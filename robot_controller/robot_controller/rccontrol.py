#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time
import atexit
import RPi.GPIO as GPIO
import serial
import os

# ===================== RC Input Pins =====================
channel_1 = 5  # Steering
channel_2 = 6  # Throttle
mode_pin = 18  # Mode switch

# RC Pulse thresholds
PULSE_TIMEOUT_MS = 25
RC_MID_MIN = 1460
RC_MID_MAX = 1530
RC_MIN = 1000
RC_MAX = 2000

# ===================== Arduino Serial =====================
try:
    arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
    time.sleep(2)
except serial.SerialException as e:
    print(f"Error opening serial port: {e}")
    arduino = None

# ===================== Motor Control =====================
class MotorController:
    def __init__(self, max_accel=20):
        self.current_left = 0
        self.current_right = 0
        self.max_accel = max_accel

    def smooth_drive(self, target_left, target_right):
        left_diff = target_left - self.current_left
        right_diff = target_right - self.current_right
        left_step = max(-self.max_accel, min(self.max_accel, left_diff))
        right_step = max(-self.max_accel, min(self.max_accel, right_diff))
        self.current_left += left_step
        self.current_right += right_step
        self.drive(self.current_left, self.current_right)

    def drive(self, left, right):
        if arduino and arduino.is_open:
            arduino.write(f"L:{int(left)} R:{int(right)}\n".encode())
        else:
            print(f"Serial unavailable. Left:{left} Right:{right}")

motor_ctrl = MotorController()

# ===================== GPIO Setup =====================
def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(channel_1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(channel_2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(mode_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# ===================== Utility Functions =====================
def pulse_in(pin, timeout_ms=PULSE_TIMEOUT_MS):
    start = time.monotonic()
    while GPIO.input(pin) == GPIO.HIGH:
        if (time.monotonic() - start) * 1000 > timeout_ms:
            return 0
    start_edge = time.monotonic()
    while GPIO.input(pin) == GPIO.LOW:
        if (time.monotonic() - start_edge) * 1000 > timeout_ms:
            return 0
    t1 = time.monotonic()
    start_fall = time.monotonic()
    while GPIO.input(pin) == GPIO.HIGH:
        if (time.monotonic() - start_fall) * 1000 > timeout_ms:
            return 0
    t2 = time.monotonic()
    return int((t2 - t1) * 1_000_000)

def map_value(x, in_min, in_max, out_min, out_max):
    if in_min == in_max:
        return out_min
    x = max(in_min, min(x, in_max))
    return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

def read_rc_channels():
    throttle = pulse_in(channel_2)
    steering = pulse_in(channel_1)
    mode = pulse_in(mode_pin)
    return throttle, steering, mode

def stop_all():
    motor_ctrl.smooth_drive(0, 0)

def cleanup():
    stop_all()
    if arduino and arduino.is_open:
        arduino.close()
    GPIO.cleanup()
    print("RCControlNode cleanup complete.")

atexit.register(cleanup)

# ===================== ROS 2 Node =====================
class RCControlNode(Node):
    def __init__(self):
        super().__init__('rc_control_node')

        # Subscribe to mode topic from main_node
        self.mode_sub = self.create_subscription(
            String,
            'mode',
            self.mode_callback,
            10
        )

        # Publisher for RC values
        self.rc_pub = self.create_publisher(Twist, 'rc_values', 10)

        self.current_mode = 'manual'
        self.timer = self.create_timer(0.05, self.publish_rc)  # 20Hz

    def mode_callback(self, msg):
        self.current_mode = msg.data
        self.get_logger().info(f'Mode changed to: {self.current_mode}')

    def publish_rc(self):
        if self.current_mode != 'manual':
            stop_all()
            return

        throttle, steering, _ = read_rc_channels()

        # Map RC input to motor speeds
        forward = 0
        correction = 0

        if throttle > RC_MID_MAX:
            forward = map_value(throttle, RC_MID_MAX, RC_MAX, 0, 100)
        elif throttle < RC_MID_MIN:
            forward = map_value(throttle, RC_MIN, RC_MID_MIN, -100, 0)

        if steering > RC_MID_MAX:
            correction = map_value(steering, RC_MID_MAX, RC_MAX, 0, 50)
        elif steering < RC_MID_MIN:
            correction = map_value(steering, RC_MIN, RC_MID_MIN, -50, 0)

        left_speed = forward + correction
        right_speed = forward - correction

        max_mag = max(abs(left_speed), abs(right_speed))
        if max_mag > 100:
            scale = 100 / max_mag
            left_speed *= scale
            right_speed *= scale

        motor_ctrl.smooth_drive(left_speed, right_speed)

        # Publish ROS topic
        msg = Twist()
        msg.linear.x = forward
        msg.angular.z = correction
        self.rc_pub.publish(msg)

        self.get_logger().info(f'Throttle: {forward}, Steering: {correction}')

# ===================== Main =====================
def main(args=None):
    setup_gpio()
    rclpy.init(args=args)
    node = RCControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()




