#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
import atexit
import serial

# ================= GPIO Setup =================
try:
    import RPi.GPIO as GPIO
except ModuleNotFoundError:
    # Mock GPIO for PC testing
    class GPIO:
        BCM = None
        IN = None
        PUD_DOWN = None
        HIGH = 1
        LOW = 0
        @staticmethod
        def setmode(mode): pass
        @staticmethod
        def setup(pin, mode, pull_up_down=None): pass
        @staticmethod
        def input(pin): return 0
        @staticmethod
        def cleanup(): pass

# ================= Constants =================
CHANNEL_1 = 5   # Steering channel
CHANNEL_2 = 6   # Throttle channel
MODE_PIN   = 18 # Mode switch
PULSE_TIMEOUT_MS = 25

RC_MIN = 1000
RC_MAX = 2000
RC_MID_MIN = 1460
RC_MID_MAX = 1530

# ================= Motor Controller =================
class MotorController:
    def __init__(self, drive_callback, max_acceleration=20):
        self.current_left = 0
        self.current_right = 0
        self.max_accel = max_acceleration
        self.drive_callback = drive_callback

    def smooth_drive(self, left, right):
        # Limit acceleration change
        delta_left = left - self.current_left
        delta_right = right - self.current_right

        if abs(delta_left) > self.max_accel:
            delta_left = self.max_accel if delta_left > 0 else -self.max_accel
        if abs(delta_right) > self.max_accel:
            delta_right = self.max_accel if delta_right > 0 else -self.max_accel

        self.current_left += delta_left
        self.current_right += delta_right

        # Send to motors
        self.drive_callback(self.current_left, self.current_right)

# ================= RCControl Node =================
class RCControlNode(Node):
    def __init__(self):
        super().__init__('rccontrol_node')

        # Serial port (Arduino / ESC controller)
        try:
            self.arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        except Exception:
            self.arduino = None
            self.get_logger().warn("Arduino not found. Using PC simulation mode.")

        # GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(CHANNEL_1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(CHANNEL_2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(MODE_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

        # Motor controller
        self.motor_ctrl = MotorController(self.drive)
        self.timer = self.create_timer(0.02, self.manual_drive)  # 50Hz
        atexit.register(self.cleanup)

        self.get_logger().info("RCControlNode initialized.")

    # ---------------- Pulse Measurement ----------------
    def pulse_in(self, pin, timeout_ms=PULSE_TIMEOUT_MS):
        """
        Measure pulse width in microseconds from RC receiver.
        Returns 0 on timeout.
        """
        start_time = time.time()
        # Wait for HIGH
        while GPIO.input(pin) == GPIO.LOW:
            if (time.time() - start_time) * 1000 > timeout_ms:
                return 0
        pulse_start = time.time()
        # Wait for LOW
        while GPIO.input(pin) == GPIO.HIGH:
            if (time.time() - start_time) * 1000 > timeout_ms:
                break
        pulse_end = time.time()
        pulse_width = (pulse_end - pulse_start) * 1_000_000  # microseconds
        return int(pulse_width)

    # ---------------- Value Mapping ----------------
    def map_value(self, x, in_min, in_max, out_min, out_max):
        if in_min == in_max:
            return out_min
        x = max(in_min, min(x, in_max))
        return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

    # ---------------- Drive Commands ----------------
    def drive(self, left, right):
        """Send motor commands via serial"""
        if self.arduino and self.arduino.is_open:
            command = f"L:{int(left)} R:{int(right)}\n"
            self.arduino.write(command.encode())
        else:
            print(f"[SIM] Drive command: L={left} R={right}")

    def stop_all(self):
        """Emergency stop"""
        if self.arduino and self.arduino.is_open:
            self.arduino.write(b"STOP\n")
        else:
            print("[SIM] Motors stopped")

    def cleanup(self):
        self.stop_all()
        if self.arduino and self.arduino.is_open:
            self.arduino.close()
        GPIO.cleanup()
        self.get_logger().info("RCControlNode cleaned up.")

    # ---------------- Manual Drive Loop ----------------
    def manual_drive(self):
        throttle_value = self.pulse_in(CHANNEL_2)
        steering_value = self.pulse_in(CHANNEL_1)
        mode_switch = self.pulse_in(MODE_PIN)

        if throttle_value == 0 or steering_value == 0:
            self.stop_all()
            return

        # Map RC values to motor speeds (-100 to 100)
        left_speed = self.map_value(throttle_value + steering_value, RC_MIN, RC_MAX, -100, 100)
        right_speed = self.map_value(throttle_value - steering_value, RC_MIN, RC_MAX, -100, 100)

        # Smooth drive to motors
        self.motor_ctrl.smooth_drive(left_speed, right_speed)

# ================= Main =================
def main(args=None):
    rclpy.init(args=args)
    node = RCControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("RCControlNode interrupted")
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()


