#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
import atexit
import serial

# === Mock RPi.GPIO if not on Raspberry Pi ===
try:
    import RPi.GPIO as GPIO
except ModuleNotFoundError:
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

# === Constants ===
CHANNEL_1 = 5
CHANNEL_2 = 6
MODE_PIN = 18
PULSE_TIMEOUT_MS = 25
RC_MID_MIN = 1460
RC_MID_MAX = 1530
RC_MIN = 1000
RC_MAX = 2000

# === Mock motor control for PC testing ===
class MotorController:
    def __init__(self, drive_callback, max_acceleration=20):
        self.current_left = 0
        self.current_right = 0
        self.max_accel = max_acceleration
        self.drive_callback = drive_callback
    def smooth_drive(self, left, right):
        print(f"[Mock] smooth_drive: L={left:.1f}, R={right:.1f}")

# --- Mock serial for PC ---
class MockSerial:
    def __init__(self, *args, **kwargs):
        self.is_open = True
    def write(self, data):
        print(f"[Mock Serial] {data.decode().strip()}")
    def close(self):
        print("[Mock Serial] closed")

# ================= RC Control Node =================
class RCControlNode(Node):
    def __init__(self):
        super().__init__('rccontrol_node')

        # Serial
        try:
            self.arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        except Exception:
            self.arduino = MockSerial()

        # GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(CHANNEL_1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(CHANNEL_2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(MODE_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

        self.motor_ctrl = MotorController(self.drive)
        self.timer = self.create_timer(0.02, self.manual_drive)  # 50Hz
        atexit.register(self.cleanup)
        self.get_logger().info("[Mock] RC Control Node initialized.")

    def pulse_in(self, pin, timeout_ms=PULSE_TIMEOUT_MS):
        """Mocked pulse measurement"""
        return 1500  # Simulate neutral RC input for testing

    def map_value(self, x, in_min, in_max, out_min, out_max):
        if in_min == in_max:
            return out_min
        x = max(in_min, min(x, in_max))
        return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

    def drive(self, left, right):
        if self.arduino and self.arduino.is_open:
            self.arduino.write(f"L:{int(left)} R:{int(right)}\n".encode())

    def stop_all(self):
        if self.arduino and self.arduino.is_open:
            self.arduino.write(b"STOP\n")

    def cleanup(self):
        self.stop_all()
        if self.arduino and self.arduino.is_open:
            self.arduino.close()
        GPIO.cleanup()
        self.get_logger().info("[Mock] GPIO cleaned up.")

    def manual_drive(self):
        throttle_value = self.pulse_in(CHANNEL_2)
        steering_value = self.pulse_in(CHANNEL_1)
        mode_switch = self.pulse_in(MODE_PIN)

        # Simple test output
        left_speed = 50
        right_speed = 50
        self.motor_ctrl.smooth_drive(left_speed, right_speed)

# ================= Main =================
def main(args=None):
    rclpy.init(args=args)
    node = RCControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("[Mock] Node interrupted")
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

