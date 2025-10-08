#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import cv2
import degirum as dg
import degirum_tools
from simple_pid import PID
import time
from rc_control import RCControlNode  # Use your hardware RCControlNode

# ================= Constants =================
VIDEO_SOURCE = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
BASE_SPEED = 50  # Use -100 to 100 for motor PWM
PID_KP, PID_KI, PID_KD = 0.5, 0.0, 0.1
CONFIDENCE_THRESHOLD = 0.6

class AutoDriveNode(Node):
    """
    Autonomous driving node using YOLO detection and motor control
    """
    def __init__(self):
        super().__init__('auto_drive_node')

        self.bridge = CvBridge()
        self.rc_node = RCControlNode()  # Use the RCControlNode to drive motors
        self.pid = PID(PID_KP, PID_KI, PID_KD, setpoint=0)
        self.pid.output_limits = (-1.0, 1.0)

        # Load DegiRum model
        self.model = dg.load_model(
            model_name="yolov8n--640x640_quant_hailort_hailo8_1",
            inference_host_address="@local",
            zoo_url="/home/tennibot/hailo_examples/models",
            device_type="HAILORT/HAILO8"
        )
        self.model.output_confidence_threshold = CONFIDENCE_THRESHOLD

        # Video capture
        self.cap = cv2.VideoCapture(VIDEO_SOURCE)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

        # Timer loop at 20Hz
        self.create_timer(0.05, self.loop)

    # ---------------- Main Loop ----------------
    def loop(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to read frame")
            return

        # Run inference
        for result in degirum_tools.predict_stream(self.model, frame):
            balls = self.process_detections(result)
            self.control_robot(balls, frame)

        # Display frame (optional)
        cv2.imshow("AutoDrive", frame)
        if cv2.waitKey(1) & 0xFF == 27:  # ESC to exit
            rclpy.shutdown()

    # ---------------- Detection Processing ----------------
    def process_detections(self, inference_result):
        tennisballs = []
        for detection in inference_result.results:
            if detection['label'] != "tennisball":
                continue
            bbox = detection['bbox']
            center_x = (bbox[0] + bbox[2]) / 2
            center_y = (bbox[1] + bbox[3]) / 2
            area = (bbox[2] - bbox[0]) * (bbox[3] - bbox[1])
            tennisballs.append({'bbox': bbox, 'center': (center_x, center_y), 'area': area})
        return tennisballs

    # ---------------- Robot Control ----------------
    def control_robot(self, balls, frame):
        frame_center_x = FRAME_WIDTH / 2

        if balls:
            target = balls[0]
            error = target['center'][0] - frame_center_x
            correction = self.pid(error / frame_center_x)

            # Map PID output to left/right motor speed
            left_speed = BASE_SPEED - int(correction * BASE_SPEED)
            right_speed = BASE_SPEED + int(correction * BASE_SPEED)
            self.rc_node.motor_ctrl.smooth_drive(left_speed, right_speed)
        else:
            # Stop motors if no target
            self.rc_node.stop_all()

    # ---------------- Cleanup ----------------
    def cleanup(self):
        self.rc_node.stop_all()
        self.cap.release()
        cv2.destroyAllWindows()
        self.get_logger().info("AutoDriveNode cleanup complete.")

# ================= Main =================
def main(args=None):
    rclpy.init(args=args)
    node = AutoDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("AutoDriveNode interrupted")
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

