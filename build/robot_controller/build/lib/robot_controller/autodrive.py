#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import cv2
import degirum as dg
import degirum_tools
import time
import math
from simple_pid import PID
from rc_control import motor_ctrl, stop_all  # Optional if controlling real motors

# Constants (reuse from your original code)
VIDEO_SOURCE = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
BASE_SPEED = 0.4  # Use 0-1 scale for Twist messages
PID_KP, PID_KI, PID_KD = 15, 0.1, 2.0
CONFIDENCE_THRESHOLD = 0.6

class AutoDriveNode(Node):
    def __init__(self):
        super().__init__('auto_drive_node')

        self.bridge = CvBridge()
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        # Load DegiRum model
        self.model = dg.load_model(
            model_name="yolov8n--640x640_quant_hailort_hailo8_1",
            inference_host_address="@local",
            zoo_url="/home/tennibot/hailo_examples/models",
            device_type="HAILORT/HAILO8"
        )
        self.model.output_confidence_threshold = CONFIDENCE_THRESHOLD

        # Robot state
        self.pid = PID(PID_KP, PID_KI, PID_KD, setpoint=0)
        self.pid.output_limits = (-1.0, 1.0)

        # Timer for processing loop
        self.create_timer(0.05, self.loop)  # 20Hz

        # Video capture
        self.cap = cv2.VideoCapture(VIDEO_SOURCE)

    def loop(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        # Run inference
        for result in degirum_tools.predict_stream(self.model, frame):
            balls = self.process_detections(result)
            self.control_robot(balls, frame)

            # Display frame
            cv2.imshow("AutoDrive ROS", frame)
            if cv2.waitKey(1) == 27:
                rclpy.shutdown()

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

    def control_robot(self, balls, frame):
        frame_center_x = FRAME_WIDTH / 2
        twist = Twist()
        if balls:
            target = balls[0]
            error = target['center'][0] - frame_center_x
            correction = self.pid(error / frame_center_x)
            twist.linear.x = BASE_SPEED
            twist.angular.z = -correction
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = AutoDriveNode()
    rclpy.spin(node)
    node.cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
