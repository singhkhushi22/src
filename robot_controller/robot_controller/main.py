#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random
import time

class RobotModeManagerNode(Node):
    """
    Publishes the current mode: 'manual' or 'auto'
    """
    def __init__(self):
        super().__init__('robot_mode_manager')
        self.mode_pub = self.create_publisher(String, 'robot_mode', 10)

        # Timer to publish mode every 0.5s
        self.create_timer(0.5, self.publish_mode)

        # Example: start in manual mode
        self.current_mode = "manual"

    def publish_mode(self):
        # Here you can read a GPIO pin or RC switch
        # For demo, let's randomly switch every few cycles
        if random.random() < 0.05:
            self.current_mode = "auto" if self.current_mode == "manual" else "manual"

        msg = String()
        msg.data = self.current_mode
        self.mode_pub.publish(msg)
        self.get_logger().info(f"Mode: {self.current_mode}")

def main(args=None):
    rclpy.init(args=args)
    node = RobotModeManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down RobotModeManagerNode")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()






