#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import random

class AutoDriveNode(Node):
    """
    Autonomous driving node
    """
    def __init__(self):
        super().__init__('auto_drive_node')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.active = False

        # Subscribe to robot mode
        self.create_subscription(String, 'robot_mode', self.mode_callback, 10)

        # Timer to publish mock autonomous commands at 10Hz
        self.create_timer(0.1, self.publish_auto)

    def mode_callback(self, msg):
        self.active = (msg.data == "auto")

    def publish_auto(self):
        if not self.active:
            return

        # Example: random autonomous velocity
        twist = Twist()
        twist.linear.x = 0.3
        twist.angular.z = random.uniform(-0.3, 0.3)
        self.pub.publish(twist)
        self.get_logger().info("AutoDriveNode publishing /cmd_vel")

def main(args=None):
    rclpy.init(args=args)
    node = AutoDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down AutoDriveNode")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()


