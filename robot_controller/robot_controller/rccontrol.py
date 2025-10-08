#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class RCControlNode(Node):
    """
    Manual driving node
    """
    def __init__(self):
        super().__init__('rc_control_node')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.active = False

        # Subscribe to robot mode
        self.create_subscription(String, 'robot_mode', self.mode_callback, 10)

        # Timer to publish mock RC commands at 10Hz
        self.create_timer(0.1, self.publish_rc)

    def mode_callback(self, msg):
        self.active = (msg.data == "manual")

    def publish_rc(self):
        if not self.active:
            return

        # Example: generate mock RC commands
        twist = Twist()
        twist.linear.x = 0.5
        twist.angular.z = 0.1
        self.pub.publish(twist)
        self.get_logger().info("RCControlNode publishing /cmd_vel")

def main(args=None):
    rclpy.init(args=args)
    node = RCControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down RCControlNode")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()



