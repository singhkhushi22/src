import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

# Import your class-based nodes
from robot_controller.rccontrol import RCControlNode
from robot_controller.autodrive import AutoDriveNode


class RobotModeManager(Node):
    """
    ROS2 Node to manage driving mode between manual and autonomous.
    """
    def __init__(self):
        super().__init__('robot_mode_manager')

        # ROS Publisher to broadcast mode status
        self.mode_pub = self.create_publisher(String, 'robot_mode', 10)

        # Timer to continuously check RC mode (every 0.1 seconds)
        self.timer = self.create_timer(0.1, self.loop)

        # Initialize the nodes
        self.rc_node = RCControlNode()
        self.auto_node = AutoDriveNode()

        self.get_logger().info("RobotModeManager node started.")

        # Track last mode for logging
        self.last_mode = None

    def loop(self):
        """
        Main loop that checks RC mode and controls the robot.
        """
        try:
            # Read RC mode switch directly from rc_node
            mode_switch = self.rc_node.pulse_in(self.rc_node.MODE_PIN)

            # Handle no signal
            if mode_switch == 0:
                self.publish_mode("no_signal")
                if self.last_mode != "no_signal":
                    self.get_logger().warn("No RC signal detected! Emergency stop.")
                    self.last_mode = "no_signal"
                self.rc_node.stop_all()
                return

            # Manual mode
            if mode_switch < 1300:
                self.publish_mode("manual")
                if self.last_mode != "manual":
                    self.get_logger().info("Manual Drive Mode Active")
                    self.last_mode = "manual"
                self.rc_node.manual_drive()  # Call manual drive method

            # Autonomous mode
            else:
                self.publish_mode("auto")
                if self.last_mode != "auto":
                    self.get_logger().info("Auto Drive Mode Active")
                    self.last_mode = "auto"
                # Run auto drive loop
                self.auto_node.loop()

        except Exception as e:
            self.get_logger().error(f"Error in loop: {e}")
            self.rc_node.stop_all()
            time.sleep(0.1)

    def publish_mode(self, mode):
        """Publishes the current robot mode to the ROS2 topic."""
        msg = String()
        msg.data = mode
        self.mode_pub.publish(msg)

    def destroy_node(self):
        """Cleanup when shutting down."""
        self.get_logger().info("Shutting down... Stopping all motors and cleaning up nodes.")
        self.rc_node.cleanup()
        self.auto_node.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RobotModeManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt detected. Exiting...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()





