import time

import rclpy
import serial
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node

from klotski_interfaces.action import GripPiece


class GripperActionServer(Node):

    def __init__(self):
        super().__init__("gripper_action_server")

        # Action server with proper topic name matching brain expectation
        self._action_server = ActionServer(
            self,
            GripPiece,
            "/gripper_manipulation/grip_piece",
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        # Initialize serial port with error handling
        self.serial_port = None
        self._init_serial_port()

        self.get_logger().info("GripperActionServer initialized")

    def _init_serial_port(self):
        """Initialize serial port with error handling"""
        try:
            self.serial_port = serial.Serial("/dev/ttyACM0", 9600, timeout=1)
            self.get_logger().info("Serial port /dev/ttyACM0 opened successfully")
        except serial.SerialException as e:
            self.get_logger().warn(f"Failed to open serial port: {e}")
            self.get_logger().warn("Gripper will operate in simulation mode")

    def goal_callback(self, goal_request):
        """Accept or reject incoming goals"""
        self.get_logger().info("Received goal request")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Handle goal cancellation"""
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Execute gripper action with feedback"""
        goal = goal_handle.request
        feedback_msg = GripPiece.Feedback()
        result = GripPiece.Result()

        # Determine gripper action
        action_name = "UNKNOWN"
        angle = 0

        if goal.grip_action == GripPiece.Goal.GRIP_OPEN:
            action_name = "OPEN"
            angle = 30
        elif goal.grip_action == GripPiece.Goal.GRIP_CLOSE:
            action_name = "CLOSE"
            angle = 1
        else:
            self.get_logger().error(f"Invalid grip action: {goal.grip_action}")
            goal_handle.abort()
            result.success = False
            return result

        self.get_logger().info(f"Executing gripper {action_name} (angle={angle})")

        try:
            # Send feedback during execution
            feedback_msg.progress = 0.0
            goal_handle.publish_feedback(feedback_msg)

            # Send command to gripper
            success = self._send_gripper_command(angle)

            if not success:
                self.get_logger().error("Failed to send gripper command")
                goal_handle.abort()
                result.success = False
                return result

            # Simulate gripper movement with progress feedback
            steps = 10
            for i in range(steps + 1):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    return result

                feedback_msg.progress = float(i) / steps
                goal_handle.publish_feedback(feedback_msg)
                time.sleep(0.1)  # Simulate gripper movement time

            # Success
            self.get_logger().info(f"Gripper {action_name} completed successfully")
            goal_handle.succeed()
            result.success = True

        except Exception as e:
            self.get_logger().error(f"Gripper execution failed: {e}")
            goal_handle.abort()
            result.success = False

        return result

    def _send_gripper_command(self, angle):
        """Send command to gripper hardware"""
        if self.serial_port is None:
            self.get_logger().info(f"Simulation mode: would set gripper angle to {angle}")
            return True

        try:
            command = f"{angle}\n"
            self.serial_port.write(command.encode("utf-8"))
            self.get_logger().debug(f"Sent gripper command: {command.strip()}")
            return True
        except serial.SerialException as e:
            self.get_logger().error(f"Serial communication error: {e}")
            return False

    def destroy_node(self):
        """Clean up resources"""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.get_logger().info("Serial port closed")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    gripper_action_server = GripperActionServer()

    try:
        rclpy.spin(gripper_action_server)
    except KeyboardInterrupt:
        pass
    finally:
        gripper_action_server.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
