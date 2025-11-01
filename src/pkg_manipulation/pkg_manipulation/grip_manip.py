import time
from typing import Optional

import rclpy
import serial
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node

from klotski_interfaces.action import GripPiece


class GripperActionServer(Node):

    def __init__(self) -> None:
        super().__init__("gripper_action_server")
        self.declare_parameter(
            "open_angle",
            descriptor=ParameterDescriptor(
                type=rclpy.Parameter.Type.INTEGER.value,
                description="Gripper open angle",
                read_only=False
            )
        )
        self.declare_parameter(
            "close_angle",
            descriptor=ParameterDescriptor(
                type=rclpy.Parameter.Type.INTEGER.value,
                description="Gripper close angle",
                read_only=False
            )
        )
        self.declare_parameter(
            "serial_port",
            descriptor=ParameterDescriptor(
                type=rclpy.Parameter.Type.STRING.value,
                description="Serial port for gripper",
                read_only=False
            )
        )
        self.declare_parameter(
            "baud_rate",
            descriptor=ParameterDescriptor(
                type=rclpy.Parameter.Type.INTEGER.value,
                description="Baud rate for serial communication",
                read_only=False
            )
        )

        self.open_angle = self.get_parameter("open_angle").value
        self.close_angle = self.get_parameter("close_angle").value
        self.serial_port_name = self.get_parameter("serial_port").value
        self.baud_rate = self.get_parameter("baud_rate").value

        self._action_server = ActionServer(
            self,
            GripPiece,
            "/gripper_manipulation/grip_piece",
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.serial_port: Optional[serial.Serial] = None
        self._init_serial_port()

        self.get_logger().info(
            f"GripperActionServer initialized with open_angle={self.open_angle}, "
            f"close_angle={self.close_angle}, serial_port={self.serial_port_name}, "
            f"baud_rate={self.baud_rate}")

    def _init_serial_port(self) -> None:
        """Initialize serial port with error handling"""
        self.serial_port = serial.Serial(self.serial_port_name, self.baud_rate)
        self.get_logger().info(f"Serial port {self.serial_port_name} opened successfully")

    def goal_callback(self, goal_request: GripPiece.Goal) -> GoalResponse:
        """Accept or reject incoming goals"""
        self.get_logger().info("Received goal request")
        return GoalResponse.ACCEPT

    def cancel_callback(self, cancel_request: ServerGoalHandle) -> CancelResponse:
        """Handle goal cancellation"""
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle: ServerGoalHandle) -> GripPiece.Result:
        """Execute gripper action with feedback"""
        goal = goal_handle.request
        feedback_msg = GripPiece.Feedback()
        result = GripPiece.Result()

        # Determine gripper action using configurable angles
        action_name = "UNKNOWN"
        angle = 0

        if goal.grip_action == GripPiece.Goal.GRIP_OPEN:
            action_name = "OPEN"
            angle = self.open_angle
        elif goal.grip_action == GripPiece.Goal.GRIP_CLOSE:
            action_name = "CLOSE"
            angle = self.close_angle
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

    def _send_gripper_command(self, angle: int) -> bool:
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

    def destroy_node(self) -> None:
        """Clean up resources"""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.get_logger().info("Serial port closed")
        super().destroy_node()


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)

    # Create gripper with default parameters (configurable via ROS parameters)
    gripper_action_server = GripperActionServer()

    try:
        rclpy.spin(gripper_action_server)
    except KeyboardInterrupt:
        gripper_action_server.get_logger().info("Shutting down due to keyboard interrupt")
    finally:
        gripper_action_server.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
