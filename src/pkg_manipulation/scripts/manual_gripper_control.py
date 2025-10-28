#!/usr/bin/env python3
"""
Manual gripper control tool for testing
Usage: 
  ros2 run pkg_manipulation manual_gripper_control open
  ros2 run pkg_manipulation manual_gripper_control close
"""

import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from klotski_interfaces.action import GripPiece


class ManualGripperControl(Node):

    def __init__(self):
        super().__init__('manual_gripper_control')
        self._action_client = ActionClient(self, GripPiece, '/gripper_manipulation/grip_piece')
        
    def control_gripper(self, action_str):
        """Send gripper command based on string input"""
        if action_str.lower() == 'open':
            grip_action = GripPiece.Goal.GRIP_OPEN
            action_name = "OPEN"
        elif action_str.lower() == 'close':
            grip_action = GripPiece.Goal.GRIP_CLOSE
            action_name = "CLOSE"
        else:
            self.get_logger().error(f"Invalid action: {action_str}. Use 'open' or 'close'")
            return False
            
        self.get_logger().info(f'Sending gripper {action_name} command...')
        
        # Wait for action server
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Gripper action server not available!')
            return False
            
        # Create and send goal
        goal_msg = GripPiece.Goal()
        goal_msg.grip_action = grip_action
        
        future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        
        # Wait for goal to be accepted
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by server')
            return False

        self.get_logger().info('Goal accepted, waiting for result...')
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result().result
        if result.success:
            self.get_logger().info(f'Gripper {action_name} completed successfully!')
            return True
        else:
            self.get_logger().error(f'Gripper {action_name} failed!')
            return False

    def feedback_callback(self, feedback_msg):
        """Print progress feedback"""
        progress = feedback_msg.feedback.progress
        bar_length = 20
        filled_length = int(bar_length * progress)
        bar = '█' * filled_length + '-' * (bar_length - filled_length)
        print(f'\rProgress: [{bar}] {progress:.1%}', end='', flush=True)
        if progress >= 1.0:
            print()  # New line when complete


def main():
    if len(sys.argv) != 2:
        print("Usage: ros2 run pkg_manipulation manual_gripper_control <open|close>")
        print("Examples:")
        print("  ros2 run pkg_manipulation manual_gripper_control open")
        print("  ros2 run pkg_manipulation manual_gripper_control close")
        return

    action = sys.argv[1]
    
    rclpy.init()
    
    controller = ManualGripperControl()
    
    try:
        success = controller.control_gripper(action)
        if success:
            print(f"✅ Gripper {action} completed successfully!")
        else:
            print(f"❌ Gripper {action} failed!")
    except KeyboardInterrupt:
        print("\n🛑 Operation cancelled by user")
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
