#!/usr/bin/env python3
"""
Test client for GripperActionServer
Tests both GRIP_OPEN and GRIP_CLOSE actions
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from klotski_interfaces.action import GripPiece


class GripperTestClient(Node):

    def __init__(self):
        super().__init__('gripper_test_client')
        self._action_client = ActionClient(self, GripPiece, '/gripper_manipulation/grip_piece')
        
    def send_goal(self, grip_action):
        """Send a goal to the gripper action server"""
        action_name = "OPEN" if grip_action == GripPiece.Goal.GRIP_OPEN else "CLOSE"
        self.get_logger().info(f'Testing gripper {action_name}...')
        
        # Wait for action server
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available!')
            return False
            
        # Create goal
        goal_msg = GripPiece.Goal()
        goal_msg.grip_action = grip_action
        
        # Send goal
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
        return True

    def goal_response_callback(self, future):
        """Handle goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Handle goal result"""
        result = future.result().result
        if result.success:
            self.get_logger().info('Gripper action completed successfully!')
        else:
            self.get_logger().error('Gripper action failed!')

    def feedback_callback(self, feedback_msg):
        """Handle feedback"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Progress: {feedback.progress:.2f}')


def main(args=None):
    rclpy.init(args=args)

    client = GripperTestClient()
    
    try:
        # Test GRIP_OPEN
        print("\n=== Testing GRIP_OPEN ===")
        if client.send_goal(GripPiece.Goal.GRIP_OPEN):
            rclpy.spin_until_future_complete(client, client._send_goal_future)
            if hasattr(client, '_get_result_future'):
                rclpy.spin_until_future_complete(client, client._get_result_future)
        
        # Wait a bit between tests
        import time
        time.sleep(2)
        
        # Test GRIP_CLOSE
        print("\n=== Testing GRIP_CLOSE ===")
        if client.send_goal(GripPiece.Goal.GRIP_CLOSE):
            rclpy.spin_until_future_complete(client, client._send_goal_future)
            if hasattr(client, '_get_result_future'):
                rclpy.spin_until_future_complete(client, client._get_result_future)
        
        print("\n=== All tests completed ===")
        
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
