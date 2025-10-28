#!/usr/bin/env python3
"""
Quick validation test for GripperActionServer
Tests basic functionality without requiring hardware
"""

import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from klotski_interfaces.action import GripPiece
from pkg_manipulation.grip_manip import GripperActionServer


def test_action_interface():
    """Test the action interface definitions"""
    print("🔍 Testing GripPiece action interface...")
    
    # Test goal constants
    assert hasattr(GripPiece.Goal, 'GRIP_OPEN')
    assert hasattr(GripPiece.Goal, 'GRIP_CLOSE')
    assert GripPiece.Goal.GRIP_OPEN == 0
    assert GripPiece.Goal.GRIP_CLOSE == 1
    print("✅ Goal constants are correct")
    
    # Test result structure
    result = GripPiece.Result()
    assert hasattr(result, 'success')
    result.success = True
    print("✅ Result structure is correct")
    
    # Test feedback structure
    feedback = GripPiece.Feedback()
    assert hasattr(feedback, 'progress')
    feedback.progress = 0.5
    print("✅ Feedback structure is correct")


def test_server_initialization():
    """Test server initialization"""
    print("\n🔍 Testing GripperActionServer initialization...")
    
    try:
        server = GripperActionServer()
        print("✅ Server initialized successfully")
        
        # Check action server setup
        assert server._action_server is not None
        print("✅ Action server created")
        
        # Clean up
        server.destroy_node()
        print("✅ Server cleanup successful")
        
    except Exception as e:
        print(f"❌ Server initialization failed: {e}")
        return False
    
    return True


def run_integration_test():
    """Run a full integration test"""
    print("\n🔍 Running integration test...")
    
    rclpy.init()
    
    # Start server in separate thread
    server = GripperActionServer()
    server_thread = threading.Thread(target=lambda: rclpy.spin(server))
    server_thread.daemon = True
    server_thread.start()
    
    # Give server time to start
    time.sleep(1)
    
    # Create client
    client_node = Node('test_client')
    action_client = ActionClient(client_node, GripPiece, '/gripper_manipulation/grip_piece')
    
    try:
        # Wait for server
        print("⏳ Waiting for action server...")
        if not action_client.wait_for_server(timeout_sec=5.0):
            print("❌ Action server not available within timeout")
            return False
        print("✅ Action server found")
        
        # Test GRIP_OPEN
        print("🔧 Testing GRIP_OPEN...")
        goal = GripPiece.Goal()
        goal.grip_action = GripPiece.Goal.GRIP_OPEN
        
        future = action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(client_node, future, timeout_sec=5.0)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            print("❌ GRIP_OPEN goal rejected")
            return False
        print("✅ GRIP_OPEN goal accepted")
        
        # Get result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(client_node, result_future, timeout_sec=10.0)
        
        result = result_future.result().result
        if not result.success:
            print("❌ GRIP_OPEN failed")
            return False
        print("✅ GRIP_OPEN completed successfully")
        
        # Test GRIP_CLOSE
        print("🔧 Testing GRIP_CLOSE...")
        goal.grip_action = GripPiece.Goal.GRIP_CLOSE
        
        future = action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(client_node, future, timeout_sec=5.0)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            print("❌ GRIP_CLOSE goal rejected")
            return False
        print("✅ GRIP_CLOSE goal accepted")
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(client_node, result_future, timeout_sec=10.0)
        
        result = result_future.result().result
        if not result.success:
            print("❌ GRIP_CLOSE failed")
            return False
        print("✅ GRIP_CLOSE completed successfully")
        
        print("🎉 All integration tests passed!")
        return True
    
    except Exception as e:
        print(f"❌ Integration test failed: {e}")
        return False
    
    finally:
        client_node.destroy_node()
        server.destroy_node()
        rclpy.shutdown()


def main():
    print("🧪 GripperActionServer Validation Tests")
    print("=" * 50)
    
    # Test 1: Action interface
    test_action_interface()
    
    # Test 2: Server initialization (without ROS spinning)
    if not test_server_initialization():
        print("❌ Server initialization test failed - stopping")
        return
    
    # Test 3: Integration test
    if run_integration_test():
        print("\n🎉 ALL TESTS PASSED!")
        print("\nThe GripperActionServer is ready for use!")
        print("\nNext steps:")
        print("1. Build: colcon build --packages-select pkg_manipulation")
        print("2. Source: source install/setup.bash")
        print("3. Run: ros2 run pkg_manipulation gripper_action_server")
        print("4. Test: ros2 run pkg_manipulation manual_gripper_control open")
    else:
        print("\n❌ SOME TESTS FAILED!")
        print("Check the error messages above for details.")


if __name__ == '__main__':
    main()
