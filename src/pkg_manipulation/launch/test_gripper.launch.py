"""
Launch file for gripper testing
Starts the gripper action server and provides testing options
"""

from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        # Launch argument for test mode
        DeclareLaunchArgument(
            'test_mode',
            default_value='false',
            description='Whether to run in test mode with automatic testing'
        ),
        
        # Gripper action server
        Node(
            package='pkg_manipulation',
            executable='gripper_action_server',
            name='gripper_action_server',
            output='screen',
            parameters=[],
        ),
        
        # Optional test client (only if test_mode is true)
        Node(
            package='pkg_manipulation',
            executable='test_gripper_client',
            name='test_gripper_client',
            output='screen',
            condition=IfCondition(LaunchConfiguration('test_mode'))
        ),
        
        LogInfo(msg="Gripper action server launched. Use manual_gripper_control for testing."),
    ])
