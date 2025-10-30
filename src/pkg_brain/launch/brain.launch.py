from launch_ros.actions import Node

from launch import LaunchDescription

DEFAULT_START_ROS_BRIDGE = 'true'


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='pkg_brain',
            executable='task_brain',
            name='task_brain',
            output='screen',
        ),
    ])
