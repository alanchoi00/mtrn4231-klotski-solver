from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='pkg_brain',
            executable='task_brain',
            name='task_brain',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare("pkg_brain"),
                    "config",
                    "brain.config.yaml"
                ])
            ],
            arguments=['--ros-args', '--log-level', 'debug'],
        ),
    ])
