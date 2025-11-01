from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='pkg_manipulation',
            executable='grip_manip',
            name='gripper_action_server',
            parameters=[PathJoinSubstitution([
                FindPackageShare('pkg_manipulation'), 'config', 'gripper.config.yaml']
            )],
            output='screen',
        ),
    ])
