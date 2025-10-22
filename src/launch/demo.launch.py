from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    start_rosbridge = LaunchConfiguration('start_rosbridge')
    return LaunchDescription([
        DeclareLaunchArgument('start_rosbridge', default_value='true'),

        Node(
            package='pkg_brain',
            executable='task_brain',
            name='task_brain',
            output='screen',
            parameters=[{
                'auto_continue': True,
                'relocalise_between_moves': True,
            }],
        ),

        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            condition=IfCondition(start_rosbridge),
        ),
    ])