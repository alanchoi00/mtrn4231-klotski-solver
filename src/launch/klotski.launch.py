from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

DEFAULT_START_ROS_BRIDGE = True
DEFAULT_OPEN_ANGLE = 30
DEFAULT_CLOSE_ANGLE = 1
DEFAULT_SERIAL_PORT = "/dev/ttyACM0"
DEFAULT_BAUD_RATE = 9600


def generate_launch_description():
    start_rosbridge = LaunchConfiguration('start_rosbridge')  # Removed comma here
    open_angle = LaunchConfiguration('open_angle')
    close_angle = LaunchConfiguration('close_angle')
    serial_port = LaunchConfiguration('serial_port')
    baud_rate = LaunchConfiguration('baud_rate')

    return LaunchDescription([
        DeclareLaunchArgument('start_rosbridge', default_value=str(DEFAULT_START_ROS_BRIDGE)),  # Convert bool to string
        DeclareLaunchArgument('open_angle', default_value=str(DEFAULT_OPEN_ANGLE)),             # Convert int to string
        DeclareLaunchArgument('close_angle', default_value=str(DEFAULT_CLOSE_ANGLE)),           # Convert int to string
        DeclareLaunchArgument('serial_port', default_value=DEFAULT_SERIAL_PORT),
        DeclareLaunchArgument('baud_rate', default_value=str(DEFAULT_BAUD_RATE)),               # Convert int to string

        Node(
            package='pkg_manipulation',
            executable='grip_manip',
            name='gripper_action_server',
            parameters=[{
                'open_angle': open_angle,
                'close_angle': close_angle,
                'serial_port': serial_port,
                'baud_rate': baud_rate,
            }],
            output='screen',
            arguments=['--ros-args', '--log-level', 'DEBUG']
        ),

        Node(
            package='pkg_brain',
            executable='task_brain',
            name='task_brain',
            output='screen',
            parameters=[{
                'auto_continue': True,
                'relocalise_between_moves': True,
            }],
            arguments=['--ros-args', '--log-level', 'DEBUG']
        ),

        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            condition=IfCondition(start_rosbridge),
        ),

        Node(
            package='pkg_plan',
            executable='klotski_solve_service',
            name='solve_service',
            output='screen',
            parameters=[{
                'use_sim_time': False,
            }],
            arguments=['--ros-args', '--log-level', 'DEBUG']
        )
    ])
