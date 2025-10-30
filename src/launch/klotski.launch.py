from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    start_rosbridge = LaunchConfiguration('start_rosbridge')

    manip_launch = PathJoinSubstitution([
        FindPackageShare('pkg_manipulation'), 'launch', 'manipulation.launch.py'
    ])

    plan_launch = PathJoinSubstitution([
        FindPackageShare('pkg_plan'), 'launch', 'plan.launch.py'
    ])

    brain_launch = PathJoinSubstitution([
        FindPackageShare('pkg_brain'), 'launch', 'brain.launch.py'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_rosbridge',
            default_value='true',
            description='Whether to start rosbridge websocket'
        ),

        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            condition=IfCondition(start_rosbridge),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(manip_launch),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(plan_launch),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(brain_launch),
        ),
    ])
