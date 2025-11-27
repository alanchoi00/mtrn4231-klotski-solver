from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def validate_launch_args(context, *args, **kwargs):
    sim_value = context.launch_configurations.get('sim', 'false').lower()
    robot_ip_value = context.launch_configurations.get('robot_ip', '')

    if sim_value == 'false' and not robot_ip_value:
        raise RuntimeError(
            "robot_ip launch argument is required when sim:=false. "
            "Please provide the IP address of your UR5e robot. "
            "Usage: ros2 launch klotski.launch.py sim:=false robot_ip:=192.168.1.100"
        )

    return []


def generate_launch_description():
    """
    Klotski robot system launch description.
    This launch file starts the UR5e robot driver, MoveIt, and all Klotski components.

    Usage:
    ```
    ros2 launch klotski.launch.py sim:=true start_rosbridge:=false
    ros2 launch klotski.launch.py sim:=false robot_ip:=192.168.1.100
    ```
    """
    # Launch configurations
    sim = LaunchConfiguration('sim')
    start_rosbridge = LaunchConfiguration('start_rosbridge')
    robot_ip = LaunchConfiguration('robot_ip')

    # Klotski component launches
    manip_launch = PathJoinSubstitution([
        FindPackageShare('pkg_manipulation'), 'launch', 'manipulation.launch.py'
    ])

    plan_launch = PathJoinSubstitution([
        FindPackageShare('pkg_plan'), 'launch', 'plan.launch.py'
    ])

    brain_launch = PathJoinSubstitution([
        FindPackageShare('pkg_brain'), 'launch', 'brain.launch.py'
    ])

    sense_launch = PathJoinSubstitution([
        FindPackageShare('pkg_sense'), 'launch', 'sense.launch.py'
    ])

    # UR5e launches
    ur_control_launch = PathJoinSubstitution([
        FindPackageShare('ur_robot_driver'), 'launch', 'ur_control.launch.py'
    ])

    ur_moveit_launch = PathJoinSubstitution([
        FindPackageShare('ur_moveit_config'), 'launch', 'ur_moveit.launch.py'
    ])

    tf_launch = PathJoinSubstitution([
        FindPackageShare('pkg_tf'),  
        'launch',
        'pkg_tf.launch.py'
    ])


    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'sim',
            default_value='false',
            description='Use simulation (fake hardware) if true, real robot if false'
        ),

        DeclareLaunchArgument(
            'start_rosbridge',
            default_value='true',
            description='Whether to start rosbridge websocket'
        ),

        DeclareLaunchArgument(
            'robot_ip',
            default_value='',
            description='IP address of the real UR5e robot (required when sim:=false)'
        ),

        # Validate launch arguments
        OpaqueFunction(function=validate_launch_args),

        # ROS Bridge (optional)
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            condition=IfCondition(start_rosbridge),
        ),

        # STEP 1: Start UR5e Robot Driver FIRST
        # UR5e Robot Driver - Simulation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ur_control_launch),
            launch_arguments={
                'ur_type': 'ur5e',
                'robot_ip': 'yyy.yyy.yyy.yyy',
                'initial_joint_controller': 'joint_trajectory_controller',
                'use_fake_hardware': 'true',
                'launch_rviz': 'false',
            }.items(),
            condition=IfCondition(sim),
        ),

        # UR5e Robot Driver - Real Robot
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ur_control_launch),
            launch_arguments={
                'ur_type': 'ur5e',
                'robot_ip': robot_ip,
                'use_fake_hardware': 'false',
                'launch_rviz': 'false',
            }.items(),
            condition=UnlessCondition(sim),
        ),

        # STEP 2: Start MoveIt after robot driver is ready
        # MoveIt Configuration - 10 second delay to ensure robot driver is ready
        TimerAction(
            period=10.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(ur_moveit_launch),
                    launch_arguments={
                        'ur_type': 'ur5e',
                        'use_fake_hardware': LaunchConfiguration('sim'),
                        'launch_rviz': 'true',
                    }.items(),
                ),
            ]
        ),

        # STEP 3: Start Klotski Components after MoveIt is ready
        TimerAction(
            period=15.0,  # Wait 15 seconds for MoveIt to be ready
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(manip_launch),
                ),

                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(plan_launch),
                ),

                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(brain_launch),
                ),

                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(sense_launch),
                ),

                IncludeLaunchDescription(  
                    PythonLaunchDescriptionSource(tf_launch),
                ),
            ],
        ),
    ])

