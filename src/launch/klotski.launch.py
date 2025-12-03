from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    """
    Klotski robot system launch description.
    This launch file starts the UR5e robot driver, MoveIt, and all Klotski components.

    Usage:
    ```
    ros2 launch klotski.launch.py sim:=true start_rosbridge:=false
    ros2 launch klotski.launch.py sim:=false
    ```
    """
    # Launch configurations
    start_rosbridge = LaunchConfiguration("start_rosbridge")
    web_video_server = LaunchConfiguration("web_video_server")

    # Klotski component launches
    manip_launch = PathJoinSubstitution(
        [FindPackageShare("pkg_manipulation"), "launch", "manipulation.launch.py"]
    )

    plan_launch = PathJoinSubstitution(
        [FindPackageShare("pkg_plan"), "launch", "plan.launch.py"]
    )

    brain_launch = PathJoinSubstitution(
        [FindPackageShare("pkg_brain"), "launch", "brain.launch.py"]
    )

    sense_launch = PathJoinSubstitution(
        [FindPackageShare("pkg_sense"), "launch", "sense.launch.py"]
    )

    ur_moveit_launch = PathJoinSubstitution(
        [FindPackageShare("ur_moveit_config"), "launch", "ur_moveit.launch.py"]
    )

    return LaunchDescription(
        [
            # Launch arguments
            DeclareLaunchArgument(
                "sim",
                default_value="false",
                description="Use simulation (fake hardware) if true, real robot if false",
            ),
            DeclareLaunchArgument(
                "start_rosbridge",
                default_value="true",
                description="Whether to start rosbridge websocket",
            ),
            DeclareLaunchArgument(
                "web_video_server",
                default_value="true",
                description="Whether to start web video server",
            ),
            # ROS Bridge (optional)
            Node(
                package="rosbridge_server",
                executable="rosbridge_websocket",
                name="rosbridge_websocket",
                output="screen",
                condition=IfCondition(start_rosbridge),
            ),
            # Web Video Server (optional)
            Node(
                package="web_video_server",
                executable="web_video_server",
                name="web_video_server",
                output="screen",
                condition=IfCondition(web_video_server),
            ),
            TimerAction(
                period=3.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(ur_moveit_launch),
                        launch_arguments={
                            "ur_type": "ur5e",
                            "use_fake_hardware": LaunchConfiguration("sim"),
                            "launch_rviz": "true",
                            "description_package": "ur_with_gripper_description",
                            "description_file": "ur_with_gripper.xacro",
                        }.items(),
                    ),
                ],
            ),
            TimerAction(
                period=3.0,  # Wait 3 seconds for MoveIt to be ready
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(manip_launch),
                    ),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(plan_launch),
                    ),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(sense_launch),
                    ),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(brain_launch),
                    ),
                ],
            ),
        ]
    )
