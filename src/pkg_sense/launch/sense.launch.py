from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="pkg_sense",
                executable="sense",
                name="sense",
                output="screen",
                parameters=[
                    PathJoinSubstitution(
                        [FindPackageShare("pkg_sense"), "config", "sense.config.yaml"]
                    )
                ],
            ),
            Node(
                package="pkg_sense",
                executable="hand_safety_monitor",
                name="hand_safety_monitor",
                output="screen",
                parameters=[
                    PathJoinSubstitution(
                        [
                            FindPackageShare("pkg_sense"),
                            "config",
                            "hand_safety.config.yaml",
                        ]
                    )
                ],
            ),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "launch",
                    "realsense2_camera",
                    "rs_launch.py",
                    "enable_color:=true",
                    "enable_depth:=true",
                    "align_depth.enable:=true",
                    "rgb_camera.exposure:=1000",
                ],
                output="screen",
                name="realsense_camera",
            ),
        ]
    )
