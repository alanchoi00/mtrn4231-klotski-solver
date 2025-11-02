from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution


def get_robot_description():
    """Generate robot description for UR5e arm"""
    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur5e", "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur5e", "default_kinematics.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur5e", "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur5e", "visual_parameters.yaml"]
    )

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro"]), " ",
        "robot_ip:=192.168.0.100 ",
        "joint_limit_params:=", joint_limit_params, " ",
        "kinematics_params:=", kinematics_params, " ",
        "physical_params:=", physical_params, " ",
        "visual_params:=", visual_params, " ",
        "safety_limits:=true ",
        "safety_pos_margin:=0.15 ",
        "safety_k_position:=20 ",
        "name:=ur ",
        "ur_type:=ur5e ",
        "prefix:=", '""', " ",
    ])

    return {"robot_description": robot_description_content}


def get_robot_description_semantic():
    """Generate robot description semantic (SRDF) for UR5e"""
    robot_description_semantic_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "srdf", "ur.srdf.xacro"]), " ",
        "name:=ur ",
        "prefix:=", '""', " ",
    ])
    return {"robot_description_semantic": robot_description_semantic_content}


def generate_launch_description():
    # Get robot descriptions
    robot_description = get_robot_description()
    robot_description_semantic = get_robot_description_semantic()
    robot_description_kinematics = PathJoinSubstitution([
        FindPackageShare('ur_moveit_config'),
        'config',
        'kinematics.yaml'
    ])

    # Gripper node with config
    gripper_node = Node(
        package="pkg_manipulation",
        executable="gripper_manipulator",
        name="gripper_manipulator",
        parameters=[
            PathJoinSubstitution([
                FindPackageShare("pkg_manipulation"),
                "config",
                "gripper.config.yaml"
            ])
        ],
    )

    # Arm manipulator node with robot description and config
    arm_node = Node(
        package="pkg_manipulation",
        executable="arm_manipulator",
        name="arm_manipulator",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            PathJoinSubstitution([
                FindPackageShare("pkg_manipulation"),
                "config",
                "arm.config.yaml"
            ])
        ],
    )

    return LaunchDescription([
        gripper_node,
        arm_node,
    ])
