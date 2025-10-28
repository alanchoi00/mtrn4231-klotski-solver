from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare

def get_robot_description():
    jl  = PathJoinSubstitution([FindPackageShare("ur_description"), "config", "ur5e", "joint_limits.yaml"])
    kin = PathJoinSubstitution([FindPackageShare("ur_description"), "config", "ur5e", "default_kinematics.yaml"])
    phys= PathJoinSubstitution([FindPackageShare("ur_description"), "config", "ur5e", "physical_parameters.yaml"])
    vis = PathJoinSubstitution([FindPackageShare("ur_description"), "config", "ur5e", "visual_parameters.yaml"])
    return {"robot_description": Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro"]), " ",
        "robot_ip:=172.17.0.2 ",
        "joint_limit_params:=", jl, " ",
        "kinematics_params:=", kin, " ",
        "physical_params:=", phys, " ",
        "visual_params:=", vis, " ",
        "safety_limits:=true safety_pos_margin:=0.15 safety_k_position:=20 ",
        "name:=ur ur_type:=ur5e prefix:=",'""'," ",
    ])}

def get_robot_description_semantic():
    return {"robot_description_semantic": Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "srdf", "ur.srdf.xacro"]), " ",
        "name:=ur prefix:=",'""'," ",
    ])}

def get_robot_description_kinematics():
    return {"robot_description_kinematics": PathJoinSubstitution(
        [FindPackageShare("ur_moveit_config"), "config", "kinematics.yaml"]
    )}

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

        Node(
            package='ur5emoveit', 
            executable='move_to_marker', 
            name='move_to_marker', 
            output='screen', 
            parameters=params),
    ])
