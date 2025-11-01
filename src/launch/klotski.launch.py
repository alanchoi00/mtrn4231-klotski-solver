from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable

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
    """
    Klotski robot system launch description.
    This launch file starts the manipulation, planning, and brain nodes,
    along with an optional rosbridge websocket server.
    Usage:
    ```
    ros2 launch klotski.launch.py start_rosbridge:=true|false
    ```
    """
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

    # Parameters for move_to_marker node
    params = [
        get_robot_description(),
        get_robot_description_semantic(),
        get_robot_description_kinematics(),
    ]

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

        Node(
            package='ur5emoveit', 
            executable='move_to_marker', 
            name='move_to_marker', 
            output='screen', 
            parameters=params
        ),
    ])
