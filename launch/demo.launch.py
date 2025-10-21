from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='pkg_sense', executable='board_perception_node', name='board_perception'),
        Node(package='pkg_plan', executable='solver_service', name='klotski_solver'),
        Node(package='pkg_manipulation', executable='arm_planner_action', name='arm_planner'),
        Node(package='pkg_manipulation', executable='gripper_node', name='gripper'),
        Node(package='pkg_brain', executable='task_brain_node', name='task_brain'),
        Node(package='pkg_ui', executable='klotski_dash_node', name='klotski_dash'),
    ])