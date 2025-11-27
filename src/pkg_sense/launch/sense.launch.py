from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pkg_sense',
            executable='sense',
            name='sense',
            output='screen',
            parameters=[{
                'frame_id': 'map'
            }]
        ),

        ExecuteProcess(
            cmd=['ros2', 'launch', 'realsense2_camera', 'rs_launch.py', 
                 'enable_color:=true', 'enable_depth:=true', 'enable_infra1:=false', 'enable_infra2:=false'],
            output='screen',
            name='realsense_camera'
        ),
    ])
