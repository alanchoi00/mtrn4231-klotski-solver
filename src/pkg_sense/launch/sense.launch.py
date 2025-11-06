from launch_ros.actions import Node
from launch import LaunchDescription

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
    ])
