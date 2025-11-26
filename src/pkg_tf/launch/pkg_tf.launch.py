from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 静态TF：相机到base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_to_base_tf',
            arguments=[
                '1.30893', '0.059849', '0.680372',
                '-0.399127', '-0.0147756', '0.916733', '-0.0089035',
                'base_link', 'camera_link'
            ],
            output='screen'
        ),
        
        # Board TF Publisher
        Node(
            package='pkg_tf',
            executable='board_tf_publisher',
            name='board_tf_publisher',
            output='screen',
            parameters=[{
                'board_width': 0.20,
                'board_length': 0.25,
                'cell_size': 0.05,
                'camera_frame': 'camera_link',
                'base_frame': 'base_link',
                'board_frame': 'board_frame',
            }]
        ),
    ])
