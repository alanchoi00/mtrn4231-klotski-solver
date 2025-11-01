from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='pkg_plan',
            executable='klotski_solve_service',
            name='solve_service',
            output='screen',
            parameters=[{
                'use_sim_time': False,
            }]
        )
    ])
