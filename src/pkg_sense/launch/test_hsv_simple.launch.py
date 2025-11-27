#!/usr/bin/env python3
"""
Simple launch file for HSV Test Sense Node only

Use this when you already have the camera and sense node running.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pkg_sense',
            executable='test_hsv_sense',
            name='hsv_test_sense_node',
            output='screen'
        ),
    ])
