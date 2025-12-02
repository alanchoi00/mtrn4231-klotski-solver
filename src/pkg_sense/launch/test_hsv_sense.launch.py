#!/usr/bin/env python3
"""
Launch file for HSV Test Sense Node

This launch file starts the HSV testing tool along with the camera and regular sense node.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
import os

def generate_launch_description():
    return LaunchDescription([
        # Start the RealSense camera
        ExecuteProcess(
            cmd=['ros2', 'launch', 'realsense2_camera', 'rs_launch.py', 
                 'enable_color:=true', 'enable_depth:=true', 'enable_infra1:=false', 'enable_infra2:=false'],
            output='screen',
            name='realsense_camera'
        ),
        
        # Wait a bit for camera to initialize
        TimerAction(
            period=3.0,
            actions=[
                # Start the regular sense node
                Node(
                    package='pkg_sense',
                    executable='sense',
                    name='sense_node',
                    output='screen',
                    parameters=[
                        {'frame_id': 'map'}
                    ]
                ),
            ]
        ),
        
        # Wait a bit more then start the HSV test node
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='pkg_sense',
                    executable='test_hsv_sense',
                    name='hsv_test_sense_node',
                    output='screen',
                    # Run in a separate terminal so GUI can be interactive
                    prefix='gnome-terminal --title="HSV Test Node" --tab --'
                ),
            ]
        ),
    ])
