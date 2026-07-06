#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag',
            remappings=[
                ('image_rect', '/front_camera/color/image_raw'),
                ('camera_info', '/front_camera/color/camera_info'),
            ],
            parameters=[{
                'family': '36h11',
                'size': 0.162,  # AprilTag尺寸(米)
                'max_hamming': 0,
            }]
        )
    ])
