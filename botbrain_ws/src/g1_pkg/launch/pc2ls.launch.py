import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('g1_pkg'),
        'config',
        'pointcloud_to_laserscan_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan_node',
            output='screen',
            parameters=[params_file],
            remappings=[('cloud_in', '/livox/lidar')],
        )
    ])
