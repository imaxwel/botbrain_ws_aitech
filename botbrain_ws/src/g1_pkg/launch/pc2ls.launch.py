import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    launch_dir = os.path.dirname(os.path.abspath(__file__))
    workspace_dir = os.path.abspath(
        os.path.join(launch_dir, '../../../../..')
    )
    config_file = os.path.join(workspace_dir, 'robot_config.yaml')
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)['robot_configuration']

    robot_name = config['robot_name']
    target_frame = (
        f'{robot_name}/base_footprint' if robot_name else 'base_footprint'
    )

    params_file = os.path.join(
        get_package_share_directory('g1_pkg'),
        'config',
        'pointcloud_to_laserscan_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='g1_pkg',
            executable='navigation_scan_projector.py',
            name='navigation_scan_projector',
            output='screen',
            parameters=[params_file, {'target_frame': target_frame}],
            remappings=[
                # The world cloud is published only after FAST-LIO accepts the
                # scan and publishes the matching timestamped odometry/TF.
                # Guarded/recovery frames publish no world cloud, so they
                # cannot enter navigation as obstacle observations.
                ('cloud_in', '/cloud_registered_1'),
                ('scan', '/scan'),
            ],
        )
    ])
