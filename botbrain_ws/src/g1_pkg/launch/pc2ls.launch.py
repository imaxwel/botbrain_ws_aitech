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
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan_node',
            output='screen',
            parameters=[params_file, {'target_frame': target_frame}],
            remappings=[
                ('cloud_in', '/cloud_registered_body_1'),
                ('scan', '/scan'),
            ],
        )
    ])
