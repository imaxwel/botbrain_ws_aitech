import os
from launch import LaunchDescription
from launch_ros.actions import Node
import yaml


def generate_launch_description():
    launch_dir = os.path.dirname(os.path.abspath(__file__))
    workspace_dir = os.path.abspath(
        os.path.join(launch_dir, '../../../../..')
    )
    config_file = os.path.join(workspace_dir, 'robot_config.yaml')
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)['robot_configuration']

    robot_name = config['robot_name']

    nav2_utils_node = Node(
        package='bot_navigation',
        executable='nav2_utils.py',
        name='nav2_utils',
        namespace=robot_name,
        output='screen',
    )

    nav_odom_relay_node = Node(
        package='bot_navigation',
        executable='nav_odom_relay.py',
        name='nav_odom_relay',
        namespace=robot_name,
        output='screen',
        parameters=[{
            'pose_topic': '/Odometry_loc',
            'twist_topic': f'/{robot_name}/odom' if robot_name else '/odom',
            'output_topic': (
                f'/{robot_name}/nav_odom' if robot_name else '/nav_odom'
            ),
            'output_frame': f'{robot_name}/odom' if robot_name else 'odom',
            'child_frame': (
                f'{robot_name}/base_footprint'
                if robot_name else 'base_footprint'
            ),
        }],
    )

    return LaunchDescription([
        nav2_utils_node,
        nav_odom_relay_node,
    ])
