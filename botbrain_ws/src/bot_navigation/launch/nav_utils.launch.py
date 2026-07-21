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

    localization_monitor_node = Node(
        package='bot_navigation',
        executable='localization_monitor.py',
        name='localization_monitor',
        namespace=robot_name,
        output='screen',
        parameters=[{
            'robot': robot_name,
            'confidence_threshold': 0.50,
            'consecutive_count': 3,
            'low_confidence_duration_sec': 5.0,
            'confidence_timeout_sec': 2.0,
            # Keep the goal alive across short sensor/TF interruptions. The
            # monitor still publishes a higher-priority zero velocity while
            # unhealthy, but Nav2 can resume the same path after recovery.
            'cancel_after_sec': 3.0,
            'auto_cancel': False,
            'publish_safety_stop': True,
            'scan_topic': '/scan',
            'startup_grace_sec': 2.0,
            'nav_odom_topic': (
                f'/{robot_name}/nav_odom' if robot_name else '/nav_odom'
            ),
            'safety_stop_topic': (
                f'/{robot_name}/cmd_vel_nav_safety'
                if robot_name else '/cmd_vel_nav_safety'
            ),
        }],
    )

    return LaunchDescription([
        nav2_utils_node,
        localization_monitor_node,
    ])
