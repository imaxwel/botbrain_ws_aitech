from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('g1_loop_closure'),
        'config',
        'loop_closure.yaml',
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_pose_graph',
            default_value='false',
            description='Enable phase-2 diagnostic SE(2) optimization. Never changes FAST-LIO TF/odom.',
        ),
        DeclareLaunchArgument(
            'export_optimized_map_path',
            default_value='',
            description='Optional explicit path used only by the export service; never overwrites maps by default.',
        ),
        Node(
            package='g1_loop_closure',
            executable='loop_closure_node',
            name='loop_closure',
            output='screen',
            parameters=[
                config,
                {
                    'enable_pose_graph': LaunchConfiguration('enable_pose_graph'),
                    'export_optimized_map_path': LaunchConfiguration('export_optimized_map_path'),
                },
            ],
        ),
    ])
