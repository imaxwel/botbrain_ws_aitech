import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_share = get_package_share_directory('unitree_g1_dex3_stack')
    launch_dir = os.path.join(package_share, 'launch')

    planning_timeout_arg = DeclareLaunchArgument(
        'planning_timeout',
        default_value='1.0',
        description='Planning timeout in seconds'
    )

    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'robot.launch.py')
        )
    )

    d435_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='d435_link_to_camera_link',
        arguments=['0', '0', '0', '0', '0', '0', 'd435_link', 'camera_link'],
    )

    planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'planner.launch.py')
        ),
        launch_arguments={
            'planning_timeout': LaunchConfiguration('planning_timeout'),
        }.items()
    )

    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'control.launch.py')
        )
    )

    delayed_actions = TimerAction(period=3.0, actions=[
        planner_launch,
        control_launch,
    ])

    return LaunchDescription([
        planning_timeout_arg,
        robot_launch,
        d435_tf_node,
        delayed_actions,
    ])
