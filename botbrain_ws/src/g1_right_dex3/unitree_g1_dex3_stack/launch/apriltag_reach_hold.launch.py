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
    config_file = os.path.join(package_share, 'config', 'apriltag_button_press.yaml')

    planning_timeout_arg = DeclareLaunchArgument(
        'planning_timeout',
        default_value='1.0',
        description='Planning timeout in seconds',
    )

    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'robot.launch.py')),
        launch_arguments={
            'tf_topic': '/unitree_g1_dex3/tf',
            'tf_static_topic': '/unitree_g1_dex3/tf_static',
        }.items(),
    )

    planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'planner.launch.py')),
        launch_arguments={
            'config_file': config_file,
            'planning_timeout': LaunchConfiguration('planning_timeout'),
            'adaptive_orientation_enabled': 'false',
            'fallback_total_timeout_s': '2.0',
            'fallback_ik_only': 'true',
            'tf_topic': '/unitree_g1_dex3/tf',
            'tf_static_topic': '/unitree_g1_dex3/tf_static',
        }.items(),
    )

    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'control.launch.py')),
        launch_arguments={
            'auto_return_to_standing': 'false',
            'hold_indefinitely': 'true',
        }.items(),
    )

    fixed_goal_node = Node(
        package='unitree_g1_dex3_stack',
        executable='publish_fixed_goal.py',
        name='fixed_goal_publisher',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'goal_x': 0.3737,
            'goal_y': -0.1123,
            'goal_z': 0.0358,
            'goal_roll': -0.0006,
            'goal_pitch': -0.0987,
            'goal_yaw': 0.0578,
            'goal_frame': 'torso_link',
            'publish_delay_s': 5.0,
        }],
    )

    delayed_actions = TimerAction(
        period=3.0,
        actions=[planner_launch, control_launch, fixed_goal_node],
    )

    return LaunchDescription([
        planning_timeout_arg,
        robot_launch,
        delayed_actions,
    ])
