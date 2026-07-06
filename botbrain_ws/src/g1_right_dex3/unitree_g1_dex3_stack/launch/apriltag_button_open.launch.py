import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_share = get_package_share_directory('unitree_g1_dex3_stack')
    launch_dir = os.path.join(package_share, 'launch')
    config_file = os.path.join(package_share, 'config', 'apriltag_button_open.yaml')
    tf_remappings = [
        ('/tf', LaunchConfiguration('tf_topic')),
        ('/tf_static', LaunchConfiguration('tf_static_topic')),
    ]

    camera_only_arg = DeclareLaunchArgument(
        'camera_only',
        default_value='false',
        description='Only launch robot description, camera TF, and V4L2 AprilTag trigger',
    )
    dry_run_arg = DeclareLaunchArgument(
        'dry_run',
        default_value='false',
        description='Skip Dex-3 subprocess calls in the button press sequencer',
    )
    planning_timeout_arg = DeclareLaunchArgument(
        'planning_timeout',
        default_value='1.0',
        description='Planning timeout in seconds',
    )
    v4l2_config_file_arg = DeclareLaunchArgument(
        'v4l2_config_file',
        default_value=config_file,
        description='Button-press V4L2 AprilTag parameter YAML file',
    )
    v4l2_video_device_arg = DeclareLaunchArgument(
        'v4l2_video_device',
        default_value='auto',
        description='Stable V4L2 RGB device path for the target D435i',
    )
    debug_image_dir_arg = DeclareLaunchArgument(
        'debug_image_dir',
        default_value='/botbrain_ws/detect_img',
        description='Directory for latest triggered AprilTag debug images',
    )
    tf_topic_arg = DeclareLaunchArgument(
        'tf_topic',
        default_value='/unitree_g1_dex3/tf',
        description='TF topic used by this launch',
    )
    tf_static_topic_arg = DeclareLaunchArgument(
        'tf_static_topic',
        default_value='/unitree_g1_dex3/tf_static',
        description='TF static topic used by this launch',
    )

    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'robot.launch.py')),
        launch_arguments={
            'tf_topic': LaunchConfiguration('tf_topic'),
            'tf_static_topic': LaunchConfiguration('tf_static_topic'),
        }.items(),
    )

    d435_to_camera_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='d435_link_to_camera_link',
        arguments=['0', '0', '0', '0', '0', '0', 'd435_link', 'camera_link'],
        remappings=tf_remappings,
    )
    camera_link_to_color_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_link_to_camera_color_frame',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_link', 'camera_color_frame'],
        remappings=tf_remappings,
    )
    camera_color_to_optical = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_color_frame_to_optical_frame',
        arguments=[
            '0', '0', '0',
            '-0.5', '0.5', '-0.5', '0.5',
            'camera_color_frame', 'camera_color_optical_frame'
        ],
        remappings=tf_remappings,
    )

    v4l2_trigger_node = Node(
        package='unitree_g1_dex3_stack',
        executable='v4l2_apriltag_trigger.py',
        name='v4l2_apriltag_trigger',
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('v4l2_config_file'),
            {
                'video_device': LaunchConfiguration('v4l2_video_device'),
                'debug_image_dir': LaunchConfiguration('debug_image_dir'),
                'detect_only': True,
                'trigger_key': '',
                'trigger_topic': '/apriltag/capture_trigger',
                'publish_intermediate_poses': False,
            },
        ],
        remappings=tf_remappings,
    )

    button_node = Node(
        package='unitree_g1_dex3_stack',
        executable='apriltag_button_press_node.py',
        name='apriltag_button_press_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('v4l2_config_file'),
            {'dry_run': LaunchConfiguration('dry_run')},
        ],
        remappings=tf_remappings,
    )

    planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'planner.launch.py')),
        launch_arguments={
            'config_file': config_file,
            'planning_timeout': LaunchConfiguration('planning_timeout'),
            'adaptive_orientation_enabled': 'false',
            'fallback_total_timeout_s': '2.0',
            'tf_topic': LaunchConfiguration('tf_topic'),
            'tf_static_topic': LaunchConfiguration('tf_static_topic'),
        }.items(),
    )

    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'control.launch.py')),
        launch_arguments={
            'auto_return_to_standing': 'false',
        }.items(),
    )

    camera_only_actions = TimerAction(
        period=3.0,
        actions=[v4l2_trigger_node],
        condition=IfCondition(LaunchConfiguration('camera_only')),
    )
    full_actions = TimerAction(
        period=3.0,
        actions=[v4l2_trigger_node, planner_launch, control_launch, button_node],
        condition=UnlessCondition(LaunchConfiguration('camera_only')),
    )

    return LaunchDescription([
        camera_only_arg,
        dry_run_arg,
        planning_timeout_arg,
        v4l2_config_file_arg,
        v4l2_video_device_arg,
        debug_image_dir_arg,
        tf_topic_arg,
        tf_static_topic_arg,
        robot_launch,
        d435_to_camera_link,
        camera_link_to_color_frame,
        camera_color_to_optical,
        camera_only_actions,
        full_actions,
    ])
