import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('unitree_g1_dex3_stack')
    launch_dir = os.path.join(package_share, 'launch')
    config_file = os.path.join(package_share, 'config', 'apriltag_button_press.yaml')
    tf_remappings = [
        ('/tf', LaunchConfiguration('tf_topic')),
        ('/tf_static', LaunchConfiguration('tf_static_topic')),
    ]

    camera_only_arg = DeclareLaunchArgument(
        'camera_only',
        default_value='false',
        description='Only launch robot description, camera TF, and button detector',
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
        description='Button-press parameter YAML file',
    )
    input_backend_arg = DeclareLaunchArgument(
        'input_backend',
        default_value='v4l2',
        description='button_detector_node input backend: v4l2 or ros',
    )
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/realsense2_camera/color/image_raw',
        description='RGB image topic for button_detector_node',
    )
    info_topic_arg = DeclareLaunchArgument(
        'info_topic',
        default_value='/camera/realsense2_camera/color/camera_info',
        description='CameraInfo topic for button_detector_node',
    )
    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic',
        default_value='/camera/realsense2_camera/depth/image_rect_raw',
        description='Depth image topic for button_detector_node',
    )
    frozen_model_dir_arg = DeclareLaunchArgument(
        'frozen_model_dir',
        default_value='/botbrain_ws/src/g1_right_dex3/yolonas_ocr/frozen_model',
        description='Frozen model directory for button_detector_node',
    )
    target_floor_arg = DeclareLaunchArgument(
        'target_floor',
        default_value='0',
        description='Target floor for button_detector_node',
    )
    det_threshold_arg = DeclareLaunchArgument(
        'det_threshold',
        default_value='0.5',
        description='Detection threshold for button_detector_node',
    )
    output_frame_arg = DeclareLaunchArgument(
        'output_frame',
        default_value='torso_link',
        description='Output frame for button_detector_node',
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
            'camera_color_frame', 'camera_color_optical_frame',
        ],
        remappings=tf_remappings,
    )

    button_detector_node = Node(
        package='unitree_g1_dex3_stack',
        executable='button_detector_node.py',
        name='button_detector_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'image_topic': LaunchConfiguration('image_topic'),
            'info_topic': LaunchConfiguration('info_topic'),
            'depth_topic': LaunchConfiguration('depth_topic'),
            'frozen_model_dir': LaunchConfiguration('frozen_model_dir'),
            'target_floor': LaunchConfiguration('target_floor'),
            'det_threshold': LaunchConfiguration('det_threshold'),
            'output_frame': LaunchConfiguration('output_frame'),
            'input_backend': LaunchConfiguration('input_backend'),
            'video_device': LaunchConfiguration('v4l2_video_device'),
            'debug_image_dir': LaunchConfiguration('debug_image_dir'),
            'fps': 3.0,
        }],
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

    button_press_node = Node(
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
        actions=[v4l2_trigger_node, planner_launch, control_launch, button_press_node],
        condition=UnlessCondition(LaunchConfiguration('camera_only')),
    )

    return LaunchDescription([
        camera_only_arg,
        dry_run_arg,
        planning_timeout_arg,
        v4l2_config_file_arg,
        input_backend_arg,
        image_topic_arg,
        info_topic_arg,
        depth_topic_arg,
        frozen_model_dir_arg,
        target_floor_arg,
        det_threshold_arg,
        output_frame_arg,
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
