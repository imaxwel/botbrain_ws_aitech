"""
elevator_button_press.launch.py
新方案：用 button_detector_node (yolonas_ocr) 替代 v4l2_apriltag_trigger
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('unitree_g1_dex3_stack')
    launch_dir = os.path.join(package_share, 'launch')
    config_file = os.path.join(package_share, 'config', 'apriltag_button_press.yaml')

    tf_topic = '/unitree_g1_dex3/tf'
    tf_static_topic = '/unitree_g1_dex3/tf_static'
    tf_remappings = [('/tf', tf_topic), ('/tf_static', tf_static_topic)]

    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'robot.launch.py')),
        launch_arguments={
            'tf_topic': tf_topic,
            'tf_static_topic': tf_static_topic,
        }.items(),
    )

    # 相机静态TF：torso_link → camera_color_optical_frame（标定逆变换）
    # 原始标定：camera_color_optical_frame → torso_link，但 URDF 中 torso_link 已有父节点
    # 故反向发布：parent=torso_link, child=camera_optical，使用逆变换值
    camera_to_robot_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='torso_to_camera_color_optical',
        arguments=[
            # 旧标定参数（回退）
            '0.057624', '0.017529', '0.429869',
            '-0.659252', '0.659252', '-0.255707', '0.255707',
            # 新标定参数 2026-06-24: '-0.003685', '0.018133', '0.454214', '-0.664050', '0.671876', '-0.233167', '0.230768',
            'torso_link', 'camera_color_optical_frame',
        ],
        remappings=tf_remappings,
    )

    button_detector_node = Node(
        executable='/botbrain_ws/src/g1_right_dex3/elevator_vision/scripts/button_detector_node.py',
        name='button_detector_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'input_backend': 'ros',
            'frozen_model_dir': '/botbrain_ws/src/g1_right_dex3/yolonas_ocr/frozen_model',
            'target_floor': LaunchConfiguration('target_floor'),
            'det_threshold': LaunchConfiguration('det_threshold'),
            'camera_frame': 'camera_color_optical_frame',
            'output_frame': 'torso_link',
            'image_topic': '/camera/camera/color/image_raw',
            'info_topic': '/camera/camera/color/camera_info',
            'depth_topic': '/camera/camera/depth/image_rect_raw',
        }],
        remappings=tf_remappings,
    )

    button_press_node = Node(
        package='unitree_g1_dex3_stack',
        executable='apriltag_button_press_node.py',
        name='apriltag_button_press_node',
        output='screen',
        emulate_tty=True,
        parameters=[config_file, {
            'dry_run': LaunchConfiguration('dry_run'),
            'capture_wait_timeout_s': 60.0,
            'press_x_offset': LaunchConfiguration('press_x_offset'),
            'press_y_offset': LaunchConfiguration('press_y_offset'),
            'press_z_offset': LaunchConfiguration('press_z_offset'),
        }],
        remappings=tf_remappings,
    )

    planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'planner.launch.py')),
        launch_arguments={
            'config_file': config_file,
            'planning_timeout': '1.0',
            'adaptive_orientation_enabled': 'false',
            'fallback_total_timeout_s': '2.0',
            'tf_topic': tf_topic,
            'tf_static_topic': tf_static_topic,
        }.items(),
    )

    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'control.launch.py')),
        launch_arguments={'auto_return_to_standing': 'false'}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('target_floor', default_value='0'),
        DeclareLaunchArgument('det_threshold', default_value='0.5'),
        DeclareLaunchArgument('dry_run', default_value='false'),
        DeclareLaunchArgument('press_x_offset', default_value='0.0'),
        DeclareLaunchArgument('press_y_offset', default_value='0.0'),
        DeclareLaunchArgument('press_z_offset', default_value='0.0'),
        robot_launch,
        camera_to_robot_tf,
        TimerAction(period=20.0, actions=[
            button_detector_node,
            planner_launch,
            control_launch,
            button_press_node,
        ]),
    ])
