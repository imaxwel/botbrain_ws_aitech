"""Standalone V4L2 AprilTag detection launch."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_share = get_package_share_directory('unitree_g1_dex3_stack')
    tf_remappings = [
        ('/tf', LaunchConfiguration('tf_topic')),
        ('/tf_static', LaunchConfiguration('tf_static_topic')),
    ]

    # ---------- launch arguments ----------
    urdf_name_arg = DeclareLaunchArgument(
        'urdf_name',
        default_value='g1_29dof_lock_waist_with_hand_rev_1_0_collision_primitives.urdf',
        description='URDF filename used by robot_state_publisher',
    )
    urdf_path_arg = DeclareLaunchArgument(
        'urdf_path',
        default_value='',
        description='Optional full URDF path that overrides urdf_name',
    )
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(package_share, 'config', 'v4l2_apriltag_trigger.yaml'),
        description='V4L2 triggered AprilTag parameter YAML file',
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
    detect_only_arg = DeclareLaunchArgument(
        'detect_only',
        default_value='true',
        description='Detect and save/publish AprilTag poses without publishing /goal_pose',
    )
    tf_topic_arg = DeclareLaunchArgument(
        'tf_topic',
        default_value='/tf',
        description='TF topic used by this launch',
    )
    tf_static_topic_arg = DeclareLaunchArgument(
        'tf_static_topic',
        default_value='/tf_static',
        description='TF static topic used by this launch',
    )

    # ---------- robot (robot_state_publisher + CycloneDDS env) ----------
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, 'launch', 'robot.launch.py')
        ),
        launch_arguments={
            'urdf_name': LaunchConfiguration('urdf_name'),
            'urdf_path': LaunchConfiguration('urdf_path'),
            'tf_topic': LaunchConfiguration('tf_topic'),
            'tf_static_topic': LaunchConfiguration('tf_static_topic'),
        }.items(),
    )

    # ---------- d435_link → camera_link static TF ----------
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
            LaunchConfiguration('config_file'),
            {
                'video_device': LaunchConfiguration('v4l2_video_device'),
                'debug_image_dir': LaunchConfiguration('debug_image_dir'),
                'detect_only': LaunchConfiguration('detect_only'),
            },
        ],
        remappings=tf_remappings,
    )

    return LaunchDescription([
        urdf_name_arg,
        urdf_path_arg,
        config_file_arg,
        v4l2_video_device_arg,
        debug_image_dir_arg,
        detect_only_arg,
        tf_topic_arg,
        tf_static_topic_arg,
        robot_launch,
        d435_to_camera_link,
        camera_link_to_color_frame,
        camera_color_to_optical,
        v4l2_trigger_node,
    ])
