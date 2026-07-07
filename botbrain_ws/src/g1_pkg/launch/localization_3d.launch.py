import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    workspace_dir = '/botbrain_ws'
    default_pcd_path  = os.path.join(workspace_dir, 'src', 'g1_pkg', 'maps', 'scans.pcd')
    default_grid_yaml = os.path.join(workspace_dir, 'src', 'g1_pkg', 'maps', 'accumulated.yaml')

    pcd_arg          = DeclareLaunchArgument('map_file',     default_value=default_pcd_path)
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')

    open3d_config = PathJoinSubstitution([FindPackageShare('open3d_loc'), 'config', 'loc_param_g1.yaml'])

    IMU_HEIGHT = 1.247  # MID360 离地高度(m)，odom z=0 对应此高度

    # Relay: /initialpose(z=0 from Foxglove/RViz 2D tool) → z corrected → /initialpose_corrected
    initialpose_z_fix = Node(
        package='g1_pkg',
        executable='initialpose_z_fix.py',
        name='initialpose_z_fix',
        output='screen',
        parameters=[{'ref_z': IMU_HEIGHT}],
    )

    # ICP 3D localization — subscribes to /initialpose_corrected (z already fixed by relay)
    global_localization = Node(
        package='open3d_loc',
        executable='global_localization_node',
        name='global_localization_node',
        output='screen',
        parameters=[
            open3d_config,
            {
                'path_map':                 LaunchConfiguration('map_file'),
                'use_sim_time':             LaunchConfiguration('use_sim_time'),
                'pcd_queue_maxsize':        10,
                'voxelsize_coarse':         0.15,
                'voxelsize_fine':           0.2,    # 走廊环境大体素=大收敛盆地，防止跳局部最优
                'threshold_fitness':        0.5,   # 0.7→0.5: 允许断流恢复时接受中等质量匹配；级联靠 dis_updatemap=5.0 防护
                'threshold_fitness_init':   0.5,
                'loc_frequence':            4.0,    # 2.5→4.0: 提高ICP修正频率，减少漂移累积
                'save_scan':                False,
                'maxpoints_source':         80000,
                'maxpoints_target':         400000,
                'initialpose':              [0.0, 0.0, IMU_HEIGHT, 0.0, 0.0, 0.0],
                'filter_odom2map':          False,
                'kalman_processVar2':       0.001,  # 0.003→0.001: 降低过程噪声，Kalman输出更稳定
                'kalman_estimatedMeasVar2': 0.06,   # 0.02→0.06: 降低Kalman对ICP大跳变的信任，防止突然漂移
                'confidence_loc_th':        0.7,
                'dis_updatemap':            5.0,    # 3.0→5.0: 降低submap更新频率，一旦漂移不会立即把错误位置固化进submap
            },
        ],
        remappings=[('initialpose', 'initialpose_corrected')],
    )

    # Static TFs required by fast_lio + open3d_loc:
    #   odom  →  camera_init  (FAST_LIO world frame alias)
    #   base_link  →  imu_link
    #   motion_link  →  base_link
    static_tf_camera_init = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='camera_init2odom',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'odom', 'camera_init'],
    )
    static_tf_imu2base = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='imulink2baselink',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'imu_link'],
    )
    static_tf_motion2base = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='base_center_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'motion_link', 'base_link'],
    )

    # 2D map server (for Nav2)
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time':  LaunchConfiguration('use_sim_time'),
            'yaml_filename': default_grid_yaml,
            'topic_name':    'map',
            'frame_id':      'map',
        }],
    )
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart':    True,
            'node_names':   ['map_server'],
        }],
    )

    # Nav2 runs in "g1_robot" namespace and expects TF frames g1_robot/map and
    # g1_robot/odom, but open3d_loc publishes plain "map" and "odom".
    # These identity bridges make both frame names available in the TF tree so
    # Nav2 can compute: g1_robot/map → map → odom → g1_robot/odom → g1_robot/base_footprint
    static_tf_map_alias = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='map_to_g1robot_map',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'g1_robot/map'],
    )
    static_tf_odom_alias = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='odom_to_g1robot_odom',
        # FAST_LIO odom/camera_init z=0 is at IMU height (1.247m above ground).
        # Unitree g1_robot/odom z=0 is at ground level.
        # Offset = -1.247m so g1_robot/odom sits at map z=0 (ground).
        arguments=['0', '0', '-1.247', '0', '0', '0', '1', 'odom', 'g1_robot/odom'],
    )

    # Bridge FAST_LIO's "body" frame (accurate global position) to g1_robot/base_footprint.
    # g1_read.py's publish_tf is disabled so only this static TF drives base_footprint.
    # body z=0 is at IMU height (1.247m above ground); base_footprint is at ground level.
    static_tf_body2footprint = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='body_to_base_footprint',
        arguments=['0', '0', '-1.247', '0', '0', '0', '1', 'body', 'g1_robot/base_footprint'],
    )

    return LaunchDescription([
        pcd_arg,
        use_sim_time_arg,
        initialpose_z_fix,
        global_localization,
        static_tf_camera_init,
        static_tf_imu2base,
        static_tf_body2footprint,
        static_tf_motion2base,
        static_tf_map_alias,
        static_tf_odom_alias,
        map_server,
        lifecycle_manager,
    ])
