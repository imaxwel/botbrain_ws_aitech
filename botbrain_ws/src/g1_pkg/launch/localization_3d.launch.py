import os
import re
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _pcd_scene_name(path):
    stem = Path(path).resolve(strict=True).stem
    return stem[:-6] if stem.endswith('_scans') else stem


def _validate_map_pair(context, maps_dir):
    scene = LaunchConfiguration('map_scene').perform(context).strip()
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]*', scene):
        raise RuntimeError(
            f'Invalid map_scene {scene!r}; use only letters, digits, "_" or "-".'
        )

    map_file = LaunchConfiguration('map_file').perform(context).strip()
    grid_map_file = LaunchConfiguration('grid_map_file').perform(context).strip()
    pcd_path = Path(map_file or os.path.join(maps_dir, f'{scene}_scans.pcd'))
    grid_path = Path(grid_map_file or os.path.join(maps_dir, f'{scene}.yaml'))
    try:
        resolved_pcd = pcd_path.resolve(strict=True)
        resolved_grid = grid_path.resolve(strict=True)
    except FileNotFoundError as exc:
        raise RuntimeError(
            f'Map scene {scene!r} is incomplete; missing {exc.filename}. '
            f'Expected {scene}_scans.pcd, {scene}.yaml and {scene}.pgm.'
        ) from exc
    if (
        not resolved_pcd.is_file()
        or resolved_pcd.stat().st_size == 0
        or resolved_pcd.suffix.lower() != '.pcd'
        or not resolved_grid.is_file()
        or resolved_grid.stat().st_size == 0
        or resolved_grid.suffix.lower() != '.yaml'
    ):
        raise RuntimeError(
            f'Map scene {scene!r} requires non-empty .pcd and .yaml files: '
            f'PCD={resolved_pcd}, YAML={resolved_grid}'
        )
    try:
        grid_data = yaml.safe_load(resolved_grid.read_text(encoding='utf-8'))
    except yaml.YAMLError as exc:
        raise RuntimeError(f'Invalid grid map YAML: {resolved_grid}: {exc}') from exc
    if grid_data is None:
        grid_data = {}
    if not isinstance(grid_data, dict):
        raise RuntimeError(
            f'Grid map YAML root must be a mapping: {resolved_grid}'
        )
    image_value = grid_data.get('image')
    if not isinstance(image_value, str) or not image_value.strip():
        raise RuntimeError(f'Grid map YAML has no image entry: {resolved_grid}')

    try:
        image_path = (resolved_grid.parent / image_value).resolve(strict=True)
    except FileNotFoundError as exc:
        raise RuntimeError(
            f'Grid map image does not exist for scene {scene!r}: '
            f'{image_value}'
        ) from exc
    grid_scene = resolved_grid.stem
    pcd_scene = _pcd_scene_name(resolved_pcd)
    image_scene = image_path.stem
    using_scene_defaults = not map_file and not grid_map_file
    selected_scene_matches = (
        not using_scene_defaults
        or (
            pcd_scene == scene
            and grid_scene == scene
            and image_scene == scene
        )
    )
    if grid_scene == 'accumulated':
        # Legacy generic pair: accumulated.yaml/pgm + scans.pcd.
        expected_pcd_scene = 'scans'
    else:
        expected_pcd_scene = grid_scene

    if (
        not selected_scene_matches
        or pcd_scene != expected_pcd_scene
        or image_scene != grid_scene
        or not image_path.is_file()
        or image_path.stat().st_size == 0
        or image_path.suffix.lower() != '.pgm'
    ):
        raise RuntimeError(
            '3D/2D map scene mismatch: '
            f'PCD={resolved_pcd} (scene={pcd_scene}), '
            f'YAML={resolved_grid} (scene={grid_scene}), '
            f'image={image_path} (scene={image_scene}). '
            f'Requested map_scene={scene}. '
            'Select matching <scene>_scans.pcd, <scene>.yaml and <scene>.pgm files.'
        )
    return [
        LogInfo(msg=(
            f'Map selection: scene={grid_scene} PCD={resolved_pcd} '
            f'YAML={resolved_grid} PGM={image_path}'
        )),
        SetLaunchConfiguration('map_file', str(resolved_pcd)),
        SetLaunchConfiguration('grid_map_file', str(resolved_grid)),
    ]


def generate_launch_description():
    workspace_dir = '/botbrain_ws'
    maps_dir = os.path.join(workspace_dir, 'src', 'g1_pkg', 'maps')

    map_scene_arg = DeclareLaunchArgument('map_scene', default_value='ug')
    pcd_arg = DeclareLaunchArgument('map_file', default_value='')
    grid_map_arg = DeclareLaunchArgument('grid_map_file', default_value='')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')

    open3d_config = PathJoinSubstitution([
        FindPackageShare('open3d_loc'), 'config', 'loc_param_g1.yaml'
    ])

    IMU_HEIGHT = 1.247  # MID360 离地高度(m)，odom z=0 对应此高度

    # Relay: /initialpose(z=0 from a 2D visualization tool) → z corrected → /initialpose_corrected
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
                'pcd_queue_maxsize':        1,
                'registered_cloud_world_frame': 'camera_init',
                'publish_planar_base_tf':   True,
                'planar_base_frame':        'g1_robot/base_footprint',
                'planar_base_height':       IMU_HEIGHT,
                'voxelsize_coarse':         0.15,
                'voxelsize_fine':           0.2,    # 走廊环境大体素=大收敛盆地，防止跳局部最优
                # Allow medium-quality recovery matches; dis_updatemap=5.0
                # limits how quickly a bad alignment can enter the submap.
                'threshold_fitness':        0.5,
                'threshold_fitness_init':   0.5,
                'loc_frequence':            4.0,    # 真实 4 Hz，即约每 250 ms 尝试一次 ICP
                'max_icp_translation_step':  1.0,
                'max_icp_rotation_step_deg': 15.0,
                'immediate_icp_translation_step': 0.10,
                'immediate_icp_rotation_step_deg': 2.0,
                'large_correction_confirmations':  2,
                'icp_candidate_consistency_translation': 0.20,
                'icp_candidate_consistency_rotation_deg': 4.0,
                'icp_candidate_max_age_sec': 1.0,
                # Pair world cloud N only with Odometry_loc N; FAST-LIO publishes
                # odometry first, so an ICP timer can otherwise mix N and N+1.
                'max_scan_odom_time_skew_sec': 0.03,
                # The finished localization PCD is shifted up by IMU_HEIGHT so
                # its floor is map z=0. Vertical walls make point-to-plane ICP
                # unable to determine this offset reliably, so keep it explicit.
                'lock_map_odom_z':          True,
                # FAST-LIO already aligns its world Z with gravity. Navigation
                # localization should correct only planar x/y/yaw, otherwise
                # corridor ICP can accumulate weakly-observed roll/pitch.
                'lock_map_odom_roll_pitch': True,
                'map_odom_z':               IMU_HEIGHT,
                'max_icp_inlier_rmse':         0.30,
                'min_initialization_fitness':  0.50,
                'max_initialization_translation_step': 2.0,
                'max_initialization_rotation_step_deg': 45.0,
                'min_icp_source_points':     100,
                'min_icp_target_points':     1000,
                # Derive map->odom from the current FAST-LIO cloud and complete
                # localization PCD. Manual /initialpose remains a fallback.
                'enable_global_initialization': True,
                'global_voxel_size':          0.40,
                # Cycle through fine/medium/coarse FPFH descriptions so map
                # selection is not tied to one building scale or wall spacing.
                'global_voxel_sizes':         [0.25, 0.40, 0.60],
                'global_ransac_max_iterations': 100000,
                'global_ransac_confidence':   0.999,
                # RANSAC is only a seed generator. Reject zero-overlap results,
                # then let fine ICP quality and three consistent confirmations
                # decide whether the absolute pose is safe to publish.
                'global_min_ransac_fitness':  0.0,
                'global_min_fitness':         0.65,
                'global_max_inlier_rmse':     0.30,
                'global_retry_interval_sec':  2.0,
                'global_initialization_confirmations': 3,
                'global_candidate_consistency_translation': 0.35,
                'global_candidate_consistency_rotation_deg': 5.0,
                'global_candidate_max_age_sec': 30.0,
                'global_min_source_points':   100,
                'global_min_target_points':   1000,
                # A short rolling world-frame window adds corners observed
                # during a slow turn; voxel filtering removes stationary duplicates.
                'global_scan_window_size':    10,
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
        remappings=[
            ('initialpose', 'initialpose_corrected'),
            # Navigation owns /scan as a LaserScan. Keep Open3D's diagnostic
            # PointCloud2 on a separate topic.
            ('scan', '/scan_loc'),
        ],
    )

    # Publish the exact odometry stream consumed by Nav2 before Navigation is
    # allowed to start. Pose follows FAST-LIO/TF; twist comes from Unitree.
    nav_odom_relay = Node(
        package='bot_navigation',
        executable='nav_odom_relay.py',
        name='nav_odom_relay',
        namespace='g1_robot',
        output='screen',
        parameters=[{
            'pose_topic': '/Odometry_loc',
            'twist_topic': '/g1_robot/odom',
            'output_topic': '/g1_robot/nav_odom',
            'output_frame': 'g1_robot/odom',
            'child_frame': 'g1_robot/base_footprint',
            'derive_twist_from_pose': False,
        }],
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
            'yaml_filename': LaunchConfiguration('grid_map_file'),
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
    # These aliases make both names available. The localization node separately
    # publishes odom -> g1_robot/base_footprint as a planar FAST-LIO projection.
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

    return LaunchDescription([
        map_scene_arg,
        pcd_arg,
        grid_map_arg,
        use_sim_time_arg,
        OpaqueFunction(
            function=_validate_map_pair,
            kwargs={'maps_dir': maps_dir},
        ),
        initialpose_z_fix,
        global_localization,
        nav_odom_relay,
        static_tf_camera_init,
        static_tf_imu2base,
        static_tf_motion2base,
        static_tf_map_alias,
        static_tf_odom_alias,
        map_server,
        lifecycle_manager,
    ])
