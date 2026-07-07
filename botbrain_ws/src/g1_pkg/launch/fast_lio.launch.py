import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    pkg_config = os.path.join(
        get_package_share_directory('fast_lio'), 'config', 'mid360.yaml'
    )

    with open(pkg_config, 'r') as f:
        mid360 = yaml.safe_load(f)
    pcd_save_en = mid360.get('/**', {}).get('ros__parameters', {}).get('pcd_save', {}).get('pcd_save_en', False)

    # imu_flip: negate IMU Y/Z before fast_lio sees it.
    # MID360 roll-180 mount → SDK corrects pointcloud (Z-up) but publishes IMU
    # in raw sensor frame (Z-down). The flip brings IMU into the same Z-up frame
    # so extrinsic_R=identity is correct and the map stays right-side-up.
    actions = [
        Node(
            package='g1_pkg',
            executable='imu_flip.py',
            name='imu_flip',
            output='screen',
        ),
        Node(
            package='fast_lio',
            executable='fastlio_mapping',
            name='fast_lio',
            output='screen',
            parameters=[pkg_config, {'use_sim_time': False}],
        ),
    ]

    if pcd_save_en:
        actions.append(ExecuteProcess(
            cmd=[
                'python3',
                '/botbrain_ws/install/g1_pkg/lib/g1_pkg/grid_accumulator.py',
                # Use /cloud_registered (already in camera_init/map frame) —
                # eliminates TF lookup and the concentric-ring artifact.
                '--cloud-topic',    '/cloud_registered_1',
                '--grid-topic',     '/accumulated_grid',
                '--map-frame',      'camera_init',
                '--resolution',     '0.05',
                # Map-frame z thresholds (camera_init origin = IMU start position,
                # floor ≈ -1.1 m, ceiling ≈ 3.0 m in typical indoor start).
                '--ground-z-min',   '-1.5',   # below → step-down / drop-off → OCCUPIED
                '--ground-z',       '0.0',    # camera_init origin=IMU(≈1.1m高); z<0全部视为地面/FREE
                '--obstacle-z',     '0.0',    # z>0以上才标记OCCUPIED; 传感器高度以上的墙体/家具
                '--obstacle-z-max', '1.5',    # 地面以上约2.6m(1.5+1.1)，覆盖室内墙体全高
                '--skip-frames',    '30',
                '--min-obs-hits',   '3',   # 需命中3次才标记OCCUPIED，减少路人误识别
            ],
            output='screen',
        ))

    return LaunchDescription(actions)
