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
                # Map-frame z thresholds (camera_init origin = IMU start position).
                # Sensor height H ≈ 1.3 m → floor is at z ≈ -1.3 m in camera_init.
                #
                # z classification layout (H=1.3m example):
                #   z > +1.0 → ignored (ceiling)
                #   z -1.0 ~ +1.0 → OCCUPIED (walls: 0.3 m above floor to 2.3 m above floor)
                #   z -1.7 ~ -1.0 → FREE (floor band, centre ≈ -1.3 m)
                #   z < -1.7 → OCCUPIED (step-down / drop-off)
                '--ground-z-min',   '-1.7',   # drops below -1.7 m → OCCUPIED
                '--ground-z',       '-1.0',   # floor FREE band: -1.7 ~ -1.0 (0.3 m above floor)
                '--obstacle-z',     '-1.0',   # walls/obstacles start 0.3 m above floor
                '--obstacle-z-max', '1.0',    # ceiling cutoff: floor+2.3 m (safe for 2.5 m rooms)
                '--skip-frames',    '30',
                '--min-obs-hits',   '3',   # 需命中3次才标记OCCUPIED，减少路人误识别
                '--map-z',          '-1.27',  # 传感器离地1.27m，地图显示在地面高度
            ],
            output='screen',
        ))

    return LaunchDescription(actions)
