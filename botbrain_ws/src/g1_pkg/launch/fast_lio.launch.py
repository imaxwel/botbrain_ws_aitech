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
                # ---- Scheme 3: wider ground band + higher confirmation ----
                # Map-frame z thresholds (camera_init origin = IMU start position).
                # Sensor height H ≈ 1.27 m → floor is at z ≈ -1.27 m in camera_init.
                #
                # z classification layout (H=1.27m example):
                #   z > +0.8 → ignored (ceiling, above floor+2.07m)
                #   z -0.8 ~ +0.8 → OCCUPIED (from 0.47m above floor to 2.07m)
                #   z -2.0 ~ -0.8 → FREE (floor band, ~1.2m wide for bipedal tilt margin)
                #   z < -2.0 → OCCUPIED (step-down / drop-off)
                '--ground-z-min',   '-2.0',   # wider lower bound (was -1.7)
                '--ground-z',       '-0.8',   # wider upper bound (was -1.0), FREE band now 1.2m
                '--obstacle-z',     '-0.8',   # match ground-z
                '--obstacle-z-max', '0.8',    # lower ceiling cutoff (was 1.0)
                '--skip-frames',    '30',
                '--min-obs-hits',   '8',      # raised from 3 → less false obstacles from noise/people
                '--map-z',          '-1.27',  # sensor-to-floor height for 3D display
                # ---- Ground-plane estimation (tilt-robust) ----
                '--use-ground-plane',          # RANSAC plane fit → height-above-plane classification
                '--ground-margin',   '0.10',   # 10cm above plane = FREE (wider for walking bounce)
                '--obstacle-margin', '0.18',   # 18cm above plane = OCCUPIED start
                '--max-obstacle-height', '2.5', # 2.5m above local floor = ceiling → ignored
                '--plane-smooth',    '0.92',   # stronger temporal smoothing
                # Note: --map-z is a fallback; grid z auto-aligns to estimated floor height
            ],
            output='screen',
        ))

    return LaunchDescription(actions)
