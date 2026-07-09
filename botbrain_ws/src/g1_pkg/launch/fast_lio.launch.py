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

    # imu_flip.py REMOVED — matches g1_3d_nav_ros2 reference.
    # The Livox SDK already publishes IMU in the correct Z-up frame;
    # the YZ-flip was likely compensating for an older SDK version and
    # introduced gravity misalignment → tilt + drift.
    actions = [
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
                # ---- Body-frame classification (matches g1_3d_nav_ros2 reference) ----
                # Classify in body frame (z relative to robot) → TF to map frame.
                # With extrinsic_T z=1.247, body origin ≈ ground level:
                #   body-frame floor z≈0, LiDAR at z≈1.247
                #   z < 0.15 → FREE (ground),  z > 0.25 → OCCUPIED
                # This is inherently TILT-PROOF — body frame is relative to robot.
                '--no-pre-transformed',            # enable TF body→map lookup
                '--cloud-topic',    '/cloud_registered_body_1',
                '--grid-topic',     '/accumulated_grid',
                '--map-frame',      'camera_init',
                '--body-frame',     'body',
                '--resolution',     '0.05',
                '--ground-z-min',   '-100',  # effectively no lower bound
                '--ground-z',       '0.15',  # body frame: z<0.15m = ground (floor at z≈0)
                '--obstacle-z',     '0.25',  # body frame: z>0.25m = obstacle
                '--obstacle-z-max', '100',   # effectively no upper bound
                '--skip-frames',    '30',    # FAST-LIO warmup
                '--min-obs-hits',   '3',     # 3 hits to confirm obstacle
                '--map-z',          '0.0',   # body origin at ground level
                '--no-ground-plane',          # body-frame is inherently tilt-proof
            ],
            output='screen',
        ))

    return LaunchDescription(actions)
