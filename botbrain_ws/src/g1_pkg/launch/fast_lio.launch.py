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

    # MID360 is physically mounted roll-180. The Livox point cloud is rotated
    # by MID360_config.json, but the IMU vectors remain in the raw sensor axes.
    # Apply the same R_x(pi) transform (Y/Z sign flip) before FAST-LIO uses it.
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
                # /cloud_registered_1 is already expressed in camera_init.
                # Accumulating this world-frame cloud avoids a second body→map
                # TF lookup and avoids treating the robot's floor height as a
                # LiDAR-to-IMU extrinsic.
                '--pre-transformed',
                '--cloud-topic',    '/cloud_registered_1',
                '--grid-topic',     '/accumulated_grid',
                '--map-frame',      'camera_init',
                '--resolution',     '0.05',
                # camera_init starts near the MID360 built-in IMU. With the
                # standing sensor about 1.247 m above the floor, map-frame
                # floor points are near z=-1.247 m. These are fallbacks while
                # the world-frame ground-plane estimator is converging.
                '--ground-z-min',   '-1.7',
                '--ground-z',       '-1.0',
                '--obstacle-z',     '-1.0',
                '--obstacle-z-max', '1.0',
                '--skip-frames',    '30',    # FAST-LIO warmup
                '--min-obs-hits',   '3',     # 3 hits to confirm obstacle
                '--map-z',          '-1.247', # fallback display height before plane fit
            ],
            output='screen',
        ))

    return LaunchDescription(actions)
