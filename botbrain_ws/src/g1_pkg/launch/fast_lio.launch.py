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

    # FAST-LIO subscribes directly to /livox/imu and applies the upside-down
    # MID360 Y/Z sign correction in C++. Do not also launch imu_flip.py, or the
    # IMU will be transformed twice.
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
                # /cloud_registered_1 is already expressed in camera_init.
                # Accumulating this world-frame cloud avoids a second body→map
                # TF lookup and avoids treating the robot's floor height as a
                # LiDAR-to-IMU extrinsic.
                '--pre-transformed',
                '--cloud-topic',    '/cloud_registered_1',
                '--odom-topic',     '/Odometry_loc',
                '--grid-topic',     '/accumulated_grid',
                '--map-frame',      'camera_init',
                '--body-frame',     'body',
                '--resolution',     '0.05',
                # Fixed-z values are used only with --no-ground-plane. In the
                # normal path, a constrained plane is initialized around the
                # known body-to-floor height and must pass quality gates.
                '--ground-z-min',   '-1.35',
                '--ground-z',       '-1.15',
                '--obstacle-z',     '-1.14',
                '--obstacle-z-max', '0.35',
                '--skip-frames',    '60',    # FAST-LIO warmup
                '--sensor-height',  '1.247',
                '--below-ground-tolerance', '0.10',
                '--ground-margin',          '0.08',
                # Keep a small 2 cm dead band above the 8 cm floor band. This
                # recovers low furniture/boxes without classifying floor noise.
                '--obstacle-margin',        '0.10',
                # Navigation only needs the lower wall/furniture band. Keeping
                # this below normal ceiling height prevents a ceiling plane
                # from being projected into a solid black floor region.
                '--max-obstacle-height',    '1.60',
                '--plane-init-frames',      '3',
                '--plane-max-tilt-deg',     '5.0',
                '--plane-max-expected-error', '0.18',
                '--plane-max-median-residual', '0.035',
                '--max-point-range',        '30.0',
                # Conservative G1 envelope in the FAST-LIO body/IMU frame.
                '--self-x-min',     '-0.40',
                '--self-x-max',     '0.40',
                '--self-y-abs',     '0.40',
                '--self-z-min',     '-1.35',
                '--self-z-max',     '0.15',
                # Point-by-point free-space generation is intentionally used
                # for saved-map editing. Ray clearing remains available as an
                # opt-in diagnostic but otherwise over-whitens the map.
                '--no-raytrace',
                # One-cell metric support keeps sparse walls connected without
                # turning each return into the previous full 3x3 black block.
                '--obstacle-spread-radius', '0.05',
                # Ground support uses a weaker update. After a person leaves,
                # repeated floor returns clear the transient footprint while
                # continuously observed walls and furniture remain occupied.
                '--free-spread-radius',     '0.05',
                '--free-update',            '0.30',
                '--min-obs-hits',   '3',     # distinct accepted scan frames
                '--debug-clouds',            # ground/obstacle/self diagnostics
                '--map-z',          '-1.247', # fallback display height before plane fit
            ],
            output='screen',
        ))

    return LaunchDescription(actions)
