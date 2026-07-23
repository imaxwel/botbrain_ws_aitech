import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo
from launch_ros.actions import Node


MAPPING_PROFILES = {
    'default': {},
    # Preserve more doorway/corner geometry and reduce the influence of
    # distant parallel corridor walls. This mitigates corridor degeneracy but
    # is not a pose-graph loop-closure backend.
    'corridor': {
        'point_filter_num': 2,
        'max_iteration': 5,
        'filter_size_surf': 0.25,
        'filter_size_map': 0.25,
        'preprocess.max_range': 20.0,
    },
}


def _environment_bool(name, fallback):
    value = os.environ.get(name, 'auto').strip().lower()
    if value in ('', 'auto'):
        return bool(fallback)
    if value in ('1', 'true', 'yes', 'on'):
        return True
    if value in ('0', 'false', 'no', 'off'):
        return False
    raise RuntimeError(
        f"Unsupported {name}={value!r}; expected auto, true or false"
    )


def generate_launch_description():
    pkg_config = os.path.join(
        get_package_share_directory('fast_lio'), 'config', 'mid360.yaml'
    )

    with open(pkg_config, 'r') as f:
        mid360 = yaml.safe_load(f)
    yaml_parameters = mid360.get('/**', {}).get('ros__parameters', {})
    yaml_pcd_save_en = yaml_parameters.get(
        'pcd_save', {}).get('pcd_save_en', False)
    mapping_save_en = _environment_bool(
        'FAST_LIO_MAPPING_SAVE', yaml_pcd_save_en)
    mapping_mode = _environment_bool(
        'FAST_LIO_MAPPING_MODE', mapping_save_en)
    map_file_override = os.environ.get('FAST_LIO_MAP_FILE', '').strip()
    effective_map_file = (
        map_file_override or yaml_parameters.get('map_file_path', '')
    )
    if mapping_save_en and not effective_map_file:
        raise RuntimeError(
            'FAST_LIO_MAPPING_SAVE=true requires FAST_LIO_MAP_FILE or '
            'map_file_path in mid360.yaml'
        )

    mapping_profile = os.environ.get(
        'FAST_LIO_MAPPING_PROFILE', 'default'
    ).strip().lower()
    if mapping_profile not in MAPPING_PROFILES:
        supported = ', '.join(sorted(MAPPING_PROFILES))
        raise RuntimeError(
            f"Unsupported FAST_LIO_MAPPING_PROFILE={mapping_profile!r}; "
            f"expected one of: {supported}"
        )
    # Profile tuning is only for live mapping. Navigation must use the
    # conservative values from mid360.yaml even though Compose carries the
    # same profile environment variable.
    profile_parameters = MAPPING_PROFILES[mapping_profile] if mapping_mode else {}

    # FAST-LIO subscribes directly to /livox/imu and applies the upside-down
    # MID360 Y/Z sign correction in C++. Do not also launch imu_flip.py, or the
    # IMU will be transformed twice.
    actions = [
        LogInfo(msg=(
            f"FAST-LIO mapping profile: {mapping_profile}; "
            f"overrides={profile_parameters or 'none'}; "
            f"mapping_mode={mapping_mode}; save={mapping_save_en}; "
            f"map_file={effective_map_file or 'disabled'}"
        )),
        Node(
            package='fast_lio',
            executable='fastlio_mapping',
            name='fast_lio',
            output='screen',
            parameters=[
                pkg_config,
                {
                    'use_sim_time': False,
                    'pcd_save.pcd_save_en': mapping_save_en,
                    'map_file_path': effective_map_file,
                },
                profile_parameters,
            ],
            # A large PCD can take well over the launch default of 5 seconds to
            # flush. Keep Docker's grace period longer than these two timeouts.
            sigterm_timeout='150',
            sigkill_timeout='20',
        ),
    ]

    if mapping_mode:
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
                # Publishing a growing OccupancyGrid at 2 Hz caused multi-
                # second Python serialization stalls on the Jetson. FAST-LIO
                # remains 10 Hz; the grid is a lower-rate visualization/save
                # product and processes every third cloud.
                '--rate',           '0.5',
                # Fixed-z values are used only with --no-ground-plane. In the
                # normal path, a constrained plane is initialized around the
                # known body-to-floor height and must pass quality gates.
                '--ground-z-min',   '-1.35',
                '--ground-z',       '-1.15',
                '--obstacle-z',     '-1.14',
                '--obstacle-z-max', '0.35',
                '--skip-frames',    '60',    # FAST-LIO warmup
                '--process-every',  '3',
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
                '--map-z',          '-1.247', # fallback display height before plane fit
            ],
            output='screen',
        ))

    return LaunchDescription(actions)
