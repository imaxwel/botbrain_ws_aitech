from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():

    launch_dir = os.path.dirname(os.path.abspath(__file__))
    workspace_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(launch_dir)))))
    config_file = os.path.join(workspace_dir, 'robot_config.yaml')

    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)['robot_configuration']

    robot_model = config['robot_model']
    robot_package_name = f"{robot_model}_pkg"
    map_name = config.get('default_map') or 'rtabmap.db'
    database_path = os.path.join(workspace_dir, 'src', robot_package_name, 'maps', map_name)

    # Ensure maps directory exists
    os.makedirs(os.path.dirname(database_path), exist_ok=True)

    if robot_model == "g1":
        return LaunchDescription([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "rtabmap_lidar_mapping.launch.py")
                ),
                launch_arguments={'database_path': database_path}.items(),
            )
        ])

    return LaunchDescription([])
