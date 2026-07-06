from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
import yaml

def generate_launch_description():

  launch_dir = os.path.dirname(os.path.abspath(__file__))
  workspace_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(launch_dir)))))
  config_file = os.path.join(workspace_dir, 'robot_config.yaml')
  with open(config_file, 'r') as f:
      config = yaml.safe_load(f)['robot_configuration']
  robot_name = config['robot_name']
  prefix = robot_name + '/' if robot_name != '' else ''

  fixed_frame_id = f'{prefix}odom'
  frame_id = f'{prefix}base_link'
  odom_topic = f'{prefix}odom'
  lidar_topic = f'{prefix}pointcloud'
  lidar_topic_deskewed = lidar_topic + '/deskewed'

  voxel_size_value = 0.1
  max_correspondence_distance = voxel_size_value * 10.0

  database_path_arg = DeclareLaunchArgument(
      'database_path',
      description='Path to save the RTAB-Map database file'
  )
  database_path = LaunchConfiguration('database_path')

  shared_parameters = {
    'use_sim_time': False,
    'frame_id': frame_id,
    'qos': 1,
    'approx_sync': True,
    'wait_for_transform': 0.5,
    'Icp/PointToPlane': 'true',
    'Icp/Iterations': '10',
    'Icp/VoxelSize': str(voxel_size_value),
    'Icp/Epsilon': '0.001',
    'Icp/PointToPlaneK': '20',
    'Icp/PointToPlaneRadius': '0',
    'Icp/MaxTranslation': '3',
    'Icp/MaxCorrespondenceDistance': str(max_correspondence_distance),
    'Icp/Strategy': '1',
    'Icp/OutlierRatio': '0.7',
  }

  rtabmap_parameters = {
    'subscribe_depth': False,
    'subscribe_rgb': False,
    'subscribe_odom_info': False,
    'subscribe_scan_cloud': True,
    'map_frame_id': f'{prefix}map',
    'database_path': database_path,
    'RGBD/ProximityMaxGraphDepth': '0',
    'RGBD/ProximityPathMaxNeighbors': '1',
    'RGBD/AngularUpdate': '0.05',
    'RGBD/LinearUpdate': '0.05',
    'RGBD/CreateOccupancyGrid': 'true',
    'Mem/NotLinkedNodesKept': 'false',
    'Mem/STMSize': '30',
    'Reg/Strategy': '1',
    'Icp/CorrespondenceRatio': '0.2',
    'Mem/IncrementalMemory': 'True',   # mapping mode
    'Mem/InitWMWithAllNodes': 'False',
  }

  nodes = [
    Node(
      package='rtabmap_slam', executable='rtabmap', output='screen',
      namespace=robot_name,
      parameters=[shared_parameters, rtabmap_parameters],
      remappings=[
        ('odom', 'odom'),
        ('scan_cloud', 'pointcloud/deskewed')
      ]
    ),
    Node(
      package='rtabmap_util', executable='lidar_deskewing', output='screen',
      namespace=robot_name,
      parameters=[{
        'use_sim_time': False,
        'fixed_frame_id': fixed_frame_id,
        'wait_for_transform': 0.5,
        'slerp': True}],
      remappings=[
          ('input_cloud', 'pointcloud'),
          ('output_cloud', 'pointcloud/deskewed')
      ])
  ]

  return LaunchDescription([database_path_arg] + nodes)
