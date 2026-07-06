import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

################### user configure parameters for ros2 start ###################
xfer_format   = 1    # 1-CustomMsg (per-point timestamps, required for humanoid motion compensation)
multi_topic   = 0
data_src      = 0
publish_freq  = 20.0
output_type   = 0
frame_id      = 'mid360_link'
lvx_file_path = '/home/livox/livox_test.lvx'
cmdline_bd_code = 'livox0000000001'

cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
cur_config_path = cur_path + '../config'
user_config_path = os.path.join(cur_config_path, 'MID360_config.json')

def generate_launch_description():
    # Accept prefix/namespace args (passed by robot_interface.launch.py) but do not use them.
    # Lidar publishes CustomMsg on /livox/lidar and IMU on /livox/imu — no namespace.
    # fast_lio mid360.yaml subscribes to these topics directly.
    prefix_arg    = DeclareLaunchArgument('prefix',    default_value='')
    namespace_arg = DeclareLaunchArgument('namespace', default_value='')

    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[
            {"xfer_format": xfer_format},
            {"multi_topic": multi_topic},
            {"data_src": data_src},
            {"publish_freq": publish_freq},
            {"output_data_type": output_type},
            {"frame_id": frame_id},
            {"lvx_file_path": lvx_file_path},
            {"user_config_path": user_config_path},
            {"cmdline_input_bd_code": cmdline_bd_code},
        ],
    )

    return LaunchDescription([prefix_arg, namespace_arg, livox_driver])

