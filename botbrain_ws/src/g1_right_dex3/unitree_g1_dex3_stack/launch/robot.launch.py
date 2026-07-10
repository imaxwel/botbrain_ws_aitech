from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def launch_setup(context, *args, **kwargs):
    urdf_path_value = LaunchConfiguration('urdf_path').perform(context)
    urdf_name_value = LaunchConfiguration('urdf_name').perform(context)
    if urdf_path_value:
        resolved_urdf_path = urdf_path_value
    else:
        resolved_urdf_path = os.path.join(
            get_package_share_directory('unitree_g1_dex3_stack'),
            'robots',
            'g1_description',
            urdf_name_value
        )
    with open(resolved_urdf_path, 'r') as infp:
        robot_description = infp.read()
    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
            remappings=[
                ('/tf', LaunchConfiguration('tf_topic')),
                ('/tf_static', LaunchConfiguration('tf_static_topic')),
            ],
        ),
#        Node(
#            package='joint_state_publisher_gui',
#            executable='joint_state_publisher_gui',
#            name='joint_state_publisher_gui',
#            output='screen',
#        ),
        Node(
            package='unitree_g1_dex3_stack',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        )
    ]


def generate_launch_description():
    urdf_name_arg = DeclareLaunchArgument(
        'urdf_name',
        default_value='g1_29dof_lock_waist_with_hand_rev_1_0_collision_primitives.urdf',
        description='URDF filename to load from robots/g1_description/'
    )
    urdf_path_arg = DeclareLaunchArgument(
        'urdf_path',
        default_value='',
        description='Full path to URDF file (overrides urdf_name if set)'
    )
    tf_topic_arg = DeclareLaunchArgument(
        'tf_topic',
        default_value='/tf',
        description='TF topic used by robot_state_publisher'
    )
    tf_static_topic_arg = DeclareLaunchArgument(
        'tf_static_topic',
        default_value='/tf_static',
        description='TF static topic used by robot_state_publisher'
    )
    return LaunchDescription([
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        SetEnvironmentVariable('CYCLONEDDS_URI',
            '<CycloneDDS><Domain><General><Interfaces>'
            '<NetworkInterface name="enP8p1s0" priority="default" multicast="default" />'
            '</Interfaces></General></Domain></CycloneDDS>'),
        urdf_name_arg,
        urdf_path_arg,
        tf_topic_arg,
        tf_static_topic_arg,
        OpaqueFunction(function=launch_setup)
    ])
