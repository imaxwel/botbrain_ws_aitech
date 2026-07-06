from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    auto_return_to_standing = (
        LaunchConfiguration('auto_return_to_standing').perform(context).lower() == 'true'
    )
    hold_indefinitely = (
        LaunchConfiguration('hold_indefinitely').perform(context).lower() == 'true'
    )
    return [
        Node(
            package='unitree_g1_dex3_stack',
            executable='joint_trajectory_executor',
            name='joint_trajectory_executor',
            output='screen',
            parameters=[{
                'auto_return_to_standing': auto_return_to_standing,
                'hold_indefinitely': hold_indefinitely,
            }]
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('auto_return_to_standing', default_value='true'),
        DeclareLaunchArgument('hold_indefinitely', default_value='false'),
        OpaqueFunction(function=launch_setup)
    ])
