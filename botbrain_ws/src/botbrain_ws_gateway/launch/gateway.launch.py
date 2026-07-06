from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare("botbrain_ws_gateway"),
        "config",
        "gateway.yaml",
    ])

    config_arg = DeclareLaunchArgument(
        "config",
        default_value=config_file,
        description="Path to botbrain_ws_gateway YAML config.",
    )

    gateway = Node(
        package="botbrain_ws_gateway",
        executable="botbrain_ws_gateway",
        name="botbrain_ws_gateway",
        output="screen",
        arguments=["--config", LaunchConfiguration("config")],
    )

    return LaunchDescription([config_arg, gateway])
