import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    robot = os.environ.get("G1_ROBOT_NAMESPACE", "g1_robot").strip() or "g1_robot"
    package_share = get_package_share_directory("bot_voice_navigation")
    # All runtime containers mount the workspace at this stable path.
    waypoint_file = "/botbrain_ws/src/bot_navigation/nav_waypoints.yaml"
    aliases_file = os.path.join(package_share, "config", "voice_destinations.yaml")
    scene_file = "/botbrain_ws/.runtime/map_scene"

    common = {
        "robot": robot,
        "waypoint_file": waypoint_file,
        "aliases_file": aliases_file,
        "scene_file": scene_file,
        "asr_topics": ["/audio_msg", "/rt/audio_msg"],
    }
    gateway = Node(
        package="bot_voice_navigation",
        executable="voice_navigation_gateway.py",
        name="voice_navigation_gateway",
        namespace=robot,
        output="screen",
        parameters=[common],
    )
    voice = Node(
        package="bot_voice_navigation",
        executable="voice_navigation_node.py",
        name="voice_navigation_input",
        output="screen",
        parameters=[common],
    )
    return LaunchDescription([gateway, voice])
