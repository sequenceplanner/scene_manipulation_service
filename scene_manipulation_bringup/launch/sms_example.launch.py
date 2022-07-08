import os
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # option 1: this will be located in the install dir of the workspace i.e.
    # ~/humble_ws/install/scene_manipulation_bringup/share/scene_manipulation_bringup/scenario_1
    scenario_dir = FindPackageShare("scene_manipulation_bringup").find(
        "scene_manipulation_bringup"
    )
    scenario_path = os.path.join(scenario_dir, "scenario_1")

    # option 2: provide the absolute path to the scenario folder, easy to manipulate, add and remove frames, for example
    # scenario_path = ~/Desktop/scenario_1

    parameters = {"scenario_path": scenario_path}

    sms_node = Node(
        package="scene_manipulation_service",
        executable="sms",
        namespace="",
        output="screen",
        parameters=[],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True,
    )

    nodes_to_start = [sms_node]

    return LaunchDescription(nodes_to_start)
