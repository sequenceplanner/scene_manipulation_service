import os
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    scenario_path = "/home/endre/ws2/src/scene_manipulation_service/scene_manipulation_bringup/scenario_1"
    parameters = {
        "scenario_path": scenario_path
    }
    
    sms_node = Node(
        package="scene_manipulation_service",
        executable="sms",
        namespace="",
        output="screen",
        parameters=[parameters],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True
    )

    nodes_to_start = [
        sms_node
    ]

    return LaunchDescription(nodes_to_start)
