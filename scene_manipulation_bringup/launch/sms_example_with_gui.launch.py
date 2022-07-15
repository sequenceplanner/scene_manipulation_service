import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # provide absolute path to the scenario directory
    # TODO: add scenario path as launch argument
    scenario_path = "/home/endre/Desktop/scenario_1"
    
    if os.path.isdir(scenario_path):
        parameters = {"scenario_path": scenario_path}

        sms_node = Node(
            package="scene_manipulation_service",
            executable="sms",
            namespace="",
            output="screen",
            parameters=[parameters],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
            emulate_tty=True,
        )

        sms_gui_node = Node(
            package="scene_manipulation_gui",
            executable="scene_manipulation_gui",
            namespace="",
            output="screen",
            parameters=[parameters],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
            emulate_tty=True,
        )

        sms_marker_node = Node(
            package="scene_manipulation_marker",
            executable="scene_manipulation_marker",
            namespace="",
            output="screen",
            parameters=[parameters],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
            emulate_tty=True,
        )

        nodes_to_start = [sms_node, sms_gui_node, sms_marker_node]

        return LaunchDescription(nodes_to_start)
    else:
        print("Failed, " + scenario_path + " is not a valid path.")
        return LaunchDescription()
