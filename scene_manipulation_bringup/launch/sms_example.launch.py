import os
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node


# joy_config = LaunchConfiguration('joy_config')
# joy_config_dec= DeclareLaunchArgument('joy_config', default_value='ps3')
# final_string_dec= DeclareLaunchArgument('final_string', default_value=[LaunchConfiguration('joy_config'), '.config.yaml'])

def generate_launch_description():
    # default_scenario_path = "~/Desktop/scenario_1"
    # scenario_path = LaunchConfiguration("scenario_path")
    # scenario_path_launch_arg = DeclareLaunchArgument(
    #     "scenario_path", default_value=TextSubstitution(text=default_scenario_path)
    # )

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

        nodes_to_start = [sms_node]

        return LaunchDescription(nodes_to_start)
    else:
        print("Failed, " + scenario_path + " is not a valid path.")
        return LaunchDescription()
