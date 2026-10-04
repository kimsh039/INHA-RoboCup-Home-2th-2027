import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = str(Path(get_package_share_directory("robocup_head_detection")) /
                 "config" / "head_detection.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=config),
        DeclareLaunchArgument("model_path", default_value=os.environ.get("ROBOCUP_MODEL", "")),
        DeclareLaunchArgument("device", default_value=os.environ.get("ROBOCUP_DEVICE", "cpu")),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(package="robocup_head_detection", executable="head_detection_node",
             name="head_detection_node", output="screen",
             parameters=[LaunchConfiguration("config"), {
                 "model_path": ParameterValue(LaunchConfiguration("model_path"), value_type=str),
                 "device": ParameterValue(LaunchConfiguration("device"), value_type=str),
                 "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
             }]),
    ])
