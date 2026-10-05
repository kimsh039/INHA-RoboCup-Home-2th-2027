import os
import sys
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('robocup_head_detection'))
    return LaunchDescription([
        DeclareLaunchArgument('model_path', default_value=os.environ.get('ROBOCUP_MODEL', '')),
        DeclareLaunchArgument('device', default_value=os.environ.get('ROBOCUP_DEVICE', 'cpu')),
        DeclareLaunchArgument('auto_send', default_value='false'),
        DeclareLaunchArgument('standoff_distance', default_value='1.0'),
        DeclareLaunchArgument('start_detection', default_value='true'),
        DeclareLaunchArgument('python_executable', default_value=sys.executable),
        DeclareLaunchArgument('config', default_value=str(share/'config'/'sim_detection.yaml')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share/'launch'/'head_detection.launch.py')),
            launch_arguments={'config': LaunchConfiguration('config'),
                              'model_path': LaunchConfiguration('model_path'),
                              'device': LaunchConfiguration('device'), 'use_sim_time': 'true',
                              'python_executable': LaunchConfiguration('python_executable')}.items(),
            condition=IfCondition(LaunchConfiguration('start_detection'))),
        Node(package='robocup_head_detection', executable='detection_nav_goal_node', output='screen',
             prefix=LaunchConfiguration('python_executable'),
             parameters=[{'use_sim_time': True,
                          'auto_send': ParameterValue(LaunchConfiguration('auto_send'), value_type=bool),
                          'standoff_distance': ParameterValue(LaunchConfiguration('standoff_distance'), value_type=float)}]),
    ])
