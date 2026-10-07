"""Closed approach after head detection: support surfaces, approach poses and docking.
Gazebo, SLAM, Nav2 and the head detector run separately (see SW/detection/CLOSED_APPROACH.md)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim = {'use_sim_time': LaunchConfiguration('use_sim_time')}
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(package='robocup_head_detection', executable='support_surface_node', output='screen', parameters=[sim]),
        Node(package='robocup_head_detection', executable='approach_node', output='screen', parameters=[sim]),
        Node(package='robocup_head_detection', executable='dock_node', output='screen', parameters=[sim]),
    ])
