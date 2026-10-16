"""Observe web approach ROS output only; no arm, camera or navigation startup."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([Node(package='robocup_real_manipulation',
        executable='control_center_input', output='screen')])
