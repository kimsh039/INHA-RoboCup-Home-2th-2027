"""Closed approach after head detection: support surfaces, approach poses, docking, wrist
observation (IK) and wrist SAM 2.1. Gazebo, SLAM, Nav2 and the head detector run separately
(see SW/detection/CLOSED_APPROACH.md)."""
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim = {'use_sim_time': LaunchConfiguration('use_sim_time')}
    gz_env = {'GZ_PARTITION': LaunchConfiguration('gz_partition'), 'GZ_IP': '127.0.0.1'}
    wrist = IfCondition(LaunchConfiguration('start_wrist'))
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('start_wrist', default_value='true',
                              description='arm observation pose + wrist SAM after docking'),
        DeclareLaunchArgument('gz_partition', default_value='robocup_motion'),
        DeclareLaunchArgument('python_executable', default_value=sys.executable,
                              description='Python with torch + sam2 for the wrist SAM node'),
        DeclareLaunchArgument('sam_checkpoint', default_value='models/sam2.1_hiera_tiny.pt'),
        Node(package='robocup_head_detection', executable='support_surface_node', output='screen', parameters=[sim]),
        Node(package='robocup_head_detection', executable='approach_node', output='screen', parameters=[sim]),
        Node(package='robocup_head_detection', executable='dock_node', output='screen', parameters=[sim]),
        Node(package='robocup_head_detection', executable='wrist_observe_node', output='screen', parameters=[sim],
             additional_env=gz_env, condition=wrist),
        Node(package='robocup_head_detection', executable='wrist_sam_node', output='screen',
             prefix=LaunchConfiguration('python_executable'), condition=wrist,
             parameters=[sim, {'checkpoint': LaunchConfiguration('sam_checkpoint')}]),
    ])
