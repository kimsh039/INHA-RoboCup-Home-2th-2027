"""Owned SLAM backend and one public map->odom clock adapter for Humble."""
from pathlib import Path
import math

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = Path(__file__).resolve().parent


def nodes(context):
    params = LaunchConfiguration('slam_params_file').perform(context)
    mode = LaunchConfiguration('mode').perform(context)
    sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    overrides = {'use_sim_time': sim_time}
    executable = 'async_slam_toolbox_node'
    if mode == 'localization':
        executable = 'localization_slam_toolbox_node'
        profile = yaml.safe_load(Path(params).read_text())
        settings = dict(profile.get('/**', {}).get('ros__parameters', {}))
        settings.update(profile.get('/slam_toolbox', profile.get('slam_toolbox', {})).get('ros__parameters', {}))
        start_pose = settings.get('map_start_pose', [0., 0., 0.])
        if len(start_pose) != 3 or not all(math.isfinite(float(v)) for v in start_pose):
            raise ValueError('map_start_pose must contain finite x, y and yaw values')
        overrides.update(mode='localization',
                         map_file_name=LaunchConfiguration('map_file_name').perform(context),
                         map_start_pose=[float(v) for v in start_pose])
    actions = [
        ExecuteProcess(cmd=['/usr/bin/python3', str(HERE / 'slam-tf-clock.py'), '--ros-args',
                            '--params-file', params, '-p', 'use_sim_time:=' + str(sim_time).lower()],
                       output='screen'),
        Node(package='slam_toolbox', executable=executable, name='slam_toolbox',
             parameters=[params, overrides], output='screen',
             # The backend's separate transform_listener_impl node keeps /tf input.
             # Applying a process-wide /tf remap would also hide EKF odometry TF.
             remappings=[('slam_toolbox:/tf', '/slam_toolbox/tf_raw')]),
    ]
    if mode == 'localization':
        actions.append(ExecuteProcess(cmd=[
            '/usr/bin/python3', str(HERE / 'slam-global-initializer.py'),
            '--backend', params, '--map', LaunchConfiguration('map_file_name').perform(context),
            '--ros-args', '--params-file', str(HERE / 'global-localization.yaml'),
            '-p', 'use_sim_time:=' + str(sim_time).lower()], output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('slam_params_file'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('mode', default_value='mapping', choices=['mapping', 'localization']),
        DeclareLaunchArgument('map_file_name', default_value=''),
        OpaqueFunction(function=nodes),
    ])
