"""Publish calibrated runtime TF; optional saved scan replay. Does not start Gazebo."""
from pathlib import Path
import sys
import os
import xml.etree.ElementTree as ET
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = Path(__file__).resolve().parent


def setup(context):
    urdf_path = HERE.parent / 'robot_description/robocup.calibrated.urdf'
    robot = ET.parse(urdf_path).getroot()
    for mesh in robot.findall('.//mesh'):
        filename = mesh.get('filename')
        if not filename.startswith(('package://', 'file://', 'http://', 'https://')):
            mesh.set('filename', (urdf_path.parent / filename).resolve().as_uri())
    actions = [Node(package='robot_state_publisher', executable='robot_state_publisher',
                    name='robot_state_publisher', output='screen',
                    parameters=[{'robot_description': ET.tostring(robot,encoding='unicode'),
                                 'use_sim_time': False}])]
    if LaunchConfiguration('joints').perform(context).lower() == 'true':
        actions.append(ExecuteProcess(
            cmd=[sys.executable, str(HERE/'publish_measured_joints.py')],
            additional_env={'GZ_IP': os.environ.get('GZ_IP','127.0.0.1'),
                            'GZ_PARTITION': os.environ.get('GZ_PARTITION','robocup_motion')},
            output='screen'))
    if LaunchConfiguration('replay').perform(context).lower() == 'true':
        actions.append(ExecuteProcess(
            cmd=[sys.executable, str(HERE/'replay_calibration_scan.py'),
                 '--scan', str(HERE/'calibrated_validation_scan.json.gz')], output='screen'))
    if LaunchConfiguration('rviz').perform(context).lower() == 'true':
        actions.append(Node(package='rviz2', executable='rviz2', name='rviz2',
                            arguments=['-d',str(HERE/'tf_calibrated.rviz')],
                            parameters=[{'use_sim_time':False}], output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rviz',default_value='true'),
        DeclareLaunchArgument('replay',default_value='false'),
        DeclareLaunchArgument('joints',default_value='true'),
        OpaqueFunction(function=setup),
    ])
