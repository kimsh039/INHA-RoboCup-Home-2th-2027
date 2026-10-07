"""Gazebo Harmonic + ROS 2 bringup for the Tracer + sensor rack + Piper robot.

ros2 launch simulation/ros2/sim.launch.py [world:=room] [gui:=false] [rviz:=false]

Loads the final URDF and generates the SDF, starts Gazebo, bridges /clock /cmd_vel /odom /tf
/joint_states /scan /mid360/points, and runs robot_state_publisher, the Mid-360
self filter and RViz on sim time.
Drive with: ros2 run teleop_twist_keyboard teleop_twist_keyboard
"""
import os
import shutil
import subprocess
import sys
import xml.etree.ElementTree as E
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = Path(__file__).resolve().parent
SIM = HERE.parent
BUILD = SIM / 'gazebo' / 'build'


def nvidia_offload():
    """Render on the NVIDIA GPU under PRIME on-demand, like gazebo/start_sim.sh."""
    if shutil.which('nvidia-smi') and subprocess.run(['nvidia-smi'], capture_output=True).returncode == 0:
        return {'__NV_PRIME_RENDER_OFFLOAD': '1', '__GLX_VENDOR_LIBRARY_NAME': 'nvidia'}
    return {}


def setup(context):
    world = LaunchConfiguration('world').perform(context)
    gui = LaunchConfiguration('gui').perform(context) == 'true'
    rviz = LaunchConfiguration('rviz').perform(context) == 'true'

    generator = [sys.executable, str(SIM / 'gazebo' / 'make_sim.py'), '--world', world]
    if LaunchConfiguration('detection_demo').perform(context) == 'true':
        generator.append('--detection-demo')
    target_image = LaunchConfiguration('target_image').perform(context)
    if target_image:
        generator += ['--target-image', target_image]
    subprocess.run(generator, check=True)
    final = SIM / 'robot_description' / 'robocup.urdf'
    robot = E.parse(final).getroot()
    for mesh in robot.findall('.//mesh'):
        mesh.set('filename', (final.parent / mesh.get('filename')).resolve().as_uri())
    urdf = E.tostring(robot, encoding='unicode')
    sim_time = {'use_sim_time': True}

    # Server and GUI run as separate processes: a GUI crash (e.g. an Ogre render-texture error
    # when the window is minimised) or closing the window no longer stops the simulation.
    # Stop everything with Ctrl+C in the launch terminal.
    gz = ExecuteProcess(cmd=['gz', 'sim', '-s', '-r', str(BUILD / 'motion.world.sdf')],
                        additional_env=nvidia_offload(), output='screen')
    actions = [
        gz,
        RegisterEventHandler(OnProcessExit(target_action=gz, on_exit=[EmitEvent(event=Shutdown())])),
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             parameters=[{'config_file': str(HERE / 'ros_bridge.yaml')}, sim_time]),
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': urdf}, sim_time]),
        # Mid-360S cloud without self hits, for the Nav2 costmaps (/mid360/points_filtered).
        ExecuteProcess(cmd=[sys.executable, str(HERE / 'cloud_self_filter.py'),
                            '--ros-args', '-p', 'use_sim_time:=true'], output='screen'),
    ]
    if gui:
        actions.append(ExecuteProcess(cmd=['gz', 'sim', '-g'], additional_env=nvidia_offload(),
                                      output='log'))
    if rviz:
        actions.append(Node(package='rviz2', executable='rviz2', output='log',
                            arguments=['-d', str(HERE / 'sim.rviz')], parameters=[sim_time]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='empty', choices=['empty', 'room']),
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('detection_demo', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('target_image', default_value=''),
        DeclareLaunchArgument('partition', default_value='robocup_motion'),
        # Gazebo and the bridge must share these, or the bridge never finds the sim.
        # GZ_IP keeps discovery on this PC so other sims on the LAN don't mix in.
        SetEnvironmentVariable('GZ_PARTITION', LaunchConfiguration('partition')),
        SetEnvironmentVariable('GZ_IP', os.environ.get('GZ_IP', '127.0.0.1')),
        OpaqueFunction(function=setup),
    ])
