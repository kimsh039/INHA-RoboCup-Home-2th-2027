"""Gazebo Harmonic + ROS 2 bringup for the Tracer + sensor rack + Piper robot.

ros2 launch simulation/ros2/sim.launch.py [world:=room] [gui:=false] [rviz:=false]

Regenerates the URDF/SDF, starts Gazebo, bridges /clock /cmd_vel /odom /tf
/joint_states /scan, and runs robot_state_publisher and RViz on sim time.
Drive with: ros2 run teleop_twist_keyboard teleop_twist_keyboard
"""
import os
import shutil
import subprocess
import sys
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

    build_cmd=[sys.executable, str(SIM / 'robot_description' / 'build.py')]
    if LaunchConfiguration('wrist_camera').perform(context)=='true':
        build_cmd.append('--with-wrist-camera')
    subprocess.run(build_cmd, check=True)
    subprocess.run([sys.executable, str(SIM / 'gazebo' / 'make_sim.py'), '--world', world], check=True)
    # make_sim.py writes absolute mesh paths; RViz needs them as file:// URIs.
    urdf = (BUILD / 'sim_local.urdf').read_text().replace('filename="/', 'filename="file:///')
    sim_time = {'use_sim_time': True}

    gz = ExecuteProcess(
        cmd=['gz', 'sim', '-r', str(BUILD / 'motion.world.sdf')] + ([] if gui else ['-s']),
        additional_env=nvidia_offload(), output='screen')
    actions = [
        gz,
        # Closing the Gazebo window ends the whole launch.
        RegisterEventHandler(OnProcessExit(target_action=gz, on_exit=[EmitEvent(event=Shutdown())])),
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             parameters=[{'config_file': str(HERE / 'ros_bridge.yaml')}, sim_time]),
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': urdf}, sim_time]),
    ]
    if rviz:
        actions.append(Node(package='rviz2', executable='rviz2', output='log',
                            arguments=['-d', str(HERE / 'sim.rviz')], parameters=[sim_time]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='empty', choices=['empty', 'room']),
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('wrist_camera', default_value='false', choices=['true', 'false']),
        # Gazebo and the bridge must share these, or the bridge never finds the sim.
        # GZ_IP keeps discovery on this PC so other sims on the LAN don't mix in.
        SetEnvironmentVariable('GZ_PARTITION', 'robocup_motion'),
        SetEnvironmentVariable('GZ_IP', os.environ.get('GZ_IP', '127.0.0.1')),
        OpaqueFunction(function=setup),
    ])
