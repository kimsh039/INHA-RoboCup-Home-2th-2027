"""Jetson bringup using committed physical calibrations and measured arm feedback."""
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = Path(__file__).resolve().parent
WORKSPACE = Path('/home/sparo')


def generate_launch_description():
    config = HERE / 'config'
    path = HERE / 'robot.physical.urdf'
    robot = ET.parse(path).getroot()
    for mesh in robot.findall('.//mesh'):
        filename = mesh.get('filename', '')
        if filename and not filename.startswith(('package:', 'file:', 'http:', 'https:')):
            mesh.set('filename', (path.parent / filename).resolve().as_uri())
    actions = [
        DeclareLaunchArgument('tracer_can', default_value='can_tracer'),
        DeclareLaunchArgument('piper_can', default_value='can_arm'),
        DeclareLaunchArgument('tracer', default_value='true'),
        DeclareLaunchArgument('calibration', default_value='true'),
        DeclareLaunchArgument('arm', default_value='true'),
        DeclareLaunchArgument('head_camera', default_value='true'),
        DeclareLaunchArgument('wrist_camera', default_value='true'),
        DeclareLaunchArgument('g2_lidar', default_value='true'),
        DeclareLaunchArgument('mid360_lidar', default_value='true'),
        Node(package='tracer_base', executable='tracer_base_node', name='tracer_base_node',
             parameters=[str(config / 'tracer.yaml'), {'port_name': LaunchConfiguration('tracer_can')}],
             remappings=[('/tf', '/wheel/tf'), ('/tf_static', '/wheel/tf_static'),
                         ('/cmd_vel', '/tracer/cmd_vel_safe')],
             condition=IfCondition(LaunchConfiguration('tracer')), output='screen'),
        Node(package='ydlidar_ros2_driver', executable='ydlidar_ros2_driver_node',
             parameters=[str(WORKSPACE / 'tracer_ws/src/ydlidar_ros2_driver/params/G2.yaml')],
             remappings=[('scan', '/scan_raw'), ('point_cloud', '/g2/points_raw')],
             condition=IfCondition(LaunchConfiguration('g2_lidar')), output='screen'),
        Node(package='livox_ros_driver2', executable='livox_ros_driver2_node', name='livox_lidar_publisher',
             parameters=[{'xfer_format': 0, 'multi_topic': 0, 'data_src': 0, 'publish_freq': 10.,
                          'output_data_type': 0, 'frame_id': 'livox_frame',
                          'user_config_path': str(WORKSPACE / 'ws_livox/src/livox_ros_driver2/config/MID360s_config.json'),
                          'cmdline_input_bd_code': 'livox0000000001'}],
             condition=IfCondition(LaunchConfiguration('mid360_lidar')), output='screen'),
        ExecuteProcess(cmd=[sys.executable, str(HERE / 'imu_base.py'), '--ros-args',
                            '--params-file', str(config / 'imu_base.yaml')],
                       condition=IfCondition(LaunchConfiguration('calibration')), output='screen'),
        ExecuteProcess(cmd=[sys.executable, str(HERE / 'rotation_fallback.py')],
                       condition=IfCondition(LaunchConfiguration('calibration')), output='screen'),
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
             parameters=[str(config / 'ekf.yaml')], remappings=[('odometry/filtered', '/odom_ekf')],
             condition=IfCondition(LaunchConfiguration('calibration')), output='screen'),
        ExecuteProcess(cmd=[sys.executable, str(HERE / 'sensor_tf.py')],
                       condition=IfCondition(LaunchConfiguration('calibration')), output='screen'),
        ExecuteProcess(cmd=[sys.executable, str(HERE / 'calibrated_streams.py')],
                       additional_env={'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1'},
                       respawn=True, respawn_delay=2.,
                       condition=IfCondition(LaunchConfiguration('calibration')), output='screen'),
        ExecuteProcess(cmd=[sys.executable, str(HERE / 'slam_scan.py'), '--ros-args',
                            '--params-file', str(config / 'slam_scan.yaml')],
                       condition=IfCondition(LaunchConfiguration('calibration')), output='screen'),
        ExecuteProcess(cmd=['bash', str(HERE / 'run-collision-cloud.sh'), '--ros-args',
                            '--params-file', str(config / 'collision_safety.yaml')],
                       additional_env={'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1'},
                       respawn=True, respawn_delay=2.,
                       condition=IfCondition(LaunchConfiguration('calibration')), output='screen'),
        ExecuteProcess(cmd=['bash', str(HERE / 'run-collision-guard.sh'), '--ros-args',
                            '--params-file', str(config / 'collision_safety.yaml')],
                       additional_env={'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1'},
                       respawn=True, respawn_delay=2.,
                       condition=IfCondition(LaunchConfiguration('calibration')), output='screen'),
        ExecuteProcess(cmd=[sys.executable, str(HERE / 'tracer_safety_watchdog.py'), '--ros-args',
                            '--params-file', str(config / 'collision_safety.yaml')],
                       condition=IfCondition(LaunchConfiguration('calibration')), output='screen'),
        ExecuteProcess(cmd=[sys.executable, str(HERE / 'piper_feedback.py'), '--ros-args',
                            '-p', ['can_port:=', LaunchConfiguration('piper_can')]],
                       respawn=True, respawn_delay=2.,
                       condition=IfCondition(LaunchConfiguration('arm')), output='screen'),
        ExecuteProcess(cmd=[sys.executable, str(HERE / 'piper_joint_bridge.py')],
                       condition=IfCondition(LaunchConfiguration('arm')), output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher', name='physical_arm_state_publisher',
             parameters=[{'robot_description': ET.tostring(robot, encoding='unicode'), 'use_sim_time': False}],
             remappings=[('joint_states', '/robot/joint_states')],
             condition=IfCondition(LaunchConfiguration('arm')), output='screen'),
    ]
    for namespace, camera_name, serial, fps in [('head', 'camera', '244622071116', 30),
                                              ('wrist', 'wrist', '243222070076', 30)]:
        actions.append(ExecuteProcess(cmd=[
            'ros2', 'launch', 'realsense2_camera', 'rs_launch.py',
            'camera_namespace:=' + namespace, 'camera_name:=' + camera_name, 'serial_no:=_' + serial,
            'rgb_camera.color_profile:=640,480,' + str(fps),
            'depth_module.depth_profile:=640,480,' + str(fps),
            'enable_color:=true', 'enable_depth:=true', 'align_depth.enable:=true',
            'enable_gyro:=false', 'enable_accel:=false', 'pointcloud.enable:=false', 'publish_tf:=true',
        ], condition=IfCondition(LaunchConfiguration(namespace + '_camera')), output='screen'))
    return LaunchDescription(actions)
