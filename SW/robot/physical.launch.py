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
        DeclareLaunchArgument('tracer_can', default_value='can2'),
        DeclareLaunchArgument('piper_can', default_value='can0'),
        DeclareLaunchArgument('arm', default_value='true'),
        DeclareLaunchArgument('head_camera', default_value='true'),
        DeclareLaunchArgument('wrist_camera', default_value='true'),
        DeclareLaunchArgument('g2_lidar', default_value='true'),
        DeclareLaunchArgument('mid360_lidar', default_value='true'),
        Node(package='tracer_base', executable='tracer_base_node', name='tracer_base_node',
             parameters=[str(config / 'tracer.yaml'), {'port_name': LaunchConfiguration('tracer_can')}],
             remappings=[('/tf', '/wheel/tf'), ('/tf_static', '/wheel/tf_static')], output='screen'),
        Node(package='ydlidar_ros2_driver', executable='ydlidar_ros2_driver_node',
             parameters=[str(WORKSPACE / 'tracer_ws/src/ydlidar_ros2_driver/params/G2.yaml')],
             remappings=[('scan', '/scan_raw')], condition=IfCondition(LaunchConfiguration('g2_lidar')), output='screen'),
        Node(package='livox_ros_driver2', executable='livox_ros_driver2_node', name='livox_lidar_publisher',
             parameters=[{'xfer_format': 0, 'multi_topic': 0, 'data_src': 0, 'publish_freq': 10.,
                          'output_data_type': 0, 'frame_id': 'livox_frame',
                          'user_config_path': str(WORKSPACE / 'ws_livox/src/livox_ros_driver2/config/MID360s_config.json'),
                          'cmdline_input_bd_code': 'livox0000000001'}],
             condition=IfCondition(LaunchConfiguration('mid360_lidar')), output='screen'),
        ExecuteProcess(cmd=[sys.executable, str(HERE / 'imu_base.py'), '--ros-args',
                            '--params-file', str(config / 'imu_base.yaml')], output='screen'),
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
             parameters=[str(config / 'ekf.yaml')], remappings=[('odometry/filtered', '/odom_ekf')], output='screen'),
        ExecuteProcess(cmd=[sys.executable, str(HERE / 'sensor_tf.py')], output='screen'),
        ExecuteProcess(cmd=[sys.executable, str(HERE / 'calibrated_streams.py')], output='screen'),
        Node(package='piper', executable='piper_read_slave_joint', name='piper_feedback',
             parameters=[{'can_port': LaunchConfiguration('piper_can'), 'gripper_exist': True}],
             remappings=[('joint_states', '/piper/joint_states_feedback')],
             condition=IfCondition(LaunchConfiguration('arm')), output='screen'),
        ExecuteProcess(cmd=[sys.executable, str(HERE / 'piper_joint_bridge.py')],
                       condition=IfCondition(LaunchConfiguration('arm')), output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher', name='physical_arm_state_publisher',
             parameters=[{'robot_description': ET.tostring(robot, encoding='unicode'), 'use_sim_time': False}],
             remappings=[('joint_states', '/robot/joint_states')],
             condition=IfCondition(LaunchConfiguration('arm')), output='screen'),
    ]
    for namespace, camera_name, serial, fps in [('head', 'camera', '244622071116', 15),
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
