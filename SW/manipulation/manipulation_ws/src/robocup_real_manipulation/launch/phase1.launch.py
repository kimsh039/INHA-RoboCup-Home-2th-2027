"""Already-docked real wrist observation + pick/place. No Nav or fake sensors."""
from pathlib import Path
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def check_config(context):
    import yaml
    from robocup_real_manipulation.core import require
    cfg=yaml.safe_load(Path(LaunchConfiguration('config').perform(context)).expanduser().read_text())
    require(cfg.get('external_sensor_tf') is True,'PHASE1_REQUIRES_EXTERNAL_SENSOR_TF')
    serial=LaunchConfiguration('wrist_serial').perform(context).lstrip('_')
    require(serial==str(cfg['wrist_camera_serial']),'CAMERA_SERIAL_CONFIG_MISMATCH')
    return []


def generate_launch_description():
    share = get_package_share_directory('robocup_real_manipulation')
    cfg = LaunchConfiguration('config'); repo = LaunchConfiguration('repo')
    tuning = LaunchConfiguration('tuning_file')
    wrapper = get_package_share_directory('realsense2_camera')
    common = {'config': cfg, 'tuning_file': tuning}
    return LaunchDescription([
        DeclareLaunchArgument('repo'), DeclareLaunchArgument('config'),
        DeclareLaunchArgument('vision_config', default_value=share+'/config/wrist.yaml'),
        DeclareLaunchArgument('tuning_file', default_value=share+'/config/phase1.tuning.yaml'),
        DeclareLaunchArgument('vision_python', default_value=sys.executable),
        DeclareLaunchArgument('start_driver', default_value='false'),
        DeclareLaunchArgument('start_camera', default_value='false'),
        DeclareLaunchArgument('start_mount_tf', default_value='false'),
        DeclareLaunchArgument('start_head_adapter', default_value='true'),
        DeclareLaunchArgument('head_topic', default_value='/detection/head/target'),
        DeclareLaunchArgument('point_topic', default_value='/detection/target_map_point'),
        DeclareLaunchArgument('point_source', default_value='fusion'),
        DeclareLaunchArgument('wrist_serial', default_value='_243222070076'),
        OpaqueFunction(function=check_config),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(share+'/launch/bringup.launch.py'),
            launch_arguments={'config':cfg, 'repo':repo, 'hardware':'true', 'tuning_file':tuning,
                              'start_driver':LaunchConfiguration('start_driver'),
                              'auto_start_on_approach':'false'}.items()),
        Node(package='robocup_real_manipulation', executable='wrist_frontend',
             prefix=LaunchConfiguration('vision_python'), output='screen',
             parameters=[{**common,'repo':repo,'vision_config':LaunchConfiguration('vision_config')}]),
        Node(package='robocup_real_manipulation', executable='head_input_adapter', output='screen',
             condition=IfCondition(LaunchConfiguration('start_head_adapter')),
             parameters=[{**common,'head_topic':LaunchConfiguration('head_topic'),
                          'point_topic':LaunchConfiguration('point_topic'),
                          'point_source':LaunchConfiguration('point_source'),
                          'freeze_to_fixed_frame':True,
                          'output_topic':'/manipulation/coarse_target'}]),
        ExecuteProcess(cmd=[sys.executable, PathJoinSubstitution([repo,'SW/robot/sensor_tf.py'])],
                       condition=IfCondition(LaunchConfiguration('start_mount_tf')),output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(wrapper)/'launch/rs_launch.py')),
            condition=IfCondition(LaunchConfiguration('start_camera')),
            launch_arguments={'camera_namespace':'wrist','camera_name':'wrist',
                'serial_no':LaunchConfiguration('wrist_serial'), 'publish_tf':'true',
                'rgb_camera.color_profile':'640,480,30','depth_module.depth_profile':'640,480,30',
                'enable_color':'true','enable_depth':'true','align_depth.enable':'true',
                'enable_gyro':'false','enable_accel':'false'}.items()),
    ])
