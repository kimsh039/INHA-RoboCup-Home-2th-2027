"""External navigation owns approach/lease. No synthetic approach or bench client."""
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def check_inputs(context):
    def enabled(name):
        return LaunchConfiguration(name).perform(context).lower() in ('true', '1')
    if enabled('start_control_center_adapter') and enabled('start_head_adapter'):
        raise RuntimeError('Choose one target source: control-center OR Head/fusion adapter')
    return []

def generate_launch_description():
    share=get_package_share_directory('robocup_real_manipulation')
    config=LaunchConfiguration('config'); tuning=LaunchConfiguration('tuning_file'); repo=LaunchConfiguration('repo')
    args=[DeclareLaunchArgument('config',default_value=share+'/config/robot.yaml'),
          DeclareLaunchArgument('tuning_file',default_value=''),DeclareLaunchArgument('repo',default_value=''),
          DeclareLaunchArgument('hardware',default_value='false'),
          DeclareLaunchArgument('start_robot_state_publisher',default_value='false'),
          DeclareLaunchArgument('auto_start_on_approach',default_value='false'),
          DeclareLaunchArgument('start_head_adapter',default_value='false'),
          DeclareLaunchArgument('start_control_center_adapter',default_value='false'),
          DeclareLaunchArgument('start_wrist_frontend',default_value='false'),
          DeclareLaunchArgument('vision_python',default_value=sys.executable),
          DeclareLaunchArgument('vision_config',default_value=share+'/config/wrist.yaml')]
    common={'config':config,'tuning_file':tuning}
    return LaunchDescription(args+[
        OpaqueFunction(function=check_inputs),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(share+'/launch/bringup.launch.py'),
            launch_arguments={'config':config,'tuning_file':tuning,'repo':repo,
                'hardware':LaunchConfiguration('hardware'),'start_driver':'false',
                'start_robot_state_publisher':LaunchConfiguration('start_robot_state_publisher'),
                'auto_start_on_approach':LaunchConfiguration('auto_start_on_approach')}.items()),
        Node(package='robocup_real_manipulation',executable='head_input_adapter',
             parameters=[common,{'point_source':'fusion','freeze_to_fixed_frame':True}],
             condition=IfCondition(LaunchConfiguration('start_head_adapter')),output='screen'),
        Node(package='robocup_real_manipulation',executable='control_center_input',
             condition=IfCondition(LaunchConfiguration('start_control_center_adapter')),output='screen'),
        Node(package='robocup_real_manipulation',executable='wrist_frontend',prefix=LaunchConfiguration('vision_python'),
             parameters=[common,{'repo':repo,'vision_config':LaunchConfiguration('vision_config')}],
             condition=IfCondition(LaunchConfiguration('start_wrist_frontend')),output='screen')])
