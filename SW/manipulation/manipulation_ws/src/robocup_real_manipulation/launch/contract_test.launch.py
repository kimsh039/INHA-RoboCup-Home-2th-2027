from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('config',default_value=get_package_share_directory('robocup_real_manipulation')+'/config/test_fixture.yaml'),
      DeclareLaunchArgument('fail_stage',default_value=''),
      Node(package='robocup_real_manipulation',executable='mock_coordinator',parameters=[{'config':LaunchConfiguration('config'),'fail_stage':LaunchConfiguration('fail_stage')}],output='screen')])
