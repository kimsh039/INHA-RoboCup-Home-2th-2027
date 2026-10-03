from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package = Path(get_package_share_directory('sensor_rack_description'))
    description = (package / 'urdf' / 'sensor_rack.urdf').read_text()
    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description}]),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', str(package / 'rviz' / 'display.rviz')]),
    ])
