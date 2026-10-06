"""Launch a chosen calibration URDF, optional measured Gazebo joints and RViz on ROS 2 (Ubuntu or native Mac)."""
from pathlib import Path
import sys
import os
import xml.etree.ElementTree as ET
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,ExecuteProcess,OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE=Path(__file__).resolve().parent
def setup(context):
    path=Path(LaunchConfiguration('urdf').perform(context)).resolve();root=ET.parse(path).getroot()
    for mesh in root.findall('.//mesh'):
        name=mesh.get('filename','')
        if name and not name.startswith(('package:','file:','http:','https:')):mesh.set('filename',(path.parent/name).resolve().as_uri())
    actions=[Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'robot_description':ET.tostring(root,encoding='unicode'),'use_sim_time':False}],output='screen')]
    if LaunchConfiguration('joints').perform(context).lower()=='true':actions.append(ExecuteProcess(cmd=[sys.executable,str(HERE/'publish_measured_joints.py')],additional_env={'GZ_IP':os.environ.get('GZ_IP','127.0.0.1'),'GZ_PARTITION':os.environ.get('GZ_PARTITION','robocup_motion')},output='screen'))
    if LaunchConfiguration('rviz').perform(context).lower()=='true':actions.append(Node(package='rviz2',executable='rviz2',arguments=['-d',str(HERE/'tf_calibrated.rviz')],parameters=[{'use_sim_time':False}],output='screen'))
    return actions
def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('urdf',default_value=str(HERE.parent/'robot_description/robocup.calibrated.urdf')),DeclareLaunchArgument('joints',default_value='true'),DeclareLaunchArgument('rviz',default_value='true'),OpaqueFunction(function=setup)])
