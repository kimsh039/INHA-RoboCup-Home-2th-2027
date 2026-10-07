"""단일 Gazebo robocup 모델 + Nav2와 호환되는 topic + MoveIt + 팔 제어."""
from pathlib import Path
import json
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,ExecuteProcess,OpaqueFunction,SetEnvironmentVariable,RegisterEventHandler,EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    from robocup_gazebo_manipulation.generate import generate,ARM
    repository=Path(LaunchConfiguration('repository').perform(context))
    generated=Path(LaunchConfiguration('generated').perform(context))
    gui=LaunchConfiguration('gui').perform(context)=='true'
    start=LaunchConfiguration('start_sim').perform(context)=='true'
    mid360=LaunchConfiguration('mid360').perform(context)=='true'
    generate(repository,generated,LaunchConfiguration('room').perform(context)=='true',mid360=mid360,head_camera=LaunchConfiguration('head_camera').perform(context)=='true',g2=LaunchConfiguration('g2').perform(context)=='true')
    common={'use_sim_time':True};description={'robot_description':(generated/'robot.urdf').read_text()}
    semantic={'robot_description_semantic':(generated/'robot.srdf').read_text()}
    kin={'robot_description_kinematics':{'arm':{'kinematics_solver':'kdl_kinematics_plugin/KDLKinematicsPlugin',
        'kinematics_solver_timeout':.2,'kinematics_solver_search_resolution':.005}}}
    limits={'robot_description_planning':{'joint_limits':{n:{'has_velocity_limits':True,'max_velocity':.5,
        'has_acceleration_limits':True,'max_acceleration':.5} for n in ARM}}}
    ompl={'planning_pipelines':['ompl'],'default_planning_pipeline':'ompl','ompl':{
        'planning_plugin':'ompl_interface/OMPLPlanner',
        'request_adapters':'default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints',
        'planner_configs':{'RRTConnectkConfigDefault':{'type':'geometric::RRTConnect'}},
        'arm':{'planner_configs':['RRTConnectkConfigDefault'],'longest_valid_segment_fraction':.001}}}
    controllers={'moveit_controller_manager':'moveit_simple_controller_manager/MoveItSimpleControllerManager',
        'moveit_simple_controller_manager':{'controller_names':['arm_controller'],
        'arm_controller':{'type':'FollowJointTrajectory','action_ns':'follow_joint_trajectory','default':True,'joints':ARM}}}
    execution={'allow_trajectory_execution':True,'moveit_manage_controllers':False,
        'trajectory_execution.allowed_execution_duration_scaling':5.,'trajectory_execution.allowed_goal_duration_margin':10.,
        'trajectory_execution.allowed_start_tolerance':.03,'publish_robot_description':True,'publish_robot_description_semantic':True,
        'publish_planning_scene':True,'publish_geometry_updates':True,'publish_state_updates':True,'publish_transforms_updates':True}
    actions=[]
    partition=LaunchConfiguration('partition').perform(context)
    render_env={k:os.environ[k] for k in ('__NV_PRIME_RENDER_OFFLOAD','__GLX_VENDOR_LIBRARY_NAME') if k in os.environ}
    render_env.update({'GZ_PARTITION':partition,'IGN_PARTITION':partition,'GZ_IP':'127.0.0.1'})
    (generated.parent/'gazebo_partition').write_text(partition+'\n')
    if start:
        # GUI and server are started by the same gz invocation with the same world.
        # Retain the separate server-only mode for gui:=false.
        cmd=['gz','sim','-r',str(generated/'world.sdf')] if gui else ['gz','sim','-s','-r','--headless-rendering',str(generated/'world.sdf')]
        server=ExecuteProcess(cmd=cmd,output='screen',additional_env=render_env)
        actions.extend([server,RegisterEventHandler(OnProcessExit(target_action=server,on_exit=[EmitEvent(event=Shutdown())]))])
        actions.extend([
            Node(package='ros_gz_bridge',executable='parameter_bridge',parameters=[common,{'config_file':str(generated/'bridge.yaml')}],output='screen'),
            Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[common,description],output='screen'),
            ])
        if mid360:
            actions.append(ExecuteProcess(cmd=['python3',str(repository/'simulation/ros2/cloud_self_filter.py'),'--ros-args','-p','use_sim_time:=true'],output='screen'))
    else:
        # 기존 내비 sim의 clock/odom/tf/joint_states/RSP/센서는 유지. 팔 명령 bridge만 추가한다.
        import yaml
        bridge=yaml.safe_load((generated/'bridge.yaml').read_text())
        arm_bridge=[b for b in bridge if b['ros_topic_name'].startswith(('/gazebo/manipulation/','/gazebo/contacts/')) or b['ros_topic_name']=='/gazebo/dynamic_poses']
        (generated/'attach_bridge.yaml').write_text(yaml.safe_dump(arm_bridge,sort_keys=False))
        actions.append(Node(package='ros_gz_bridge',executable='parameter_bridge',parameters=[common,{'config_file':str(generated/'attach_bridge.yaml')}],output='screen'))
    actions.extend([
        Node(package='robocup_gazebo_manipulation',executable='controller',parameters=[common,{'manifest':str(generated/'manifest.json'),'allow_base_drift_experiment':LaunchConfiguration('slip_experiment').perform(context)=='true'}],output='screen'),
        Node(package='moveit_ros_move_group',executable='move_group',parameters=[common,description,semantic,kin,limits,ompl,controllers,execution],output='screen')])
    if LaunchConfiguration('task_server').perform(context)=='true':
        actions.append(Node(package='robocup_gazebo_manipulation',executable='task',parameters=[common,{'repository':str(repository),'generated':str(generated),'task_config':LaunchConfiguration('task_config').perform(context)}],output='screen'))
    if LaunchConfiguration('rviz').perform(context)=='true':
        actions.append(Node(package='rviz2',executable='rviz2',parameters=[common,description,semantic,kin],output='log'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('slip_experiment',default_value='true',choices=['true','false']),DeclareLaunchArgument('repository'),DeclareLaunchArgument('generated'),DeclareLaunchArgument('task_config',default_value=''),
        DeclareLaunchArgument('gui',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('rviz',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('head_camera',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('g2',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('task_server',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('mid360',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('room',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('start_sim',default_value='true',choices=['true','false']),
        DeclareLaunchArgument('partition',default_value='robocup_manipulation'),
        SetEnvironmentVariable('GZ_PARTITION',LaunchConfiguration('partition')),
        SetEnvironmentVariable('GZ_IP','127.0.0.1'),OpaqueFunction(function=setup)])
