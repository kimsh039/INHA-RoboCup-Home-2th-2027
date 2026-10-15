from pathlib import Path
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def start(context):
    config=LaunchConfiguration('config').perform(context)
    tuning=LaunchConfiguration('tuning_file').perform(context)
    hardware=LaunchConfiguration('hardware').perform(context).lower()=='true'
    automatic=LaunchConfiguration('auto_start_on_approach').perform(context).lower()=='true'
    nodes=[Node(package='robocup_real_manipulation',executable='coordinator',parameters=[{'config':config,'tuning_file':tuning,'hardware_mode':hardware,'auto_start_on_approach':automatic}],output='screen')]
    if not hardware: return nodes
    from robocup_real_manipulation.core import validate
    from robocup_real_manipulation.description import prepare
    cfg=yaml.safe_load(Path(config).read_text())
    if tuning:
        from robocup_real_manipulation.tuning import overlay
        cfg=overlay(cfg,yaml.safe_load(Path(tuning).read_text()))
    validate(cfg,hardware=True)
    repo=Path(LaunchConfiguration('repo').perform(context)); out=Path(LaunchConfiguration('description_output').perform(context)).expanduser()
    urdf,srdf=prepare(repo,cfg,out)
    limits={n:{'has_velocity_limits':True,'max_velocity':cfg['max_joint_speed_rad_s'],'has_acceleration_limits':True,'max_acceleration':cfg['max_joint_acceleration_rad_s2']} for n in cfg['joint_names']}
    params={'robot_description':urdf,'robot_description_semantic':srdf,
            'robot_description_kinematics':{cfg['planning_group']:{'kinematics_solver':'kdl_kinematics_plugin/KDLKinematicsPlugin','kinematics_solver_timeout':0.1}},
            'robot_description_planning':{'joint_limits':limits},
            'planning_pipelines':['ompl'],'default_planning_pipeline':'ompl',
            'ompl':{'planning_plugin':'ompl_interface/OMPLPlanner','request_adapters':'default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints',
                    'start_state_max_bounds_error':float(cfg.get('startup_bounds_tolerance_rad',0.)),
                    cfg['planning_group']:{'planner_configs':['RRTConnectkConfigDefault']},
                    'planner_configs':{'RRTConnectkConfigDefault':{'type':'geometric::RRTConnect','range':0.0}}},
            'moveit_controller_manager':'moveit_simple_controller_manager/MoveItSimpleControllerManager',
            'moveit_simple_controller_manager':{'controller_names':['arm_controller'],'arm_controller':{'type':'FollowJointTrajectory','action_ns':'follow_joint_trajectory','default':True,'joints':cfg['joint_names']}},
            'trajectory_execution.allowed_execution_duration_scaling':3.0,'trajectory_execution.allowed_goal_duration_margin':3.0,
            'publish_robot_description':True,'publish_robot_description_semantic':True}
    nodes += [Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'robot_description':urdf}],condition=IfCondition(LaunchConfiguration('start_robot_state_publisher'))),
              Node(package='moveit_ros_move_group',executable='move_group',parameters=[params],output='screen'),
              Node(package='robocup_real_manipulation',executable='piper_bridge',parameters=[{'config':config,'tuning_file':tuning,'hardware_mode':hardware}],output='screen')]
    if LaunchConfiguration('start_driver').perform(context).lower()=='true':
        # Do not use upstream launch: it auto-enables and remaps command to /joint_states.
        nodes.append(Node(package='piper',executable='piper_single_ctrl',parameters=[{'can_port':cfg['can_port'],'auto_enable':False,'gripper_exist':True,'gripper_val_mutiple':1}],
                          remappings=[('joint_ctrl_single',cfg['command_topic']),('joint_states_feedback',cfg['feedback_topic']),('gripper_states_feedback',cfg['gripper_feedback_topic']),('arm_status',cfg['arm_status_topic']),('enable_srv',cfg['enable_service'])],output='screen'))
    return nodes

def generate_launch_description():
    share=get_package_share_directory('robocup_real_manipulation')
    return LaunchDescription([DeclareLaunchArgument('config',default_value=share+'/config/robot.yaml'),
        DeclareLaunchArgument('tuning_file',default_value=''),
        DeclareLaunchArgument('start_robot_state_publisher',default_value='true'),
        DeclareLaunchArgument('hardware',default_value='false'),DeclareLaunchArgument('start_driver',default_value='false'),
        DeclareLaunchArgument('auto_start_on_approach',default_value='false'),
        DeclareLaunchArgument('repo',default_value=''),DeclareLaunchArgument('description_output',default_value='~/.ros/robocup_real_manipulation/model'),OpaqueFunction(function=start)])
