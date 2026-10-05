"""Launch MoveIt with the model exported from the actual compiled MuJoCo robot."""
from pathlib import Path
import json
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    root=Path(__file__).resolve().parents[1]
    generated=root/"simulation/generated"
    if not all((generated/name).exists() for name in ("piper.urdf","piper.srdf")):
        raise RuntimeError("MoveIt model files are missing. From simulation/mujoco, run: source /opt/ros/humble/setup.bash && .ros_venv/bin/python run_mujoco_moveit_bridge.py --prepare-only. Then start the MuJoCo bridge and retry this launch.")
    cfg=json.loads((root/"config/grasp_simulation.json").read_text())
    description={"robot_description":(generated/"piper.urdf").read_text()}
    semantic={"robot_description_semantic":(generated/"piper.srdf").read_text()}
    kinematics={"robot_description_kinematics":{"arm":{"kinematics_solver":"kdl_kinematics_plugin/KDLKinematicsPlugin",
                 "kinematics_solver_search_resolution":0.005,"kinematics_solver_timeout":0.2}}}
    limits={"robot_description_planning":{"joint_limits":{f"joint{i}":{"has_velocity_limits":True,
             "max_velocity":cfg["joint_velocity_limit_rad_s"],"has_acceleration_limits":True,
             "max_acceleration":cfg["joint_acceleration_limit_rad_s2"]} for i in range(1,7)}}}
    ompl={"planning_pipelines":["ompl"],"default_planning_pipeline":"ompl",
          "ompl":{"planning_plugin":"ompl_interface/OMPLPlanner",
                  "request_adapters":"default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints",
                  "start_state_max_bounds_error":0.1,
                  "planner_configs":{"RRTConnectkConfigDefault":{"type":"geometric::RRTConnect","range":0.0}},
                  # 프레임 근처에서 성긴 관절 보간이 충돌 구간을 건너뛰지 않도록 세분한다.
                  "arm":{"planner_configs":["RRTConnectkConfigDefault"],"longest_valid_segment_fraction":0.001}}}
    controllers={"moveit_controller_manager":"moveit_simple_controller_manager/MoveItSimpleControllerManager",
                 "moveit_simple_controller_manager":{"controller_names":["arm_controller"],
                     "arm_controller":{"type":"FollowJointTrajectory","action_ns":"follow_joint_trajectory",
                                       "default":True,"joints":[f"joint{i}" for i in range(1,7)]}}}
    execution={"allow_trajectory_execution":True,"moveit_manage_controllers":False,
               # MuJoCo가 실시간보다 느려도 정상 궤적을 성급하게 취소하지 않는다.
               # 실행 시간 제한과 브리지의 충돌/추종 오차 검사는 유지한다.
               "trajectory_execution.allowed_execution_duration_scaling":10.0,
               "trajectory_execution.allowed_goal_duration_margin":3.0,
               "trajectory_execution.allowed_start_tolerance":0.03,
               "publish_robot_description":True,"publish_robot_description_semantic":True,
               "publish_planning_scene":True,"publish_geometry_updates":True,
               "publish_state_updates":True,"publish_transforms_updates":True}
    return LaunchDescription([
        Node(package="robot_state_publisher",executable="robot_state_publisher",parameters=[description]),
        Node(package="moveit_ros_move_group",executable="move_group",output="screen",
             parameters=[description,semantic,kinematics,limits,ompl,controllers,execution])])
