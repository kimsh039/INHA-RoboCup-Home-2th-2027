"""MoveIt plans a top-down observation; MuJoCo executes without teleporting."""
import json,time
import numpy as np
import rclpy
from moveit_msgs.msg import Constraints,JointConstraint
from run_grasp_moveit import Demo
from manipulation.mid360_cloud import validate_scene_metadata,validate_scene_status
from manipulation.sim_model import ROOT,JOINTS,load_model,body_transform
from scripts.generate_mid360_cloud import apply_snapshot


def main():
    meta=json.loads((ROOT/'data/pointclouds/cube_wrist_camera/metadata.json').read_text())
    validate_scene_metadata(meta)
    rclpy.init();node=Demo()
    try:
        end=time.monotonic()+15
        while node.status is None or node.joints is None:
            node.tick(.1)
            if time.monotonic()>end:raise RuntimeError('START_MUJOCO_BRIDGE_FIRST')
        validate_scene_status(meta,node.status)
        if node.status['base_state']!='STOPPED':raise RuntimeError('STOP_BASE_BEFORE_OBSERVATION')
        if not node.scene.wait_for_service(timeout_sec=10) or not node.move.wait_for_server(timeout_sec=10):
            raise RuntimeError('START_MOVEIT_FIRST')
        node.world_scene();node.stage('PLAN_WRIST_OBSERVATION')
        c=Constraints();c.name='wrist_top_down_observation'
        for name,value in zip(JOINTS,meta['observation_plan']['joint_positions_rad']):
            c.joint_constraints.append(JointConstraint(joint_name=name,position=float(value),
                tolerance_above=.003,tolerance_below=.003,weight=1.))
        # OMPL이 찾은 경로도 최종 검증에서 충돌하면 실행하지 않고 새 경로를 찾는다.
        for attempt in range(3):
            try:
                trajectory=node.move_goal(c,True).planned_trajectory
                break
            except RuntimeError as error:
                if str(error)!='MOVEIT_FAILED: -2' or attempt==2:raise
                print(f'관측 경로 최종 검증 실패: 새 경로 탐색 {attempt+2}/3',flush=True)
        node.stage('MOVE_TO_WRIST_OBSERVATION');node.execute_trajectory(trajectory);node.tick(1.)
        # Reconstruct measured FK in scratch data, never overwrite live joints.
        m,d=load_model();apply_snapshot(m,d,node.status)
        actual=body_transform(m,d,'wrist_camera_optical_frame');target=np.array(meta['T_world_camera'])
        if np.linalg.norm(actual[:3,3]-target[:3,3])>.005 or np.linalg.norm(actual[:3,2]-[0,0,-1])>.02:
            raise RuntimeError('WRIST_OBSERVATION_NOT_REACHED')
        node.stage('WRIST_OBSERVATION_READY')
        print('Observation reached. Now generate_wrist_camera_cloud.py --live; upload the resulting new ZIP.',flush=True)
    finally:
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
