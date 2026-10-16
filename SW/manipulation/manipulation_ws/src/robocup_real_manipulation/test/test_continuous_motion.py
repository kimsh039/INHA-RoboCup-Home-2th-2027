"""Offline regression cases: no ROS initialization, driver or motor connection."""
import copy
import unittest
import tempfile
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PolygonStamped, Point32
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from robocup_real_manipulation.core import Fault, rotation, support
from robocup_real_manipulation.trajectory import Spline, from_message, retime, set_time
from robocup_real_manipulation.table_scene import measured_table
from robocup_real_manipulation.grasp_yaw import candidates
from robocup_real_manipulation.motion import Motion
from robocup_real_manipulation.node import Coordinator
from test_core import CFG


class ContinuousMotionTests(unittest.TestCase):
    def setUp(self):
        self.cfg=copy.deepcopy(CFG); self.cfg['joint_limits']=[[-2.,2.]]*6

    def trajectory(self):
        traj=JointTrajectory(joint_names=self.cfg['joint_names'])
        for t,x in [(0.,0.),(.1,.01),(.2,.02)]:
            p=JointTrajectoryPoint(positions=[x]*6); set_time(p,t); traj.points.append(p)
        return traj

    def test_internal_knots_continue_and_endpoints_stop(self):
        spline=Spline([0,1,2],[[0]*6,[.01]*6,[.02]*6])
        self.assertGreater(spline.evaluate(1,1)[0],.009)
        np.testing.assert_allclose(spline.evaluate(1-1e-8,1),spline.evaluate(1+1e-8,1),atol=1e-8)
        np.testing.assert_allclose(spline.evaluate(0,1),0,atol=1e-12)
        np.testing.assert_allclose(spline.evaluate(2,1),0,atol=1e-12)

    def test_supplied_velocity_and_acceleration_preserved_at_knots(self):
        v=np.array([[0]*6,[.01]*6,[0]*6]); a=np.array([[0]*6,[.002]*6,[0]*6])
        spline=Spline([0,1,2],[[0]*6,[.01]*6,[.02]*6],v,a)
        for i in range(3):
            np.testing.assert_allclose(spline.evaluate(i,1),v[i],atol=1e-12)
            np.testing.assert_allclose(spline.evaluate(i,2),a[i],atol=1e-12)

    def test_retime_scales_whole_path_and_enforces_actual_extrema(self):
        traj=self.trajectory(); original=from_message(traj,self.cfg,np.zeros(6),False)
        spline=retime(traj,self.cfg,np.zeros(6))
        self.assertGreater(spline.times[-1],original.times[-1])
        for u in np.linspace(0,1,21):
            np.testing.assert_allclose(spline.evaluate(u*spline.times[-1]),original.evaluate(u*original.times[-1]),atol=1e-8)
        self.assertLessEqual(spline.extrema(1),self.cfg['max_joint_speed_rad_s'])
        self.assertLessEqual(spline.extrema(2),self.cfg['max_joint_acceleration_rad_s2'])

    def test_position_overshoot_between_safe_knots_is_rejected(self):
        cfg=dict(self.cfg,joint_limits=[[-.02,.02]]*6)
        spline=Spline([0,1,2],[[0]*6]*3,[[0]*6,[.2]*6,[0]*6])
        with self.assertRaisesRegex(Fault,'TRAJECTORY_LIMITS'):
            spline.validate(cfg,np.zeros(6))

    def test_missing_or_nonfinite_derivatives_rejected(self):
        traj=self.trajectory(); traj.points[1].velocities=[.01]*6
        with self.assertRaisesRegex(Fault,'VELOCITIES'): from_message(traj,self.cfg,np.zeros(6))
        for p in traj.points: p.velocities=[float('nan')]*6
        with self.assertRaisesRegex(Fault,'VELOCITIES'): from_message(traj,self.cfg,np.zeros(6))

    def test_collision_sampling_bounds_step_and_includes_extrema(self):
        spline=Spline([0,1,2],[[0]*6,[.1]*6,[0]*6])
        samples=spline.positions(step=.01)
        self.assertLessEqual(np.max(np.abs(np.diff(samples,axis=0))),.01)
        self.assertAlmostEqual(np.max(samples),.1)

    def test_measured_droop_prefix_stays_inward(self):
        cfg=dict(self.cfg,joint_limits=[[-2,2],[0,2],[-2,0],[-2,2],[-2,2],[-2,2]],
                 startup_bounds_tolerance_rad=.04,observation_joints=[0,.1,-.1,0,0,0])
        current=np.array([0,-.02,.02,0,0,0])
        spline=Spline([0,2,4],[current,np.zeros(6),cfg['observation_joints']])
        spline.validate(cfg,current)
        self.assertTrue(np.all(np.diff(spline.positions(step=.002)[:,1])>=-1e-9))


class TableAndYawTests(unittest.TestCase):
    def setUp(self):
        self.cfg=dict(CFG,table_thickness_m=.12,obstacles=[])
        self.diamond=[[0,-1,.7],[1,0,.7],[0,1,.7],[-1,0,.7]]

    def test_rotated_support_rejects_inside_aabb_but_outside_table(self):
        cfg=dict(self.cfg,**measured_table(self.diamond,self.cfg))
        self.assertTrue(support([0,0,.74],.04,cfg))
        self.assertFalse(support([.8,.8,.74],.04,cfg))
        self.assertFalse(support([.95,0,.74],.04,cfg))
        reversed_cfg=measured_table(self.diamond[::-1],self.cfg)
        self.assertTrue(support([0,0,.74],.04,dict(self.cfg,**reversed_cfg)))

    def test_bad_table_geometry_is_rejected(self):
        for points in ([self.diamond[i] for i in [0,2,1,3]],
                       self.diamond[:3],[[0,-1,.7],[1,0,.8],[0,1,.7],[-1,0,.7]]):
            with self.assertRaises(Fault): measured_table(points,self.cfg)

    def test_scene_mesh_preserves_measured_corners_and_thickness(self):
        motion=Motion.__new__(Motion); motion.placed=[]; motion.wrist_ids=set(); motion.c=dict(self.cfg,**measured_table(self.diamond,self.cfg))
        scenes=[]; motion.apply=lambda scene,guard:scenes.append(scene)
        motion.setup_scene(None,None,lambda:None)
        mesh=scenes[0].world.collision_objects[0].meshes[0]
        self.assertEqual(len(mesh.triangles),12)
        np.testing.assert_allclose([[p.x,p.y,p.z] for p in mesh.vertices[:4]],self.diamond)
        np.testing.assert_allclose([p.z for p in mesh.vertices[4:]],.58)

    def test_yaw_sampling_keeps_grip_fixed_and_banana_axis(self):
        cfg=dict(self.cfg,tcp_to_grip_center_m=[.02,.01,.03])
        q=np.array([1.,0,0,0]); tcp=np.array([.4,0,.8])
        grip=tcp+rotation(q)@np.array(cfg['tcp_to_grip_center_m'])
        rule={'yaw_sample_degrees':[0,30,60,90,120,150]}
        poses=list(candidates(grip,tcp,q,rule,cfg)); self.assertEqual(len(poses),6)
        for p,orientation in poses:
            np.testing.assert_allclose(p+rotation(orientation)@np.array(cfg['tcp_to_grip_center_m']),grip)
            np.testing.assert_allclose(rotation(orientation)[:,2],[0,0,-1],atol=1e-12)
        rule['axis_required']=True
        self.assertEqual(len(list(candidates(grip,tcp,q,rule,cfg))),1)

    def test_surface_handoff_waits_for_matching_time_and_pins_in_gravity_frame(self):
        with tempfile.TemporaryDirectory() as directory:
            cfg=dict(self.cfg,table_source='dock_surface',coarse_fixed_frame='odom')
            msg=PolygonStamped(); msg.header.frame_id='odom'; msg.header.stamp=Time(sec=10)
            msg.polygon.points=[Point32(x=float(x),y=float(y),z=float(z)) for x,y,z in self.diamond]
            target=SimpleNamespace(task_id='task',header=SimpleNamespace(stamp=Time(sec=9)))
            approach=SimpleNamespace(header=SimpleNamespace(stamp=Time(sec=10)))
            node=SimpleNamespace(c=cfg,dock_surface=None,coarse=np.array([.3,0,.74]),
                rule={'radius_m':.04},now=lambda:10.2,log=Path(directory)/'events.jsonl',
                to_gravity=lambda p,header,stamp:np.asarray(p)+[.3,0,0])
            self.assertFalse(Coordinator.pin_table(node,target,approach))
            node.dock_surface=msg; msg.header.stamp=Time(sec=5)
            self.assertFalse(Coordinator.pin_table(node,target,approach))
            msg.header.stamp=Time(sec=10)
            self.assertTrue(Coordinator.pin_table(node,target,approach))
            np.testing.assert_allclose(np.mean(cfg['table_polygon_xy'],axis=0),[.3,0],atol=1e-8)
            snapshot=copy.deepcopy(cfg['table_polygon_xy'])
            msg.polygon.points[0].x=99.
            self.assertEqual(cfg['table_polygon_xy'],snapshot)
            msg.header.frame_id='base_link'
            with self.assertRaisesRegex(Fault,'DOCK_SURFACE_FRAME'):
                Coordinator.pin_table(node,target,approach)


if __name__=='__main__': unittest.main()
