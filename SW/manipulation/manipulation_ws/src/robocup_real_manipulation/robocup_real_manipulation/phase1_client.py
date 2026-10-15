"""Already-docked, physically isolated base trial; wrist inputs always real."""
import argparse
import copy
import time
import uuid
from pathlib import Path
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from moveit_msgs.srv import ApplyPlanningScene
from robocup_real_manipulation_msgs.action import Manipulate
from robocup_real_manipulation_msgs.msg import Target, ApproachResult, BaseLease
from .core import fresh, rule_for
from .ros_util import seconds


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('class_name', choices=['green_apple','peach','tomato','fanta_can','cup','banana'])
    p.add_argument('--config', type=Path, required=True)
    p.add_argument('--already-docked', action='store_true')
    p.add_argument('--base-drive-isolated', action='store_true')
    p.add_argument('--input-topic', default='/manipulation/coarse_target')
    p.add_argument('--input-durable', action='store_true', help='Use for the supplied Head/fusion adapter (retains six targets)')
    p.add_argument('--timeout', type=float, default=300.)
    args=p.parse_args()
    if not args.already_docked or not args.base_drive_isolated:
        p.error('Confirm actual close approach and PHYSICALLY isolated base drive with both flags. No mobile-base mode.')
    cfg=yaml.safe_load(args.config.expanduser().read_text())
    rule=cfg['classes'].get(args.class_name)
    if not rule or not rule['enabled']: p.error('Disabled/unknown grasp class')
    class_id=rule['class_id']; rule_for(cfg,class_id)
    # Keep the ROS context alive long enough to send cancel and receive stop/result.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)

    class Trial(Node):
        def __init__(self):
            super().__init__('already_docked_pick_place')
            self.task=uuid.uuid4().hex; self.target=None; self.lock=False
            self.goal=None; self.sent=None; self.done=False; self.success=False
            self.wrist_ready=0.
            self.arm_feedback_stamp=0.
            self.create_subscription(JointState,'/joint_states',
                lambda m:setattr(self,'arm_feedback_stamp',seconds(m.header.stamp)),
                QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT))
            qos=QoSProfile(depth=6,durability=(DurabilityPolicy.TRANSIENT_LOCAL if args.input_durable else DurabilityPolicy.VOLATILE),
                           reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(Target,args.input_topic,self.input,qos)
            self.target_pub=self.create_publisher(Target,'/manipulation/target',10)
            self.approach_pub=self.create_publisher(ApproachResult,'/manipulation/approach_result',10)
            self.lease=self.create_publisher(BaseLease,cfg['base_lease_topic'],10)
            self.create_subscription(BaseLease,cfg['base_lock_topic'],self.lock_request,
                QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.create_subscription(String,'/manipulation/wrist/status',self.wrist_status,10)
            self.client=ActionClient(self,Manipulate,'/manipulation/run')
            self.scene_client=self.create_client(ApplyPlanningScene,'/apply_planning_scene')
            self.create_timer(.05,self.tick)

        def input(self,msg):
            if self.sent is not None or msg.class_id!=class_id: return
            try:
                fresh(seconds(msg.header.stamp),self.get_clock().now().nanoseconds/1e9,cfg['max_target_age_s'])
                rule_for(cfg,msg.class_id)
                self.target=copy.deepcopy(msg)
            except Exception as exc: self.get_logger().warning(str(exc))

        def wrist_status(self,msg):
            if msg.data=='READY': self.wrist_ready=time.monotonic()

        def lock_request(self,msg):
            if msg.task_id==self.task: self.lock=msg.locked

        def tick(self):
            if self.lock:
                msg=BaseLease(task_id=self.task,locked=True,stopped=True)
                msg.header.stamp=self.get_clock().now().to_msg(); self.lease.publish(msg)
            if self.sent is not None or self.target is None: return
            try:
                fresh(self.arm_feedback_stamp,self.get_clock().now().nanoseconds/1e9,cfg['feedback_age_s'])
            except Exception: return
            if not self.scene_client.service_is_ready(): return
            if not self.client.server_is_ready() or time.monotonic()-self.wrist_ready>.5: return
            if not self.target_pub.get_subscription_count() or not self.approach_pub.get_subscription_count(): return
            try:
                fresh(seconds(self.target.header.stamp),self.get_clock().now().nanoseconds/1e9,cfg['max_target_age_s'])
            except Exception: self.target=None; return
            msg=copy.deepcopy(self.target); msg.task_id=self.task
            self.target_pub.publish(msg)
            result=ApproachResult(task_id=self.task,outcome='SUCCESS',base_stopped=True,
                reason='OPERATOR_CONFIRMED_ALREADY_DOCKED_PHYSICAL_BASE_ISOLATION')
            result.header.stamp=self.get_clock().now().to_msg(); self.approach_pub.publish(result)
            self.get_logger().info('Using REAL coarse input stamp/frame; task_id='+self.task)
            self.sent=self.client.send_goal_async(Manipulate.Goal(task_id=self.task),feedback_callback=self.feedback)
            self.sent.add_done_callback(self.accepted)

        def feedback(self,msg):
            s=msg.feedback.status; self.get_logger().info(s.stage+' '+s.outcome+' '+s.reason)

        def accepted(self,f):
            self.goal=f.result()
            if not self.goal.accepted:
                self.get_logger().error('Manipulation rejected: inspect config/recovery journal')
                self.done=True; return
            self.goal.get_result_async().add_done_callback(self.finished)

        def finished(self,f):
            s=f.result().result.status
            self.success=s.outcome=='SUCCESS' and s.place_success and s.return_success
            self.get_logger().info(f'FINAL {s.outcome}: {s.reason}; place={s.place_success}, return={s.return_success}, holding={s.holding}, stop={s.stop_confirmed}')
            self.done=True

    node=Trial(); deadline=time.monotonic()+args.timeout
    try:
        while not node.done and time.monotonic()<deadline: rclpy.spin_once(node,timeout_sec=.1)
    except KeyboardInterrupt:
        print('Cancel requested; keep base isolated and inspect real arm holding/stop state.')
    finally:
        # Resolve a late acceptance too; do not abandon an accepted physical goal.
        if node.sent is not None and not node.done:
            limit=time.monotonic()+5.
            while not node.sent.done() and time.monotonic()<limit: rclpy.spin_once(node,timeout_sec=.1)
            if node.sent.done():
                h=node.sent.result()
                if h and h.accepted:
                    h.cancel_goal_async()
                    while not node.done and time.monotonic()<limit: rclpy.spin_once(node,timeout_sec=.1)
            else: node.get_logger().error('Acceptance/stop unconfirmed; manual stop and recovery required')
        success=node.success
        node.destroy_node(); rclpy.shutdown()
    raise SystemExit(0 if success else 1)
