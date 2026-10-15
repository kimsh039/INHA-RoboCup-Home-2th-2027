#!/usr/bin/env python3
"""External, operator-controlled bench inputs; automatic package stays unchanged.

Publishes NO joint feedback, TF, arm commands, or motor enable requests.
The base lease is an operator attestation that base drive is physically isolated,
not sensor-derived navigation feedback. Never use it for a mobile-base trial.
"""
import argparse
import math
import uuid
import time


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--center', type=float, nargs=3, required=True,
                   help='Measured object geometric center in --frame, meters')
    p.add_argument('--coarse', type=float, nargs=3, help='Defaults to --center')
    p.add_argument('--frame', default='base_link')
    p.add_argument('--class-id', type=int, default=0)
    p.add_argument('--base-drive-isolated', action='store_true',
                   help='Operator has physically disabled base drive and confirmed stopped')
    p.add_argument('--run', action='store_true', help='Send physical Manipulate goal')
    p.add_argument('--timeout', type=float, default=600.)
    return p


def main():
    a = parser().parse_args()
    if not all(math.isfinite(x) for x in a.center+(a.coarse or a.center)):
        raise SystemExit('Coordinates must be finite meters')
    if not a.run:
        print('PREVIEW ONLY: no ROS traffic. center=',a.center,'coarse=',a.coarse or a.center,'frame=',a.frame)
        print('Add --run --base-drive-isolated only for a prepared stationary bench trial.')
        return
    if not a.base_drive_isolated:
        raise SystemExit('Physically isolate base drive before acknowledging --base-drive-isolated')
    if not .1 <= a.timeout <= 1800: raise SystemExit('Timeout outside 0.1..1800s')
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from robocup_real_manipulation_msgs.msg import Target, ApproachResult, BaseLease, WristObservation, ObservationRequest
    from robocup_real_manipulation_msgs.action import Manipulate
    rclpy.init()

    class Bench(Node):
        def __init__(self):
            super().__init__('operator_bench_inputs')
            self.task = 'bench-'+uuid.uuid4().hex
            self.lock = ''; self.pending = []; self.started = False; self.goal = None; self.done = False
            self.target = self.create_publisher(Target,'/manipulation/target',10)
            self.approach = self.create_publisher(ApproachResult,'/manipulation/approach_result',10)
            self.lease = self.create_publisher(BaseLease,'/manipulation/base_lease',10)
            self.wrist = self.create_publisher(WristObservation,'/manipulation/wrist/result',10)
            qos = QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(BaseLease,'/manipulation/base_lock_request',self.lock_request,qos)
            self.create_subscription(ObservationRequest,'/manipulation/wrist/request',self.observe,10)
            self.client = ActionClient(self,Manipulate,'/manipulation/run')
            self.create_timer(.1,self.tick)
            self.get_logger().warning('MANUAL BENCH: base drive must remain physically isolated. Wrist positions are synthetic inputs.')

        def stamp(self, msg):
            msg.header.stamp = self.get_clock().now().to_msg(); msg.header.frame_id = a.frame
            return msg

        def lock_request(self, msg):
            # Never acknowledge an unrelated/replayed task's retained lock.
            if msg.task_id == self.task: self.lock = msg.task_id if msg.locked else ''

        def observe(self, req):
            if req.task_id != self.task: return
            self.pending = [req]*3

        def tick(self):
            if self.lock:
                self.lease.publish(self.stamp(BaseLease(task_id=self.task,locked=True,stopped=True)))
            if self.pending:
                req = self.pending.pop(0)
                msg = self.stamp(WristObservation(task_id=self.task,observation_id=req.observation_id,
                    class_id=a.class_id,instance_id='bench-object',candidate_count=1,candidate_index=0,
                    confidence=1.,position_kind='CENTER',has_position=True,position_std_m=.002))
                # Synthetic acquisition timestamps, no claim of actual camera measurement.
                msg.position.x,msg.position.y,msg.position.z = a.center
                self.wrist.publish(msg)
            if not self.started and self.client.server_is_ready() and self.target.get_subscription_count() and self.approach.get_subscription_count():
                self.started = True
                msg = self.stamp(Target(task_id=self.task,class_id=a.class_id,instance_id='bench-object'))
                msg.coarse_position.x,msg.coarse_position.y,msg.coarse_position.z = a.coarse or a.center
                self.target.publish(msg)
                # Operator placed the robot at close approach; this does not claim Nav2 ran.
                self.approach.publish(self.stamp(ApproachResult(task_id=self.task,outcome='SUCCESS',base_stopped=True)))
                self.get_logger().info('task_id='+self.task)
                self.client.send_goal_async(Manipulate.Goal(task_id=self.task),feedback_callback=self.feedback).add_done_callback(self.accepted)

        def feedback(self, future):
            status = future.feedback.status
            self.get_logger().info('stage='+status.stage+' outcome='+status.outcome+' reason='+status.reason)

        def accepted(self, future):
            self.goal = future.result()
            if not self.goal.accepted:
                self.get_logger().error('GOAL REJECTED: inspect coordinator configuration/journal'); self.done = True; return
            self.goal.get_result_async().add_done_callback(self.finished)

        def finished(self, future):
            s = future.result().result.status
            self.get_logger().info(f'FINAL {s.outcome} stage={s.stage} holding={s.holding} place={s.place_success} return={s.return_success} stop={s.stop_confirmed} reason={s.reason}')
            self.done = True

    node = Bench(); deadline = time.monotonic()+a.timeout
    try:
        while not node.done and time.monotonic() < deadline: rclpy.spin_once(node,timeout_sec=.1)
    except KeyboardInterrupt:
        print('Cancel requested; physical emergency stop remains independent.')
    finally:
        if node.goal and node.goal.accepted and not node.done:
            node.goal.cancel_goal_async()
            end = time.monotonic()+5
            while not node.done and time.monotonic() < end: rclpy.spin_once(node,timeout_sec=.1)
        node.destroy_node(); rclpy.shutdown()
        print('Keep base drive isolated. On failure/cancel, inspect holding/stop and perform manual recovery before another trial.')


if __name__ == '__main__': main()
