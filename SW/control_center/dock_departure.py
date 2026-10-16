"""Exclusive measured reverse -> stop -> turn -> stop -> Nav2 home departure."""
from concurrent.futures import ThreadPoolExecutor
import copy
import json
import math
import time
import numpy as np
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from std_msgs.msg import String
from collision_distance import PreparedBoxes, build_scene


def wrap(value): return math.atan2(math.sin(value), math.cos(value))


def clearance_plan(boxes, pose, base_z, plan, points, rear_points):
    prepared = PreparedBoxes(boxes)
    edge = np.asarray(plan['edge_origin'])
    outward = np.asarray(plan['outward'])
    heading = np.array([math.cos(pose[2]), math.sin(pose[2])])
    if float((-heading) @ outward) < .98:
        raise RuntimeError('도킹 면에 수직인 후진 방향 미확인')
    distance = max(.08, prepared.extent+.05-float((np.asarray(pose[:2])-edge) @ outward))
    if distance > 1.:
        raise RuntimeError('제자리 회전 공간 확보에 필요한 후진이 1m를 초과합니다.')
    exit_pose = np.array([pose[0]-distance*heading[0], pose[1]-distance*heading[1], pose[2]])
    # Fresh returns in each rear sector are required; absent observations are not free space.
    rear = rear_points[:, :2]-pose[:2]
    along = -rear @ heading
    lateral = rear @ np.array([-heading[1], heading[0]])
    angles = np.arctan2(lateral, along)
    for lo, hi in zip(np.linspace(-.65, .65, 9)[:-1], np.linspace(-.65, .65, 9)[1:]):
        if not np.any((angles >= lo) & (angles < hi) & (along > distance+prepared.extent)):
            raise RuntimeError('후방 이탈 구간 관측 부족 · 후진하지 않습니다.')
    scene_points = points.copy(); scene_points[:, 2] -= base_z
    scene = build_scene(scene_points)
    deadline = time.monotonic()+2.
    previous = np.asarray(pose)
    for travel in np.linspace(0., distance, max(2, math.ceil(distance/.01)+1))[1:]:
        current = np.array([pose[0]-travel*heading[0], pose[1]-travel*heading[1], pose[2]])
        if prepared.swept_contact(scene, previous, current, .011, deadline):
            raise RuntimeError('실측 후방 궤적에 장애물')
        previous = current
    # A full turn is conservative and allows either direction without a hidden corner swing.
    for angle in np.linspace(0., 2*math.pi, max(36, math.ceil(2*math.pi*prepared.extent/.01))):
        query = exit_pose.copy(); query[2] += angle
        nearest, _, _ = prepared.nearest(scene, query, .035, deadline)
        if nearest < .025:
            raise RuntimeError('후진 종료점에서 차체·랙·팔 회전 공간 부족')
    return float(distance), exit_pose.tolist()


class DockDeparture:
    MESSAGES = dict(hold='도킹 유지', planning='후진·회전 공간 계획', path='홈 복귀 경로 계획',
        prepare='팔 수납 확인·베이스 잠금 인계', reverse='직선 후진', stop_reverse='후진 후 정지 확인',
        turn='제자리 회전·Head 진행 방향 정렬', stop_turn='정렬 후 정지 확인', finishing='홈 정지·최종 게이트 확인',
        returning='지정 홈으로 복귀', done='홈 위치·방향 도착 및 정지 완료')

    def __init__(self, node, state, config):
        self.node, self.state, self.config = node, state, config
        self.a = node.detection_approach
        self.phase = 'idle'; self.message = ''; self.docked = False
        self.dock_id = ''; self.task_id = ''; self.plan = None; self.request_data = None
        self.gate = {}; self.gate_received = 0.; self.drive = {}; self.drive_received = 0.
        self.entered = time.monotonic(); self.stopped_since = None
        self.geometry_future = None; self.path_future = None; self.path_handle = None
        self.path_generation = 0; self.distance = 0.; self.target_yaw = None
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='dock-departure-plan')
        self.path_client = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
        self.control_pub = node.create_publisher(String, '/navigation/departure/control', 1)
        self.release_pub = node.create_publisher(String, '/manipulation/departure/request', 1)
        node.create_subscription(String, '/navigation/departure/gate', lambda m: self.receive('gate', m), 1)
        node.create_subscription(String, '/manipulation/drive_ready', lambda m: self.receive('drive', m), 1)
        node.create_timer(.05, self.tick)

    @property
    def active(self): return self.phase not in ('idle', 'hold', 'done', 'blocked', 'canceled')

    def receive(self, key, msg):
        try:
            value = json.loads(msg.data)
            if not isinstance(value, dict): return
            setattr(self, key, value); setattr(self, key+'_received', time.monotonic())
            if (key == 'gate' and value.get('phase') not in ('done', None) and self.phase == 'idle'
                    and not (self.a.phase == 'succeeded' and self.a.task_id == value.get('dock_id'))):
                self.docked = True
                self.dock_id = value.get('dock_id', '')
                self.phase, self.message = 'blocked', '이전 도킹 잠금 복원 · 자동 재출발 금지'
        except (ValueError, TypeError): pass

    def latch(self):
        if self.a.phase == 'succeeded' and self.a.task_id and self.dock_id != self.a.task_id:
            self.dock_id = self.a.task_id
            self.plan = copy.deepcopy(self.a.plan)
            self.docked = True
            self.set_phase('hold')

    def set_phase(self, phase):
        self.phase, self.entered, self.stopped_since = phase, time.monotonic(), None
        self.message = self.MESSAGES.get(phase, phase)
        self.state.log('도킹 이탈 · '+self.message, source='navigation')

    def start(self, request):
        self.latch()
        if self.active: raise RuntimeError('이탈 작업 진행 중')
        if not self.docked or self.phase != 'hold' or self.plan is None:
            raise RuntimeError('현재 도킹 완료 문맥이 없습니다.')
        if request.get('dock_id') != self.dock_id or not request.get('task_id'):
            raise RuntimeError('도킹·팔 작업 ID 불일치')
        goal = request.get('goal', {})
        if not all(type(goal.get(k)) in (float, int) and math.isfinite(goal[k]) for k in ('x','y','yaw')):
            raise RuntimeError('유효한 홈 위치·방향 필요')
        if self.a.active or self.a.nav_busy or self.node.nav_status['state'] in ('active', 'sending', 'canceling'):
            raise RuntimeError('이탈 전 다른 이동 작업이 남아 있습니다.')
        self.task_id = request['task_id']; self.request_data = copy.deepcopy(request)
        self.reversed = False
        self.node.front_retreat.cancel('도킹 이탈 제어로 인계')
        self.epoch = self.a.lidar.odometry()[1].get('epoch')
        self.reference = self.a.transform('odom')
        self.set_phase('planning')

    def command(self, linear=0., angular=0.):
        phase = {'planning':'hold', 'path':'stop_reverse' if self.reversed else 'hold',
                 'returning':'return', 'finishing':'done', 'canceled':'blocked'}.get(self.phase, self.phase)
        value = dict(stamp=self.node.get_clock().now().nanoseconds/1e9, dock_id=self.dock_id,
            task_id=self.task_id, phase=phase, reverse_distance=self.distance,
            target_yaw=self.target_yaw, linear=linear, angular=angular)
        self.control_pub.publish(String(data=json.dumps(value)))
        if self.phase != 'returning': self.a.command(linear, angular)

    def cancel(self, reason='도킹 이탈 취소'):
        if not self.active: return
        self.path_generation += 1
        if self.path_handle: self.path_handle.cancel_goal_async()
        if self.phase == 'returning': self.node.cancel_goal()
        self.phase, self.message = 'canceled', reason
        self.command()

    def inputs(self):
        self.a.require_protection_ready()
        pose, odom = self.node.front_retreat.inputs(time.monotonic())
        if self.a.lidar.odometry()[1].get('epoch') != self.epoch:
            raise RuntimeError('이탈 중 오도메트리 기준 변경')
        r, t = self.a.transform('odom')
        old_r, old_t = self.reference
        if np.linalg.norm(t-old_t) > .08 or abs(wrap(math.atan2(r[1,0], r[0,0])-math.atan2(old_r[1,0],old_r[0,0]))) > .08:
            raise RuntimeError('복귀 중 map→odom 기준 변화 · 경로 재확인 필요')
        now = time.monotonic()
        if (now-self.drive_received > .4 or not self.drive.get('ready') or self.drive.get('task_id') != self.task_id):
            raise RuntimeError('팔 작업 완료·현재 수납 상태 확인 없음')
        if now-self.gate_received > .6 or self.gate.get('dock_id') != self.dock_id:
            raise RuntimeError('최종 도킹 이탈 게이트 연결 없음')
        if self.gate.get('phase') == 'blocked': raise RuntimeError('최종 이탈 게이트가 잠겼습니다.')
        if self.phase not in ('planning','path','prepare') and not self.drive.get('released'):
            raise RuntimeError('베이스 잠금 인계 미완료')
        return pose, odom

    def stopped(self, odom):
        now = time.monotonic()
        if self.node.front_retreat.stopped(odom):
            self.stopped_since = self.stopped_since or now
        else: self.stopped_since = None
        return self.stopped_since is not None and now-self.stopped_since >= .6

    def cloud_snapshot(self):
        now = self.node.get_clock().now().nanoseconds/1e9
        with self.a.lidar.lock:
            clouds = list(self.a.lidar.clouds)
            rear = list(self.a.lidar.surface_clouds)
        if not clouds or not rear or not all(-.05 <= now-x[-1][0] <= .4 for x in (clouds, rear)):
            raise RuntimeError('이탈 계획용 최신 점군 없음')
        return (np.concatenate([x[1] for x in clouds if now-x[0] <= .3]),
                np.concatenate([x[1] for x in rear if now-x[0] <= .3]))

    def request_path(self, start_pose):
        if not self.path_client.server_is_ready(): raise RuntimeError('홈 경로 계획 서버 연결 없음')
        goal = ComputePathToPose.Goal()
        goal.planner_id = 'GridBased'
        goal.goal = self.node.pose_msg(self.request_data['goal'], type(goal.goal))
        r, t = self.a.transform('odom')
        xy = r @ np.array([start_pose[0], start_pose[1], 0.])+t
        start = dict(x=float(xy[0]), y=float(xy[1]), yaw=start_pose[2]+math.atan2(r[1,0],r[0,0]))
        goal.start = self.node.pose_msg(start, type(goal.start)); goal.use_start = True
        self.path_generation += 1; generation = self.path_generation
        self.path_future = None; self.path_error = ''; self.path_started = time.monotonic()
        def accepted(future):
            try:
                handle = future.result()
                if generation != self.path_generation:
                    if handle.accepted: handle.cancel_goal_async()
                    return
                if not handle.accepted: raise RuntimeError('홈 복귀 경로 요청 거부')
                self.path_handle = handle; self.path_future = handle.get_result_async()
            except Exception as exc: self.path_error = str(exc)
        self.path_client.send_goal_async(goal).add_done_callback(accepted)
        self.set_phase('path')

    def advance(self):
        pose, odom = self.inputs()
        elapsed = time.monotonic()-self.entered
        if elapsed > (180. if self.phase == 'returning' else 90.): raise RuntimeError('이탈 단계 시간 초과')
        if self.phase != 'returning' and (self.a.active or self.a.nav_busy or self.node.nav_status['state'] in ('active','sending','canceling')):
            raise RuntimeError('이탈 중 다른 이동 소유자 감지')
        if self.phase in ('reverse', 'stop_reverse', 'turn', 'stop_turn') and self.gate.get('phase') != self.phase:
            self.command()
            if elapsed > 3.: raise RuntimeError('이탈 단계의 최종 게이트 ACK 시간 초과')
            return
        if self.phase == 'planning':
            self.command()
            if not self.stopped(odom): return
            if self.geometry_future is None:
                positions = self.a.arm_feedback.positions()
                if self.a.planner.model.required_joints-set(positions): raise RuntimeError('회전 형상용 관절 피드백 없음')
                points, rear = self.cloud_snapshot()
                _, base_z = self.a.robot()
                self.origin = pose; self.arm_positions = positions
                self.geometry_future = self.pool.submit(clearance_plan, self.a.planner.model.boxes(positions),
                    pose, base_z, self.plan, points, rear)
            if not self.geometry_future.done(): return
            self.distance, self.exit_pose = self.geometry_future.result()
            self.geometry_future = None
            self.request_path(self.exit_pose)
        elif self.phase == 'path':
            self.command()
            if self.path_error: raise RuntimeError(self.path_error)
            if time.monotonic()-self.path_started > 10.: raise RuntimeError('홈 복귀 경로 계획 시간 초과')
            if not self.path_future or not self.path_future.done(): return
            result = self.path_future.result()
            if result.status != 4 or len(result.result.path.poses) < 2: raise RuntimeError('홈 복귀 경로 없음')
            path = result.result.path
            if path.header.frame_id != self.node.fixed: raise RuntimeError('복귀 경로 좌표계 오류')
            r, t = self.a.transform(self.node.fixed, target='odom')
            points = [r @ np.array([p.pose.position.x,p.pose.position.y,0.])+t for p in path.poses]
            start = np.asarray(pose[:2] if self.reversed else self.exit_pose[:2])
            point = next((p for p in points if np.linalg.norm(p[:2]-start) >= .15), None)
            if point is None: raise RuntimeError('Head 정렬용 전진 경로가 너무 짧습니다.')
            # Actual optical +Z must look along base forward; do not assume mount yaw.
            head_r, _ = self.a.transform(self.config.get('head_optical_frame','camera_color_optical_frame'), target=self.node.base)
            axis = head_r[:, 2]
            self.head_offset = math.atan2(axis[1],axis[0])
            if np.linalg.norm(axis[:2]) < .5 or abs(self.head_offset) > math.radians(4):
                raise RuntimeError('고정 Head와 차체 진행 방향 정렬 범위 불일치')
            self.route_yaw = math.atan2(point[1]-start[1], point[0]-start[0])
            self.target_yaw = wrap(self.route_yaw-self.head_offset/2)
            self.path_time = time.monotonic()
            self.set_phase('turn' if self.reversed else 'prepare')
        elif self.phase == 'prepare':
            self.command()
            if self.gate.get('phase') != 'prepare': return
            self.release_pub.publish(String(data=json.dumps(dict(task_id=self.task_id,
                stamp=self.node.get_clock().now().nanoseconds/1e9))))
            if self.drive.get('released') and self.stopped(odom):
                self.origin = pose; self.progress_at = time.monotonic(); self.best = 0.
                self.set_phase('reverse')
        elif self.phase == 'reverse':
            dx, dy = pose[0]-self.origin[0], pose[1]-self.origin[1]
            yaw = self.origin[2]
            progress = -(dx*math.cos(yaw)+dy*math.sin(yaw))
            if abs(-dx*math.sin(yaw)+dy*math.cos(yaw)) > .03 or abs(wrap(pose[2]-yaw)) > .06:
                raise RuntimeError('직선 후진 궤적 이탈')
            if progress >= self.distance-.003:
                self.reversed = True; self.set_phase('stop_reverse'); self.command(); return
            if progress > self.best+.002: self.best, self.progress_at = progress, time.monotonic()
            if time.monotonic()-self.progress_at > 3.: raise RuntimeError('후진 진행 불가')
            # The general guard may use chassis-only geometry. Include the rack
            # and measured stowed arm here for the next braking-length segment.
            points, _ = self.cloud_snapshot()
            positions = self.a.arm_feedback.positions()
            if any(abs(positions.get(n, math.inf)-v) > .03 for n,v in self.arm_positions.items()):
                raise RuntimeError('후진 중 팔 수납 자세 변화')
            _, base_z = self.a.robot()
            points[:,2] -= base_z
            boxes = PreparedBoxes(self.a.planner.model.boxes(positions))
            future = np.array([pose[0]-.04*math.cos(yaw), pose[1]-.04*math.sin(yaw), yaw])
            if boxes.swept_contact(build_scene(points), np.asarray(pose), future, .04, time.monotonic()+.025):
                raise RuntimeError('후방 차체·랙·팔 궤적 차단')
            self.command(-min(.05, max(.015, .5*(self.distance-progress))), 0.)
        elif self.phase == 'stop_reverse':
            self.command()
            if self.stopped(odom): self.request_path(pose)
        elif self.phase == 'turn':
            # Recheck the complete current rotation envelope against fresh 3D returns.
            points, _ = self.cloud_snapshot()
            positions = self.a.arm_feedback.positions()
            if any(abs(positions.get(n, math.inf)-v) > .03 for n,v in self.arm_positions.items()):
                raise RuntimeError('수납 팔 자세 변화')
            boxes = self.a.planner.model.boxes(positions)
            radius = max(np.linalg.norm(b.vertices()[:,:2],axis=1).max() for b in boxes)
            _, base_z = self.a.robot()
            zmin = min(b.vertices()[:,2].min() for b in boxes)+base_z
            zmax = max(b.vertices()[:,2].max() for b in boxes)+base_z
            if np.any((np.linalg.norm(points[:,:2]-pose[:2],axis=1) < radius+.025)
                      & (points[:,2] >= zmin) & (points[:,2] <= zmax)):
                raise RuntimeError('제자리 회전 영역에 장애물 관측')
            error = wrap(self.target_yaw-pose[2])
            aligned = (abs(wrap(self.route_yaw-pose[2])) <= math.radians(2) and
                       abs(wrap(self.route_yaw-pose[2]-self.head_offset)) <= math.radians(2))
            if aligned: self.set_phase('stop_turn'); self.command(); return
            self.command(0., max(-.15, min(.15, .6*error)))
        elif self.phase == 'stop_turn':
            self.command()
            if not self.stopped(odom): return
            self.set_phase('returning'); self.command()
            # Wait for the final gate's return ACK before handing Nav2 a goal.
            self.return_goal = None
        elif self.phase == 'returning':
            self.command()
            if self.gate.get('phase') != 'return': return
            if self.return_goal is None:
                self.return_goal = self.node.send_goal(dict(self.request_data['goal'], owner='dock_return'))
            nav = self.node.nav_status
            if nav.get('goal') is not self.return_goal: raise RuntimeError('홈 복귀 목표 소유권 상실')
            if nav['state'] in ('aborted','error','canceled','rejected'): raise RuntimeError(nav['message'])
            if nav['state'] == 'succeeded' and self.stopped(odom):
                r, t = self.a.transform(self.node.base)
                goal = self.request_data['goal']
                if math.hypot(t[0]-goal['x'],t[1]-goal['y']) > .1 or abs(wrap(math.atan2(r[1,0],r[0,0])-goal['yaw'])) > .05:
                    raise RuntimeError('홈 위치·방향 실측 오차 초과')
                self.set_phase('finishing'); self.command()
        elif self.phase == 'finishing':
            self.command()
            if self.gate.get('phase') == 'done' and self.stopped(odom):
                self.set_phase('done')
                self.docked = False

    def tick(self):
        self.latch()
        if not hasattr(self, 'reversed'): self.reversed = False
        if self.active:
            try: self.advance()
            except Exception as exc:
                self.cancel(str(exc)); self.phase = 'blocked'; self.command()
        self.state.put('dock_departure', dict(phase=self.phase, active=self.active, docked=self.docked,
            task_id=self.task_id, dock_id=self.dock_id, message=self.message,
            reverse_distance=self.distance, target_yaw=self.target_yaw))

    def close(self):
        self.cancel('제어기 종료 · 도킹 잠금 유지')
        self.pool.shutdown(wait=False, cancel_futures=True)
