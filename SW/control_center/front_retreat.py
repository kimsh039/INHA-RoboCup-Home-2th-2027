"""Short, 3D-guarded reverse before ordinary Nav2 / docking recovery."""
import math
import time
import json


class FrontRetreat:
    DETECTION_PHASES = {'navigating', 'navigating_target', 'aligning', 'approaching',
                        'point_aligning', 'point_approaching', 'final_align', 'realigning',
                        'visual_aligning', 'visual_approaching'}

    def __init__(self, node, state, settings):
        self.node, self.state = node, state
        self.approach = node.detection_approach
        self.distance = float(settings.get('distance', .08))
        if not math.isfinite(self.distance) or not .05 <= self.distance <= .10:
            raise ValueError('front_retreat.distance는 0.05~0.10m입니다.')
        self.run = None
        self.status = None
        self.context = None
        self.attempts = 0
        self.blocked_since = self.rearm_pose = None
        self.cooldown_until = 0.
        self.last_front_pose = None
        self.route_changes = 0
        self.zones = []
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        self.route_pub = node.create_publisher(self.approach.String, '/navigation/blocked_routes',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.publish_routes()

    def publish_routes(self):
        self.route_pub.publish(self.approach.String(data=json.dumps(dict(frame='odom', zones=self.zones), allow_nan=False)))

    def clear_routes(self):
        if self.zones:
            self.zones = []
            self.publish_routes()

    @property
    def active(self):
        return self.run is not None

    @property
    def detection_owned(self):
        return self.active and self.run['detection']

    def report(self, phase, reason, **details):
        self.status = dict(active=self.active, phase=phase, reason=reason,
                           requested_distance=self.distance, source='front_obstacle_retreat', **details)
        self.node.nav_status['front_retreat'] = self.status
        self.state.log(reason, 'warn' if phase in ('blocked', 'no_progress', 'error') else 'info', 'navigation')

    def cancel(self, reason='전방 장애물 후진 취소'):
        if self.active:
            self.approach.command()
            self.run = None
            self.report('canceled', reason)
        self.blocked_since = None
        self.clear_routes()

    def collision(self, now):
        value, _, received_at = self.approach.lidar.protection()
        received = received_at.get('guard')
        stamp = value.get('stamp')
        ros_now = self.node.get_clock().now().nanoseconds/1e9
        fresh = (received is not None and now-received <= 1.
                 and isinstance(stamp, (int, float)) and -.1 <= ros_now-stamp <= 1.)
        return value, fresh

    @staticmethod
    def front_blocked(value, allow_slowed=True):
        point = value.get('blocking_source_point_base')
        command = value.get('command') or []
        obstruction = ((value.get('state') == 'blocked'
                        and value.get('reason') == 'predicted_clearance_violation')
                       or (allow_slowed and value.get('state') == 'slowed' and value.get('speed_scale', 1.) <= .25))
        return (value.get('ready') and obstruction
                and isinstance(point, list) and len(point) == 3
                and all(isinstance(v, (int, float)) and math.isfinite(v) for v in point)
                and point[0] > 0.  # base_link: front half is +x, rear half is -x
                and len(command) == 2 and command[0] >= 0.
                and (command[0] > .005 or abs(command[1]) > .01))

    def inputs(self, now):
        odom, meta = self.approach.lidar.odometry()
        received, stamp = meta.get('received'), meta.get('stamp')
        ros_now = self.node.get_clock().now().nanoseconds/1e9
        if (received is None or now-received > 1. or stamp is None or not -.1 <= ros_now-stamp <= 1.):
            raise RuntimeError('짧은 후진용 현재 오도메트리 수신 지연')
        pose, _ = self.approach.robot()
        if not all(math.isfinite(v) for v in pose):
            raise RuntimeError('짧은 후진용 현재 위치 오류')
        return pose, odom

    @staticmethod
    def stopped(odom):
        return abs(odom.get('linear', 1.)) <= .01 and abs(odom.get('angular', 1.)) <= .02

    def consider(self, now, collision, fresh):
        nav = self.node.nav_status
        detection = self.approach.active
        # Precision docking owns its 15 cm recovery and completion checks.
        # Do not inject an independent retreat or route exclusion mid-dock.
        if detection and self.approach.phase in ('docking_handoff', 'settling', 'observing_table',
                'aligning', 'approaching', 'final_align', 'realigning', 'recovering',
                'final_reverse', 'checking_reverse'):
            self.blocked_since = None
            return
        goal = nav.get('goal')
        if detection:
            eligible = (self.approach.phase in self.DETECTION_PHASES
                        and (not self.approach.nav_busy or goal is self.approach.stage_goal))
            context = ('detection', self.approach.started)
        else:
            eligible = (nav.get('state') == 'active' and goal is not None
                        and goal.get('owner', 'navigation') == 'navigation')
            context = ('navigation', id(goal))
        if context != self.context:
            self.context, self.attempts, self.rearm_pose = context, 0, None
            self.last_front_pose, self.route_changes = None, 0
            self.clear_routes()
            self.cooldown_until = 0.
            self.blocked_since = None
        fine = detection and self.approach.phase in ('aligning', 'approaching', 'point_aligning',
                                                      'point_approaching', 'final_align', 'realigning')
        # A measured grasp-distance limit can legitimately end forward travel
        # near furniture. Let precision control finish alignment and the stop
        # handoff instead of pulling the base 8 cm away first.
        if fine and (self.approach.grasp_gap_satisfied() or self.approach.heading_only()):
            self.blocked_since = None
            return
        if fine and self.approach.plan is not None:
            pose, _ = self.approach.robot()
            dock = self.approach.plan['dock']
            if math.hypot(pose[0]-dock[0], pose[1]-dock[1]) <= self.approach.position_tolerance:
                self.blocked_since = None
                return  # Let the approach accept this pose and align in place.
        if not eligible or not fresh or not self.front_blocked(collision, allow_slowed=not fine):
            self.blocked_since = None
            if not detection and nav.get('state') in ('succeeded', 'aborted', 'canceled', 'rejected', 'error', 'idle'):
                self.clear_routes()
            return
        pose, _ = self.inputs(now)
        if self.rearm_pose is not None:
            previous = self.rearm_pose
            if math.hypot(pose[0]-previous[0], pose[1]-previous[1]) >= .15:
                self.attempts, self.rearm_pose = 0, None
                self.last_front_pose, self.route_changes = None, 0
        self.blocked_since = self.blocked_since or now
        if now-self.blocked_since < .15:
            return
        # Only interrupt our own goal. An unrelated Nav2 publisher is not a
        # reason to emit reverse commands from this control center.
        owned_goal = self.approach.stage_goal if detection else goal
        if nav.get('state') in ('active', 'sending', 'canceling') and goal is not owned_goal:
            return
        previous = self.last_front_pose
        repeated = (previous is not None and math.hypot(pose[0]-previous[0], pose[1]-previous[1]) <= .25
                    and abs(math.atan2(math.sin(pose[2]-previous[2]), math.cos(pose[2]-previous[2]))) <= .5)
        if now < self.cooldown_until and not repeated:
            self.blocked_since = None
            return
        skip_reverse = repeated or self.attempts >= 2
        if not skip_reverse:
            self.attempts += 1
            self.last_front_pose = pose
        self.run = dict(detection=detection, detection_started=self.approach.started,
                        goal=owned_goal, previous_phase=self.approach.phase,
                        phase='waiting', began=now, wall_started=now, stopped_since=None,
                        last_pose=None, distance=0., outcome=None,
                        failed_pose=pose, alternate=skip_reverse, skip_reverse=skip_reverse,
                        odometry_epoch=self.approach.lidar.odometry()[1].get('epoch'))
        self.approach.command()
        if nav.get('state') == 'active':
            self.node.cancel_goal()
        message = ('후진 후 같은 진입 구간 재차 차단 · 추가 후진 없이 다른 경로 선택' if skip_reverse else
                   f'전방 절반 충돌점 감지 · 회전보다 먼저 {self.distance*100:.0f}cm 후진 · Nav2 정지 확인')
        self.report('waiting', message, blocking_point=collision['blocking_source_point_base'],
                    blocking_link=collision.get('blocking_link'))
        if detection:
            self.approach.last_pose = None
            self.approach.report('rerouting_obstacle' if skip_reverse else 'front_retreat', message)
            self.approach.assess_reverse('waiting', message)

    def abort(self, reason):
        run = self.run
        self.approach.command()
        self.run = None
        self.clear_routes()
        self.report('error', reason)
        if run['detection']:
            self.approach.cancel(reason, 'error')
        else:
            if self.node.nav_status.get('goal') is run['goal']:
                self.node.cancel_goal()
            self.node.nav_update(state='error', message=reason)

    def finish_motion(self, now, outcome, reason, collision):
        self.approach.command()
        self.run.update(phase='stopping', stopped_since=None, outcome=outcome, stopped_at=now)
        self.report(outcome, reason, distance=self.run['distance'],
                    blocking_point=collision.get('blocking_source_point_base'),
                    blocking_link=collision.get('blocking_link'))
        if self.run['detection']:
            self.approach.assess_reverse(outcome, reason, distance=self.run['distance'])

    def resume(self, now, pose):
        run = self.run
        run['alternate'] |= run['outcome'] != 'moved'
        if run['alternate'] and not run.get('route_ready'):
            if self.route_changes >= 3:
                self.abort('후진과 대체 경로 3회로도 진행 불가 · 같은 경로 재시도 중지')
                return
            self.route_changes += 1
            ros_now = self.node.get_clock().now().nanoseconds/1e9
            self.zones = [z for z in self.zones if z['expires'] > ros_now][-5:]
            self.zones.append(dict(pose=list(run['failed_pose']), expires=ros_now+30.))
            self.publish_routes()
            run.update(phase='rerouting', route_until=now+.5)
            self.report('rerouting', '후진 효과 없음 · 실패한 진입 구간을 30초 제외하고 대체 경로 선택',
                        distance=run['distance'], reroutes=self.route_changes)
            if run['detection']:
                self.approach.report('rerouting_obstacle', self.status['reason'])
            return
        self.run = None
        self.blocked_since = None
        self.cooldown_until, self.rearm_pose = now+3., pose
        result = dict(state=run['outcome'], reason=self.status['reason'],
                      distance=run['distance'], source='front_obstacle_retreat')
        self.report('done', result['reason']+' · 정지 완료·경로 재계획', distance=run['distance'])
        try:
            self.resume_goal(now, run, result)
        except Exception:
            self.run = run  # Let tick stop the owner and report the failed restart.
            raise

    def resume_goal(self, now, run, result):
        if run['detection']:
            approach = self.approach
            approach.stage_goal = None
            approach.last_pose = None
            approach.progress_best, approach.progress_at = math.inf, now
            if run['alternate']:
                approach.reroute_after_retreat(now)
            elif run['previous_phase'] in ('final_align', 'realigning'):
                approach.finish_request = None
                approach.arrived_since = None
                approach.report('realigning', '짧은 후진 후 물체 방향 선정렬 · 정렬 후 재접근')
            elif run['outcome'] != 'moved' and run['previous_phase'] not in ('navigating', 'navigating_target'):
                approach.begin_recovery(now, '짧은 후진 진행 불가 · 직선 후진 복구로 전환')
            else:
                approach.resume_navigation(now)
        else:
            target = self.node.send_goal(run['goal'])
            self.context = ('navigation', id(target))  # Preserve this congestion's attempt count.
            self.node.nav_status['reverse_assessment'] = result

    def advance(self, now, collision, fresh):
        run = self.run
        if now-run['wall_started'] > 20.:
            self.abort('짧은 후진 전체 시간 초과 · 정지')
            return
        if run['detection'] and (not self.approach.active or self.approach.started != run['detection_started']):
            self.cancel('접근 취소·목표 변경 · 짧은 후진 중지')
            return
        nav = self.node.nav_status
        if nav.get('goal') is not run['goal'] and (run['goal'] is not None
                                                    or nav.get('state') in ('active', 'sending', 'canceling')):
            self.abort('다른 Nav2 목표 감지 · 짧은 후진 중지')
            return
        if run['phase'] == 'waiting' and run['goal'] is not None and nav.get('state') == 'succeeded':
            self.cancel('Nav2 목표 도착 확인 · 추가 후진 없이 종료')
            if run['detection']:
                self.approach.report(run['previous_phase'], 'Nav2 도착 확인 · 접근 단계 계속')
            return
        if not fresh or not collision.get('ready'):
            self.wait_inputs(now, collision.get('reason', 'collision_status_missing'))
            return
        _, actuator, received_at = self.approach.lidar.protection()
        if (not actuator.get('ready') or now-received_at.get('actuator', -math.inf) > .5):
            self.wait_inputs(now, actuator.get('reason', 'actuator_status_missing'))
            return
        try:
            pose, odom = self.inputs(now)
        except RuntimeError as exc:
            self.wait_inputs(now, str(exc))
            return
        epoch = self.approach.lidar.odometry()[1].get('epoch')
        if run.get('odometry_epoch') is not None and epoch != run['odometry_epoch']:
            self.abort('후진 중 실제 오도메트리 프레임 초기화 · 정지')
            return
        if run.get('input_wait') is not None:
            waited = now-run.pop('input_wait')
            for key in ('began', 'started', 'progress_at', 'stopped_at', 'route_until'):
                if key in run:
                    run[key] += waited
            run['last_pose'] = None
            self.report(run['phase'], '후진 입력 수신 복구 · 남은 거리 계속 진행', distance=run['distance'])
        previous = run['last_pose']
        if previous:
            then, old = previous
            angle = math.atan2(math.sin(pose[2]-old[2]), math.cos(pose[2]-old[2]))
            elapsed = max(0., now-then)
            if math.hypot(pose[0]-old[0], pose[1]-old[1]) > .05+.3*elapsed or abs(angle) > .05+.75*elapsed:
                self.abort('후진 중 오도메트리 연속성 손실 · 정지')
                return
        run['last_pose'] = now, pose
        if run['phase'] == 'waiting':
            self.approach.command()
            if now-run['began'] > 8.:
                self.abort('짧은 후진 전 Nav2·차체 정지 확인 시간 초과')
                return
            if nav.get('state') in ('active', 'sending', 'canceling'):
                if nav['state'] == 'active':
                    self.node.cancel_goal()
                run['stopped_since'] = None
                return
            if self.approach.nav_busy or not self.stopped(odom):
                run['stopped_since'] = None
                return
            run['stopped_since'] = run['stopped_since'] or now
            if now-run['stopped_since'] < .2:
                return
            if run['skip_reverse']:
                run['outcome'] = 'no_progress'
                if run['detection']:
                    self.approach.assess_reverse('no_progress', '후진 이후 같은 전방 차단 재발 · 다른 경로 선택')
                self.resume(now, pose)
                return
            run.update(phase='backing', origin=pose, started=now, best=0., progress_at=now)
            self.report('backing', f'전방 장애물에서 {self.distance*100:.0f}cm 저속 후진 · 후방 궤적 3D 보호')
            if run['detection']:
                self.approach.assess_reverse('checking', self.status['reason'])
        if self.approach.nav_busy or nav.get('state') in ('active', 'sending', 'canceling'):
            self.abort('후진 중 다른 Nav2 이동 감지 · 정지')
            return
        if run['phase'] == 'rerouting':
            self.approach.command()
            if now >= run['route_until']:
                run['route_ready'] = True
                self.resume(now, pose)
            return
        if run['phase'] == 'stopping':
            self.approach.command()
            if now-run['stopped_at'] > 5.:
                self.abort('짧은 후진 후 실제 정지 확인 시간 초과')
                return
            run['stopped_since'] = (run['stopped_since'] or now) if self.stopped(odom) else None
            if run['stopped_since'] is not None and now-run['stopped_since'] >= .2:
                self.resume(now, pose)
            return
        origin = run['origin']
        dx, dy = pose[0]-origin[0], pose[1]-origin[1]
        progress = -(dx*math.cos(origin[2])+dy*math.sin(origin[2]))
        run['distance'] = max(0., progress)
        angle = math.atan2(math.sin(pose[2]-origin[2]), math.cos(pose[2]-origin[2]))
        if abs(-dx*math.sin(origin[2])+dy*math.cos(origin[2])) > .03 or abs(angle) > .10:
            self.abort('직선 후진 궤적에서 벗어남 · 정지')
            return
        if progress >= self.distance-.003:
            self.finish_motion(now, 'moved', f'전방 장애물 회피 후진 {progress*100:.1f}cm 완료', collision)
            return
        command = collision.get('command') or []
        if (collision.get('state') == 'blocked' and collision.get('reason') == 'predicted_clearance_violation'
                and len(command) == 2 and command[0] < -.005):
            self.finish_motion(now, 'blocked', '후방 3D 궤적 막힘 · 짧은 후진 중지', collision)
            return
        if progress > run['best']+.002:
            run.update(best=progress, progress_at=now)
        if now-run['progress_at'] > 2.5 or now-run['started'] > 6.:
            outcome = 'moved' if progress >= .05 else 'no_progress'
            self.finish_motion(now, outcome, f'짧은 후진 진행 {max(0., progress)*100:.1f}cm · 동작 종료', collision)
            return
        self.approach.command(-min(.05, max(.02, .8*(self.distance-progress))), 0.)

    def wait_inputs(self, now, reason):
        self.approach.command()
        run = self.run
        if run.get('input_wait') is None:
            run['input_wait'] = now
            run['stopped_since'] = None
            self.report('waiting_input', '후진 일시 정지 · 입력 복구 후 계속 · '+reason, distance=run['distance'])
        elif now-run['input_wait'] > 3.:
            self.abort('후진 입력 대기 3초 초과 · '+reason)

    def tick(self):
        now = time.monotonic()
        try:
            collision, fresh = self.collision(now)
            if self.active:
                self.advance(now, collision, fresh)
            else:
                self.consider(now, collision, fresh)
        except Exception as exc:
            if self.active:
                self.abort('짧은 후진 중지 · '+str(exc)[:200])
            else:
                self.blocked_since = None
