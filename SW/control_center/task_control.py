"""Single-command table service. Runs on the bridge executor, never in HTTP callbacks."""
from collections import deque
import copy
import json
import math
import os
from pathlib import Path
import threading
import time
import uuid

from detection_approach import target_label


class TaskControl:
    TERMINAL = {'idle', 'succeeded', 'failed', 'canceled', 'interrupted'}
    LABELS = dict(sensors='센서 준비', mapping='SLAM·홈 지도 위치추정', navigation='NAV 준비',
                  manipulation_ready='팔·손목 실행 준비', detection='물체 탐지·closed approach',
                  manipulation='집기·손목 SAM 접시 관측·접시에 놓기·팔 수납',
                  departure='후진·정지·Head 정렬·홈 복귀', succeeded='접시 배치 및 홈 복귀 완료')

    def __init__(self, node, state, manipulation):
        from std_msgs.msg import String
        self.String = String
        self.heartbeat = node.create_publisher(String, '/task/orchestrator', 1)
        self.node, self.state, self.manipulation = node, state, manipulation
        self.config = state.config.get('task', {})
        self.path = Path(self.config.get('state_file', '~/.ros/inha_control_center/task.json')).expanduser()
        self.lock = threading.Lock()
        self.requests = deque(maxlen=8)
        self.home = None
        self.home_pending = None
        self.home_message = 'SLAM을 시작하면 시작 위치를 홈으로 등록합니다.'
        self.run = dict(phase='idle', active=False, message='물체를 선택하세요.')
        self.map_session = uuid.uuid4().hex
        self.map_owner = None
        try:
            if self.path.exists():
                saved = json.loads(self.path.read_text())
                self.home = saved.get('home')
                if saved.get('run', {}).get('active'):
                    self.run = dict(saved['run'], active=False, phase='interrupted',
                                    message='이전 전체 Task 중 실행기 재시작 · 자동 재실행 금지')
        except (OSError, ValueError):
            self.run = dict(phase='interrupted', active=False, message='전체 Task 저장 상태 오류')
        state.task_request = self.request
        state.task_interlock = lambda: self.run['active'] or self.home_pending is not None
        state.home_on_slam_start = self.queue_slam_home
        node.create_timer(.1, self.tick)

    def queue_slam_home(self):
        with self.lock:
            self.requests.append(dict(action='slam_home'))

    def persist(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temp = self.path.with_suffix('.tmp')
        with temp.open('w') as f:
            json.dump(dict(home=self.home, run=self.run), f, ensure_ascii=False, allow_nan=False)
            f.flush(); os.fsync(f.fileno())
        temp.replace(self.path)

    def request(self, value):
        if value.get('action') not in ('start', 'cancel', 'set_home'):
            raise ValueError('action은 start/cancel/set_home 중 하나입니다.')
        request = dict(action=value['action'])
        if request['action'] == 'start':
            request['label'] = target_label(value.get('label'))
        with self.lock:
            if request['action'] == 'cancel': self.requests.clear()
            if len(self.requests) >= 8:
                raise RuntimeError('전체 Task 요청 처리 대기 중')
            self.requests.append(request)

    def value(self, key):
        with self.state.condition:
            return copy.deepcopy(self.state.values.get(key) or {})

    def fresh(self, key, age=1.5):
        with self.state.condition:
            meta = self.state.channels.get(key, {})
            received = meta.get('received')
            source_age = meta.get('source_age')
        return (received is not None and 0 <= time.monotonic()-received <= age
                and (source_age is None or -.1 <= source_age+time.monotonic()-received <= age))

    def map_id(self):
        mapping = self.value('service_control').get('mapping', {})
        if mapping.get('mode') == 'localization' and mapping.get('map'):
            name = mapping['map']
            base = self.state.sensor_manager.map_base(name)
            revision = []
            for suffix in ('.yaml', '.posegraph'):
                try:
                    stat = base.with_suffix(suffix).stat()
                    revision.extend((str(stat.st_size), str(stat.st_mtime_ns)))
                except OSError: revision.append('missing')
            return 'saved:'+name+':'+'.'.join(revision)
        # Starting -> running and publisher discovery do not create new maps.
        owner = mapping.get('session_id') or mapping.get('pid') or 'external'
        if owner != self.map_owner:
            self.map_owner, self.map_session = owner, uuid.uuid4().hex
        return 'session:'+self.map_session

    def transition(self, phase, message=None):
        self.run.update(phase=phase, active=phase not in self.TERMINAL,
                        message=message or self.LABELS.get(phase, phase), since=time.monotonic())
        self.persist()
        self.state.log('전체 Task · '+self.run['message'],
                       'warn' if phase in ('failed', 'interrupted') else 'info', 'task')

    def cancel(self, phase='canceled', message='전체 Task 취소 · 정지 요청'):
        if self.home_pending is not None:
            self.home_pending = None
            self.home_message = '홈 등록 예약 취소'
        # Invalidate queued starts as well as accepted goals.
        self.manipulation.request(dict(action='cancel'))
        for send in (self.state.detection_request, self.state.navigation_request):
            if send:
                try: send(dict(action='cancel'))
                except RuntimeError: pass
        self.transition(phase, message)

    def service(self, key):
        service = self.value('service_control').get(key, {})
        if service.get('error'):
            raise RuntimeError(key+': '+service['error'])
        if service.get('managed') or service.get('publishers'):
            return True
        if not service.get('busy'):
            manager = self.state.sensor_manager
            if key == 'mapping':
                home = self.run.get('home') or {}
                map_id = home.get('map_id', '')
                if map_id.startswith('saved:'):
                    manager.service_request(key, 'localize', map_id.split(':', 2)[1])
                else:
                    self.run['home'] = None
                    manager.service_request(key, 'start')
            else:
                manager.service_request(key, 'start')
        return False

    def handle(self, req):
        if req['action'] == 'cancel':
            self.cancel(); return
        if req['action'] == 'slam_home':
            if self.run['active'] and self.run['phase'] not in ('sensors', 'mapping'):
                raise RuntimeError('진행 중인 작업의 홈을 변경할 수 없습니다.')
            if self.state.manipulation_interlock():
                raise RuntimeError('팔 작업·잠금 유지 중 홈 등록 불가')
            if self.home_pending is None: self.begin_home(start_mapping=False)
            return
        if self.run['active'] or self.state.manipulation_interlock():
            raise RuntimeError('기존 작업/베이스 잠금 유지 중')
        if req['action'] == 'set_home':
            if self.home_pending is None: self.begin_home(start_mapping=True)
            return
        if self.home_pending is not None:
            raise RuntimeError('SLAM 시작 위치를 홈으로 등록 중입니다. 완료 후 실행하세요.')
        if self.value('detection_control').get('active') or self.value('navigation').get('state') in ('active', 'sending', 'canceling'):
            raise RuntimeError('기존 이동을 종료한 뒤 전체 Task를 시작하세요.')
        if self.value('dock_departure').get('docked'):
            raise RuntimeError('이전 도킹 문맥이 남아 있습니다. 이탈 상태를 먼저 확인하세요.')
        self.run = dict(id=uuid.uuid4().hex, label=req['label'], home=copy.deepcopy(self.home),
                        active=True, phase='sensors', manip_task=None, started=time.time(),
                        requested=[])
        self.transition('sensors')

    def odom_pose(self):
        from rclpy.time import Time
        tf = self.node.tf.lookup_transform('odom', self.node.base, Time())
        stamp = tf.header.stamp.sec+tf.header.stamp.nanosec*1e-9
        now = self.node.get_clock().now().nanoseconds/1e9
        if not -.1 <= now-stamp <= .5: raise RuntimeError('시작 위치의 최신 odom TF 대기')
        p, q = tf.transform.translation, tf.transform.rotation
        if not all(math.isfinite(x) for x in (p.x,p.y,q.x,q.y,q.z,q.w)):
            raise RuntimeError('시작 위치 TF 오류')
        return dict(x=p.x, y=p.y, yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))

    def begin_home(self, start_mapping):
        if self.value('detection_control').get('active') or self.value('navigation').get('state') in ('active','sending','canceling'):
            raise RuntimeError('이동을 종료한 뒤 시작 위치를 홈으로 등록하세요.')
        self.home_pending = dict(started=time.monotonic(),
            after_ros=self.node.get_clock().now().nanoseconds/1e9, anchor=None, map_id=None)
        self.home_message = 'SLAM 시작·첫 위치 수신 대기 · 로봇을 정지해 주세요.'
        # Capture odom before map exists. Convert this fixed start point once SLAM supplies map->odom.
        try:
            if self.stopped_for_home(): self.home_pending['anchor'] = self.odom_pose()
        except Exception: pass
        if start_mapping:
            mapping = self.value('service_control').get('mapping', {})
            if not (mapping.get('managed') or mapping.get('publishers') or mapping.get('busy')):
                try: self.state.sensor_manager.service_request('mapping', 'start')
                except Exception:
                    self.home_pending = None
                    raise

    def stopped_for_home(self):
        odom = self.value('odom')
        values = [odom.get(k) for k in ('linear','angular')]
        return (self.fresh('odom', .5) and all(type(x) in (float,int) and math.isfinite(x) for x in values)
                and abs(values[0]) <= .01 and abs(values[1]) <= .02)

    def advance_home(self):
        from rclpy.time import Time
        pending = self.home_pending
        if time.monotonic()-pending['started'] > 90.:
            raise RuntimeError('홈 등록 시간 초과 · SLAM·LiDAR·odom 입력을 확인하세요.')
        mapping = self.value('service_control').get('mapping', {})
        if mapping.get('error'): raise RuntimeError('SLAM 시작 실패 · '+mapping['error'])
        if not self.stopped_for_home():
            self.home_message = '홈 등록 대기 · 실측 정지·odom 입력 확인 중'
            return
        try: current = self.odom_pose()
        except Exception:
            self.home_message = '홈 등록 대기 · odom 위치 수신 중'
            return
        if pending['anchor'] is None: pending['anchor'] = current
        anchor = pending['anchor']
        angle = math.atan2(math.sin(current['yaw']-anchor['yaw']),math.cos(current['yaw']-anchor['yaw']))
        if math.hypot(current['x']-anchor['x'],current['y']-anchor['y']) > .05 or abs(angle) > .08:
            raise RuntimeError('홈 등록 중 시작 위치가 바뀌었습니다. 정지 후 다시 등록하세요.')
        if mapping.get('busy') or not (mapping.get('managed') or mapping.get('publishers')):
            self.home_message = 'SLAM 시작 대기 · 지도를 완성하거나 저장할 필요는 없습니다.'
            return
        if mapping.get('mode') == 'localization' and self.value('localization').get('state') != 'localized':
            self.home_message = '저장 지도 위치추정 완료 대기'
            return
        try: tf = self.node.tf.lookup_transform('map', 'odom', Time())
        except Exception:
            self.home_message = 'SLAM 첫 위치 수신 대기 · 첫 스캔 처리 후 홈을 저장합니다.'
            return
        stamp = tf.header.stamp.sec+tf.header.stamp.nanosec*1e-9
        now = self.node.get_clock().now().nanoseconds/1e9
        if stamp < pending['after_ros'] or not -.1 <= now-stamp <= 1.:
            self.home_message = 'SLAM 최신 좌표계 수신 대기'
            return
        p, q = tf.transform.translation, tf.transform.rotation
        if not all(math.isfinite(v) for v in (p.x,p.y,q.x,q.y,q.z,q.w)):
            raise RuntimeError('홈 등록용 map TF 오류')
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        c, s = math.cos(yaw), math.sin(yaw)
        self.home = dict(x=p.x+c*anchor['x']-s*anchor['y'], y=p.y+s*anchor['x']+c*anchor['y'],
            yaw=math.atan2(math.sin(yaw+anchor['yaw']),math.cos(yaw+anchor['yaw'])),
            map_id=self.map_id(), frame='map', saved_at=time.time(), source='slam_start', odom_anchor=anchor)
        self.home_message = 'SLAM 시작 위치·방향을 홈으로 등록했습니다.'
        self.home_pending = None
        self.persist()
        self.state.log(self.home_message, source='task')

    def advance(self):
        phase = self.run['phase']
        deadline = 360. if phase in ('detection', 'manipulation', 'departure') else 120.
        if time.monotonic()-self.run['since'] > deadline:
            raise RuntimeError(self.LABELS.get(phase, phase)+' 시간 초과')
        if phase == 'sensors':
            controls = self.value('sensor_control')
            pending = False
            for key in ('head_camera', 'wrist_camera', 'mid360_lidar', 'g2_lidar'):
                sensor = controls.get(key, {})
                if sensor.get('error'): raise RuntimeError(key+': '+sensor['error'])
                if not (sensor.get('managed') or sensor.get('publishers')):
                    pending = True
                    if key not in self.run['requested'] and sensor.get('can_start'):
                        self.state.sensor_manager.request(key, 'start')
                        self.run['requested'].append(key)
            if not pending: self.transition('mapping')
        elif phase == 'mapping':
            if not self.service('mapping'): return
            if self.home_pending is not None: return
            if not self.run.get('home'):
                if self.home and self.home.get('map_id') == self.map_id():
                    self.run['home'] = copy.deepcopy(self.home)
                else:
                    self.begin_home(start_mapping=False)
                    return
            elif self.map_id() != self.run['home']['map_id']:
                if self.run['home']['map_id'].startswith('saved:'):
                    raise RuntimeError('현재 지도와 홈 등록 지도가 다릅니다.')
                self.run['home'] = None
                return
            mapping = self.value('service_control').get('mapping', {})
            if mapping.get('mode') == 'localization' and self.value('localization').get('state') != 'localized': return
            if self.fresh('pose') and self.value('pose').get('frame') == 'map': self.transition('navigation')
        elif phase == 'navigation':
            if self.service('navigation') and self.fresh('navigation') and self.value('navigation').get('available'):
                self.transition('manipulation_ready')
        elif phase == 'manipulation_ready':
            if not self.manipulation.client or not self.manipulation.client.server_is_ready():
                self.service('manipulation'); return
            errors = self.manipulation.blockers(require_handoff=False)
            ready = self.manipulation.monitor.values.get('readiness', {})
            if ready.get('hardware_mode') is False:
                raise RuntimeError('팔 실행기가 읽기 전용입니다. 실물 프로파일로 실행기를 준비하세요.')
            if errors:
                self.run['message'] = '팔 준비 대기 · '+' / '.join(errors); return
            if not ready.get('table_plate_ready'):
                raise RuntimeError('접시 관측·팔 수납 프로파일 미설정: '+str(ready.get('table_plate_error', '실행기 업데이트 필요')))
            if self.run['label'] not in ready.get('enabled_classes', []):
                raise RuntimeError('실물 파지가 설정되지 않은 물체: '+self.run['label'])
            if not self.manipulation.monitor.values.get('wrist', {}).get('plate_ready'):
                raise RuntimeError('손목 SAM 접시 관측 실행기 미준비')
            self.state.sensor_manager.require_collision_ready()
            self.state.detection_request(dict(action='find_and_approach', label=self.run['label'],
                                              edge_clearance=0., task_id=self.run['id']))
            self.transition('detection')
        elif phase == 'detection':
            if not self.fresh('detection_control'): raise RuntimeError('도킹 제어 연결 지연')
            approach = self.value('detection_control')
            if approach.get('task_id') != self.run['id']: return
            if approach.get('phase') in ('error', 'canceled'): raise RuntimeError(approach.get('message', '접근 중단'))
            if approach.get('phase') != 'succeeded': return
            task = self.manipulation.candidate()
            if task is None: return
            self.run['manip_task'] = task
            self.manipulation.request(dict(action='run', task_id=task, operation='table_plate'))
            self.transition('manipulation')
        elif phase == 'manipulation':
            status = self.manipulation.monitor.values.get('status', {})
            if self.manipulation.last_error: raise RuntimeError(self.manipulation.last_error)
            if status.get('task_id') != self.run['manip_task']: return
            if status.get('outcome') in ('FAILED', 'CANCELED', 'REJECTED'):
                raise RuntimeError(status.get('reason', '팔 작업 실패'))
            if status.get('outcome') != 'SUCCESS': return
            if not all(status.get(k) is True for k in ('place_success', 'return_success', 'stop_confirmed')) or status.get('holding') != 'EMPTY':
                raise RuntimeError('접시 배치·팔 수납 완료 증거 누락')
            self.state.navigation_request(dict(action='departure', task_id=self.run['manip_task'],
                dock_id=self.run['id'], goal=self.run['home']))
            self.transition('departure')
        elif phase == 'departure':
            if not self.fresh('dock_departure'): raise RuntimeError('이탈 제어 연결 지연')
            departure = self.value('dock_departure')
            if departure.get('task_id') != self.run['manip_task']: return
            self.run['message'] = departure.get('message', '홈 복귀 중')
            if departure.get('phase') in ('blocked', 'canceled'): raise RuntimeError(self.run['message'])
            if departure.get('phase') == 'done': self.transition('succeeded')
        if phase not in ('sensors', 'mapping') and self.map_id() != self.run['home']['map_id']:
            raise RuntimeError('작업 중 지도 변경 · 정지')

    def tick(self):
        with self.lock:
            requests = list(self.requests); self.requests.clear()
        for req in requests:
            try: self.handle(req)
            except Exception as exc:
                self.state.log('전체 Task 요청 거부 · '+str(exc), 'warn', 'task')
                if not self.run['active']: self.run['message'] = str(exc)
        if self.home_pending is not None:
            try: self.advance_home()
            except Exception as exc:
                self.home_pending = None
                self.home_message = str(exc)
                self.state.log('홈 등록 실패 · '+str(exc), 'warn', 'task')
                if self.run['active']: self.cancel('failed', str(exc))
        if self.run['active']:
            try: self.advance()
            except Exception as exc: self.cancel('failed', str(exc))
        self.state.put('task', dict(self.run, home=self.home, current_map_id=self.map_id(),
            home_pending=self.home_pending is not None, home_message=self.home_message))
        self.heartbeat.publish(self.String(data=json.dumps(dict(stamp=self.node.get_clock().now().nanoseconds/1e9,
            active=self.run['active'], task_id=self.run.get('manip_task'), phase=self.run['phase']))))
