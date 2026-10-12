"""Prepare CAN links and supervise owned feedback nodes without sending motion."""
from collections import deque
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import threading
import time

from can_devices import devices


class CanMonitor:
    def __init__(self, state, manager, with_robot):
        self.state, self.manager = state, manager
        self.settings = state.config.get('can', {})
        self.active = with_robot and self.settings.get('auto_connect', True)
        self.with_robot = with_robot
        self.closed = threading.Event()
        self.previous_rx, self.last_rx, self.next_attempt = {}, {}, {}
        self.last_status, self.core_signature = {}, None
        self.next_core_attempt = 0
        self.ip = shutil.which('ip')
        self.links = {}
        for role, interface, bitrate in [('tracer', 'can_tracer', 500000), ('piper', 'can_arm', 1000000)]:
            entry = self.settings.get(role, {})
            interface = str(entry.get('interface', interface))
            bitrate = int(entry.get('bitrate', bitrate))
            if not re.fullmatch(r'can[A-Za-z0-9_]{1,11}', interface) or not 10000 <= bitrate <= 1000000:
                raise ValueError('CAN 인터페이스 이름 또는 bitrate 설정이 잘못됐습니다.')
            self.links[role] = (interface, bitrate)
        if self.links['tracer'][0] == self.links['piper'][0]:
            raise ValueError('Tracer와 PiPER에 서로 다른 CAN 인터페이스를 지정하세요.')
        self.worker = threading.Thread(target=self._work, name='can-monitor', daemon=True)
        self.worker.start()

    def _read(self, role, interface, bitrate):
        status = dict(interface=interface, expected_bitrate=bitrate, up=False, ready=False,
                      receiving=False, rx_age=None, rx_packets=None, bitrate=None, error='', state='missing')
        driver = (Path('/sys/class/net') / interface / 'device/driver').resolve().name
        status['hardware'] = {'gs_usb': 'USB CAN', 'mttcan': 'Jetson 내장 CAN'}.get(driver, 'CAN 장치')
        entry = self.settings.get(role, {})
        if 'usb_serial' in entry:
            expected = str(entry['usb_serial'])
            connected = devices()
            actual = next((device for device in connected if device['interface'] == interface), None)
            status.update(usb_serial=actual['usb_serial'] if actual else '', expected_serial=expected)
            if actual and actual['usb_path']:
                status['hardware'] = 'USB CAN · serial ' + (actual['usb_serial'] or '없음')
            if not expected:
                status.update(state='error', error='USB CAN 역할 미등록 · 두 어댑터 연결 후 setup-can.sh 실행 필요')
                return status
            if actual is None:
                located = next((device for device in connected if device['usb_serial'] == expected), None)
                status['error'] = ('USB CAN 발견(' + located['interface'] + ') · setup-can.sh로 이름 고정 필요'
                                   if located else '등록된 USB CAN 장치 없음 · USB 연결 확인 필요')
                return status
            if actual['usb_serial'] != expected:
                status.update(state='error', error='다른 로봇의 USB CAN이 해당 이름을 사용 중 · setup-can.sh 실행 필요')
                return status
        if not self.ip:
            status.update(state='error', error='iproute2가 필요합니다. setup-jetson.sh를 실행하세요.')
            return status
        result = subprocess.run([self.ip, '-details', '-json', 'link', 'show', 'dev', interface],
                                capture_output=True, text=True, timeout=2)
        if result.returncode:
            self.previous_rx.pop(role, None)
            self.last_rx.pop(role, None)
            status['error'] = 'CAN 장치 없음 · 전원과 연결을 확인하세요.'
            return status
        link = json.loads(result.stdout)[0]
        info = link.get('linkinfo', {})
        if info.get('info_kind') != 'can':
            status.update(state='error', error='해당 인터페이스가 CAN 장치가 아닙니다.')
            return status
        details = info.get('info_data', {})
        status.update(up='UP' in link.get('flags', []), bitrate=details.get('bittiming', {}).get('bitrate'),
                      bus_state=details.get('state', 'UNKNOWN'), ifindex=link.get('ifindex'))
        now = time.monotonic()
        rx = int((Path('/sys/class/net') / interface / 'statistics/rx_packets').read_text())
        previous = self.previous_rx.get(role)
        identity = (status['ifindex'], rx)
        if previous and previous[0] == identity[0] and rx > previous[1] and status['up']:
            self.last_rx[role] = now
        elif previous is None or previous[0] != identity[0] or rx < previous[1] or not status['up']:
            self.last_rx.pop(role, None)
        self.previous_rx[role] = identity
        age = now - self.last_rx[role] if role in self.last_rx else None
        status.update(rx_packets=rx, rx_age=age, receiving=age is not None and age <= 3)
        if status['bus_state'] == 'BUS-OFF':
            status.update(state='bus_off', error='BUS-OFF · 배선/종단저항/bitrate 확인 필요')
        elif not status['up'] or status['bus_state'] == 'STOPPED':
            status['state'] = 'down'
        elif status['bitrate'] != bitrate:
            status.update(state='mismatch', error='bitrate 불일치 · 관제센터 kill 후 setup-can.sh 실행 필요')
        else:
            status.update(state='up', ready=True)
        return status

    def _prepare(self, role, status):
        now = time.monotonic()
        if not self.active or status['state'] not in ('down', 'bus_off') or now < self.next_attempt.get(role, 0):
            return
        self.next_attempt[role] = now + 5
        interface, bitrate = self.links[role]
        if status['state'] == 'bus_off':
            if status['bitrate'] != bitrate:
                return
            args = ['link', 'set', 'dev', interface, 'type', 'can', 'restart']
        else:
            args = ['link', 'set', 'dev', interface, 'up', 'type', 'can', 'bitrate', str(bitrate), 'restart-ms', '100']
        command = [self.ip] + args
        if os.geteuid() != 0:
            command = ['sudo', '-n'] + command
        result = subprocess.run(command, capture_output=True, text=True, timeout=2)
        if result.returncode:
            status['error'] = ('CAN 자동 연결 권한/설정 오류 · 최초 1회 bash SW/control_center/setup-can.sh 실행 필요. '
                               + result.stderr.strip())[:400]
        else:
            self.state.log(interface + ' CAN 연결/복구 요청', source='can')

    def _robot(self, statuses):
        if not self.with_robot or self.closed.is_set():
            return
        tracer, piper = statuses['tracer'], statuses['piper']
        signature = (tracer.get('ifindex') if tracer['ready'] else None,
                     piper.get('ifindex') if piper['ready'] else None)
        if signature == (None, None):
            signature = None
        with self.manager.lock:
            process = self.manager.jobs.get('robot_core')
            running = process is not None and process.poll() is None
        if signature == self.core_signature and (signature is None or running):
            return
        now = time.monotonic()
        if now < self.next_core_attempt:
            return
        self.next_core_attempt = now + 5
        command = None
        if signature is not None:
            command = ['ros2', 'launch', str(Path(__file__).resolve().parent / '../robot/physical.launch.py'),
                       'head_camera:=false', 'wrist_camera:=false', 'g2_lidar:=false', 'mid360_lidar:=false',
                       'calibration:=false', 'tracer:=' + str(tracer['ready']).lower(),
                       'tracer_can:=' + tracer['interface'], 'piper_can:=' + piper['interface'],
                       'arm:=' + str(piper['ready']).lower()]
        self.manager.replace_robot_core(command)
        self.core_signature = signature
        self.state.log('CAN 상태에 따라 차체/팔 피드백 ' + ('시작 요청' if command else '중지 요청'), source='can')

    def _work(self):
        while not self.closed.is_set():
            statuses = {}
            for role, (interface, bitrate) in self.links.items():
                try:
                    status = self._read(role, interface, bitrate)
                    self._prepare(role, status)
                except (OSError, ValueError, KeyError, IndexError, subprocess.SubprocessError) as exc:
                    status = dict(interface=interface, expected_bitrate=bitrate, ready=False,
                                  receiving=False, state='error', error=str(exc)[:400])
                statuses[role] = status
                identity = (status['state'], status.get('bus_state'), status.get('error', ''))
                if identity != self.last_status.get(role):
                    self.last_status[role] = identity
                    self.state.log(interface + ': ' + status['state'] + ' ' + status.get('bus_state', '') + ' ' + status.get('error', ''),
                                   'warn' if not status['ready'] else 'info', 'can')
            try:
                self._robot(statuses)
            except (OSError, subprocess.SubprocessError) as exc:
                self.state.log('CAN 피드백 실행 오류: ' + str(exc), 'error', 'can')
            self.state.put('can_status', dict(automatic=self.active, interfaces=statuses))
            self.closed.wait(1)

    def close(self):
        self.closed.set()
        self.worker.join()


class OdometryMonitor:
    """Track the two actual odometry sources independently of UI topic selection."""
    def __init__(self, state):
        self.state = state
        self.samples = {key: dict(arrivals=deque(maxlen=100), received=None, advancing=None,
                                  stamp=None, valid=False, value={}) for key in ('wheel', 'fused')}
        self.rotation = {key: dict(received=None, stamp=None, valid=False, angular=None)
                         for key in ('imu', 'wheel_fallback')}
        self.last_summary = None

    def receive(self, key, msg, ros_now):
        sample = self.samples[key]
        now = time.monotonic()
        p, q, twist = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        numbers = [p.x, p.y, p.z, q.x, q.y, q.z, q.w,
                   twist.linear.x, twist.linear.y, twist.linear.z,
                   twist.angular.x, twist.angular.y, twist.angular.z]
        valid = (all(math.isfinite(v) for v in numbers) and
                 0.5 <= math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w) <= 1.5 and
                 msg.header.frame_id == 'odom' and msg.child_frame_id == self.state.config.get('base_frame', 'base_link'))
        if stamp > 0 and (sample['stamp'] is None or stamp > sample['stamp']):
            sample['advancing'] = now
        elif sample['stamp'] is not None and stamp < sample['stamp']:
            sample['advancing'] = None
        sample.update(received=now, stamp=stamp, source_age=ros_now-stamp, valid=valid,
                      value=dict(x=p.x, y=p.y, linear=twist.linear.x, angular=twist.angular.z,
                                 yaw=math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z)),
                                 frame=msg.header.frame_id, child_frame=msg.child_frame_id))
        sample['arrivals'].append(now)

    def receive_rotation(self, key, msg, ros_now):
        angular = msg.angular_velocity.z if key == 'imu' else msg.twist.twist.angular.z
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        frame = 'base_link' if key == 'imu' else 'odom'
        valid = math.isfinite(angular) and msg.header.frame_id == frame and stamp > 0
        self.rotation[key].update(received=time.monotonic(), stamp=stamp,
                                  valid=valid, angular=angular)

    def publish(self, ros_now, tf_status):
        now, readings = time.monotonic(), {}
        for key, sample in self.samples.items():
            age = None if sample['received'] is None else now - sample['received']
            source_age = None if sample['stamp'] is None else ros_now - sample['stamp']
            advancing = sample['advancing'] is not None and now - sample['advancing'] <= 3
            fresh = age is not None and age <= 3 and source_age is not None and -0.5 <= source_age <= 3
            recent = [t for t in sample['arrivals'] if now-t <= 3]
            hz = (len(recent)-1)/(recent[-1]-recent[0]) if len(recent)>1 and recent[-1]>recent[0] else 0
            status = 'waiting' if age is None else 'invalid' if not sample['valid'] else 'live' if fresh and advancing else 'stale'
            readings[key] = dict(sample['value'], state=status, age=age, source_age=source_age, hz=hz)
        with self.state.condition:
            can = self.state.values.get('can_status', {})
            meta = self.state.channels.get('can_status', {})
            can_fresh = meta.get('received') is not None and now-meta['received'] <= 3
            tracer = can.get('interfaces', {}).get('tracer', {})
        connected = can_fresh and tracer.get('ready') and tracer.get('receiving')
        healthy = (connected and tracer.get('bus_state') == 'ERROR-ACTIVE' and
                   all(r['state']=='live' for r in readings.values()) and tf_status['state']=='live')
        summary = 'ok' if healthy else 'waiting' if all(r['state']=='waiting' for r in readings.values()) else 'degraded'
        rotation = {}
        for key, sample in self.rotation.items():
            age = None if sample['received'] is None else now - sample['received']
            source_age = None if sample['stamp'] is None else ros_now - sample['stamp']
            live = (sample['valid'] and age is not None and age <= 0.5
                    and source_age is not None and -0.5 <= source_age <= 0.5)
            rotation[key] = dict(state='live' if live else 'waiting' if age is None else 'stale',
                                 age=age, source_age=source_age, angular=sample['angular'])
        rotation['source'] = ('imu' if rotation['imu']['state'] == 'live' else
                              'wheel_fallback' if rotation['wheel_fallback']['state'] == 'live' else 'waiting')
        self.state.put('odometry_health', dict(state=summary, wheel=readings['wheel'], fused=readings['fused'], tf=tf_status,
                                             rotation=rotation, can_receiving=bool(connected)))
        if summary != self.last_summary:
            self.last_summary = summary
            self.state.log('CAN·오도메트리: ' + ('CAN 수신·바퀴 odom·융합 odom·TF 수신 중' if healthy else
                           '입력 대기' if summary=='waiting' else '입력 누락/지연 · CAN·오도메트리 패널 참조'),
                           'info' if healthy else 'warn', 'odometry')
