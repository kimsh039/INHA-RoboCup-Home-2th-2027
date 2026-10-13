"""Own separate ROS driver process groups; never kill unrelated ROS processes."""
from datetime import datetime
from concurrent.futures import ThreadPoolExecutor
import os
from pathlib import Path
import queue
import re
import signal
import subprocess
import sys
import threading
import time

ROOT = Path(__file__).resolve().parent
SENSORS = {
    'head_camera': ('Head D435', '/head/camera/color/image_raw', 'head_image'),
    'wrist_camera': ('Wrist D435', '/wrist/wrist/color/image_raw', 'wrist_image'),
    'g2_lidar': ('YDLIDAR G2', '/scan_raw', 'scan'),
    'mid360_lidar': ('Livox MID-360', '/livox/lidar', 'cloud'),
}
# Owned SLAM / Nav2 process groups: (label, topic whose publishers show it is running).
SERVICES = {
    'mapping': ('SLAM Toolbox', '/map'),
    'navigation': ('Nav2', '/local_costmap/costmap'),
}
MAP_DIR = Path(os.environ.get('ROBOCUP_MAP_DIR', str(Path.home() / 'maps')))
MAP_NAME = re.compile(r'[A-Za-z0-9_-]{1,48}')
NAV2_SHARE = Path('/opt/ros/humble/share/nav2_bringup')


class SensorManager:
    def __init__(self, state):
        self.state = state
        self.config = state.config.get('sensors', {})
        self.lock = threading.RLock()
        self.queue = queue.Queue()
        self.closed = threading.Event()
        self.jobs = {}
        self.log_offsets = {}
        self.exiting = set()
        self.status = {key: dict(label=label, state='off', error='', publishers=0, topic=topic)
                       for key, (label, topic, _) in SENSORS.items()}
        self.services = {key: dict(label=label, state='off', error='', publishers=0, topic=topic, mode='', map='')
                         for key, (label, topic) in SERVICES.items()}
        self.saving = None
        self.last_save = None
        self.maps = []
        self.maps_checked = 0
        directory = os.environ.get('XDG_STATE_HOME', str(Path.home() / '.local/state'))
        self.logs = Path(directory) / 'inha-control-center' / (datetime.now().strftime('%Y%m%d-%H%M%S') + '-' + str(os.getpid()))
        self.logs.mkdir(parents=True, exist_ok=True)
        self.worker = threading.Thread(target=self._work, name='sensor-control', daemon=True)
        self.worker.start()
        self._publish()

    def command(self, key):
        settings = self.config.get(key, {})
        if key in ('head_camera', 'wrist_camera'):
            head = key == 'head_camera'
            namespace, camera = ('head', 'camera') if head else ('wrist', 'wrist')
            serial = str(settings.get('serial', '244622071116' if head else '243222070076'))
            fps = int(settings.get('fps', 15 if head else 30))
            if not serial.isdigit() or not 1 <= fps <= 60:
                raise ValueError('카메라 serial은 숫자, FPS는 1~60으로 설정하세요.')
            return ['ros2', 'launch', 'realsense2_camera', 'rs_launch.py',
                    'camera_namespace:=' + namespace, 'camera_name:=' + camera, 'serial_no:=_' + serial,
                    f'rgb_camera.color_profile:=640,480,{fps}', f'depth_module.depth_profile:=640,480,{fps}',
                    'enable_color:=true', 'enable_depth:=true', 'align_depth.enable:=true',
                    'enable_gyro:=false', 'enable_accel:=false', 'pointcloud.enable:=false', 'publish_tf:=true']
        if key == 'g2_lidar':
            path = self._path(settings.get('params_file', '~/tracer_ws/src/ydlidar_ros2_driver/params/G2.yaml'))
            return ['ros2', 'run', 'ydlidar_ros2_driver', 'ydlidar_ros2_driver_node',
                    '--ros-args', '--params-file', path, '-r', 'scan:=/scan_raw']
        if key == 'mid360_lidar':
            path = self._path(settings.get('config_file', '~/ws_livox/src/livox_ros_driver2/config/MID360s_config.json'))
            return [sys.executable, str(ROOT / 'start-mid360.py'), '--config', path,
                    '--connection', str(settings.get('connection', 'livox'))]
        raise ValueError('알 수 없는 센서입니다.')

    def service_command(self, key, mode='mapping', map_name=''):
        if key == 'mapping':
            if mode == 'localization':
                base = self.map_base(map_name)
                if not base.with_suffix('.posegraph').is_file():
                    raise ValueError('저장된 pose graph가 없습니다: ' + str(base) + '.posegraph')
                return ['bash', str(ROOT / 'start-localization.sh'), str(base)]
            return ['bash', str(ROOT / 'start-mapping.sh')]
        if key == 'navigation':
            if not NAV2_SHARE.is_dir():
                raise ValueError('Nav2가 설치되지 않았습니다. 젯슨에서 sudo apt-get install -y ros-humble-navigation2 ros-humble-nav2-bringup')
            return ['bash', str(ROOT / 'start-navigation.sh')]
        raise ValueError('알 수 없는 기능입니다.')

    @staticmethod
    def map_base(name):
        if not isinstance(name, str) or not MAP_NAME.fullmatch(name):
            raise ValueError('지도 이름은 영문·숫자·_·- 1~48자로 지정하세요.')
        return MAP_DIR / name

    def list_maps(self):
        maps = {}
        try:
            for path in MAP_DIR.iterdir():
                if path.suffix in ('.posegraph', '.yaml') and MAP_NAME.fullmatch(path.stem):
                    item = maps.setdefault(path.stem, dict(name=path.stem, posegraph=False, image=False, time=0))
                    item['posegraph' if path.suffix == '.posegraph' else 'image'] = True
                    item['time'] = max(item['time'], path.stat().st_mtime)
        except OSError:
            pass
        return sorted(maps.values(), key=lambda item: -item['time'])

    def _service_running(self, key):
        process = self.jobs.get(key)
        return process is not None and process.poll() is None

    def service_request(self, key, action, map_name=''):
        if key not in SERVICES or action not in ('start', 'stop', 'reset', 'localize'):
            raise ValueError('지원하는 기능과 동작을 지정하세요.')
        if action in ('reset', 'localize') and key != 'mapping':
            raise ValueError('초기화와 위치추정은 SLAM에서만 지원합니다.')
        with self.lock:
            if self.closed.is_set():
                raise RuntimeError('관제센터 종료 중입니다.')
            current = self.services[key]
            running = self._service_running(key)
            if current['state'] in ('starting', 'stopping'):
                raise RuntimeError(current['label'] + '의 이전 요청을 처리 중입니다.')
            if key == 'mapping' and self._service_running('navigation'):
                # Replacing map -> odom under a running Nav2 would move its goal frame.
                raise RuntimeError('Nav2 실행 중에는 SLAM을 바꾸거나 끌 수 없습니다. 먼저 Nav2를 중지하세요.')
            mode, name = 'mapping', ''
            if action == 'start':
                if running:
                    raise RuntimeError('이미 관제센터에서 실행 중입니다.')
                if current['publishers']:
                    raise RuntimeError('외부에서 실행 중인 ' + current['label'] + '이 있습니다. 그 실행을 종료한 뒤 켜세요.')
                if key == 'navigation' and not (self._service_running('mapping') or self.services['mapping']['publishers']):
                    raise RuntimeError('SLAM 또는 저장된 지도 위치추정을 먼저 시작하세요.')
            elif action == 'reset':
                if not running:
                    raise RuntimeError('관제센터가 실행한 SLAM만 초기화할 수 있습니다.')
                mode, name = current['mode'] or 'mapping', current['map']
            elif action == 'localize':
                if not running and current['publishers']:
                    raise RuntimeError('외부에서 실행 중인 SLAM이 있습니다. 그 실행을 종료한 뒤 켜세요.')
                mode, name = 'localization', map_name
            elif not running:
                raise RuntimeError('관제센터가 실행한 ' + current['label'] + '만 끌 수 있습니다.')
            command = None if action == 'stop' else self.service_command(key, mode, name)
            current.update(state='stopping' if action == 'stop' else 'starting', error='',
                           pending=dict(mode=mode if key == 'mapping' else '', map=name if mode == 'localization' else ''))
            self.queue.put((key, 'stop' if action == 'stop' else 'start', command))
            self._publish()

    def navigation_available(self):
        with self.lock:
            return self._service_running('navigation') or bool(self.services['navigation']['publishers'])

    def save_map(self, name):
        base = self.map_base(name)
        with self.lock:
            if self.saving:
                raise RuntimeError('지도를 저장하는 중입니다.')
            if not (self._service_running('mapping') or self.services['mapping']['publishers']):
                raise RuntimeError('실행 중인 SLAM이 없습니다.')
            self.saving = name
            self._publish()
        threading.Thread(target=self._save, args=(name, base), name='map-save', daemon=True).start()

    def _save(self, name, base):
        log = self.logs / 'map_save.log'
        try:
            MAP_DIR.mkdir(parents=True, exist_ok=True)
            with log.open('ab', buffering=0) as output:
                result = subprocess.run(['bash', str(ROOT / 'save-map.sh'), name], cwd=ROOT, stdin=subprocess.DEVNULL,
                                        stdout=output, stderr=subprocess.STDOUT, timeout=90,
                                        env=dict(os.environ, ROBOCUP_MAP_DIR=str(MAP_DIR)))
            ok = (result.returncode == 0 and base.with_suffix('.posegraph').is_file()
                  and base.with_suffix('.yaml').is_file())
            message = f'지도 저장 {"완료" if ok else "실패"} · {base}' + (
                '' if ok else f' · 종료 코드 {result.returncode} · 로그 {log}')
        except (OSError, subprocess.SubprocessError) as exc:
            ok, message = False, f'지도 저장 실패 · {exc}'
        self.state.log(message, 'info' if ok else 'error', 'mapping')
        with self.lock:
            self.saving = None
            self.last_save = dict(name=name, ok=ok, message=message, time=time.time())
            self.maps, self.maps_checked = self.list_maps(), time.monotonic()
            self._publish()

    @staticmethod
    def _path(value):
        path = Path(os.path.expandvars(str(value))).expanduser().resolve()
        if not path.is_file():
            raise ValueError('센서 설정 파일이 없습니다: ' + str(path))
        return str(path)

    def start_auxiliary(self, key, command):
        with self.lock:
            self._spawn(key, command)

    def replace_robot_core(self, command):
        """Reconnect only our own physical feedback process group."""
        with self.lock:
            if self.closed.is_set():
                return
            process = self.jobs.get('robot_core')
            if process is not None:
                self._stop(process)
                self.jobs.pop('robot_core', None)
            self.exiting.discard('robot_core')
            if command is not None:
                self._spawn('robot_core', command)

    def request(self, key, action):
        if key not in SENSORS or action not in ('start', 'stop'):
            raise ValueError('지원하는 센서와 start/stop을 지정하세요.')
        with self.lock:
            if self.closed.is_set():
                raise RuntimeError('관제센터 종료 중입니다.')
            current = self.status[key]
            process = self.jobs.get(key)
            running = process is not None and process.poll() is None
            if current['state'] in ('starting', 'stopping'):
                raise RuntimeError('이 센서의 이전 요청을 처리 중입니다.')
            if action == 'start':
                if running:
                    raise RuntimeError('이미 관제센터에서 실행 중입니다.')
                if current['publishers']:
                    raise RuntimeError('외부 드라이버 실행 중입니다. 기존 센서 실행을 종료한 뒤 관제센터에서 켜세요.')
                command = self.command(key)
            else:
                if not running:
                    raise RuntimeError('관제센터가 시작한 드라이버만 끌 수 있습니다. README의 기존 실행 전환 절차를 사용하세요.')
                command = None
            current.update(state='starting' if action == 'start' else 'stopping', error='')
            self.queue.put((key, action, command))
            self._publish()

    def _spawn(self, key, command):
        log = self.logs / (key + '.log')
        with log.open('ab', buffering=0) as output:
            self.log_offsets[key] = output.tell()
            process = subprocess.Popen(command, cwd=ROOT, stdin=subprocess.DEVNULL,
                                       stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
        self.jobs[key] = process
        self.state.log(f'{key} 실행 요청 · PID {process.pid} · 로그 {log}', source='sensor-control')
        return process

    @staticmethod
    def _group_exists(process):
        try:
            os.killpg(process.pid, 0)
            return True
        except ProcessLookupError:
            return False

    @staticmethod
    def _stop(process):
        # The process was launched in its own session. Signal only that owned group.
        for sig, seconds in ((signal.SIGINT, 6), (signal.SIGTERM, 3), (signal.SIGKILL, 2)):
            try:
                os.killpg(process.pid, sig)
            except ProcessLookupError:
                process.wait()
                return
            try:
                process.wait(timeout=seconds)
            except subprocess.TimeoutExpired:
                continue
            if not SensorManager._group_exists(process):
                return
        process.wait(timeout=2)

    def _work(self):
        while True:
            item = self.queue.get()
            if item is None:
                return
            key, action, command = item
            if self.closed.is_set():
                continue
            try:
                with self.lock:
                    old = self.jobs.pop(key, None)
                if old:
                    self._stop(old)
                if action == 'start':
                    with self.lock:
                        if self.closed.is_set():
                            continue
                        self._spawn(key, command)
                        if key in self.status:
                            self.status[key].update(state='running', error='')
                        if key in self.services:
                            current = self.services[key]
                            current.update(state='running', error='', **current.pop('pending', {}))
                    if key == 'mapping':
                        # A new SLAM process starts a new map; drop the previous image from the browser.
                        self.state.clear('map')
                else:
                    with self.lock:
                        if key in self.status:
                            previous_error = self.status[key]['error']
                            self.status[key].update(state='error' if previous_error else 'off')
                        if key in self.services:
                            current = self.services[key]
                            current.pop('pending', None)
                            current.update(state='error' if current['error'] else 'off', mode='', map='')
                    self.state.log(key + (' 중지' if key in self.services else ' 드라이버 중지'), source='sensor-control')
            except (OSError, ValueError, subprocess.SubprocessError) as exc:
                with self.lock:
                    if key in self.status:
                        self.status[key].update(state='error', error=str(exc))
                    if key in self.services:
                        self.services[key].pop('pending', None)
                        self.services[key].update(state='error', error=str(exc))
                self.state.log(f'{key}: {exc}', 'error', 'sensor-control')
            with self.lock:
                self.exiting.discard(key)
                self._publish()

    def observe(self, publishers):
        # Called by the ROS graph timer. Publisher presence and data reception differ.
        with self.lock:
            for key, (_, topic, _) in SENSORS.items():
                self.status[key]['publishers'] = len(publishers.get(topic, []))
            for key, (_, topic) in SERVICES.items():
                self.services[key]['publishers'] = len(publishers.get(topic, []))
            self._publish()

    def refresh(self):
        with self.lock:
            for key, process in list(self.jobs.items()):
                code = process.poll()
                details = self._log_tail(key) if key in SENSORS or code is not None else ''
                if key == 'mid360_lidar' and code is None and self.status[key]['state'] != 'stopping':
                    if 'Init lds lidar fail!' in details or 'Failed to init livox lidar sdk' in details:
                        self.status[key].update(state='stopping', error='Livox SDK 초기화 실패 · ' + details[-700:])
                        self.queue.put((key, 'stop', None))
                        self.state.log('MID-360 초기화 실패 · 드라이버 정리 · ' + details[-700:], 'error', 'sensor-control')
                if code is None:
                    continue
                # Keep ownership until cleanup has signalled any remaining children.
                if key in SENSORS and self.status[key]['state'] != 'stopping':
                    self.status[key].update(state='stopping', error=f'드라이버 종료 코드 {code} · ' + details[-700:])
                    self.queue.put((key, 'stop', None))
                    self.state.log(f'{key} 종료 코드 {code} · 로그 {self.logs / (key + ".log")}', 'error', 'sensor-control')
                elif key in SERVICES and key not in self.exiting:
                    if self.services[key]['state'] != 'stopping':
                        self.services[key].update(state='error', error=f'종료 코드 {code} · ' + details[-600:])
                        self.state.log(f'{key} 종료 코드 {code} · 로그 {self.logs / (key + ".log")}', 'error', 'sensor-control')
                    self.exiting.add(key)
                    self.queue.put((key, 'stop', None))
                elif key == 'robot_core' and key not in self.exiting:
                    # The CAN supervisor alone replaces this group; queued cleanup
                    # could otherwise kill a newly reconnected feedback process.
                    self.state.log(f'{key} 프로세스 종료 코드 {code} · CAN 감독기가 재연결합니다.', 'error', 'can')
                    self.exiting.add(key)
                elif key not in SENSORS and key not in self.exiting:
                    self.state.log(f'{key} 프로세스 종료 코드 {code} · {details[-600:]} · 로그 {self.logs / (key + ".log")}', 'error', 'sensor-control')
                    self.exiting.add(key)
                    self.queue.put((key, 'stop', None))
            if time.monotonic() - self.maps_checked > 5:
                self.maps, self.maps_checked = self.list_maps(), time.monotonic()
            self._publish()

    def _log_tail(self, key):
        try:
            with (self.logs / (key + '.log')).open('rb') as output:
                end = output.seek(0, os.SEEK_END)
                output.seek(max(self.log_offsets.get(key, 0), end - 8192))
                return output.read().decode('utf-8', errors='replace').strip()
        except OSError:
            return ''

    def _publish(self):
        controls = {}
        with self.state.condition:
            for key, (_, _, channel) in SENSORS.items():
                current = dict(self.status[key])
                process = self.jobs.get(key)
                owned = process is not None and process.poll() is None
                busy = current['state'] in ('starting', 'stopping')
                if not owned and not busy and current['publishers']:
                    current['state'] = 'external'
                elif not owned and not busy and current['state'] == 'external':
                    current['state'] = 'off'
                meta = self.state.channels.get(channel, {})
                received = meta.get('received')
                current.update(managed=owned, pid=process.pid if owned else None,
                               receiving=owned and received is not None and time.monotonic() - received < 3,
                               can_start=not owned and not busy and not current['publishers'] and not self.closed.is_set(),
                               can_stop=owned and not busy and not self.closed.is_set(),
                               log=str(self.logs / (key + '.log')))
                controls[key] = current
            services = {}
            for key, current in self.services.items():
                current = {k: v for k, v in current.items() if k != 'pending'}
                process = self.jobs.get(key)
                owned = process is not None and process.poll() is None
                busy = current['state'] in ('starting', 'stopping')
                if not owned and not busy and current['publishers']:
                    current['state'] = 'external'
                elif not owned and not busy and current['state'] == 'external':
                    current['state'] = 'off'
                current.update(managed=owned, pid=process.pid if owned else None, busy=busy,
                               log=str(self.logs / (key + '.log')))
                services[key] = current
            services['maps'] = dict(directory=str(MAP_DIR), items=self.maps, saving=self.saving,
                                    last=self.last_save, nav2_installed=NAV2_SHARE.is_dir())
        self.state.put('sensor_control', controls)
        self.state.put('service_control', services)

    def close(self):
        self.closed.set()
        self.queue.put(None)
        self.worker.join(timeout=15)
        with self.lock:
            jobs, self.jobs = list(self.jobs.items()), {}
        # Every job owns a separate process group; signal them together on shutdown.
        def stop_job(item):
            key, process = item
            try:
                self._stop(process)
            except (OSError, subprocess.SubprocessError) as exc:
                self.state.log(f'{key} 종료 오류: {exc}', 'error', 'sensor-control')
        with ThreadPoolExecutor(max_workers=max(1, len(jobs))) as executor:
            list(executor.map(stop_job, jobs))
