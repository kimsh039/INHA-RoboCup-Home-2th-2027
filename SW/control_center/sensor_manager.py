"""Own separate ROS driver process groups; never kill unrelated ROS processes."""
from datetime import datetime
import os
from pathlib import Path
import queue
import signal
import subprocess
import threading
import time

ROOT = Path(__file__).resolve().parent
SENSORS = {
    'head_camera': ('Head D435', '/head/camera/color/image_raw', 'head_image'),
    'wrist_camera': ('Wrist D435', '/wrist/wrist/color/image_raw', 'wrist_image'),
    'g2_lidar': ('YDLIDAR G2', '/scan_raw', 'scan'),
    'mid360_lidar': ('Livox MID-360', '/livox/lidar', 'cloud'),
}


class SensorManager:
    def __init__(self, state):
        self.state = state
        self.config = state.config.get('sensors', {})
        self.lock = threading.RLock()
        self.queue = queue.Queue()
        self.closed = threading.Event()
        self.jobs = {}
        self.exiting = set()
        self.status = {key: dict(label=label, state='off', error='', publishers=0, topic=topic)
                       for key, (label, topic, _) in SENSORS.items()}
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
            return ['ros2', 'run', 'livox_ros_driver2', 'livox_ros_driver2_node', '--ros-args',
                    '-r', '__node:=livox_lidar_publisher', '-p', 'xfer_format:=0', '-p', 'multi_topic:=0',
                    '-p', 'data_src:=0', '-p', 'publish_freq:=10.0', '-p', 'output_data_type:=0',
                    '-p', 'frame_id:=livox_frame', '-p', 'user_config_path:=' + path,
                    '-p', 'cmdline_input_bd_code:=livox0000000001']
        raise ValueError('알 수 없는 센서입니다.')

    @staticmethod
    def _path(value):
        path = Path(os.path.expandvars(str(value))).expanduser().resolve()
        if not path.is_file():
            raise ValueError('센서 설정 파일이 없습니다: ' + str(path))
        return str(path)

    def start_auxiliary(self, key, command):
        with self.lock:
            self._spawn(key, command)

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
                else:
                    with self.lock:
                        if key in self.status:
                            previous_error = self.status[key]['error']
                            self.status[key].update(state='error' if previous_error else 'off')
                    self.state.log(key + ' 드라이버 중지', source='sensor-control')
            except (OSError, ValueError, subprocess.SubprocessError) as exc:
                with self.lock:
                    if key in self.status:
                        self.status[key].update(state='error', error=str(exc))
                self.state.log(f'{key}: {exc}', 'error', 'sensor-control')
            with self.lock:
                self.exiting.discard(key)
                self._publish()

    def observe(self, publishers):
        # Called by the ROS graph timer. Publisher presence and data reception differ.
        with self.lock:
            for key, (_, topic, _) in SENSORS.items():
                self.status[key]['publishers'] = len(publishers.get(topic, []))
            self._publish()

    def refresh(self):
        with self.lock:
            for key, process in list(self.jobs.items()):
                code = process.poll()
                if code is None:
                    continue
                # Keep ownership until cleanup has signalled any remaining children.
                if key in SENSORS and self.status[key]['state'] != 'stopping':
                    self.status[key].update(state='stopping', error=f'드라이버 종료 코드 {code}')
                    self.queue.put((key, 'stop', None))
                    self.state.log(f'{key} 종료 코드 {code} · 로그 {self.logs / (key + ".log")}', 'error', 'sensor-control')
                elif key not in SENSORS and key not in self.exiting:
                    self.state.log(f'{key} 프로세스 종료 코드 {code} · 로그 {self.logs / (key + ".log")}', 'error', 'sensor-control')
                    self.exiting.add(key)
                    self.queue.put((key, 'stop', None))
            self._publish()

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
        self.state.put('sensor_control', controls)

    def close(self):
        self.closed.set()
        self.queue.put(None)
        self.worker.join(timeout=15)
        with self.lock:
            jobs, self.jobs = list(self.jobs.items()), {}
        # Stop inference and mapping before sensors and the physical TF/odom core.
        jobs.sort(key=lambda item: 2 if item[0] == 'robot_core' else 1 if item[0] in SENSORS else 0)
        for key, process in jobs:
            try:
                self._stop(process)
            except (OSError, subprocess.SubprocessError) as exc:
                self.state.log(f'{key} 종료 오류: {exc}', 'error', 'sensor-control')
