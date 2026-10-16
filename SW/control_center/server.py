#!/usr/bin/env python3
"""Local HTTP/SSE transport. ROS is imported only by the live entry point."""
import argparse
import base64
from collections import deque
import json
import math
from pathlib import Path
import re
import secrets
import socket
import struct
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse
import zlib

from detection_approach import target_label

ROOT = Path(__file__).resolve().parent


def png_gray(width, height, pixels):
    def chunk(kind, data):
        return struct.pack('!I', len(data)) + kind + data + struct.pack('!I', zlib.crc32(kind + data) & 0xffffffff)
    rows = b''.join(b'\x00' + bytes(pixels[y * width:(y + 1) * width]) for y in range(height))
    return (b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('!2I5B', width, height, 8, 0, 0, 0, 0))
            + chunk(b'IDAT', zlib.compress(rows, 3)) + chunk(b'IEND', b''))


def finite_json(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {k: finite_json(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [finite_json(v) for v in value]
    return value


class State:
    def __init__(self, mode='offline', config=None):
        self.mode, self.config = mode, config or {}
        self.condition = threading.Condition(threading.RLock())
        self.closed = threading.Event()
        self.shutdown_requested = threading.Event()
        self.started = time.monotonic()
        self.values, self.versions, self.channels, self.frames = {}, {}, {}, {}
        self.logs = deque(maxlen=60)
        self.pending = {}
        self.sensor_manager = None
        self.navigation_request = None  # set by the ROS bridge; queues goal / cancel
        self.detection_request = None
        self.manipulation_request = None
        self.manipulation_interlock = lambda: False
        self.control_token = secrets.token_urlsafe(32)
        self.map_png = None
        self.clients = 0
        for key, candidates in self.config.get('channels', {}).items():
            self.channels[key] = dict(topic='', type='', candidates=candidates, error='',
                                      publishers=0, received=None, arrivals=deque(maxlen=40))

    def log(self, message, level='info', source='bridge'):
        with self.condition:
            self.logs.append(dict(time=time.time(), level=level, source=source, message=str(message)[:800]))

    def put(self, key, value, source_stamp=None, source_age=None, finite=False):
        # finite=True: caller already removed NaN/inf (large point arrays); skip the recursive scan under the lock.
        if not finite:
            value = finite_json(value)
        with self.condition:
            self.values[key] = value
            self.versions[key] = self.versions.get(key, 0) + 1
            meta = self.channels.setdefault(key, dict(topic='', type='', candidates=[], publishers=0,
                                                     received=None, arrivals=deque(maxlen=40)))
            now = time.monotonic()
            meta.update(received=now, stamp=source_stamp, source_age=source_age, error='')
            meta['arrivals'].append(now)
            self.condition.notify_all()

    def put_frame(self, key, data, mime, width, height, stamp=None, source_age=None):
        with self.condition:
            self.frames[key] = (data, mime)
            self.put(key, dict(width=width, height=height, mime=mime), stamp, source_age)

    def clear(self, key):
        with self.condition:
            self.values[key] = None
            self.versions[key] = self.versions.get(key, 0) + 1
            self.frames.pop(key, None)
            if key == 'map':
                self.map_png = None
            meta = self.channels[key]
            meta.update(received=None, stamp=None, source_age=None, error='')
            meta['arrivals'].clear()
            self.condition.notify_all()

    def error(self, key, error):
        with self.condition:
            previous = self.channels[key].get('error', '')
            self.channels[key]['error'] = str(error)[:300]
            if previous != str(error)[:300]:
                self.log(f'{key}: {error}', 'warn')

    def snapshot(self, known=None):
        known = known or {}
        with self.condition:
            now = time.monotonic()
            metadata = {}
            for key, channel in self.channels.items():
                m = {k: v for k, v in channel.items() if k not in ('received', 'arrivals', 'source_age')}
                received = channel.get('received')
                m['age'] = None if received is None else now - received
                a = channel.get('source_age')
                m['source_age'] = None if a is None or received is None else a + now - received
                arrivals = list(channel['arrivals'])
                m['hz'] = ((len(arrivals) - 1) / (arrivals[-1] - arrivals[0])
                           if len(arrivals) > 1 and now - arrivals[-1] < 3 and arrivals[-1] > arrivals[0] else 0)
                metadata[key] = m
            return dict(mode=self.mode, hostname=socket.gethostname(), uptime=now - self.started,
                        clients=self.clients, channels=metadata, versions=dict(self.versions),
                        changes={k: v for k, v in self.values.items() if known.get(k) != self.versions[k]},
                        logs=list(self.logs), fixed_frame=self.config.get('fixed_frame', 'map'),
                        control_token=self.control_token, shutting_down=self.shutdown_requested.is_set())


def make_server(state, host='127.0.0.1', port=8765):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def log_message(self, *args):
            pass

        def send(self, body, mime='application/json; charset=utf-8', status=200, download=None):
            self.send_response(status)
            self.send_header('Content-Type', mime)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            if download:
                self.send_header('Content-Disposition', f'attachment; filename="{download}"')
            self.end_headers()
            self.wfile.write(body)

        def send_json(self, value, status=200):
            self.send(json.dumps(finite_json(value), ensure_ascii=False, allow_nan=False).encode(), status=status)

        def do_GET(self):
            if urlparse('http://' + self.headers.get('Host', '')).hostname not in ('localhost', '127.0.0.1', '::1'):
                self.send_json({'error': 'Use a localhost SSH tunnel'}, 403)
                return
            path = urlparse(self.path).path
            if path == '/api/events':
                self.events()
            elif path.startswith('/api/stream/'):
                self.stream(path.rsplit('/', 1)[-1])
            elif path.startswith('/api/frame/'):
                with state.condition:
                    frame = state.frames.get(path.rsplit('/', 1)[-1])
                if frame:
                    self.send(frame[0], frame[1])
                else:
                    self.send_json({'error': 'Waiting for an image'}, 404)
            elif path == '/api/snapshot':
                self.send_json(state.snapshot())
            elif path == '/api/map.png':
                with state.condition:
                    png = state.map_png
                if png:
                    self.send(png, 'image/png', download='slam-map.png')
                else:
                    self.send_json({'error': 'No map has been received'}, 404)
            else:
                files = {'/': ('index.html', 'text/html; charset=utf-8'),
                         '/app.js': ('app.js', 'text/javascript; charset=utf-8'),
                         '/styles.css': ('styles.css', 'text/css; charset=utf-8')}
                if path not in files:
                    self.send_json({'error': 'Not found'}, 404)
                    return
                name, mime = files[path]
                self.send((ROOT / 'static' / name).read_bytes(), mime)

        def do_POST(self):
            path = urlparse(self.path).path
            if path not in ('/api/select', '/api/sensors', '/api/shutdown', '/api/mapping', '/api/navigation', '/api/detection', '/api/manipulation'):
                self.send_json({'error': 'Not found'}, 404)
                return
            if state.mode != 'live':
                self.send_json({'error': '실시간 ROS 모드에서만 제어할 수 있습니다.'}, 409)
                return
            origin = self.headers.get('Origin')
            host = self.headers.get('Host', '')
            if (urlparse('http://' + host).hostname not in ('localhost', '127.0.0.1', '::1')
                    or (origin and urlparse(origin).netloc != host)
                    or not secrets.compare_digest(self.headers.get('X-Control-Token', ''), state.control_token)
                    or self.headers.get('Content-Type', '').split(';')[0] != 'application/json'):
                self.send_json({'error': 'Origin mismatch'}, 403)
                return
            try:
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 < length <= 8192:
                    raise ValueError('Invalid request size')
                items = json.loads(self.rfile.read(length))
                if not isinstance(items, dict):
                    raise ValueError('Expected a channel/topic object')
                if state.manipulation_interlock() and (
                        path == '/api/shutdown'
                        or path == '/api/sensors'
                        or path == '/api/mapping'
                        or (path == '/api/navigation' and items.get('action') != 'cancel')
                        or (path == '/api/detection' and items.get('action') not in ('cancel',))):
                    raise RuntimeError('매니퓰 작업/베이스 잠금 유지 중입니다. 작업 상태와 정지를 확인하세요.')
                if path == '/api/shutdown':
                    if items != {'action': 'shutdown'}:
                        raise ValueError('action=shutdown을 지정하세요.')
                    self.send_json({'state': 'shutting_down'}, 202)
                    state.log('웹에서 서버 종료 요청 · 센서·SLAM·검출 프로세스 정리', source='server')
                    state.shutdown_requested.set()
                    with state.condition:
                        state.condition.notify_all()
                    return
                if state.shutdown_requested.is_set():
                    self.send_json({'error': '관제센터 종료 중입니다.'}, 409)
                    return
                if path == '/api/manipulation':
                    if set(items) != {'action'} or items.get('action') not in ('run', 'cancel', 'arm'):
                        raise ValueError('action은 arm/run/cancel 중 하나입니다.')
                    if not state.manipulation_request:
                        raise RuntimeError('매니퓰 ROS 연결이 준비되지 않았습니다.')
                    state.manipulation_request(dict(action=items['action']))
                    self.send_json({'queued': items}, 202)
                    return
                if path == '/api/sensors':
                    if not state.sensor_manager:
                        self.send_json({'error': '센서 제어 관리자가 없습니다.'}, 409)
                        return
                    if set(items) != {'sensor', 'action'} or not all(isinstance(v, str) for v in items.values()):
                        raise ValueError('sensor와 action 문자열을 지정하세요.')
                    state.sensor_manager.request(items['sensor'], items['action'])
                    self.send_json({'queued': items}, 202)
                    return
                if path == '/api/detection':
                    if not state.detection_request or not state.sensor_manager:
                        raise RuntimeError('ROS 검출·접근 연결이 준비되지 않았습니다.')
                    action = items.get('action')
                    request = dict(action=action)
                    if action in ('select', 'find_and_approach'):
                        request['label'] = target_label(items.get('label'))
                    if action in ('approach', 'find_and_approach'):
                        if 'distance' in items:
                            raise ValueError('접근 방식이 책상 기준으로 바뀌었습니다. 화면을 새로고침하세요.')
                        clearance = items.get('edge_clearance', 0.)
                        if (not isinstance(clearance, (int, float)) or isinstance(clearance, bool)
                                or not math.isfinite(clearance) or not 0 <= clearance <= .1):
                            raise ValueError('책상 추가 이격은 0~0.1m입니다.')
                        request['edge_clearance'] = float(clearance)
                    elif action not in ('start', 'stop', 'select', 'cancel', 'clear'):
                        raise ValueError('action은 start/stop/select/find_and_approach/approach/cancel/clear 중 하나입니다.')
                    state.detection_request(request)
                    self.send_json({'queued': request}, 202)
                    return
                if path in ('/api/mapping', '/api/navigation'):
                    if not state.sensor_manager:
                        self.send_json({'error': '프로세스 관리자가 없습니다.'}, 409)
                        return
                    action = items.get('action')
                    if path == '/api/mapping':
                        if action == 'save':
                            state.sensor_manager.save_map(items.get('name'))
                        elif action in ('start', 'stop', 'reset', 'localize'):
                            state.sensor_manager.service_request('mapping', action, items.get('map', ''))
                        else:
                            raise ValueError('action은 start/stop/reset/localize/save 중 하나입니다.')
                    elif action in ('start', 'stop'):
                        if action == 'stop' and state.navigation_request:
                            state.navigation_request(dict(action='cancel'))
                        state.sensor_manager.service_request('navigation', action)
                    elif action in ('goal', 'cancel'):
                        if not state.navigation_request:
                            raise RuntimeError('ROS 내비게이션 연결이 준비되지 않았습니다.')
                        request = dict(action=action)
                        if action != 'cancel':
                            values = [items.get(k) for k in ('x', 'y', 'yaw')]
                            if not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
                                       for v in values) or max(abs(values[0]), abs(values[1])) > 500:
                                raise ValueError('x, y, yaw 숫자를 지정하세요.')
                            request.update(x=float(values[0]), y=float(values[1]), yaw=float(values[2]))
                        if action == 'goal' and not state.sensor_manager.navigation_available():
                            raise RuntimeError('Nav2가 실행 중이 아닙니다. 먼저 Nav2를 시작하세요.')
                        if action == 'goal':
                            state.sensor_manager.require_collision_ready()
                        state.navigation_request(request)
                    else:
                        raise ValueError('action은 start/stop/goal/cancel 중 하나입니다.')
                    self.send_json({'queued': items}, 202)
                    return
                for key, topic in items.items():
                    if key not in state.config.get('channels', {}) or not isinstance(topic, str):
                        raise ValueError('Unknown channel')
                    if topic and not re.fullmatch(r'/[A-Za-z0-9_/]+', topic):
                        raise ValueError('Invalid topic name')
                with state.condition:
                    state.pending.update(items)
                self.send_json({'queued': items}, 202)
            except (ValueError, TypeError) as exc:
                self.send_json({'error': str(exc)}, 400)
            except RuntimeError as exc:
                self.send_json({'error': str(exc)}, 409)

        def events(self):
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Connection', 'keep-alive')
            self.send_header('X-Accel-Buffering', 'no')
            self.end_headers()
            self.connection.settimeout(10)
            known = {}
            with state.condition:
                state.clients += 1
            try:
                while not state.closed.is_set():
                    payload = state.snapshot(known)
                    known = payload['versions']
                    packet = json.dumps(payload, ensure_ascii=False, separators=(',', ':'), allow_nan=False)
                    self.wfile.write(f'data: {packet}\n\n'.encode())
                    self.wfile.flush()
                    state.closed.wait(0.2)
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
            finally:
                with state.condition:
                    state.clients -= 1

        def stream(self, key):
            if key not in state.config.get('channels', {}):
                self.send_json({'error': 'Unknown image channel'}, 404)
                return
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.connection.settimeout(10)
            version = -1
            try:
                while not state.closed.is_set():
                    with state.condition:
                        state.condition.wait_for(lambda: state.closed.is_set() or state.versions.get(key, 0) != version, 2)
                        current = state.versions.get(key, 0)
                        frame = state.frames.get(key)
                        if not frame or version == current:
                            version = current
                            continue
                        version = current
                    if frame[1] != 'image/jpeg':
                        continue
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                                     + str(len(frame[0])).encode() + b'\r\n\r\n' + frame[0] + b'\r\n')
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass

    server = ThreadingHTTPServer((host, port), Handler)
    server.daemon_threads = True
    return server


def demo_camera(t, wrist=False, debug=False):
    move = 12 * math.sin(t * 0.3)
    objects = [(170 + move, 190, 36, '#80bd55', 'green_apple'),
               (340 + move, 198, 32, '#edaa75', 'peach'),
               (455 + move, 178, 35, '#90d5e2', 'cup')]
    shapes = []
    for x, y, r, color, label in objects:
        shapes.append(f'<ellipse cx="{x}" cy="{y + r}" rx="{r}" ry="8" fill="#000" opacity=".2"/>'
                      f'<circle cx="{x}" cy="{y}" r="{r}" fill="{color}"/>')
        if debug or wrist:
            shapes.append(f'<rect x="{x-r-8}" y="{y-r-8}" width="{2*r+16}" height="{2*r+16}" '
                          f'fill="none" stroke="#b6f776" stroke-width="2"/>'
                          f'<text x="{x-r-8}" y="{y-r-16}" fill="#b6f776" font-size="13">{label} · SAMPLE</text>')
    return (f'<svg xmlns="http://www.w3.org/2000/svg" width="640" height="360" viewBox="0 0 640 360">'
            '<defs><linearGradient id="g" x2="0" y2="1"><stop stop-color="#364250"/><stop offset="1" stop-color="#202b37"/></linearGradient></defs>'
            '<rect width="640" height="360" fill="url(#g)"/><path d="M80 100H560V240H80Z" fill="#4a5865"/>'
            '<path d="M40 150H600L640 330H0Z" fill="#b6aa96"/><path d="M0 330H640V360H0Z" fill="#7e7568"/>'
            + ''.join(shapes) + '<rect x="16" y="16" width="250" height="28" rx="5" fill="#131c28"/>'
            '<text x="27" y="35" fill="#f9c577" font-family="sans-serif" font-size="12">SAMPLE SCENE · 실제 카메라 아님</text></svg>').encode()


def run_demo(state):
    width, height, resolution = 180, 140, 0.07
    pixels = bytearray([95] * (width * height))
    for y in range(15, 126):
        for x in range(15, 166):
            occupied = x in (15, 16, 164, 165) or y in (15, 16, 124, 125)
            occupied |= 65 <= x <= 68 and 15 <= y <= 70
            occupied |= 112 <= x <= 115 and 60 <= y <= 125
            occupied |= 32 <= x <= 47 and 82 <= y <= 96
            occupied |= 127 <= x <= 148 and 35 <= y <= 51
            pixels[y * width + x] = 20 if occupied else 230
    png = png_gray(width, height, pixels)
    state.map_png = png_gray(width, height, b''.join(pixels[y*width:(y+1)*width] for y in range(height-1, -1, -1)))
    state.put('map', dict(image='data:image/png;base64,' + base64.b64encode(png).decode(),
                          width=width, height=height, resolution=resolution, frame='map',
                          origin=[-6.3, -4.9, 0], source_width=width, source_height=height,
                          known_area=74.4, occupied_area=7.8))
    state.log('샘플 모드입니다. 지도·위치·영상·검출·점군은 모두 가상 데이터입니다.', 'warn', 'demo')
    cloud = []
    for n in range(2400):
        a = n * 0.037
        wall = n % 4
        x = (-5.1 if wall == 0 else 5.1) if wall < 2 else 5.1 * math.sin(a)
        y = 3.8 * math.sin(a) if wall < 2 else (-3.8 if wall == 2 else 3.8)
        z = (n % 31) * 0.065
        cloud.append([round(x, 3), round(y, 3), round(z, 3)])
    state.put('cloud', dict(points=cloud, frame='map', source_frame='map', total_points=len(cloud)))
    state.put('path', dict(frame='map', points=[[-3.8, -2], [-1, -2], [0.6, 0], [3.6, 2.5]]))
    trail = []
    last_cloud = -1.0
    while not state.closed.is_set():
        t = time.monotonic() - state.started
        if t - last_cloud >= 0.5:
            state.put('cloud', dict(points=cloud, frame='map', source_frame='map', total_points=len(cloud)))
            last_cloud = t
        x, y, yaw = -3.0 + 1.4 * math.sin(t * 0.08), -1.0 + math.cos(t * 0.08), t * 0.08
        trail.append([x, y])
        state.put('pose', dict(x=x, y=y, yaw=yaw, frame='map', trail=trail[-500:]))
        state.put('odom', dict(linear=0.12, angular=0.08, frame='odom'))
        state.put('scan', dict(frame='map', points=[[x + 2.1 * math.cos(a / 30), y + 2.1 * math.sin(a / 30)] for a in range(190)]))
        state.put('detections', dict(state='TRACK', valid=True, measured=True, reason='SAMPLE_DATA', processing_ms=28.4,
                                    objects=[dict(label='green_apple', confidence=0.94, x=126, y=146, width=88, height=88),
                                             dict(label='peach', confidence=0.89, x=300, y=158, width=80, height=80),
                                             dict(label='cup', confidence=0.92, x=412, y=135, width=86, height=86)]))
        state.put('wrist_status', dict(text='SAMPLE · SAM 분할 표시 예시'))
        for key, wrist, debug in [('head_image', False, False), ('head_debug', False, True),
                                  ('wrist_image', True, False), ('wrist_debug', True, True)]:
            state.put_frame(key, demo_camera(t, wrist, debug), 'image/svg+xml', 640, 360)
        state.closed.wait(0.25)


def load_config(path=None):
    path = Path(path) if path else ROOT / 'config.json'
    if not path.exists():
        path = ROOT / 'config.example.json'
    return json.loads(path.read_text())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--demo', action='store_true', help='Explicit synthetic sample mode, no ROS imports')
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8080)
    args = parser.parse_args()
    state = State('demo' if args.demo else 'offline', load_config())
    server = make_server(state, args.host, args.port)
    if args.demo:
        threading.Thread(target=run_demo, args=(state,), daemon=True).start()
    else:
        state.log('ROS 입력 대기 화면입니다. 실시간 연결은 connect.command를 실행하세요.', 'warn')
    print(f'INHA_CONTROL_CENTER_READY http://{args.host}:{args.port}', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        state.closed.set()
        with state.condition:
            state.condition.notify_all()
        server.server_close()


if __name__ == '__main__':
    main()
