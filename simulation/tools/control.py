#!/usr/bin/env python3
"""Local web controls (http://127.0.0.1:8081). Commands only target the robocup_motion partition."""
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import robocup_gz as rg

ADDRESS = ('127.0.0.1', 8081)
DRIVE_SECONDS = 1.0     # each drive button moves for this long, then stops
PAGE = Path(__file__).parent / 'controls.html'

robot = rg.RobotCommands(rg.DEFAULT_PARTITION)
robot.advertise_all()
lock = threading.Lock()
drive_until = 0.0


def watchdog():
    global drive_until
    while True:
        time.sleep(0.1)
        with lock:
            if drive_until and time.monotonic() > drive_until:
                drive_until = 0.0
                robot.stop()


def handle(command):
    global drive_until
    kind = command['type']
    if kind == 'drive':
        v, w = float(command['v']), float(command['w'])
        rg.check_drive(v, w)
        with lock:
            robot.drive(v, w)
            drive_until = time.monotonic() + DRIVE_SECONDS if v or w else 0.0
    elif kind == 'joint':
        robot.arm_joint(int(command['i']), float(command['q']))
    elif kind == 'grip':
        robot.grip(float(command['q']))
    else:
        raise ValueError('Unknown command')


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        body = PAGE.read_bytes()
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        try:
            handle(json.loads(self.rfile.read(int(self.headers['Content-Length']))))
            self.send_response(200)
            self.end_headers()
            self.wfile.write(b'OK')
        except Exception as e:
            self.send_response(400)
            self.end_headers()
            self.wfile.write(str(e).encode())


def main():
    threading.Thread(target=watchdog, daemon=True).start()
    print(f'Controls: http://{ADDRESS[0]}:{ADDRESS[1]}', flush=True)
    ThreadingHTTPServer(ADDRESS, Handler).serve_forever()


if __name__ == '__main__':
    main()
