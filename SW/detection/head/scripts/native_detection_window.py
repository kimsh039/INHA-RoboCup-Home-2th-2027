#!/usr/bin/env python3
"""Display the existing Jetson MJPEG stream in a local OpenCV window."""
import argparse
import threading
import time
from urllib.request import urlopen

import cv2
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default='http://127.0.0.1:8787/stream.mjpg')
    args = parser.parse_args()
    incoming = threading.Lock()
    closed = threading.Event()
    latest = None
    error = 'Waiting for Jetson detection frames...'

    def receive():
        nonlocal latest, error
        while not closed.is_set():
            try:
                with urlopen(args.url, timeout=5) as stream:
                    while not closed.is_set():
                        line = stream.readline()
                        if not line:
                            raise OSError('Jetson stream disconnected')
                        if not line.startswith(b'--frame'):
                            continue
                        length = 0
                        while True:
                            header = stream.readline()
                            if not header:
                                raise OSError('Jetson stream disconnected')
                            if header in (b'\r\n', b'\n'):
                                break
                            if header.lower().startswith(b'content-length:'):
                                length = int(header.split(b':', 1)[1])
                        if not 0 < length <= 16 * 1024 * 1024:
                            raise ValueError('Invalid JPEG frame size')
                        data = stream.read(length)
                        if len(data) != length:
                            raise OSError('Incomplete JPEG frame')
                        frame = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
                        if frame is None:
                            continue
                        with incoming:
                            latest, error = frame, ''
            except (OSError, ValueError) as exc:
                with incoming:
                    error = str(exc)
                closed.wait(1)

    title = 'RoboCup Head detection - OpenCV (Q / Esc to close)'
    cv2.namedWindow(title, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(title, 960, 720)
    cv2.imshow(title, np.zeros((480, 640, 3), dtype=np.uint8))
    threading.Thread(target=receive, name='jetson-preview', daemon=True).start()
    print('OpenCV window opened. Reusing existing Jetson detections; Q / Esc to close.', flush=True)
    report_at = time.monotonic()
    count = 0
    seen_visible = False
    try:
        while not closed.is_set():
            with incoming:
                frame, latest = latest, None
                reason = error
            if frame is not None:
                cv2.imshow(title, frame)
                count += 1
            elif reason:
                waiting = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(waiting, reason[:75], (12, 40), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 180, 255), 1)
                cv2.imshow(title, waiting)
            key = cv2.waitKey(5) & 0xff
            if key in (27, ord('q')):
                break
            visible = cv2.getWindowProperty(title, cv2.WND_PROP_VISIBLE)
            if visible >= 1:
                seen_visible = True
            elif seen_visible:
                break
            now = time.monotonic()
            if now - report_at >= 1:
                print(f'Window FPS={count / (now - report_at):.1f}' +
                      (f' | {reason}' if reason else ''), flush=True)
                count, report_at = 0, now
    except KeyboardInterrupt:
        pass
    finally:
        closed.set()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
