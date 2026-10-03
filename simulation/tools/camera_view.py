#!/usr/bin/env python3
"""Live Gazebo RGB and depth popup windows (GTK3)."""
import argparse
import os
import threading
import time

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--partition', default=os.environ.get('GZ_PARTITION', 'robocup_motion'))
p.add_argument('--camera', choices=['head', 'wrist'], default='head', help='Select the rack or wrist camera')
a = p.parse_args()
os.environ.setdefault('GZ_IP', '127.0.0.1')
os.environ['GZ_PARTITION'] = a.partition
import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, GdkPixbuf, GLib
import numpy as np
from PIL import Image as PILImage
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

node = Node()
latest = {}
lock = threading.Lock()
windows = {}

def receive(kind):
    def callback(msg):
        with lock:
            latest[kind] = (msg, time.monotonic())
    return callback

prefix = '/robocup/wrist_camera' if a.camera == 'wrist' else '/robocup/camera'
for kind, topic in [('RGB', prefix + '/color/image'), ('Depth', prefix + '/depth/image')]:
    win = Gtk.Window(title='RoboCup ' + a.camera + ' Camera — ' + kind)
    win.set_default_size(800, 490)
    box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=4)
    view = Gtk.Image()
    label = Gtk.Label(label='Waiting for camera frames… partition: ' + a.partition)
    box.pack_start(view, True, True, 0)
    box.pack_start(label, False, False, 4)
    win.add(box)
    windows[kind] = (win, view, label)
    def close(widget, key=kind):
        windows.pop(key, None)
        if not windows:
            Gtk.main_quit()
    win.connect('destroy', close)
    win.show_all()
    if not node.subscribe(Image, topic, receive(kind)):
        label.set_text('Subscription failed: ' + topic)

def convert(msg, kind):
    if kind == 'Depth':
        if msg.pixel_format_type != 13:  # gz PixelFormatType R_FLOAT32
            raise ValueError('Expected float32 depth, received format ' + str(msg.pixel_format_type))
        depth = np.ndarray((msg.height, msg.width), dtype='<f4', buffer=msg.data, strides=(msg.step, 4))
        valid = np.isfinite(depth) & (depth > 0)
        d = np.clip(np.nan_to_num(depth, nan=3, posinf=3, neginf=3) / 3, 0, 1)
        rgb = np.stack((255*(1-d), 255*(1-np.abs(2*d-1)), 255*d), axis=-1).astype(np.uint8)
        rgb[~valid] = 0
        return PILImage.fromarray(rgb), 'Depth: near=red, far=blue; 0–3 m; black=no return'
    modes = {3: 'RGB', 4: 'RGBA', 5: 'BGRA', 8: 'BGR'}
    mode = modes.get(msg.pixel_format_type)
    if mode is None:
        raise ValueError('Unsupported RGB format ' + str(msg.pixel_format_type))
    return PILImage.frombytes('RGBA' if len(mode)==4 else 'RGB', (msg.width, msg.height), msg.data, 'raw', mode, msg.step).convert('RGB'), 'RGB'

def update():
    with lock:
        frames = dict(latest)
    for kind, (win, view, label) in list(windows.items()):
        if kind not in frames:
            continue
        msg, received = frames[kind]
        try:
            picture, description = convert(msg, kind)
            picture.thumbnail((min(960, max(320, win.get_allocated_width() - 16)), min(540, max(180, win.get_allocated_height() - 50))))
            w, h = picture.size
            pixbuf = GdkPixbuf.Pixbuf.new_from_bytes(GLib.Bytes.new(picture.tobytes()), GdkPixbuf.Colorspace.RGB, False, 8, w, h, w*3)
            view.set_from_pixbuf(pixbuf)
            age = time.monotonic() - received
            label.set_text(f'{description} | {msg.width}×{msg.height} | last frame {age:.1f}s ago')
        except ValueError as exc:
            label.set_text(str(exc))
    return True

GLib.timeout_add(100, update)
Gtk.main()
