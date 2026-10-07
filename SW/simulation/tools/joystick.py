#!/usr/bin/env python3
"""GTK joystick and Piper controls over Gazebo Transport."""
import argparse
import math

import robocup_gz as rg


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--partition', default=rg.default_partition())
    args = parser.parse_args()
    robot = rg.RobotCommands(args.partition)
    import gi
    gi.require_version('Gtk', '3.0')
    from gi.repository import Gtk, Gdk, GLib

    class Controls(Gtk.Window):
        def __init__(self):
            super().__init__(title='RoboCup — Joystick / Piper')
            self.set_default_size(480, 720)
            robot.advertise_all()
            self.active = False
            self.x = self.y = 0
            box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8)
            box.set_border_width(12)
            self.add(box)
            box.pack_start(Gtk.Label(label='Tracer: drag the stick • release to stop\nUp/down: forward/reverse • left/right: turn'), False, False, 0)
            self.pad = Gtk.DrawingArea()
            self.pad.set_size_request(300, 260)
            self.pad.add_events(Gdk.EventMask.BUTTON_PRESS_MASK | Gdk.EventMask.BUTTON_RELEASE_MASK | Gdk.EventMask.POINTER_MOTION_MASK)
            self.pad.connect('draw', self.draw)
            self.pad.connect('button-press-event', self.press)
            self.pad.connect('motion-notify-event', self.motion)
            self.pad.connect('button-release-event', lambda *_: self.stop())
            box.pack_start(self.pad, True, True, 0)
            self.speed = Gtk.Label(label='v = 0.00 m/s   ω = 0.00 rad/s')
            box.pack_start(self.speed, False, False, 0)
            stop = Gtk.Button(label='STOP Tracer')
            stop.connect('clicked', lambda *_: self.stop())
            box.pack_start(stop, False, False, 0)
            self.status = Gtk.Label()
            box.pack_start(self.status, False, False, 0)
            box.pack_start(Gtk.Label(label='Piper target angles (rad) — press Apply'), False, False, 0)
            grid = Gtk.Grid(column_spacing=8, row_spacing=4)
            self.spins = []
            for i, (lo, hi) in enumerate(rg.ARM_LIMITS, 1):
                spin = Gtk.SpinButton.new_with_range(lo, hi, .05)
                spin.set_digits(3)
                self.spins.append(spin)
                button = Gtk.Button(label='Apply')
                button.connect('clicked', lambda _, number=i, value=spin: self.joint(f'piper_joint{number}', value.get_value()))
                grid.attach(Gtk.Label(label=f'Joint {i}'), 0, i-1, 1, 1)
                grid.attach(spin, 1, i-1, 1, 1)
                grid.attach(button, 2, i-1, 1, 1)
            box.pack_start(grid, False, False, 0)
            row = Gtk.Box(spacing=8)
            row.pack_start(Gtk.Label(label='Gripper (m / finger)'), False, False, 0)
            self.grip = Gtk.SpinButton.new_with_range(0, rg.GRIPPER_MAX, .005)
            self.grip.set_digits(3)
            row.pack_start(self.grip, True, True, 0)
            button = Gtk.Button(label='Apply')
            button.connect('clicked', lambda *_: self.gripper(self.grip.get_value()))
            row.pack_start(button, False, False, 0)
            box.pack_start(row, False, False, 0)
            home = Gtk.Button(label='Arm / gripper HOME + Tracer stop')
            home.connect('clicked', self.home)
            box.pack_start(home, False, False, 0)
            self.connect('focus-out-event', lambda *_: self.stop())
            self.connect('key-press-event', self.key)
            self.connect('delete-event', self.close)
            self.timer = GLib.timeout_add(50, self.tick)
            self.show_all()

        def draw(self, widget, cr):
            cx, cy = widget.get_allocated_width()/2, widget.get_allocated_height()/2
            radius = min(cx, cy)-24
            cr.set_source_rgb(.16, .19, .24)
            cr.arc(cx, cy, radius, 0, 2*math.pi)
            cr.fill()
            cr.set_source_rgb(.5, .55, .6)
            cr.move_to(cx-radius, cy); cr.line_to(cx+radius, cy)
            cr.move_to(cx, cy-radius); cr.line_to(cx, cy+radius); cr.stroke()
            cr.set_source_rgb(.05, .65, .9)
            cr.arc(cx+self.x*radius, cy+self.y*radius, 22, 0, 2*math.pi)
            cr.fill()

        def move(self, event):
            cx, cy = self.pad.get_allocated_width()/2, self.pad.get_allocated_height()/2
            radius = min(cx, cy)-24
            x, y = (event.x-cx)/radius, (event.y-cy)/radius
            length = max(1, math.hypot(x, y))
            self.x, self.y = x/length, y/length
            self.pad.queue_draw()

        def press(self, _, event):
            if event.button == 1 and robot.drive_connected():
                self.active = True
                self.pad.grab_add()
                self.move(event)
            return True

        def motion(self, _, event):
            if self.active:
                if event.state & Gdk.ModifierType.BUTTON1_MASK:
                    self.move(event)
                else:
                    self.stop()
            return True

        def drive(self, v, w):
            robot.drive(v, w)
            self.speed.set_text(f'v = {v:.2f} m/s   ω = {w:.2f} rad/s')

        def stop(self):
            self.active = False
            self.x = self.y = 0
            if self.pad.has_grab(): self.pad.grab_remove()
            self.drive(0, 0)
            self.pad.queue_draw()
            return False

        def tick(self):
            connected = robot.drive_connected()
            self.status.set_text(('Connected • ' if connected else 'Waiting for Gazebo • ') + args.partition)
            if self.active:
                if connected:
                    self.drive(-self.y * rg.MAX_LINEAR, -self.x * rg.MAX_ANGULAR)
                else:
                    self.stop()
            return True

        def joint(self, name, value):
            if not robot.joint_connected(name):
                self.status.set_text('No joint controller: ' + name)
                return
            robot.joint(name, value)

        def gripper(self, value):
            self.joint('piper_gripper_joint1', value)
            self.joint('piper_gripper_joint2', -value)

        def home(self, *_):
            self.stop()
            for i, spin in enumerate(self.spins, 1):
                spin.set_value(0); self.joint(f'piper_joint{i}', 0)
            self.grip.set_value(0); self.gripper(0)

        def key(self, _, event):
            if event.keyval in (Gdk.KEY_space, Gdk.KEY_Escape):
                self.stop(); return True
            return False

        def close(self, *_):
            self.stop()
            GLib.source_remove(self.timer)
            # Allow the final stop to leave Transport before the process exits.
            GLib.timeout_add(150, lambda: Gtk.main_quit())
            return False

    controls = Controls()
    try:
        Gtk.main()
    finally:
        controls.stop()


if __name__ == '__main__':
    main()
