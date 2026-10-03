#!/usr/bin/env python3
"""Direct Gazebo Transport terminal controls; no browser or HTTP server needed."""
import argparse
import select
import sys
import termios
import time
import tty

import robocup_gz as rg

TELEOP_KEYS = {'w': (0.15, 0.0), 's': (-0.15, 0.0), 'a': (0.0, 0.4), 'd': (0.0, -0.4),
               'x': (0.0, 0.0), ' ': (0.0, 0.0)}
TELEOP_TIMEOUT = 0.4   # stop when no key arrives for this long, s


def parse_args():
    p = argparse.ArgumentParser(description='Gazebo Tracer / Piper terminal controls')
    p.add_argument('--partition', default=rg.default_partition())
    s = p.add_subparsers(dest='command', required=True)
    d = s.add_parser('drive')
    d.add_argument('linear', type=float, help='m/s')
    d.add_argument('angular', type=float, help='rad/s')
    d.add_argument('--seconds', type=float, default=1)
    s.add_parser('stop')
    s.add_parser('home')
    s.add_parser('teleop')
    j = s.add_parser('joint')
    j.add_argument('number', type=int, choices=range(1, 7))
    j.add_argument('radians', type=float)
    g = s.add_parser('grip')
    g.add_argument('meters', type=float, help=f'one finger travel, 0..{rg.GRIPPER_MAX} m')
    args = p.parse_args()
    try:
        if args.command == 'drive':
            rg.check_drive(args.linear, args.angular)
            rg.check_range(args.seconds, 0.05, 30, 'seconds')
        elif args.command == 'joint':
            rg.check_joint(args.number, args.radians)
        elif args.command == 'grip':
            rg.check_grip(args.meters)
    except ValueError as e:
        p.error(str(e))
    return args


def teleop(robot):
    if not sys.stdin.isatty():
        raise RuntimeError('teleop requires an interactive terminal')
    robot.open_drive()
    print('W: forward / S: reverse / A: left / D: right / X or Space: stop / Q: quit\n'
          f'Press repeatedly to continue; stops after {TELEOP_TIMEOUT} seconds without a key.', flush=True)
    old = termios.tcgetattr(sys.stdin)
    last, v, w = 0.0, 0.0, 0.0
    try:
        tty.setcbreak(sys.stdin.fileno())
        while True:
            if select.select([sys.stdin], [], [], 0.05)[0]:
                key = sys.stdin.read(1).lower()
                if key == 'q':
                    break
                if key in TELEOP_KEYS:
                    v, w = TELEOP_KEYS[key]
                    last = time.monotonic()
            if time.monotonic() - last > TELEOP_TIMEOUT:
                v = w = 0.0
            robot.drive(v, w)
    finally:
        try:
            robot.stop()
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)


def main():
    args = parse_args()
    robot = rg.RobotCommands(args.partition, wait_for_gazebo=4.0)
    try:
        if args.command == 'drive':
            robot.open_drive()
            try:
                end = time.monotonic() + args.seconds
                while time.monotonic() < end:
                    robot.drive(args.linear, args.angular)
                    time.sleep(0.05)
            finally:
                robot.stop()
                time.sleep(0.1)
        elif args.command == 'stop':
            robot.stop()
            time.sleep(0.1)
        elif args.command == 'joint':
            robot.arm_joint(args.number, args.radians)
            time.sleep(0.1)
        elif args.command == 'grip':
            robot.grip(args.meters)
            time.sleep(0.1)
        elif args.command == 'home':
            robot.home()
            time.sleep(0.1)
        elif args.command == 'teleop':
            teleop(robot)
        if args.command != 'teleop':
            print('Command sent:', args.command, 'partition:', args.partition)
    except KeyboardInterrupt:
        print('\nStopped.')
    except RuntimeError as e:
        print(str(e), file=sys.stderr)
        sys.exit(1)


if __name__ == '__main__':
    main()
