#!/usr/bin/env python3
"""Stop-and-go /cmd_vel program for the base <-> G2 <-> Mid-360S calibration (one session, ~3 min).

Usage (on the robot, someone at the e-stop, ~1 m free in front and behind):
  python3 calib_drive.py --dry-run
  python3 calib_drive.py
  python3 calib_drive.py --straights 2       # extra forward/reverse cycles only (driving-direction yaw)
  python3 calib_drive.py --poses             # 16 static poses at different positions / headings in a ~0.6 m box (G2 <-> Mid-360S)
Record at the same time: ~/livox_g2_calib/odomval.sh start lidar ... odomval.sh stop

  spin CCW : 8 x +45 deg in place, STOP_S still after each   (rotation centre, CW/CCW consistency)
  spin CW  : 8 x -45 deg
  forward  : 6 x 0.15 m                                       (driving direction -> yaw)
  reverse  : 6 x 0.15 m                                       (back to the start)
Every still period is also a static pose for the G2 <-> Mid-360S fit (16 headings, 7 positions).
Slow speeds (0.25 rad/s, 0.08 m/s) with 0.5 s ramps keep wheel slip negligible; only geometry is used.
"""
import sys, time, math
import rclpy
from geometry_msgs.msg import Twist

W, V, RAMP_S, STOP_S = 0.25, 0.08, 0.5, 3.5
STEP_YAW, STEP_D, N_SPIN, N_STR = math.radians(45), 0.15, 8, 6

def plan_poses():
    t = lambda deg: ('turn', 0, math.copysign(W, deg), math.radians(abs(deg)) / W)
    f = lambda d: ('move', math.copysign(V, d), 0, abs(d) / V)
    return [f(0.4), t(90), f(0.4), t(90), f(0.4), t(90), f(0.4), t(135), f(0.3), t(-90), f(0.3), t(-90), f(0.3), t(-60), f(-0.3), t(-75)]

def plan(cycles=None):
    straight = [('forward', +V, 0, STEP_D / V)] * N_STR + [('reverse', -V, 0, STEP_D / V)] * N_STR
    if cycles: return straight * cycles
    return [('spin CCW', 0, +W, STEP_YAW / W)] * N_SPIN + [('spin CW', 0, -W, STEP_YAW / W)] * N_SPIN + straight

def main():
    cycles = int(sys.argv[sys.argv.index('--straights') + 1]) if '--straights' in sys.argv else None
    steps = plan_poses() if '--poses' in sys.argv else plan(cycles)
    print(f'{len(steps)} moves, ~{sum(d + RAMP_S + STOP_S for *_, d in steps) / 60 + 0.2:.1f} min')
    if '--dry-run' in sys.argv:
        for s in steps: print(f'  {s[0]:9s} v {s[1]:+.2f} w {s[2]:+.2f} for {s[3]:.2f} s')
        return
    rclpy.init(); node = rclpy.create_node('calib_drive'); pub = node.create_publisher(Twist, '/cmd_vel', 10)
    def send(v, w):
        m = Twist(); m.linear.x = float(v); m.angular.z = float(w); pub.publish(m)
    def move(v, w, d):                       # trapezoid, ramps of RAMP_S, commanded integral = (v, w) * d
        total = d + RAMP_S; t0 = time.monotonic()
        while (t := time.monotonic() - t0) < total:
            f = min(1.0, t / RAMP_S, (total - t) / RAMP_S); send(v * f, w * f); time.sleep(0.02)
    def hold(d):
        t0 = time.monotonic()
        while time.monotonic() - t0 < d: send(0, 0); time.sleep(0.02)
    try:
        time.sleep(1.0); hold(5.0)
        for k, (name, v, w, d) in enumerate(steps, 1):
            print(f'[{k}/{len(steps)}] {name}', flush=True); move(v, w, d); hold(STOP_S)
    except KeyboardInterrupt:
        print('interrupted')
    finally:
        for _ in range(20): send(0, 0); time.sleep(0.02)
        node.destroy_node(); rclpy.try_shutdown()

if __name__ == '__main__':
    main()
