#!/usr/bin/env python3
"""Stop-and-go speed sweep for Tracer odometry calibration (publishes /cmd_vel, open loop by time).

Usage (on the robot, someone at the e-stop, ~2 m x 2 m free floor):
  python3 odom_speed_sweep.py --dry-run          # print the plan only
  python3 odom_speed_sweep.py [rot|str|all]      # default all
  python3 odom_speed_sweep.py tape               # 2 x 1 m forward at 0.1 m/s, 20 s stops (mark the floor)
Record at the same time with ~/livox_g2_calib/odomval.sh start lidar, then stop.

Each speed level: +turn, stop, -turn, stop (net yaw 0) and forward, stop, reverse, stop (net 0),
so the robot ends near its start. Every motion is followed by a STOP_S still period, which the
scan-matching stop detector needs (>= 1 s). Ctrl+C publishes zero velocity.
"""
import sys, time, math
import rclpy
from geometry_msgs.msg import Twist

ROT_SPEEDS = [0.15, 0.30, 0.50, 0.80]      # rad/s
ROT_ANGLE = math.pi / 2                    # per turn
STR_SPEEDS = [0.10, 0.20, 0.35, 0.50]      # m/s
STR_DIST = 1.0                             # m per straight
REPS = 2
STOP_S = 2.5
RAMP_S = 0.3                               # linear ramp at start and end (firmware also limits acceleration)
MAX_V, MAX_W = 0.6, 1.0

def plan(which):
    steps = []
    if which == 'tape':                     # ground-truth check: 2 x 1.0 m forward at 0.10 m/s, 20 s still for marking
        return [('str', 0.10, 1.0 / 0.10, 20.0), ('str', 0.10, 1.0 / 0.10, 20.0)]
    if which in ('rot', 'all'):
        for w in ROT_SPEEDS:
            for _ in range(REPS):
                steps += [('rot', +w, ROT_ANGLE / w, STOP_S), ('rot', -w, ROT_ANGLE / w, STOP_S)]
    if which in ('str', 'all'):
        for v in STR_SPEEDS:
            for _ in range(REPS):
                steps += [('str', +v, STR_DIST / v, STOP_S), ('str', -v, STR_DIST / v, STOP_S)]
    return steps

def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]; which = args[0] if args else 'all'
    steps = plan(which)
    total = sum(d + sp for _, _, d, sp in steps)
    for k, (kind, s, d, _) in enumerate(steps, 1):
        print(f'{k:2d} {kind} {s:+.2f} {"rad/s" if kind == "rot" else "m/s"} for {d:.1f} s')
    print(f'total ~{total / 60:.1f} min')
    if '--dry-run' in sys.argv: return
    assert all(abs(s) <= (MAX_W if k == 'rot' else MAX_V) for k, s, _, _ in steps)
    rclpy.init(); node = rclpy.create_node('odom_speed_sweep'); pub = node.create_publisher(Twist, '/cmd_vel', 10)
    def send(v, w):
        m = Twist(); m.linear.x = float(v); m.angular.z = float(w); pub.publish(m)
    def hold(v, w, dur, ramp):
        t0 = time.monotonic()
        while (t := time.monotonic() - t0) < dur:
            f = 1.0 if not ramp else min(1.0, t / RAMP_S, max(0.0, (dur - t) / RAMP_S))
            send(v * f, w * f); time.sleep(0.02)
    try:
        time.sleep(1.0); hold(0, 0, 5.0, False)          # still start: gyro bias, reference scan
        for k, (kind, s, d, sp) in enumerate(steps, 1):
            print(f'[{k}/{len(steps)}] {kind} {s:+.2f}', flush=True)
            hold(s if kind == 'str' else 0, s if kind == 'rot' else 0, d + RAMP_S, True)   # +RAMP_S keeps the commanded integral
            if sp > STOP_S: print(f'    stopped: mark the floor now ({sp:.0f} s)', flush=True)
            hold(0, 0, sp, False)
    except KeyboardInterrupt:
        print('interrupted')
    finally:
        for _ in range(20): send(0, 0); time.sleep(0.02)
        node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
