#!/usr/bin/env python3
"""Odometry path test on /cmd_vel: long still (gyro drift), stop-and-go arcs, continuous loops.

Usage (on the robot, someone at the e-stop, ~2.5 m x 2.5 m free floor):
  python3 odom_path_test.py --dry-run
  python3 odom_path_test.py [--still 300]
Record at the same time with ~/livox_g2_calib/odomval.sh start lidar (MAXS=900 for > 10 min).

1. still   : STILL seconds without motion (gyro bias / yaw drift)
2. arcs    : simultaneous v and w, each arc followed by its exact reverse (back to the start), 2.5 s stops
3. loops   : continuous rounded square (1.0 m straights, r = 0.5 m corners at 0.25 m/s), 2 laps CCW, stop, 2 laps CW
Velocities ramp over RAMP_S at both ends of every move (acceleration <= 0.5 m/s^2 for 0.25 m/s), the
commanded integral is kept. Ctrl+C publishes zero velocity.
"""
import sys, time, math
import rclpy
from geometry_msgs.msg import Twist

RAMP_S = 0.5
STOP_S = 2.5
ARCS = [(0.20, 0.30), (0.30, 0.50), (0.15, 0.50), (0.30, 0.20)]   # (v m/s, w rad/s), 90 deg each
LOOP_V, LOOP_R, LOOP_L = 0.25, 0.5, 1.0

def plan(still):
    st = [('still', 0.0, 0.0, still, 0.0)]
    for v, w in ARCS:
        for sgn in (+1, -1):
            d = (math.pi / 2) / w
            st += [(f'arc v{v:.2f} w{sgn * w:+.2f}', v, sgn * w, d, STOP_S), (f'arc back v{-v:.2f} w{-sgn * w:+.2f}', -v, -sgn * w, d, STOP_S)]
    for name, sgn in [('loop CCW', +1), ('loop CW', -1)]:
        lap = []
        for _ in range(4):
            lap += [(name + ' straight', LOOP_V, 0.0, LOOP_L / LOOP_V, 0.0), (name + ' corner', LOOP_V, sgn * LOOP_V / LOOP_R, (math.pi / 2) * LOOP_R / LOOP_V, 0.0)]
        seq = lap * 2; seq[-1] = seq[-1][:4] + (STOP_S + 2.0,)
        st += seq
    return st

def main():
    still = 300.0
    if '--still' in sys.argv: still = float(sys.argv[sys.argv.index('--still') + 1])
    steps = plan(still)
    total = sum(d + p for _, _, _, d, p in steps)
    print(f'{len(steps)} steps, total ~{total / 60:.1f} min (+ ramps)')
    if '--dry-run' in sys.argv:
        for s in steps: print(f'  {s[0]:28s} v {s[1]:+.2f} w {s[2]:+.2f} for {s[3]:.1f} s, then stop {s[4]:.1f} s')
        return
    rclpy.init(); node = rclpy.create_node('odom_path_test'); pub = node.create_publisher(Twist, '/cmd_vel', 10)
    def send(v, w):
        m = Twist(); m.linear.x = float(v); m.angular.z = float(w); pub.publish(m)
    def run(segs):
        """Continuous sequence of (v, w, dur): ramp up at the start and down at the end only."""
        total = sum(d for _, _, d in segs) + RAMP_S; t0 = time.monotonic()
        bounds = []; acc = 0.0
        for v, w, d in segs: bounds.append((acc, acc + d, v, w)); acc += d
        while (t := time.monotonic() - t0) < total:
            tc = min(max(t - RAMP_S / 2, 0.0), acc - 1e-9)
            v, w = next((b[2], b[3]) for b in bounds if b[0] <= tc < b[1] or b is bounds[-1])
            f = min(1.0, t / RAMP_S, max(0.0, (total - t) / RAMP_S))
            send(v * f, w * f); time.sleep(0.02)
    def hold(dur):
        t0 = time.monotonic()
        while time.monotonic() - t0 < dur: send(0, 0); time.sleep(0.02)
    try:
        time.sleep(1.0); hold(5.0)
        i = 0
        while i < len(steps):
            name, v, w, d, pause = steps[i]
            if name == 'still':
                print(f'still {d:.0f} s', flush=True); hold(d); i += 1; continue
            j = i                                   # merge consecutive moves without a stop into one continuous run
            while steps[j][4] == 0.0 and j + 1 < len(steps): j += 1
            print(f'[{i + 1}-{j + 1}/{len(steps)}] {name}' + (f' ... {steps[j][0]}' if j > i else ''), flush=True)
            run([(s[1], s[2], s[3]) for s in steps[i:j + 1]]); hold(steps[j][4]); i = j + 1
    except KeyboardInterrupt:
        print('interrupted')
    finally:
        for _ in range(20): send(0, 0); time.sleep(0.02)
        node.destroy_node(); rclpy.try_shutdown()

if __name__ == '__main__':
    main()
