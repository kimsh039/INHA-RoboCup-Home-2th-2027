#!/usr/bin/env python3
"""Write the odometry correction from odom_calibration.json into tracer_base.launch.py defaults.

Usage (on the robot, after the patch is applied):
  python3 set_launch_defaults.py odom_calibration.json ~/tracer_ws/src/tracer_ros2/tracer_base/launch/tracer_base.launch.py
Then commit in tracer_ros2 and restart tracer_base. Use 0 offsets / 1.0 scales to disable.
"""
import json, re, sys
cal = json.load(open(sys.argv[1])); path = sys.argv[2]
values = {'linear_velocity_scale': cal['linear_velocity_scale'], 'angular_velocity_scale': cal['angular_velocity_scale'],
          'angular_velocity_bias': cal['angular_velocity_bias_rad_s'], 'linear_velocity_offset': cal['linear_velocity_offset_m_s'],
          'angular_velocity_offset': cal['angular_velocity_offset_rad_s'], 'angular_offset_threshold': cal['angular_offset_threshold_rad_s']}
s = open(path).read()
for name, value in values.items():
    pattern = re.compile(r"(DeclareLaunchArgument\('%s', default_value=')([^']*)(')" % name)
    if len(pattern.findall(s)) != 1:
        raise SystemExit(f'{name}: launch argument not found exactly once (is the odom-correction patch applied?)')
    s = pattern.sub(lambda m: m.group(1) + repr(float(value)) + m.group(3), s)
open(path, 'w').write(s)
print({k: float(v) for k, v in values.items()})
