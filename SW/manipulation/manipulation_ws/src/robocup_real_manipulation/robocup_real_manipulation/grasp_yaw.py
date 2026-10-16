"""Yaw alternatives keep the measured grip center fixed, including TCP offset."""
import math
import numpy as np
from .core import require, rotation


def candidates(center, tcp, quaternion, rule, cfg):
    angles = [0] if rule.get('axis_required') else rule.get('yaw_sample_degrees', [0])
    require(isinstance(angles,list) and 1<=len(angles)<=12 and
            all(isinstance(a,(int,float)) and math.isfinite(a) and 0<=a<180 for a in angles),
            'GRASP_YAW_SAMPLES')
    grip = np.asarray(tcp)+rotation(quaternion)@np.asarray(cfg['tcp_to_grip_center_m'])
    x,y,z,w = quaternion
    for angle in angles:
        a = math.radians(angle)/2; s,c = math.sin(a),math.cos(a)
        q = np.array([c*x-s*y,c*y+s*x,c*z+s*w,c*w-s*z])
        yield grip-rotation(q)@np.asarray(cfg['tcp_to_grip_center_m']),q
