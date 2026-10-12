#!/usr/bin/env python3
"""Use the recorded gyro-bias relay with the current physical Mid360 rotation."""
from pathlib import Path
import sys

import numpy as np
import rclpy

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'HW/calibration/physical/tracer_odom'))
from imu_base_relay import Relay


def main():
    rclpy.init()
    node = Relay()
    node.declare_parameter('r_base_imu', [1., 0., 0., 0., 1., 0., 0., 0., 1.])
    node.R = np.array(node.get_parameter('r_base_imu').value).reshape(3, 3)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
