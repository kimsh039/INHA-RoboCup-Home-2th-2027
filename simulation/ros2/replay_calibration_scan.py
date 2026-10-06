"""Replay the saved validation scan for TF visualization; never reads live Gazebo."""
import argparse
import gzip
import json
import math
import signal
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanReplay(Node):
    def __init__(self,path):
        super().__init__('calibration_scan_replay')
        with gzip.open(path,'rt') as stream:
            data=json.load(stream)
        self.message=LaserScan()
        self.message.header.frame_id='laser_frame'
        self.message.angle_min=float(data['angleMin'])
        self.message.angle_max=float(data['angleMax'])
        self.message.angle_increment=float(data['angleStep'])
        self.message.range_min=float(data['rangeMin'])
        self.message.range_max=float(data['rangeMax'])
        self.message.time_increment=0.0
        self.message.scan_time=0.1
        self.message.ranges=[float(x) for x in data['ranges']]
        if len(self.message.ranges)!=int(data['count']):
            raise ValueError('Saved scan count mismatch')
        # The simulator's single scan layer has a 1-degree elevation. LaserScan
        # describes a horizontal plane, so project its slant ranges onto that plane.
        low=float(data.get('verticalAngleMin',0))
        high=float(data.get('verticalAngleMax',0))
        if int(data.get('verticalCount',1))!=1 or abs(low-high)>1e-10:
            raise ValueError('Replay expects one scan layer with fixed elevation')
        scale=math.cos(low)
        self.message.ranges=[r*scale if math.isfinite(r) else r for r in self.message.ranges]
        self.message.range_min*=scale
        self.message.range_max*=scale
        self.publisher=self.create_publisher(LaserScan,'/calibration/scan_replay',qos_profile_sensor_data)
        self.create_timer(0.1,self.publish_scan)
        self.get_logger().info('Saved validation scan replay; timestamps are playback time, not capture time. No live sensor subscription.')

    def publish_scan(self):
        self.message.header.stamp=self.get_clock().now().to_msg()
        self.publisher.publish(self.message)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--scan',required=True)
    args=parser.parse_args()
    rclpy.init()
    node=ScanReplay(args.scan)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT,signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__=='__main__':
    main()
