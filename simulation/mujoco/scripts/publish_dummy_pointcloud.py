"""Publish generated points.npy for ROS 2 Humble RViz preview."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header


def load_message(path: Path) -> PointCloud2:
    metadata = json.loads(path.with_name("metadata.json").read_text())
    if metadata["units"] != "meters" or metadata["source_frame"] != "object":
        raise ValueError("Expected metadata in meters and object frame")
    if hashlib.sha256(path.read_bytes()).hexdigest() != metadata["npy_sha256"]:
        raise ValueError("Cloud does not match its metadata SHA-256")
    points = np.load(path, allow_pickle=False)
    if points.shape != (metadata["num_points"], 3) or not np.isfinite(points).all():
        raise ValueError("Expected finite N×3 cloud matching metadata count")
    return create_cloud_xyz32(Header(frame_id=metadata["source_frame"]), points)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("cloud", type=Path)
    parser.add_argument("--topic", default="/dummy_object/points")
    parser.add_argument("--check", action="store_true", help="Validate message without starting ROS middleware")
    args = parser.parse_args()
    message = load_message(args.cloud)
    if args.check:
        print(f"PointCloud2 verified: frame={message.header.frame_id}, points={message.width}, bytes={len(message.data)}")
        return
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    rclpy.init()
    node = None
    try:
        node = rclpy.create_node("dummy_pointcloud_publisher")
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        publisher = node.create_publisher(PointCloud2, args.topic, qos)

        def publish() -> None:
            message.header.stamp = node.get_clock().now().to_msg()
            publisher.publish(message)

        timer = node.create_timer(0.5, publish)
        node.get_logger().info(f"Publishing {message.width} points on {args.topic}; RViz Fixed Frame: object")
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
