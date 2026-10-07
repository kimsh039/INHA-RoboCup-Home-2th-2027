"""Wrist re-observation: SAM 2.1 mask of the docked target and its 3D points from wrist depth.

When the arm reports OBSERVING, the head-camera target (/detection/dock/target, base_link) is
projected into the wrist colour image and used as SAM's prompt (a point and a box sized for a
`object_size` object). Mask pixels are looked up in the wrist depth image, lifted to 3D, kept
above the table top and published in base_link:

  /detection/wrist/target         PointStamped   object centre (x, y from the visible points)
  /detection/wrist/object_cloud   PointCloud2    masked object points
  /detection/wrist/debug_image    Image          mask and prompt overlay
  /detection/wrist/status         String         WAITING / SEGMENTED:<details> / FAILED:<reason>

The Gazebo wrist RGB and depth share one optical frame with different FOVs (mapped through
the two CameraInfos, as in nav_geometry); a real D435f stream would use aligned depth.
"""
import math
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PointStamped, PolygonStamped
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from tf2_ros import Buffer, TransformListener, TransformException

from .nav_geometry import depth_array
from .support_surface_node import matrix


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec*1e-9


class WristSamNode(Node):
    def __init__(self):
        super().__init__('wrist_sam_node')
        get = lambda name, default: self.declare_parameter(name, default).value
        checkpoint = get('checkpoint', 'models/sam2.1_hiera_tiny.pt')
        config = get('model_config', 'configs/sam2.1/sam2.1_hiera_t.yaml')
        device = get('device', 'cuda')
        self.object_size = get('object_size', 0.12)       # prompt box edge, m (prior object size)
        self.min_above = get('min_above_table', 0.005)    # drop table points, m
        self.passes = get('passes', 2)                    # SAM prompt refinement passes
        self.regrow = get('regrow', 0.3)                  # grow the mask box by this share per side
        import torch
        from sam2.build_sam import build_sam2
        from sam2.sam2_image_predictor import SAM2ImagePredictor
        self.torch = torch
        self.predictor = SAM2ImagePredictor(build_sam2(config, checkpoint, device=device))
        self.color = self.depth = self.color_info = self.depth_info = None
        self.target = self.surface = None
        self.done_for = None                     # stamp of the target already segmented
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.target_pub = self.create_publisher(PointStamped, '/detection/wrist/target', latched)
        self.cloud_pub = self.create_publisher(PointCloud2, '/detection/wrist/object_cloud', latched)
        self.debug_pub = self.create_publisher(Image, '/detection/wrist/debug_image', latched)
        self.status_pub = self.create_publisher(String, '/detection/wrist/status', 10)
        sensor = qos_profile_sensor_data
        self.create_subscription(Image, '/wrist_camera/color/image_raw', lambda m: setattr(self, 'color', m), sensor)
        self.create_subscription(Image, '/wrist_camera/depth/image_raw', lambda m: setattr(self, 'depth', m), sensor)
        self.create_subscription(CameraInfo, '/wrist_camera/color/camera_info', lambda m: setattr(self, 'color_info', m), sensor)
        self.create_subscription(CameraInfo, '/wrist_camera/depth/camera_info', lambda m: setattr(self, 'depth_info', m), sensor)
        self.create_subscription(PointStamped, '/detection/dock/target', lambda m: setattr(self, 'target', m), latched)
        self.create_subscription(PolygonStamped, '/detection/dock/surface', lambda m: setattr(self, 'surface', m), latched)
        self.create_subscription(String, '/detection/wrist/observe_status', self.on_observe, 10)
        self.get_logger().info('SAM 2.1 ready')

    def report(self, text):
        self.status_pub.publish(String(data=text))
        self.get_logger().info(text)

    def on_observe(self, msg):
        if msg.data != 'OBSERVING' or self.target is None:
            return
        if self.done_for == stamp_seconds(self.target.header.stamp):
            return
        if None in (self.color, self.depth, self.color_info, self.depth_info, self.surface):
            return
        if abs(stamp_seconds(self.color.header.stamp) - stamp_seconds(self.depth.header.stamp)) > 0.1:
            return
        try:
            self.segment()
        except TransformException as exc:
            self.report(f'FAILED:TF:{exc}')
        self.done_for = stamp_seconds(self.target.header.stamp)

    def segment(self):
        cam = self.color.header.frame_id
        T = self.tf.lookup_transform(cam, 'base_link', Time.from_msg(self.color.header.stamp), timeout=Duration(seconds=0.2))
        R, t = matrix(T.transform)                         # base_link -> camera
        K = np.array(self.color_info.k).reshape(3, 3)
        p = R @ np.array([self.target.point.x, self.target.point.y, self.target.point.z]) + t
        if p[2] <= 0.05:
            return self.report('FAILED:TARGET_BEHIND_CAMERA')
        u, v = K[0, 0]*p[0]/p[2] + K[0, 2], K[1, 1]*p[1]/p[2] + K[1, 2]
        half = K[0, 0]*self.object_size/2/p[2]
        box = np.array([u-half, v-half, u+half, v+half])
        rgb = np.frombuffer(self.color.data, np.uint8).reshape(self.color.height, self.color.step)[:, :self.color.width*3]
        rgb = rgb.reshape(self.color.height, self.color.width, 3)
        if not (0 <= u < self.color.width and 0 <= v < self.color.height):
            return self.report(f'FAILED:TARGET_OUT_OF_VIEW:{u:.0f},{v:.0f}')
        with self.torch.inference_mode():
            self.predictor.set_image(rgb)
            # Pass 1 prompts with the projected head estimate; it can be off by a few cm, so the
            # box may cut the object. Pass 2 re-prompts with the first mask's centre and its
            # bounding box grown by `regrow`.
            for _ in range(self.passes):
                masks, scores, _ = self.predictor.predict(point_coords=np.array([[u, v]]), point_labels=np.array([1]),
                                                          box=box, multimask_output=False)
                mask = masks[0].astype(bool)
                if not mask.any():
                    break
                rows, cols = np.nonzero(mask)
                u, v = float(np.median(cols)), float(np.median(rows))
                gx, gy = (cols.max()-cols.min())*self.regrow/2, (rows.max()-rows.min())*self.regrow/2
                box = np.array([cols.min()-gx, rows.min()-gy, cols.max()+gx, rows.max()+gy])
        score = float(scores[0])
        # Mask -> depth pixels (same optical frame, different FOV / resolution).
        D = np.array(self.depth_info.k).reshape(3, 3)
        depth = depth_array(self.depth)
        vd, ud = np.mgrid[0:depth.shape[0], 0:depth.shape[1]]
        uc = np.round((ud-D[0, 2])*K[0, 0]/D[0, 0] + K[0, 2]).astype(int)
        vc = np.round((vd-D[1, 2])*K[1, 1]/D[1, 1] + K[1, 2]).astype(int)
        inside = (uc >= 0) & (uc < mask.shape[1]) & (vc >= 0) & (vc < mask.shape[0])
        sel = np.zeros_like(inside)
        sel[inside] = mask[vc[inside], uc[inside]]
        z = depth[sel]
        ok = np.isfinite(z) & (z > 0.05)
        z = z[ok]
        x = (ud[sel][ok]-D[0, 2])*z/D[0, 0]
        y = (vd[sel][ok]-D[1, 2])*z/D[1, 1]
        cam_pts = np.c_[x, y, z]
        base_pts = (cam_pts - t) @ R                        # camera -> base_link
        table_z = float(np.mean([c.z for c in self.surface.polygon.points]))
        obj = base_pts[base_pts[:, 2] > table_z + self.min_above]
        self.publish_debug(rgb, mask, (u, v), box)
        if len(obj) < 50:
            return self.report(f'FAILED:FEW_OBJECT_POINTS:{len(obj)}_score_{score:.2f}')
        centre = np.array([*np.median(obj[:, :2], axis=0), obj[:, 2].max()])
        header = Header(frame_id='base_link', stamp=self.color.header.stamp)
        out = PointStamped(header=header)
        out.point.x, out.point.y, out.point.z = map(float, centre)
        self.target_pub.publish(out)
        self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, obj.astype(np.float32)))
        shift = math.hypot(centre[0]-self.target.point.x, centre[1]-self.target.point.y)
        self.report(f'SEGMENTED:score_{score:.2f}_pixels_{int(mask.sum())}_points_{len(obj)}_'
                    f'centre_{centre[0]:.3f}_{centre[1]:.3f}_top_{centre[2]-table_z:.3f}_head_shift_{shift*1000:.0f}mm')

    def publish_debug(self, rgb, mask, uv, box):
        img = rgb.copy()
        img[mask] = (0.5*img[mask] + 0.5*np.array([0, 255, 0])).astype(np.uint8)
        x0, y0, x1, y1 = [int(round(v)) for v in box]
        img[max(y0, 0):y1, max(x0, 0):max(x0, 0)+3] = (255, 255, 0)
        img[max(y0, 0):y1, max(x1-3, 0):x1] = (255, 255, 0)
        img[max(y0, 0):max(y0, 0)+3, max(x0, 0):x1] = (255, 255, 0)
        img[max(y1-3, 0):y1, max(x0, 0):x1] = (255, 255, 0)
        u, v = int(uv[0]), int(uv[1])
        img[max(v-6, 0):v+6, max(u-6, 0):u+6] = (255, 0, 0)
        msg = Image(header=self.color.header, height=img.shape[0], width=img.shape[1], encoding='rgb8',
                    step=img.shape[1]*3, data=img.tobytes())
        self.debug_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WristSamNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
