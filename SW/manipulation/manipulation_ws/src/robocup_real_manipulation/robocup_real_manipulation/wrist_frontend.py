"""ROS request adapter using existing YOLO and generic SAM2 weights.

RealSense ROS owns the camera. No pyrealsense2 pipeline, camera reopening, arm
commands, fabricated classes, or copied coarse positions appear in this node.
"""
from contextlib import nullcontext
from pathlib import Path
import threading
import time
import cv2
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from rclpy.time import Time
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener
from std_msgs.msg import String
from robocup_real_manipulation_msgs.msg import ObservationRequest, WristObservation, TaskStatus
from robocup_head_detection.backends import YoloDetector
from robocup_head_detection.class_schema import class_mapping
from .ros_util import read_config, seconds
from .core import fresh, require, rotation
from .wrist_geometry import surface_point, banana_point_axis


class WristFrontend(Node):
    def __init__(self):
        super().__init__('real_wrist_frontend')
        self.c = read_config(self)
        repo = Path(self.declare_parameter('repo', '').value).expanduser().resolve()
        vision_file = Path(self.declare_parameter('vision_config', '').value).expanduser()
        self.v = yaml.safe_load(vision_file.read_text())
        if self.c['classes']['banana']['enabled']:
            require(isinstance(self.v.get('banana'),dict),'BANANA_VISION_CONFIG_MISSING:copy banana settings from updated wrist.yaml')
        self.bridge = CvBridge()
        self.tf=Buffer(); self.tf_listener=TransformListener(self.tf,self)
        self.mutex = threading.Lock()
        self.frames = {}; self.req = None; self.last_capture = 0.; self.worker_error = ''
        self.closed = threading.Event()
        self.results = self.create_publisher(WristObservation, '/manipulation/wrist/result', 10)
        self.diagnostics = self.create_publisher(String, '/manipulation/wrist/status', 10)
        self.create_subscription(ObservationRequest, '/manipulation/wrist/request', self.request, 10)
        self.create_subscription(TaskStatus, '/manipulation/status', self.task_status, 10)
        for kind, topic, typ in [('color', self.v['color_topic'], Image),
                                 ('depth', self.v['depth_topic'], Image),
                                 ('info', self.v['camera_info_topic'], CameraInfo)]:
            self.create_subscription(typ, topic, lambda msg, k=kind: self.frame(k, msg), qos_profile_sensor_data)
        self.create_timer(.2, self.status)
        def local_path(value):
            path = Path(value).expanduser()
            return path if path.is_absolute() else repo / path
        weights = local_path(self.v['yolo_model'])
        self.detector = YoloDetector(str(weights), confidence=self.c['min_confidence'],
            nms_iou=self.v['nms_iou'], imgsz=self.v['imgsz'], device=self.v['device'], task=self.v['yolo_task'])
        require(self.detector.ready, 'YOLO_NOT_READY:' + self.detector.error)
        schema=class_mapping(self.detector.model.names)
        active={pair[0]:pair[1] for pair in schema.values() if pair is not None}
        require(all(active.get(i)==name for i,name in enumerate(self.c['model_names'])), 'WRIST_CLASS_SCHEMA_MISMATCH')
        self.predictor = None
        if self.v['mask_source'] == 'sam2':
            checkpoint = local_path(self.v['sam_checkpoint'])
            require(checkpoint.is_file(), 'SAM_CHECKPOINT_MISSING:' + str(checkpoint))
            import torch
            from sam2.build_sam import build_sam2
            from sam2.sam2_image_predictor import SAM2ImagePredictor
            self.torch = torch
            device = 'cpu' if str(self.v['device']) == 'cpu' else 'cuda'
            self.predictor = SAM2ImagePredictor(build_sam2(self.v['sam_config'], str(checkpoint), device=device))
        else:
            require(self.v['mask_source'] == 'yolo_seg' and self.v['yolo_task'] == 'segment', 'MASK_SOURCE_CONFIG')
        self.worker = threading.Thread(target=self.work, daemon=True)
        self.worker.start()

    def frame(self, kind, msg):
        with self.mutex:
            self.frames[kind] = msg

    def request(self, msg):
        with self.mutex:
            if self.req is None or (msg.task_id, msg.observation_id) != (self.req.task_id, self.req.observation_id):
                self.req = msg
                self.last_capture = seconds(msg.header.stamp)
                self.worker_error = ''

    def task_status(self,msg):
        with self.mutex:
            if self.req and msg.task_id==self.req.task_id and (msg.stage!='WRIST_REDETECT' or msg.outcome!='RUNNING'):
                self.req=None; self.worker_error=''

    def status(self):
        with self.mutex:
            color = self.frames.get('color')
            ready = all(k in self.frames for k in ('color', 'depth', 'info'))
        if ready and color:
            try:
                fresh(seconds(color.header.stamp), self.get_clock().now().nanoseconds / 1e9, self.c['max_observation_age_s'])
                with self.mutex: depth=self.frames['depth']; info=self.frames['info']
                require(depth.header.frame_id==color.header.frame_id==info.header.frame_id==self.v['optical_frame'],'CAMERA_FRAME')
                require(max(abs(seconds(color.header.stamp)-seconds(m.header.stamp)) for m in (depth,info))<=self.v['sync_tolerance_s'],'CAMERA_SYNC')
            except Exception:
                ready = False
        text = ('READY' if ready else 'WAITING_CAMERA')
        if self.worker_error: text += ':' + self.worker_error
        self.diagnostics.publish(String(data=text))

    def infer(self, color, depth_msg, info, req):
        require(depth_msg.header.frame_id == color.header.frame_id == info.header.frame_id == self.v['optical_frame'], 'ALIGNED_DEPTH_FRAME_MISMATCH')
        stamp = seconds(color.header.stamp)
        require(max(abs(stamp-seconds(m.header.stamp)) for m in (depth_msg, info)) <= self.v['sync_tolerance_s'], 'UNSYNCHRONIZED_CAMERA')
        fresh(stamp, self.get_clock().now().nanoseconds / 1e9, self.c['max_observation_age_s'])
        require(depth_msg.encoding in ('16UC1', '32FC1'), 'DEPTH_ENCODING')
        bgr = self.bridge.imgmsg_to_cv2(color, 'bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg).astype(float)
        if depth_msg.encoding == '16UC1': depth *= .001
        require(depth.shape == bgr.shape[:2], 'DEPTH_GRID_MISMATCH')
        result = self.detector.model.predict(bgr, conf=self.c['min_confidence'],
            iou=self.v['nms_iou'], imgsz=self.v['imgsz'], device=self.v['device'], verbose=False)[0]
        mapping = class_mapping(result.names)
        active = {pair[0]: pair[1] for pair in mapping.values() if pair is not None}
        require(all(active.get(i) == name for i, name in enumerate(self.c['model_names'])), 'WRIST_CLASS_SCHEMA_MISMATCH')
        if result.boxes is None: return []
        # Preserve ALL candidates of the requested class; do not use Head's preview
        # policy that keeps one highest-confidence box per class.
        boxes = [(i, box) for i, box in enumerate(result.boxes)
                 if mapping[int(box.cls[0])] is not None and mapping[int(box.cls[0])][0] == req.class_id]
        observations = []
        if self.predictor:
            torch = self.torch
            device_cuda = str(self.v['device']) != 'cpu'
            autocast = (torch.autocast('cuda', dtype=torch.bfloat16 if torch.cuda.is_bf16_supported() else torch.float16)
                        if device_cuda else nullcontext())
            with torch.inference_mode(), autocast:
                self.predictor.set_image(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
                for index, box in boxes:
                    masks, scores, _ = self.predictor.predict(box=np.asarray(box.xyxy[0].tolist()), multimask_output=False)
                    if float(scores[0]) >= self.v['sam_min_score']:
                        observations.append(self.observation(color, depth, info, req, box, masks[0], float(scores[0])))
        else:
            require(result.masks is not None, 'SEGMENTATION_MASK_MISSING')
            for index, box in boxes:
                mask = np.zeros(depth.shape, np.uint8)
                cv2.fillPoly(mask, [np.rint(result.masks.xy[index]).astype(np.int32)], 255)
                observations.append(self.observation(color, depth, info, req, box, mask, 1.))
        fresh(stamp, self.get_clock().now().nanoseconds / 1e9, self.c['max_observation_age_s'])
        for index, obs in enumerate(observations):
            obs.candidate_count, obs.candidate_index = len(observations), index
        return observations

    def observation(self, color, depth, info, req, box, mask, mask_score):
        require(np.asarray(mask).shape == depth.shape, 'MASK_GRID_MISMATCH')
        yaw=None
        if self.c['classes']['banana']['class_id']==req.class_id:
            tf=self.tf.lookup_transform(self.c['gravity_frame'],color.header.frame_id,
                Time.from_msg(color.header.stamp),timeout=Duration(seconds=.1))
            r=tf.transform.rotation
            p,std,yaw,details=banana_point_axis(depth,mask,info,self.c,self.v['position_error_floor_m'],
                rotation([r.x,r.y,r.z,r.w]),self.v['banana'])
            self.get_logger().info('Banana PCA/body: '+str(details)+' yaw='+str(yaw))
        else:
            p, std = surface_point(depth, np.asarray(mask).astype(bool), info, self.c, self.v['position_error_floor_m'])
        require(std <= self.c['max_position_std_m'], 'DEPTH_POSITION_QUALITY')
        obs = WristObservation(header=color.header, task_id=req.task_id, observation_id=req.observation_id,
            class_id=req.class_id, confidence=min(float(box.conf[0]), mask_score),
            position_kind='SURFACE', has_position=True, position_std_m=float(std))
        # A new wrist view has no externally proven Head track identity. Geometry
        # association considers all candidates; no copied/fabricated instance ID.
        obs.instance_id = ''
        if yaw is not None: obs.axis_valid=True; obs.axis_yaw=yaw
        obs.position.x, obs.position.y, obs.position.z = map(float, p)
        return obs

    def work(self):
        while not self.closed.wait(.03):
            with self.mutex:
                req = self.req; snapshot = dict(self.frames)
                previous = self.last_capture
            if req is None or not all(k in snapshot for k in ('color', 'depth', 'info')): continue
            stamp = seconds(snapshot['color'].header.stamp)
            if stamp <= max(previous, seconds(req.header.stamp)): continue
            with self.mutex: self.last_capture = stamp
            try:
                observations = self.infer(snapshot['color'], snapshot['depth'], snapshot['info'], req)
                with self.mutex:
                    current = self.req
                    if current is None or (current.task_id, current.observation_id) != (req.task_id, req.observation_id): continue
                for obs in observations: self.results.publish(obs)
                self.worker_error = '' if observations else 'NO_REQUESTED_CLASS'
            except Exception as exc:
                self.worker_error = str(exc)
                self.get_logger().warning('Wrist input rejected: ' + str(exc))


def main():
    rclpy.init()
    node = None
    try:
        node = WristFrontend()
        rclpy.spin(node)
    finally:
        if node:
            node.closed.set(); node.worker.join(timeout=5.)
            node.destroy_node()
        rclpy.shutdown()
