"""ROS request adapter using existing YOLO and generic SAM2 weights.

RealSense ROS owns the camera. No pyrealsense2 pipeline, camera reopening, arm
commands, fabricated classes, or copied coarse positions appear in this node.
"""
from contextlib import nullcontext
from collections import deque
import json
from pathlib import Path
from collections import deque
import json
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
from .integration import ProcessOwner
from .timing import Timing
from .wrist_geometry import surface_point, banana_point_axis


class WristFrontend(Node):
    def __init__(self):
        super().__init__('real_wrist_frontend')
        self.c = read_config(self)
        repo = Path(self.declare_parameter('repo', '').value).expanduser().resolve()
        vision_file = Path(self.declare_parameter('vision_config', '').value).expanduser()
        self.v = yaml.safe_load(vision_file.read_text())
        require(isinstance(self.v.get('max_candidates',8),int) and 1<=self.v.get('max_candidates',8)<=8,'CANDIDATE_LIMIT')
        require(isinstance(self.v.get('max_frames_per_request',12),int) and 3<=self.v.get('max_frames_per_request',12)<=24,'REQUEST_FRAME_LIMIT')
        if self.c['classes']['banana']['enabled']:
            require(isinstance(self.v.get('banana'),dict),'BANANA_VISION_CONFIG_MISSING:copy banana settings from updated wrist.yaml')
        self.owner=ProcessOwner(self.c.get('wrist_owner_path','~/.ros/robocup_real_manipulation/wrist.owner'))
        self.timing=Timing(self)
        self.bridge = CvBridge()
        self.tf=Buffer(); self.tf_listener=TransformListener(self.tf,self)
        self.mutex = threading.Lock()
        self.frames = {}; self.history={k:deque(maxlen=6) for k in ('color','depth','info')}; self.received={}
        self.req_started=0.; self.attempted_frames=0; self.seen_requests=deque(maxlen=128)
        self.req = None; self.last_capture = 0.; self.worker_error = ''
        self.metrics={}; self.latencies=deque(maxlen=100)
        self.closed = threading.Event()
        self.results = self.create_publisher(WristObservation, '/manipulation/wrist/result', 10)
        self.debug=self.create_publisher(Image,'/manipulation/wrist/debug_image',1)
        self.mask_pub=self.create_publisher(Image,'/manipulation/wrist/mask',1)
        self.diagnostics = self.create_publisher(String, '/manipulation/wrist/status', 10)
        self.profile_pub=self.create_publisher(String,'/manipulation/wrist/profile',1)
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
            self.history[kind].append(msg)
            self.received[kind]=self.get_clock().now().nanoseconds/1e9
            self.received[(kind,msg.header.stamp.sec,msg.header.stamp.nanosec)]=self.received[kind]
            valid={(k,m.header.stamp.sec,m.header.stamp.nanosec) for k,items in self.history.items() for m in items}
            for key in list(self.received):
                if isinstance(key,tuple) and key not in valid: del self.received[key]
            self.timing.sample('camera_'+kind,seconds(msg.header.stamp),self.received[kind])

    def request(self, msg):
        try:
            fresh(seconds(msg.header.stamp),self.get_clock().now().nanoseconds/1e9,self.c['max_observation_age_s'])
            require(msg.task_id and msg.observation_id and 0<=msg.class_id<len(self.c['model_names']),'INVALID_REQUEST')
        except Exception as exc:
            self.get_logger().warning(str(exc)); return
        with self.mutex:
            if self.req is not None and self.req.task_id!=msg.task_id:
                self.get_logger().warning('WRIST_BUSY'); return
            if self.req is None or (msg.task_id, msg.observation_id) != (self.req.task_id, self.req.observation_id):
                key=(msg.task_id,msg.observation_id)
                if key in self.seen_requests: return
                self.seen_requests.append(key)
                self.req = msg
                self.req_started=time.monotonic(); self.attempted_frames=0
                self.last_capture = seconds(msg.header.stamp)
                self.worker_error = ''

    def task_status(self,msg):
        with self.mutex:
            if self.req and msg.task_id==self.req.task_id and seconds(msg.header.stamp)>=seconds(self.req.header.stamp) and (msg.stage!='WRIST_REDETECT' or msg.outcome!='RUNNING'):
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
        with self.mutex: req=self.req
        self.diagnostics.publish(String(data=json.dumps(dict(state=text,valid=ready and not bool(self.worker_error),reason=self.worker_error,
            task_id=req.task_id if req else '',observation_id=req.observation_id if req else '',
            class_id=req.class_id if req else -1,frame=color.header.frame_id if color else '',
            capture_stamp=seconds(color.header.stamp) if color else 0.))))
        with self.mutex: metrics=dict(self.metrics)
        if metrics: self.profile_pub.publish(String(data=json.dumps(metrics,allow_nan=False)))

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
        inference_started=time.monotonic()
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
        require(len(boxes)<=self.v.get('max_candidates',8),'TOO_MANY_CANDIDATES')
        # CPU box transfer synchronizes YOLO before measuring its latency.
        boxes=[(i,box.cpu()) for i,box in boxes]
        yolo_ms=(time.monotonic()-inference_started)*1000.
        sam_started=time.monotonic()
        observations = []
        if not boxes:
            self.record_profile(stamp,yolo_ms,0.,0)
            return []
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
        self.record_profile(stamp,yolo_ms,(time.monotonic()-sam_started)*1000.,len(boxes))
        for index, obs in enumerate(observations):
            obs.candidate_count, obs.candidate_index = len(observations), index
            obs.aligned_depth=depth_msg
        return observations

    def record_profile(self,stamp,yolo_ms,mask_depth_ms,candidates):
        total=yolo_ms+mask_depth_ms
        with self.mutex:
            self.latencies.append(total)
            self.metrics=dict(source_stamp=stamp,source_age_ms=(self.get_clock().now().nanoseconds/1e9-stamp)*1000.,
                yolo_ms=yolo_ms,mask_depth_ms=mask_depth_ms,perception_ms=total,candidates=candidates,
                p50_ms=float(np.percentile(self.latencies,50)),p95_ms=float(np.percentile(self.latencies,95)),
                samples=len(self.latencies),mask_source=self.v['mask_source'],
                detector_backend=Path(self.v['yolo_model']).suffix,
                budget_ms=float(self.v.get('latency_budget_ms',50.)),
                within_budget=total<=self.v.get('latency_budget_ms',50.))

    def observation(self, color, depth, info, req, box, mask, mask_score):
        require(np.asarray(mask).shape == depth.shape, 'MASK_GRID_MISMATCH')
        binary=np.asarray(mask).astype(bool)
        require(binary.any(),'EMPTY_MASK')
        require(not (binary[0].any() or binary[-1].any() or binary[:,0].any() or binary[:,-1].any()),'OBJECT_CLIPPED_BY_IMAGE')
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
        obs.target_mask=self.bridge.cv2_to_imgmsg(binary.astype(np.uint8)*255,'mono8')
        obs.target_mask.header=color.header; obs.camera_info=info
        obs.instance_id = ''
        if yaw is not None: obs.axis_valid=True; obs.axis_yaw=yaw
        obs.position.x, obs.position.y, obs.position.z = map(float, p)
        return obs

    def work(self):
        while not self.closed.wait(.02):
            with self.mutex:
                req=self.req
                if req is None: continue
                if time.monotonic()-self.req_started>self.c['observation_timeout_s'] or self.attempted_frames>=self.v.get('max_frames_per_request',12):
                    self.req=None; self.worker_error='REQUEST_EXPIRED'; continue
                # Choose newest COMPLETE tuple; never queue inference jobs.
                chosen=None
                for color in reversed(self.history['color']):
                    stamp=seconds(color.header.stamp)
                    if stamp<=max(self.last_capture,seconds(req.header.stamp)): break
                    depth=min(self.history['depth'],key=lambda m:abs(seconds(m.header.stamp)-stamp),default=None)
                    info=min(self.history['info'],key=lambda m:abs(seconds(m.header.stamp)-stamp),default=None)
                    if depth is not None and info is not None and max(abs(seconds(m.header.stamp)-stamp) for m in (depth,info))<=self.v['sync_tolerance_s']:
                        chosen=(color,depth,info); break
                if chosen is None: continue
                self.last_capture=stamp; self.attempted_frames+=1
                received=self.received[('color',color.header.stamp.sec,color.header.stamp.nanosec)]
            start=self.get_clock().now().nanoseconds/1e9; outcome=''; count=0
            try:
                observations=self.infer(*chosen,req)
                count=len(observations)
                with self.mutex:
                    current=self.req
                    if self.closed.is_set() or current is None or (current.task_id,current.observation_id)!=(req.task_id,req.observation_id): continue
                for obs in observations:
                    self.results.publish(obs); self.mask_pub.publish(obs.target_mask)
                    if self.debug.get_subscription_count():
                        image=self.bridge.imgmsg_to_cv2(chosen[0],'bgr8').copy()
                        mask=self.bridge.imgmsg_to_cv2(obs.target_mask).astype(bool)
                        image[mask]=(image[mask]*.5+np.array([0,120,0])).astype(np.uint8)
                        p=obs.position
                        cv2.putText(image,f'{req.task_id[:8]} class={req.class_id} ({p.x:.3f},{p.y:.3f},{p.z:.3f}) {obs.header.frame_id}',(5,22),cv2.FONT_HERSHEY_SIMPLEX,.4,(0,255,255),1)
                        debug=self.bridge.cv2_to_imgmsg(image,'bgr8'); debug.header=obs.header; self.debug.publish(debug)
                outcome='' if observations else 'NO_REQUESTED_CLASS'
            except Exception as exc:
                outcome=str(exc)
                self.get_logger().warning('Wrist input rejected: '+outcome,throttle_duration_sec=1.)
            finally:
                if self.predictor is not None: self.predictor.reset_image()
                self.worker_error=outcome
                end=self.get_clock().now().nanoseconds/1e9
                self.timing.sample('inference',stamp,received,task_id=req.task_id,observation_id=req.observation_id,
                    inference_start=start,inference_end=end,receive_to_start_s=start-received,
                    inference_s=end-start,capture_to_result_s=end-stamp,candidates=count,reason=outcome)


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
