from contextlib import nullcontext
from pathlib import Path
from types import SimpleNamespace as NS
import copy
import unittest
import cv2
import numpy as np
import yaml
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from std_msgs.msg import Header
from robocup_real_manipulation_msgs.msg import ObservationRequest
from robocup_real_manipulation.wrist_frontend import WristFrontend
from robocup_real_manipulation.core import Fault

PACKAGE=Path(__file__).resolve().parents[1]
CFG=yaml.safe_load((PACKAGE/'config/robot.yaml').read_text())


class WristFrontendTests(unittest.TestCase):
    def setup_infer(self,now_sequence=None):
        header=Header(frame_id='wrist_color_optical_frame'); header.stamp.sec=100
        color=Image(header=header,encoding='bgr8',width=160,height=120)
        depth=Image(header=copy.deepcopy(header),encoding='16UC1',width=160,height=120)
        info=CameraInfo(header=copy.deepcopy(header),width=160,height=120,
            k=[200.,0.,80.,0.,200.,60.,0.,0.,1.],d=[],distortion_model='plumb_bob')
        masks=[]
        for x in (50,110):
            mask=np.zeros((120,160),bool)
            canvas=np.zeros(mask.shape,np.uint8); cv2.circle(canvas,(x,60),12,1,-1)
            masks.append(canvas.astype(bool))
        def box(x): return NS(cls=np.array([0]),conf=np.array([.95]),xyxy=np.array([[x-12,48,x+12,72]]))
        result=NS(names=dict(enumerate(CFG['model_names'])),boxes=[box(50),box(110)])
        queue=iter(now_sequence or [100.,100.])
        node=NS(c=copy.deepcopy(CFG),v={'optical_frame':'wrist_color_optical_frame','sync_tolerance_s':.025,
                'device':'cpu','nms_iou':.45,'imgsz':640,'sam_min_score':.85,'position_error_floor_m':.003},
            get_clock=lambda:NS(now=lambda:NS(nanoseconds=int(next(queue)*1e9))),
            detector=NS(model=NS(predict=lambda *a,**kw:[result])),
            torch=NS(inference_mode=nullcontext),
            bridge=NS(cv2_to_imgmsg=CvBridge().cv2_to_imgmsg,imgmsg_to_cv2=lambda msg,*a:np.zeros((120,160,3),np.uint8) if msg.encoding=='bgr8' else np.full((120,160),300,np.uint16)))
        node.c['camera_min_depth_m']=.1
        count=iter(masks)
        node.predictor=NS(set_image=lambda rgb:None,predict=lambda **kw:(np.array([next(count)]),np.array([.9]),None))
        node.observation=lambda *a:WristFrontend.observation(node,*a)
        req=ObservationRequest(task_id='real-task',observation_id='fresh-request',class_id=0,instance_id='head-track')
        return node,color,depth,info,req

    def test_every_real_candidate_keeps_acquisition_and_request_ids_without_fake_track_identity(self):
        node,color,depth,info,req=self.setup_infer()
        outputs=WristFrontend.infer(node,color,depth,info,req)
        self.assertEqual(len(outputs),2)
        self.assertEqual([o.candidate_index for o in outputs],[0,1])
        for output in outputs:
            self.assertEqual(output.candidate_count,2)
            self.assertEqual(output.header.stamp.sec,100)
            self.assertEqual(output.task_id,'real-task')
            self.assertEqual(output.observation_id,'fresh-request')
            self.assertEqual(output.position_kind,'SURFACE')
            self.assertEqual(output.instance_id,'')
        self.assertLess(outputs[0].position.x,outputs[1].position.x)

    def test_slow_inference_is_rejected_without_relabeling_its_capture_time(self):
        node,color,depth,info,req=self.setup_infer([100.,104.])
        with self.assertRaisesRegex(Fault,'STALE'): WristFrontend.infer(node,color,depth,info,req)
        self.assertEqual(color.header.stamp.sec,100)

    def test_unaligned_or_unsynchronized_depth_is_rejected_before_inference(self):
        node,color,depth,info,req=self.setup_infer()
        depth.header.frame_id='wrist_depth_optical_frame'
        with self.assertRaisesRegex(Fault,'ALIGNED'): WristFrontend.infer(node,color,depth,info,req)
        depth.header.frame_id=color.header.frame_id; depth.header.stamp.nanosec=50000000
        with self.assertRaisesRegex(Fault,'UNSYNCHRONIZED'): WristFrontend.infer(node,color,depth,info,req)


if __name__=='__main__': unittest.main()
