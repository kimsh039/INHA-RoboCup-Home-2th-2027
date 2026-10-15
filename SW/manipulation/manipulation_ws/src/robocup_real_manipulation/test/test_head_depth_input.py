import copy
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
import numpy as np
import yaml
from sensor_msgs.msg import Image,CameraInfo
from std_msgs.msg import Header
from robocup_detection_msgs.msg import HeadTarget
from robocup_real_manipulation.head_input_adapter import HeadInput,freeze_point
from robocup_real_manipulation.external_input import capture_key

CFG=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/robot.yaml').read_text())


class HeadDepthTests(unittest.TestCase):
    def test_preapproach_coordinate_is_frozen_in_odom_without_refreshing_capture_stamp(self):
        from geometry_msgs.msg import PointStamped
        header=Header(frame_id='base_link'); header.stamp.sec=10
        point=PointStamped(header=header); point.point.x=.5
        tf=NS(translation=NS(x=1.,y=2.,z=0.),rotation=NS(x=0.,y=0.,z=0.,w=1.))
        frozen=freeze_point(point,'odom',tf)
        self.assertEqual(frozen.header.frame_id,'odom')
        self.assertEqual(frozen.header.stamp.sec,10)
        self.assertEqual(frozen.point.x,1.5)
        self.assertEqual(frozen.point.y,2.)
        self.assertEqual(point.header.frame_id,'base_link')
    def receiver(self,depth_mm):
        header=Header(frame_id='camera_color_optical_frame'); header.stamp.sec=100
        head=HeadTarget(header=header,class_id=0,class_name='green_apple',target_id='existing-head-id',
                        valid=True,measured=True,confidence=.95,x=60.,y=40.,width=40.,height=40.)
        info=CameraInfo(header=copy.deepcopy(header),width=160,height=120,
                        k=[200.,0.,80.,0.,200.,60.,0.,0.,1.],d=[],distortion_model='plumb_bob')
        depth=Image(header=copy.deepcopy(header),encoding='16UC1')
        key=capture_key(header); outputs=[]; warnings=[]
        c=copy.deepcopy(CFG); c['table_z_m']=.75
        transform=NS(transform=NS(translation=NS(x=.5,y=0.,z=1.2),rotation=NS(x=1.,y=0.,z=0.,w=0.)))
        node=NS(c=c,heads={key:head},depths={key:depth},infos={key:info},
                cv=NS(imgmsg_to_cv2=lambda msg:np.full((120,160),depth_mm,np.uint16)),
                tf=NS(lookup_transform=lambda *a,**kw:transform),
                receive=lambda kind,msg:outputs.append(msg),get_logger=lambda:NS(warning=warnings.append))
        return node,key,outputs,warnings

    def test_existing_box_plus_real_depth_keeps_stamp_and_estimates_coarse_tabletop_center(self):
        node,key,outputs,warnings=self.receiver(382)
        HeadInput.make_depth_point(node,key)
        self.assertFalse(warnings)
        self.assertEqual(len(outputs),1)
        self.assertEqual(outputs[0].header.stamp.sec,100)
        self.assertEqual(outputs[0].header.frame_id,'base_link')
        self.assertLess(abs(outputs[0].point.x-.5),.002)
        self.assertAlmostEqual(outputs[0].point.z,.784,places=4)

    def test_bbox_background_below_support_is_not_published_as_an_object(self):
        node,key,outputs,warnings=self.receiver(1200)
        HeadInput.make_depth_point(node,key)
        self.assertFalse(outputs)
        self.assertTrue(warnings)


if __name__=='__main__': unittest.main()
