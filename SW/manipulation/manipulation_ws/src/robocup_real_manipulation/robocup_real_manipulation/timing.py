"""Bounded latest-sample telemetry; source stamps are never rewritten."""
import json
import threading
from std_msgs.msg import String

class Timing:
    def __init__(self,node):
        self.node=node; self.values={}; self.mutex=threading.Lock()
        self.pub=node.create_publisher(String,'/manipulation/timing',1)
        self.timer=node.create_timer(1.,self.publish)
    def sample(self,channel,capture,received,**extra):
        with self.mutex:
            self.values[channel]=dict(source_stamp=capture,received_stamp=received,
                source_to_receive_s=received-capture,**extra)
    def publish(self):
        with self.mutex: value=dict(self.values)
        self.pub.publish(String(data=json.dumps(dict(node=self.node.get_name(),channels=value))))
