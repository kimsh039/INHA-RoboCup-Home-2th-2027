#!/usr/bin/env python3
"""Local Gazebo controls. Commands only target the robocup_motion partition."""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json, subprocess, os, threading, time
from pathlib import Path
os.environ['GZ_PARTITION']='robocup_motion'
os.environ.setdefault('GZ_IP','127.0.0.1')  # keep discovery off the LAN
from gz.transport13 import Node
from gz.msgs10.double_pb2 import Double
from gz.msgs10.twist_pb2 import Twist
node=Node()
lock=threading.Lock()
drive_until=0
drive_pub=node.advertise('/robocup/cmd_vel',Twist)
joint_pubs={name:node.advertise(f'/model/robocup/joint/{name}/0/cmd_pos',Double) for name in [*[f'piper_joint{i}' for i in range(1,7)],'piper_gripper_joint1','piper_gripper_joint2']}
def drive(v,w):
 msg=Twist(); msg.linear.x=v; msg.angular.z=w; drive_pub.publish(msg)
def joint(name,value):
 msg=Double(); msg.data=value; joint_pubs[name].publish(msg)
def watchdog():
 global drive_until
 while True:
  time.sleep(.1)
  with lock:
   if drive_until and time.monotonic()>drive_until:
    drive_until=0; drive(0,0)
class Handler(BaseHTTPRequestHandler):
 def do_GET(self):
  b=(Path(__file__).parent/'controls.html').read_bytes(); self.send_response(200); self.send_header('Content-Type','text/html; charset=utf-8'); self.end_headers(); self.wfile.write(b)
 def do_POST(self):
  global drive_until
  try:
   a=json.loads(self.rfile.read(int(self.headers['Content-Length'])))
   if a['type']=='drive':
    v=float(a['v']); w=float(a['w']); assert abs(v)<=.2 and abs(w)<=.5
    with lock: drive(v,w); drive_until=time.monotonic()+1 if v or w else 0
   elif a['type']=='joint':
    i=int(a['i']); q=float(a['q']); limits=[(-2.617,2.617),(0,3.141),(-2.967,0),(-1.745,1.745),(-1.221,1.221),(-2.094,2.094)]
    assert 1<=i<=6 and limits[i-1][0]<=q<=limits[i-1][1]; joint(f'piper_joint{i}',q)
   elif a['type']=='grip':
    q=float(a['q']); assert 0<=q<=.05; joint('piper_gripper_joint1',q); joint('piper_gripper_joint2',-q)
   else: raise ValueError('Unknown command')
   self.send_response(200); self.end_headers(); self.wfile.write(b'OK')
  except Exception as e:
   self.send_response(400); self.end_headers(); self.wfile.write(str(e).encode())
threading.Thread(target=watchdog,daemon=True).start()
print('Controls: http://127.0.0.1:8081',flush=True)
ThreadingHTTPServer(('127.0.0.1',8081),Handler).serve_forever()
