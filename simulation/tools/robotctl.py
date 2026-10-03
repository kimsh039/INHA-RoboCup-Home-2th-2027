#!/usr/bin/env python3
"""Direct Gazebo Transport terminal controls; no browser or HTTP server needed."""
import argparse
import math
import os
import select
import sys
import termios
import time
import tty

LIMITS=[(-2.6179938,2.6179938),(0,3.1415926),(-2.9670597,0),(-1.7453292,1.7453292),(-1.2217304,1.2217304),(-2.0943951,2.0943951)]
p=argparse.ArgumentParser(description='Gazebo Tracer / Piper terminal controls')
p.add_argument('--partition',default=os.environ.get('GZ_PARTITION','robocup_projectsh_sensor_view_20261003'))
s=p.add_subparsers(dest='command',required=True)
d=s.add_parser('drive'); d.add_argument('linear',type=float,help='m/s'); d.add_argument('angular',type=float,help='rad/s'); d.add_argument('--seconds',type=float,default=1)
s.add_parser('stop'); s.add_parser('home'); s.add_parser('teleop')
j=s.add_parser('joint'); j.add_argument('number',type=int,choices=range(1,7)); j.add_argument('radians',type=float)
g=s.add_parser('grip'); g.add_argument('meters',type=float,help='one finger travel, 0..0.05 m')
a=p.parse_args()
def bounded(value,lo,hi,label):
 if not math.isfinite(value) or not lo<=value<=hi: p.error(f'{label}: expected {lo}..{hi}')
if a.command=='drive':
 bounded(a.linear,-.2,.2,'linear'); bounded(a.angular,-.5,.5,'angular'); bounded(a.seconds,.05,30,'seconds')
if a.command=='joint': bounded(a.radians,*LIMITS[a.number-1],'joint position')
if a.command=='grip': bounded(a.meters,0,.05,'finger travel')
os.environ.setdefault('GZ_IP', '127.0.0.1')
os.environ['GZ_PARTITION']=a.partition
from gz.transport13 import Node
from gz.msgs10.double_pb2 import Double
from gz.msgs10.twist_pb2 import Twist
node=Node(); publishers={}
def publisher(topic,kind):
 if topic not in publishers:
  pub=node.advertise(topic,kind); publishers[topic]=pub
  until=time.monotonic()+4
  while not pub.has_connections() and time.monotonic()<until: time.sleep(.05)
  if not pub.has_connections(): raise RuntimeError(f'No Gazebo subscriber on {topic}; check partition {a.partition} and world controllers.')
 return publishers[topic]
def drive(v,w):
 msg=Twist(); msg.linear.x=v; msg.angular.z=w
 publisher('/robocup/cmd_vel',Twist).publish(msg)
def joint(name,q):
 msg=Double(); msg.data=q
 publisher(f'/model/robocup/joint/{name}/0/cmd_pos',Double).publish(msg)
def teleop():
 if not sys.stdin.isatty(): raise RuntimeError('teleop requires an interactive terminal')
 publisher('/robocup/cmd_vel',Twist)
 print('W: forward / S: reverse / A: left / D: right / X or Space: stop / Q: quit\nPress repeatedly to continue; stops after 0.4 seconds without a key.',flush=True)
 old=termios.tcgetattr(sys.stdin); last=0; v=w=0
 try:
  tty.setcbreak(sys.stdin.fileno())
  while True:
   if select.select([sys.stdin],[],[],.05)[0]:
    key=sys.stdin.read(1).lower()
    if key=='q': break
    if key in 'wsadx ':
     v,w={'w':(.15,0),'s':(-.15,0),'a':(0,.4),'d':(0,-.4),'x':(0,0),' ':(0,0)}[key]; last=time.monotonic()
   if time.monotonic()-last>.4: v=w=0
   drive(v,w)
 finally:
  try: drive(0,0)
  finally: termios.tcsetattr(sys.stdin,termios.TCSADRAIN,old)
try:
 if a.command=='drive':
  publisher('/robocup/cmd_vel',Twist)
  try:
   end=time.monotonic()+a.seconds
   while time.monotonic()<end: drive(a.linear,a.angular); time.sleep(.05)
  finally:
   drive(0,0); time.sleep(.1)
 elif a.command=='stop': drive(0,0); time.sleep(.1)
 elif a.command=='joint': joint(f'piper_joint{a.number}',a.radians); time.sleep(.1)
 elif a.command=='grip':
  joint('piper_gripper_joint1',a.meters); joint('piper_gripper_joint2',-a.meters); time.sleep(.1)
 elif a.command=='home':
  for i in range(1,7): joint(f'piper_joint{i}',0)
  for i in (1,2): joint(f'piper_gripper_joint{i}',0)
  drive(0,0); time.sleep(.1)
 elif a.command=='teleop': teleop()
 if a.command!='teleop': print('Command sent:',a.command,'partition:',a.partition)
except KeyboardInterrupt:
 print('\nStopped.')
except RuntimeError as e:
 print(str(e),file=sys.stderr); sys.exit(1)
