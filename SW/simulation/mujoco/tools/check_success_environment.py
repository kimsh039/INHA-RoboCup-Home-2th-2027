# 성공 실행의 CPU/ROS 환경을 확인한다. CUDA는 로컬에서 필요하지 않다.
import sys, importlib.metadata
import rclpy
from moveit_msgs.msg import PlanningScene
from control_msgs.action import FollowJointTrajectory
print("Python:", sys.version)
for name in ("mujoco", "numpy", "scipy"):
    print(name, importlib.metadata.version(name))
print("ROS 메시지 import: PASS")
