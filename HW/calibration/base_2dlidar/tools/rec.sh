#!/bin/bash
# 사용: rec.sh poseNN   (정지 확인 후 10초 녹화)
source /opt/ros/humble/setup.bash; cd ~/livox_g2_calib/bags
N=$1; [ -d "$N" ] && { echo "$N already exists"; exit 1; }
timeout 6 ros2 topic echo --once /odom 2>/dev/null | python3 -c "
import sys,yaml,math; d=yaml.safe_load(sys.stdin.read().split(\"---\")[0]); p=d[\"pose\"][\"pose\"]; q=p[\"orientation\"]; t=d[\"twist\"][\"twist\"]
print(\"odom x=%.3f y=%.3f yaw=%.1fdeg | v=%.3f w=%.3f\"%(p[\"position\"][\"x\"],p[\"position\"][\"y\"],math.degrees(2*math.atan2(q[\"z\"],q[\"w\"]))%360,t[\"linear\"][\"x\"],t[\"angular\"][\"z\"]))"
timeout -s INT 12 ros2 bag record -o $N /livox/lidar /livox/imu /scan /odom > /tmp/rec_$N.log 2>&1
ros2 bag info $N | grep -E "Duration|Count" | sed "s/| Serialization Format: cdr//;s/Topic information://" | awk "{\$1=\$1};1"
# 끊김 간이 검사: 10초 녹화면 점군 ~109개가 정상
C=$(ros2 bag info $N | grep "/livox/lidar" | sed "s/.*Count: \([0-9]*\).*/\1/")
[ "$C" -ge 104 ] && echo "CHECK OK (lidar msgs=$C)" || echo "CHECK FAIL (lidar msgs=$C) -> 다시 녹화 권장"
