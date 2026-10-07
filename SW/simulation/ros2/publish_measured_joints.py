#!/usr/bin/env python3
"""Display-only joint-state bridge: Gazebo CLI measured positions -> ROS. Wall-clock timestamps."""
import json, signal, subprocess, threading
import rclpy
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import JointState

def main():
    rclpy.init();node=rclpy.create_node('gazebo_measured_joint_display');pub=node.create_publisher(JointState,'/joint_states',10)
    proc=subprocess.Popen(['gz','topic','-e','--json-output','-t','/robocup/joint_states'],stdout=subprocess.PIPE,text=True,start_new_session=True)
    stopped=threading.Event()
    def read():
        buffer='';decoder=json.JSONDecoder()
        for line in proc.stdout:
            if stopped.is_set():break
            buffer+=line
            while buffer.strip():
                buffer=buffer.lstrip()
                try:v,end=decoder.raw_decode(buffer)
                except json.JSONDecodeError:break
                buffer=buffer[end:];joints=v.get('joint',[])
                if not joints:continue
                msg=JointState();msg.header.stamp=node.get_clock().now().to_msg()
                for j in joints:
                    if not j.get('name'):continue
                    # Protobuf JSON may omit default zero scalar fields.
                    axis=j.get('axis1',{})
                    msg.name.append(j['name']);msg.position.append(float(axis.get('position',0)));msg.velocity.append(float(axis.get('velocity',0)))
                if rclpy.ok() and msg.name:pub.publish(msg)
            if len(buffer)>16_000_000:
                node.get_logger().error('Malformed Gazebo JSON stream');break
    thread=threading.Thread(target=read,daemon=True);thread.start()
    node.get_logger().info('Measured joint display; wall-clock timestamps. This is not an image/cloud bridge or synchronized acquisition.')
    try:rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        signal.signal(signal.SIGINT,signal.SIG_IGN);stopped.set()
        if proc.poll() is None:
            proc.terminate()
            try:proc.wait(timeout=3)
            except subprocess.TimeoutExpired:proc.kill();proc.wait()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
