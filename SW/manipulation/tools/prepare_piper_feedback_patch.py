#!/usr/bin/env python3
"""Write a reviewable patch; never modifies/starts the user's installed driver."""
import argparse, difflib, subprocess
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('piper_checkout',type=Path);p.add_argument('output',type=Path);a=p.parse_args()
sha=subprocess.check_output(['git','-C',str(a.piper_checkout),'rev-parse','HEAD'],text=True).strip()
if sha!='017ffefa64511bc6325bd77ddc4e16065c152051':raise SystemExit('Pinned driver SHA mismatch; inspect newer API before generating patch')
relative='src/piper/piper/piper_ctrl_single_node.py';original=(a.piper_checkout/relative).read_text()
modified=original.replace("self.joint_feedback_pub = self.create_publisher(JointState, 'joint_states_feedback', 1)","self.joint_feedback_pub = self.create_publisher(JointState, 'joint_states_feedback', 1)\n        self.gripper_feedback_pub = self.create_publisher(JointState, 'gripper_states_feedback', 1)")
modified=modified.replace('new_time = max(self.piper.GetArmJointMsgs().time_stamp, self.piper.GetArmHighSpdInfoMsgs().time_stamp)', 'new_time = self.piper.GetArmJointMsgs().time_stamp')
needle='self.joint_states_feedback.header.stamp = self.joint_states.header.stamp'
modified=modified.replace(needle,needle+'''\n        grip = self.piper.GetArmGripperMsgs()
        grip_msg = JointState()
        grip_msg.header.stamp = self.float_to_ros_time(grip.time_stamp)
        grip_msg.name = ['gripper']
        grip_msg.position = [grip.gripper_state.grippers_angle / 1000000.0]
        grip_msg.effort = [grip.gripper_state.grippers_effort / 1000.0]
        self.gripper_feedback_pub.publish(grip_msg)''')
if modified.count('self.gripper_feedback_pub.publish(grip_msg)')!=1:raise SystemExit('Patch anchor mismatch')
a.output.parent.mkdir(parents=True,exist_ok=True)
a.output.write_text(''.join(difflib.unified_diff(original.splitlines(True),modified.splitlines(True),fromfile='a/'+relative,tofile='b/'+relative)))
print(a.output)
