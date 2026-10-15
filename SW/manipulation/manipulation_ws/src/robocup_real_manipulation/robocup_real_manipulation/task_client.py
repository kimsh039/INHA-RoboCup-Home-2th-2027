"""Task-ID-only physical action client. Never publishes target/approach/lease."""
import argparse
import json
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from robocup_real_manipulation_msgs.action import Manipulate

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--task-id',required=True); parser.add_argument('--timeout',type=float,default=900.)
    args=parser.parse_args(); rclpy.init(args=[]); node=Node('manipulation_task_client')
    client=ActionClient(node,Manipulate,'/manipulation/run'); handle=None
    def wait(future,timeout):
        rclpy.spin_until_future_complete(node,future,timeout_sec=timeout)
        if not future.done(): raise TimeoutError('ACTION_RESPONSE_TIMEOUT')
        return future.result()
    try:
        if not client.wait_for_server(timeout_sec=5.): raise RuntimeError('MANIPULATION_ACTION_UNAVAILABLE')
        handle=wait(client.send_goal_async(Manipulate.Goal(task_id=args.task_id),
            feedback_callback=lambda m:node.get_logger().info(m.feedback.status.stage+':'+m.feedback.status.reason)),5.)
        if not handle.accepted: raise RuntimeError('TASK_REJECTED:inspect /manipulation/status and coordinator log')
        result=wait(handle.get_result_async(),args.timeout).result.status
        print(json.dumps({k:getattr(result,k) for k in ('task_id','stage','outcome','reason','holding','place_success','return_success','stop_confirmed')}))
        if result.outcome!='SUCCESS': raise SystemExit(2)
    except (KeyboardInterrupt,TimeoutError):
        if handle and handle.accepted:
            wait(handle.cancel_goal_async(),3.)
            try: wait(handle.get_result_async(),5.)
            except TimeoutError: node.get_logger().error('CANCEL_RESULT_UNCONFIRMED:keep base locked; inspect hardware')
        raise SystemExit(130)
    finally:
        node.destroy_node(); rclpy.shutdown()
