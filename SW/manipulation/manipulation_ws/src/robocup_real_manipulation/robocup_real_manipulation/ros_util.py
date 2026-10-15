import time
import yaml
import rclpy
from rclpy.executors import MultiThreadedExecutor
from .core import require, Fault

def seconds(t): return t.sec+t.nanosec/1e9

# [PARAMETER_ENTRY] robot.measured.yaml + optional tuning_file; read once at startup.
def read_config(node):
    node.declare_parameter('config','')
    with open(node.get_parameter('config').value) as f: cfg=yaml.safe_load(f)
    node.declare_parameter('tuning_file','')
    path=node.get_parameter('tuning_file').value
    if path:
        from .tuning import overlay
        with open(path) as f: cfg=overlay(cfg,yaml.safe_load(f))
    return cfg

def wait_future(future, timeout, guard=lambda:None):
    deadline=time.monotonic()+timeout
    while not future.done():
        guard(); require(time.monotonic()<deadline,'TIMEOUT'); time.sleep(0.01)
    guard()
    return future.result()

def action_call(client, goal, timeout, guard, allow_aborted=False):
    require(client.wait_for_server(timeout_sec=1.0),'ACTION_UNAVAILABLE')
    sent=client.send_goal_async(goal)
    try:
        handle=wait_future(sent,timeout,guard)
    except Exception:
        # If acceptance arrives after caller aborts, cancel it too.
        def cancel_late(f):
            try:
                h=f.result()
                if h and h.accepted: h.cancel_goal_async()
            except Exception: pass
        sent.add_done_callback(cancel_late)
        raise
    require(handle.accepted,'GOAL_REJECTED')
    try:
        result=wait_future(handle.get_result_async(),timeout,guard)
        require(result.status==4 or (allow_aborted and result.status==6),'ACTION_NOT_SUCCEEDED')
        return result.result
    except Exception:
        handle.cancel_goal_async()
        raise

def spin(node):
    ex=MultiThreadedExecutor(num_threads=8); ex.add_node(node)
    try: ex.spin()
    finally:
        ex.shutdown(timeout_sec=3)
        pool=getattr(ex,'_executor',None)
        if pool is not None: pool.shutdown(wait=True)
        node.destroy_node(); rclpy.shutdown()
