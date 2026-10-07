"""Left is +Y of the supplied chassis frame; world and chassis yaw are aligned."""
import numpy as np

def place_targets(T_world_grasp_tcp, T_world_base, distance=.10):
    if not np.isfinite(distance) or distance<=0:raise ValueError('Invalid placement offset')
    left=np.asarray(T_world_base)[:3,1]
    targets=[]
    for side,sign in (("left",1),("right",-1)):
        T=np.asarray(T_world_grasp_tcp).copy();T[:3,3]+=sign*distance*left
        targets.append((side,T))
    return targets
