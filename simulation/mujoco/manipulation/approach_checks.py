"""Geometric early rejection only; MoveIt still checks the entire robot path."""
import numpy as np

def segment_intersects_box(start,end,center,half_size):
    start=np.asarray(start,dtype=float);delta=np.asarray(end,dtype=float)-start
    lower=np.asarray(center)-np.asarray(half_size);upper=np.asarray(center)+np.asarray(half_size)
    lo,hi=0.,1.
    for i in range(3):
        if abs(delta[i])<1e-12:
            if not lower[i]<=start[i]<=upper[i]:return False
        else:
            values=sorted(((lower[i]-start[i])/delta[i],(upper[i]-start[i])/delta[i]))
            lo=max(lo,values[0]);hi=min(hi,values[1])
            if lo>hi:return False
    return True
