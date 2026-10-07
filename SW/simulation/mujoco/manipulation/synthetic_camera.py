"""Explicit virtual camera coordinates for a synthetic convex cube view.
Camera X right, Y down, Z forward. Output grasps must be mapped back to object.
This is not a real camera acquisition or a claim that inference will succeed.
"""
import numpy as np

def camera_transform(camera_position_object):
    camera=np.asarray(camera_position_object,dtype=float)
    forward=-camera/np.linalg.norm(camera)
    right=np.cross(forward,[0,0,1.])
    if np.linalg.norm(right)<1e-8:raise ValueError('Use an oblique virtual camera')
    right/=np.linalg.norm(right);down=np.cross(forward,right)
    T=np.eye(4);T[:3,:3]=np.stack([right,down,forward]);T[:3,3]=-T[:3,:3]@camera
    return T

def visible_cube_surface(cloud,dimensions,camera_position_object):
    points=np.asarray(cloud);half=np.asarray(dimensions)/2;camera=np.asarray(camera_position_object)
    visible=np.zeros(len(points),dtype=bool)
    for axis in range(3):
        for sign in (-1,1):
            surface=np.isclose(points[:,axis],sign*half[axis],atol=1e-8,rtol=0)
            visible|=surface & (sign*(camera[axis]-points[:,axis])>0)
    if visible.sum()<2048:raise ValueError('Not enough points in synthetic view')
    return points[visible]

def grasps_camera_to_object(raw,T_camera_object):
    output=np.array(raw,copy=True)
    inverse=np.linalg.inv(T_camera_object)
    R=output[:,4:13].reshape(-1,3,3)
    output[:,4:13]=np.einsum('ij,njk->nik',inverse[:3,:3],R).reshape(-1,9)
    output[:,13:16]=output[:,13:16]@inverse[:3,:3].T+inverse[:3,3]
    return output
