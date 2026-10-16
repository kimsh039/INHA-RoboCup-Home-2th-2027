"""Shared continuous splines for MoveIt preflight and the measured Piper bridge.

Use quintics when both derivatives are supplied, cubic Hermite otherwise.
Missing velocities use monotone harmonic tangents, with stationary endpoints.
All derivative/position extrema are evaluated analytically, not at servo ticks.
"""
import numpy as np
from .core import require, finite
from .startup_bounds import check_trajectory_bounds


def stamp_seconds(stamp):
    return stamp.sec+stamp.nanosec/1e9


def tangents(times, points):
    dt=np.diff(times); slope=np.diff(points,axis=0)/dt[:,None]
    v=np.zeros_like(points)
    for i in range(1,len(points)-1):
        same=slope[i-1]*slope[i]>0
        w1=2*dt[i]+dt[i-1]; w2=dt[i]+2*dt[i-1]
        v[i,same]=(w1+w2)/(w1/slope[i-1,same]+w2/slope[i,same])
    return v


def derivative(c):
    return c[1:]*np.arange(1,len(c))[:,None]


def roots(c):
    out=[]
    for j in range(c.shape[1]):
        for r in np.polynomial.polynomial.polyroots(c[:,j]):
            if abs(r.imag)<1e-8 and 0<r.real<1: out.append(float(r.real))
    return out


class Spline:
    def __init__(self,times,points,velocities=None,accelerations=None):
        self.times=np.asarray(times,float); self.points=np.asarray(points,float)
        require(len(self.times)>=2 and self.points.shape==(len(self.times),6) and
                finite(self.times) and finite(self.points) and self.times[0]==0 and
                np.all(np.diff(self.times)>0),'TRAJECTORY_TIMES')
        v=tangents(self.times,self.points) if velocities is None else np.asarray(velocities,float)
        require(v.shape==self.points.shape and finite(v),'TRAJECTORY_VELOCITIES')
        require(np.max(np.abs(v[[0,-1]]))<=1e-6,'TRAJECTORY_ENDPOINT_SPEED')
        a=None if accelerations is None else np.asarray(accelerations,float)
        require(a is None or (a.shape==v.shape and finite(a)),'TRAJECTORY_ACCELERATIONS')
        self.coefficients=[]
        for i,h in enumerate(np.diff(self.times)):
            delta=self.points[i+1]-self.points[i]; v0=v[i]*h; v1=v[i+1]*h
            if a is None:
                c=np.array([self.points[i],v0,3*delta-2*v0-v1,-2*delta+v0+v1])
            else:
                a0=a[i]*h*h; a1=a[i+1]*h*h
                c=np.array([self.points[i],v0,a0/2,
                            10*delta-6*v0-4*v1-1.5*a0+.5*a1,
                            -15*delta+8*v0+7*v1+1.5*a0-a1,
                            6*delta-3*v0-3*v1-.5*a0+.5*a1])
            self.coefficients.append(c)

    def evaluate(self,t,order=0):
        i=min(len(self.coefficients)-1,max(0,int(np.searchsorted(self.times,t,side='right')-1)))
        h=self.times[i+1]-self.times[i]; u=np.clip((t-self.times[i])/h,0,1)
        c=self.coefficients[i]
        for _ in range(order): c=derivative(c)/h
        return np.polynomial.polynomial.polyval(u,c)

    def extrema(self,order):
        peak=0.
        for c,h in zip(self.coefficients,np.diff(self.times)):
            for _ in range(order): c=derivative(c)/h
            for u in [0.,1.]+roots(derivative(c)):
                peak=max(peak,float(np.max(np.abs(np.polynomial.polynomial.polyval(u,c)))))
        return peak

    def positions(self,step=None):
        samples=[]
        for i,(c,h) in enumerate(zip(self.coefficients,np.diff(self.times))):
            us=[0.,1.]+roots(derivative(c))
            if step is not None:
                # Bound joint displacement between collision samples by the
                # polynomial's derivative envelope (triangle inequality).
                bound=float(np.max(np.sum(np.abs(derivative(c)),axis=0)))
                us.extend(np.linspace(0.,1.,max(2,int(np.ceil(bound/step))+1)))
            samples.extend(np.polynomial.polynomial.polyval(u,c) for u in sorted(set(us)))
        return np.asarray(samples)

    def validate(self,cfg,current,limits=True):
        require(self.times[-1]<cfg['stage_timeout_s']-cfg['joint_stable_s'],'TRAJECTORY_TIMES')
        check_trajectory_bounds(self.positions(),np.asarray(current),cfg)
        if limits:
            require(self.extrema(1)<=cfg['max_joint_speed_rad_s']*(1+1e-8),'TRAJECTORY_SPEED')
            require(self.extrema(2)<=cfg['max_joint_acceleration_rad_s2']*(1+1e-8),'TRAJECTORY_ACCELERATION')


def from_message(traj,cfg,current,check_limits=True):
    require(current is not None,'JOINTS_NOT_READY')
    require(list(traj.joint_names)==cfg['joint_names'] and len(traj.points)>0,'TRAJECTORY_NAMES')
    require(stamp_seconds(traj.header.stamp)==0,'SCHEDULED_TRAJECTORY_UNSUPPORTED')
    times=[stamp_seconds(p.time_from_start) for p in traj.points]
    points=[list(p.positions) for p in traj.points]
    require(times[0]>=0 and np.all(np.diff(times)>0),'TRAJECTORY_TIMES')
    require(all(len(q)==6 for q in points),'TRAJECTORY_SHAPE')
    def fields(name):
        values=[list(getattr(p,name)) for p in traj.points]
        require(all(len(v)==0 for v in values) or all(len(v)==6 for v in values),'TRAJECTORY_'+name.upper())
        return values if values[0] else None
    velocities=fields('velocities'); accelerations=fields('accelerations')
    require(accelerations is None or velocities is not None,'TRAJECTORY_DERIVATIVES')
    if times[0]>0:
        times.insert(0,0.); points.insert(0,list(current))
        if velocities is not None: velocities.insert(0,[0.]*6)
        if accelerations is not None: accelerations.insert(0,[0.]*6)
    else:
        require(np.max(np.abs(np.asarray(points[0])-current))<=cfg['goal_tolerance_rad'],'TRAJECTORY_START')
    if len(times)==1:
        times.append(.01); points.append(points[0])
        if velocities is not None: velocities.append([0.]*6)
        if accelerations is not None: accelerations.append([0.]*6)
    spline=Spline(times,points,velocities,accelerations)
    spline.validate(cfg,current,check_limits)
    return spline


def retime(traj,cfg,current):
    # Recover only the intentional t=0 measured startup prefix inserted by
    # plan_to. Reject every other duplicate/nonmonotonic timestamp.
    points=traj.points
    require(bool(points),'EMPTY_PLAN')
    if len(points)>1 and stamp_seconds(points[0].time_from_start)==stamp_seconds(points[1].time_from_start)==0:
        delta=np.max(np.abs(np.asarray(points[1].positions)-points[0].positions))
        offset=max(.01,1.5*delta/cfg['max_joint_speed_rad_s'],np.sqrt(6*delta/cfg['max_joint_acceleration_rad_s2']))
        for p in points[1:]: set_time(p,stamp_seconds(p.time_from_start)+offset)
        for p in points[:2]:
            if p.velocities: p.velocities=[0.]*6
            if p.accelerations: p.accelerations=[0.]*6
    spline=from_message(traj,cfg,current,False)
    scale=max(1.,spline.extrema(1)/cfg['max_joint_speed_rad_s'],
              np.sqrt(spline.extrema(2)/cfg['max_joint_acceleration_rad_s2']))*1.02
    for p in points:
        set_time(p,stamp_seconds(p.time_from_start)*scale)
        p.velocities=[v/scale for v in p.velocities]
        p.accelerations=[a/scale**2 for a in p.accelerations]
    return from_message(traj,cfg,current)


def set_time(point,t):
    ns=round(t*1e9)
    point.time_from_start.sec,point.time_from_start.nanosec=divmod(ns,1_000_000_000)
