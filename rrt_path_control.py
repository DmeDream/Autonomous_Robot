"""Simulator-independent interpolated-path follower for a differential drive."""
import math

import numpy as np

from rrt_star_plan import interpolate, wrap


class PathFollower:
    """Track ordered small segments, turn first, then align the final heading."""
    def __init__(self, poses, spacing=.10, speed=.20, turn_speed=.65,
                 position_tolerance=.04, heading_tolerance=.06):
        poses = np.asarray(poses,dtype=float)
        if poses.ndim != 2 or poses.shape[1] != 3 or len(poses)<2 or not np.isfinite(poses).all():
            raise ValueError('Expected at least two finite [x,y,theta] poses')
        if not all(math.isfinite(v) and v>0 for v in
                   (speed,turn_speed,spacing,position_tolerance,heading_tolerance)):
            raise ValueError('Controller speeds, spacing and tolerances must be positive')
        if position_tolerance >= spacing/2:
            raise ValueError('Position tolerance must be less than half the interpolation spacing')
        self.path = interpolate(poses,spacing)
        self.index = 1
        self.speed,self.turn_speed = speed,turn_speed
        self.position_tolerance,self.heading_tolerance = position_tolerance,heading_tolerance

    def command(self, pose):
        if len(pose)!=3 or not np.isfinite(pose).all():
            raise ValueError('Invalid measured robot pose')
        pose = np.asarray(pose)
        while (self.index < len(self.path)-1 and
               np.linalg.norm(self.path[self.index,:2]-pose[:2]) < self.position_tolerance):
            self.index += 1
        delta = self.path[self.index,:2]-pose[:2]
        distance = float(np.linalg.norm(delta))
        if self.index == len(self.path)-1 and distance < self.position_tolerance:
            error = wrap(self.path[-1,2]-pose[2])
            if abs(error)<self.heading_tolerance:
                return 0.,0.,True
            return 0.,float(np.clip(1.8*error,-self.turn_speed,self.turn_speed)),False
        error = wrap(math.atan2(delta[1],delta[0])-pose[2])
        angular = float(np.clip(1.8*error,-self.turn_speed,self.turn_speed))
        linear = min(self.speed,1.6*distance)*max(0.,math.cos(error)) if abs(error)<.30 else 0.
        return float(linear),angular,False


def wheel_velocities(linear, angular, wheel_radius, wheel_separation, maximum=8.):
    """Convert m/s and rad/s to left/right joint rad/s; preserve curvature on saturation."""
    if not all(math.isfinite(v) for v in (linear,angular,wheel_radius,wheel_separation,maximum)):
        raise ValueError('Wheel conversion requires finite inputs')
    if min(wheel_radius,wheel_separation,maximum)<=0:
        raise ValueError('Wheel radius, separation and velocity limit must be positive')
    left = (linear-angular*wheel_separation/2)/wheel_radius
    right = (linear+angular*wheel_separation/2)/wheel_radius
    scale = max(1.,abs(left)/maximum,abs(right)/maximum)
    return left/scale,right/scale
