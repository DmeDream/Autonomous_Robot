#!/usr/bin/env python3
"""Execute an offline RRT* plan through CoppeliaSim's ZeroMQ joint interface.

Requires a matching, already-open scene and a JSON configuration with verified
robot/joint paths, wheel dimensions, joint signs and map-to-scene calibration.
This module does not load or replace a scene and never connects to ROS hardware.
"""
import argparse
import csv
import json
import math
from pathlib import Path
import time

import numpy as np

from rrt_path_control import PathFollower, wheel_velocities
from rrt_star_plan import load_plan, wrap


def validate_config(config):
    for key in ('robot','left_joints','right_joints','wheel_radius','wheel_separation','footprint_radius','map_to_scene'):
        if key not in config:
            raise ValueError(f'Missing scene configuration field: {key}')
    paths = [config['robot'],*config['left_joints'],*config['right_joints']]
    if not config['left_joints'] or not config['right_joints'] or not all(isinstance(p,str) and p.startswith('/') for p in paths):
        raise ValueError('Supply absolute scene paths for the robot and both sets of wheel joints')
    if len(paths)!=len(set(paths)):
        raise ValueError('Robot and joint paths must be distinct')
    if not all(isinstance(config[k],(int,float)) and math.isfinite(config[k]) and config[k]>0
               for k in ('wheel_radius','wheel_separation','footprint_radius')):
        raise ValueError('Set actual wheel radius, separation and circular footprint radius in metres')
    for side in ('left','right'):
        if config.get(side+'_sign',1) not in (-1,1):
            raise ValueError('Wheel signs must be +1 or -1')
    transform = np.asarray(config['map_to_scene'],dtype=float)
    if transform.shape!=(3,) or not np.isfinite(transform).all():
        raise ValueError('map_to_scene must be [world_x_offset,world_y_offset,yaw_offset]')
    if not math.isfinite(config.get('robot_forward_yaw',0)):
        raise ValueError('robot_forward_yaw must be finite radians')


def to_scene(pose, config):
    tx,ty,angle = config['map_to_scene']
    c,s = math.cos(angle),math.sin(angle)
    return [tx+c*pose[0]-s*pose[1],ty+s*pose[0]+c*pose[1],
            wrap(pose[2]+angle-config.get('robot_forward_yaw',0))]


def from_scene(position, orientation, config):
    tx,ty,angle = config['map_to_scene']
    c,s = math.cos(angle),math.sin(angle)
    x,y = position[0]-tx,position[1]-ty
    return [c*x+s*y,-s*x+c*y,wrap(orientation[2]+config.get('robot_forward_yaw',0)-angle)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan',type=Path,required=True)
    parser.add_argument('--config',type=Path,required=True)
    parser.add_argument('--host',default='localhost')
    parser.add_argument('--port',type=int,default=23000)
    parser.add_argument('--speed',type=float,default=.20)
    parser.add_argument('--duration',type=float,default=600.,help='Maximum simulation seconds')
    parser.add_argument('--check',action='store_true',help='Validate files without connecting to the simulator')
    args = parser.parse_args()
    try:
        plan,grid,poses = load_plan(args.plan)
        config = json.loads(args.config.read_text())
        validate_config(config)
        if config['footprint_radius'] > grid.robot_radius:
            raise ValueError('Scene robot is larger than the planned footprint; replan with a larger --robot-radius')
        if not 0<args.speed<=.4 or not math.isfinite(args.duration) or args.duration<=0:
            raise ValueError('Use 0 < speed <= 0.4 and a positive duration')
        follower = PathFollower(poses,speed=args.speed)
        if grid.margin < .10:
            raise ValueError('This follower requires a plan with margin >= 0.10 m for tracking error')
        if args.check:
            print(f'Files valid: {len(poses)} RRT* nodes -> {len(follower.path)} interpolated poses; no connection made')
            return
        # Current package name plus the older module name used in course handouts.
        try:
            from coppeliasim_zmqremoteapi_client import RemoteAPIClient
        except ImportError:
            try:
                from zmqRemoteApi import RemoteAPIClient
            except ImportError:
                raise RuntimeError('Install coppeliasim-zmqremoteapi-client in the Python environment running this script')
        client = RemoteAPIClient(host=args.host,port=args.port)
        sim = client.require('sim') if hasattr(client,'require') else client.getObject('sim')
        if sim.getSimulationState()!=sim.simulation_stopped:
            raise RuntimeError('Stop the current CoppeliaSim run before placing the random start pose')
        robot = sim.getObject(config['robot'])
        left = [sim.getObject(p) for p in config['left_joints']]
        right = [sim.getObject(p) for p in config['right_joints']]
        if len(set([robot,*left,*right]))!=len([robot,*left,*right]):
            raise RuntimeError('Scene paths resolved to duplicate handles')
        def motors(linear=0.,angular=0.):
            vl,vr = wheel_velocities(linear,angular,config['wheel_radius'],config['wheel_separation'])
            for joint in left:
                sim.setJointTargetVelocity(joint,vl*config.get('left_sign',1))
            for joint in right:
                sim.setJointTargetVelocity(joint,vr*config.get('right_sign',1))
        start = to_scene(poses[0],config)
        z = sim.getObjectPosition(robot,sim.handle_world)[2]
        # One placement while stopped; all subsequent motion is from wheel joints.
        sim.setObjectPose(robot,[start[0],start[1],z,0.,0.,math.sin(start[2]/2),math.cos(start[2]/2)],sim.handle_world)
        sim.setStepping(True)
        started = False
        reached = False
        run = args.plan.parent/f'coppelia_run_{time.time_ns()}'
        run.mkdir()
        try:
            motors()
            sim.startSimulation()
            started = True
            sim.step()
            started_at = sim.getSimulationTime()
            last_progress = started_at
            last_index = follower.index
            last_distance = math.inf
            progress_pose = None
            with (run/'actual_poses.csv').open('w',newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['time_s','x','y','theta','linear_m_s','angular_rad_s','target_index'])
                while True:
                    stamp = sim.getSimulationTime()
                    pose = from_scene(sim.getObjectPosition(robot,sim.handle_world),
                                      sim.getObjectOrientation(robot,sim.handle_world),config)
                    linear,angular,reached = follower.command(pose)
                    target = follower.path[follower.index,:2]
                    if not grid.segment_free(pose[:2],target):
                        raise RuntimeError('Measured pose or its next segment left the validated free space')
                    distance = math.dist(pose[:2],target)
                    moved = (progress_pose is None or math.dist(pose[:2],progress_pose[:2])>.03 or
                             abs(wrap(pose[2]-progress_pose[2]))>.04)
                    if follower.index!=last_index or distance<last_distance-.01 or moved:
                        last_progress,last_index,last_distance = stamp,follower.index,distance
                        progress_pose = pose
                    if stamp-started_at>args.duration or stamp-last_progress>20:
                        raise RuntimeError('Motion timeout or no progress; joint velocities stopped')
                    writer.writerow([stamp,*pose,linear,angular,follower.index])
                    motors(linear,angular)
                    if reached:
                        break
                    sim.step()
            print(f'Goal reached; measured trajectory: {run / "actual_poses.csv"}')
        finally:
            # A failed wheel call must not prevent attempts to stop the other motors.
            for joint in [*left,*right]:
                try:
                    sim.setJointTargetVelocity(joint,0.)
                except Exception:
                    pass
            if started:
                try:
                    sim.step()
                finally:
                    sim.stopSimulation()
            sim.setStepping(False)
            (run/'result.json').write_text(json.dumps(dict(reached=reached,plan=str(args.plan.resolve())),indent=2))
    except (ValueError,RuntimeError,OSError,KeyError) as error:
        parser.exit(1,f'Coppelia execution stopped: {error}\n')


if __name__=='__main__':
    main()
