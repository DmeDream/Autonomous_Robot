#!/usr/bin/env python3
"""Follow room poses with collision-aware paths while a separate mapper records."""
import argparse
import csv
import fcntl
import heapq
import json
import math
import os
from pathlib import Path
import signal
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from rclpy.signals import SignalHandlerOptions
from auto_map import Driver, ROOT, velocity, verify_world, wrap


class Planner:
    """Use scene collisions only to plan motion, never to fill measured Gridmap."""
    def __init__(self, world, radius=0.5, resolution=0.15):
        self.resolution = resolution
        self.xmin, self.ymin = -10.5, -8.5
        ny, nx = math.ceil(17 / resolution), math.ceil(21 / resolution)
        yy, xx = np.mgrid[:ny, :nx]
        xx = self.xmin + (xx + 0.5) * resolution
        yy = self.ymin + (yy + 0.5) * resolution
        self.free = (abs(xx) < 9.4) & (abs(yy) < 7.4)
        for model in ET.parse(world).findall('world/model'):
            for link in model.findall('link'):
                if model.find('pose') is not None or link.find('pose') is not None:
                    raise ValueError('This route planner expects the current flat five-room world')
                for collision in link.findall('collision'):
                    size = [float(v) for v in collision.findtext('geometry/box/size').split()]
                    pose = [float(v) for v in collision.findtext('pose').split()]
                    if pose[2] + size[2] / 2 <= 0.06 or pose[2] - size[2] / 2 >= 0.45:
                        continue
                    dx, dy = xx - pose[0], yy - pose[1]
                    u = math.cos(pose[5]) * dx + math.sin(pose[5]) * dy
                    v = -math.sin(pose[5]) * dx + math.cos(pose[5]) * dy
                    distance = np.hypot(np.maximum(abs(u) - size[0]/2, 0),
                                        np.maximum(abs(v) - size[1]/2, 0))
                    # Cover the entire cell footprint, including diagonals.
                    self.free &= distance > radius + resolution / math.sqrt(2)

    def cell(self, point):
        return (math.floor((point[0] - self.xmin) / self.resolution),
                math.floor((point[1] - self.ymin) / self.resolution))

    def point(self, cell):
        return (self.xmin + (cell[0] + .5) * self.resolution,
                self.ymin + (cell[1] + .5) * self.resolution)

    def clear(self, cell):
        x, y = cell
        return 0 <= y < self.free.shape[0] and 0 <= x < self.free.shape[1] and self.free[y, x]

    def visible(self, a, b):
        n = max(2, math.ceil(math.dist(a, b) / (self.resolution / 3)))
        return all(self.clear(self.cell((a[0]+t*(b[0]-a[0]), a[1]+t*(b[1]-a[1]))))
                   for t in np.linspace(0, 1, n))

    def plan(self, start, goal):
        s, g = self.cell(start), self.cell(goal)
        if not self.clear(s) or not self.clear(g):
            raise RuntimeError(f'Pose is too close to furniture/wall: {start} -> {goal}')
        queue, cost, parent = [(0, s)], {s: 0}, {}
        while queue:
            _, current = heapq.heappop(queue)
            if current == g:
                cells = [g]
                while cells[-1] != s:
                    cells.append(parent[cells[-1]])
                points = [start] + [self.point(c) for c in reversed(cells[:-1])] + [goal]
                result, i = [], 0
                while i < len(points)-1:
                    j = len(points)-1
                    while j > i+1 and not self.visible(points[i], points[j]):
                        j -= 1
                    result.append(points[j])
                    i = j
                return result
            for dx, dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
                neighbor = current[0]+dx, current[1]+dy
                if not self.clear(neighbor) or (dx and dy and
                    (not self.clear((current[0]+dx,current[1])) or not self.clear((current[0],current[1]+dy)))):
                    continue
                new_cost = cost[current] + math.hypot(dx,dy)
                if new_cost < cost.get(neighbor, math.inf):
                    cost[neighbor], parent[neighbor] = new_cost, current
                    heapq.heappush(queue, (new_cost + math.dist(neighbor,g), neighbor))
        raise RuntimeError(f'No safe route to {goal}')


class RouteDriver(Driver):
    """Keep each measured world pose as pose[t] = [x, y, r]."""
    def __init__(self, pose_topic, directory):
        self.history = []
        self.history_file = (directory / 'poses_t.csv').open('x', newline='')
        self.history_writer = csv.writer(self.history_file)
        self.history_writer.writerow(['t', 'time_s', 'x', 'y', 'r', 'dx', 'dy', 'dr'])
        super().__init__(pose_topic, node_name=f'room_route_{os.getpid()}')

    def receive_pose(self, message):
        previous_stamp = self.pose_stamp
        super().receive_pose(message)
        if self.pose is None or self.pose_stamp == previous_stamp:
            return
        previous = self.history[-1] if self.history else self.pose
        delta = [self.pose[0]-previous[0], self.pose[1]-previous[1], wrap(self.pose[2]-previous[2])]
        stamp = self.pose_stamp[0] + self.pose_stamp[1] * 1e-9
        self.history_writer.writerow([len(self.history), stamp, *self.pose, *delta])
        self.history.append(self.pose)
        if len(self.history) % 10 == 0:
            self.history_file.flush()

    def save_history(self, directory):
        self.history_file.close()
        np.save(directory / 'poses_t.npy', np.asarray(self.history, dtype=float).reshape(-1, 3))
        (directory / 'poses_t.meta.json').write_text(json.dumps(dict(
            columns=['x', 'y', 'r'], frame='world', angle_unit='radians',
            time='Gazebo simulation seconds', samples=len(self.history),
            delta='pose[t] - pose[t-1], with dr wrapped to [-pi, pi]'), indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--poses', type=Path, default=ROOT / 'room_poses.json')
    parser.add_argument('--speed', type=float, default=0.3)
    parser.add_argument('--duration', type=float, default=1200)
    parser.add_argument('--session', type=Path, default=ROOT / 'maps/active_session.json')
    parser.add_argument('--pose-topic', default='/ocgm/world_pose_array')
    parser.add_argument('--dwell', type=float, default=0.4, help='Seconds to collect scans at each pose')
    parser.add_argument('--check-route', action='store_true', help='Validate all poses and paths without ROS or motion')
    args = parser.parse_args()
    if (not math.isfinite(args.speed) or not 0 < args.speed <= .4 or
            not math.isfinite(args.duration) or args.duration <= 0 or
            not math.isfinite(args.dwell) or args.dwell < 0):
        parser.error('0 < speed <= 0.4, positive duration and nonnegative dwell required')
    goals = json.loads(args.poses.read_text())
    if not goals or any(not all(math.isfinite(float(g[k])) for k in ('x','y')) or
                        ('yaw' in g and not math.isfinite(float(g['yaw']))) for g in goals):
        parser.error('poses must be a nonempty list of finite x, y and optional yaw')
    planner = Planner(ROOT / 'world_design/five_rooms.sdf')
    previous = (-1,0)
    for goal in goals:
        path = planner.plan(previous, (goal['x'],goal['y']))
        if args.check_route:
            print(f'Checked {goal["name"]}: {len(path)} path segments', flush=True)
        previous = goal['x'],goal['y']
    if args.check_route:
        return
    verify_world()
    session = json.loads(args.session.read_text())
    directory = Path(session['directory'])
    os.kill(session['pid'], 0)
    if not session.get('ready') or (directory / 'survey.json').exists():
        raise SystemExit('Start a fresh run_gridmap.sh session first')
    lock = (ROOT / 'maps/.drive.lock').open('w')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise SystemExit('A room driver is already running')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    def interrupted(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    node = RouteDriver(args.pose_topic, directory)
    reached, rooms, reason, failure = 0, set(), 'route complete', None
    start = time.monotonic()
    def check_live():
        now = time.monotonic()
        if now-start > args.duration:
            raise RuntimeError('Survey duration limit reached')
        os.kill(session['pid'], 0)
        if time.time() - (directory / 'data.csv').stat().st_mtime > 3:
            raise RuntimeError('Gridmap recorder is no longer receiving scans')
        if now-node.pose_time > 1 or now-node.scan_time > 1:
            raise RuntimeError('Pose or scan stale; stopped')
        if any(p.node_name != node.get_name() or p.node_namespace != node.get_namespace()
               for p in node.get_publishers_info_by_topic('/cmd_vel')):
            raise RuntimeError('Another command publisher is running; close teleop')
        return now
    try:
        while time.monotonic()-start < 3:
            rclpy.spin_once(node, timeout_sec=.05)
        if node.pose is None or node.scan is None:
            raise RuntimeError('No live pose/scan; start run_ros.sh')
        t = len(goals)
        for index in range(t):
            goal = goals[index]
            segments = planner.plan(node.pose[:2], (goal['x'],goal['y']))
            print(f'waypoints[{index}] / {t-1}: [{goal["x"]}, {goal["y"]}, '
                  f'{goal.get("yaw", "follow path")}] — {goal["name"]}', flush=True)
            for target in segments:
                progress_pose, progress_time = node.pose, time.monotonic()
                best_distance = math.dist(node.pose[:2], target)
                approach_time = progress_time
                segment_deadline = progress_time + max(30, 3*best_distance/args.speed + 20)
                print(f'  Path target: {tuple(round(v, 3) for v in target)}', flush=True)
                while True:
                    rclpy.spin_once(node, timeout_sec=.05)
                    now = check_live()
                    if math.dist(node.pose[:2], progress_pose[:2]) > .06 or abs(wrap(node.pose[2]-progress_pose[2])) > .08:
                        progress_pose, progress_time = node.pose, now
                    if now-progress_time > 15:
                        raise RuntimeError('Robot is stuck or cannot turn')
                    distance = math.dist(node.pose[:2], target)
                    if distance < best_distance - .03:
                        best_distance, approach_time = distance, now
                    if now-approach_time > 20 or now > segment_deadline:
                        raise RuntimeError('Robot is not approaching the path target; stopped')
                    # Grid vertices are 0.15 m apart. A 0.20 m arrival radius
                    # avoids circling a tiny corner while retaining body clearance.
                    linear, angular, arrived = velocity(node.pose, target, node.scan, args.speed,
                                                         arrival_tolerance=.20)
                    node.command(linear, angular)
                    if arrived:
                        break
            if 'yaw' in goal:
                deadline = time.monotonic()+20
                while abs(wrap(goal['yaw']-node.pose[2])) > .08:
                    rclpy.spin_once(node, timeout_sec=.05)
                    now = check_live()
                    if now>deadline:
                        raise RuntimeError('Final heading timed out')
                    node.command(0., max(-.6,min(.6,1.5*wrap(goal['yaw']-node.pose[2]))))
            node.command()
            deadline = time.monotonic() + args.dwell
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.05)
                check_live()
                node.command()
            reached += 1
            if 'room' in goal:
                rooms.add(goal['room'])
            print(f'Reached {reached}/{t}: {goal["name"]}; pose[{len(node.history)-1}] = '
                  f'{[round(v, 3) for v in node.pose]}; rooms visited: {sorted(rooms)}', flush=True)
    except KeyboardInterrupt:
        reason = 'Ctrl+C'
    except Exception as error:
        reason, failure = str(error), error
    finally:
        for _ in range(5):
            node.command()
            rclpy.spin_once(node, timeout_sec=.05)
        node.save_history(directory)
        node.destroy_node()
        rclpy.shutdown()
        status = dict(reason=reason, reached=reached, total=len(goals), rooms_visited=sorted(rooms),
                      poses=goals, coverage='Measured LiDAR; occluded areas remain unknown')
        temp = directory / 'survey.tmp'
        temp.write_text(json.dumps(status, indent=2))
        temp.replace(directory / 'survey.json')
    print(f'Robot stopped: {reason}', flush=True)
    if failure:
        raise SystemExit(reason)


if __name__ == '__main__':
    main()
