#!/usr/bin/env python3
"""Survey the clear corridor of the five-room Gazebo world and save a measured map."""
import argparse
from datetime import datetime
import json
import math
import re
import signal
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET

import rclpy
from geometry_msgs.msg import PoseArray, Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan

from collect_map import ROOT, ROS_PYTHON, build, stop

POSE_TOPIC = '/ocgm/auto_world_pose_array'
# Current skid-steer barely turns in simulation. Use forward/reverse along
# the corridor; room interiors behind obstacles remain unknown.
TARGETS = [(-8, 0), (8, 0), (-1, 0)]


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def velocity(pose, goal, scan, speed, arrival_tolerance=0.12):
    """Return a command; invalid rays in the direction of travel prevent motion."""
    if not all(math.isfinite(v) for v in (*pose, scan.angle_min, scan.angle_increment, scan.range_min)) or scan.angle_increment <= 0:
        raise RuntimeError('Invalid pose or LiDAR geometry; robot stopped')
    x, y, yaw = pose
    distance = math.hypot(goal[0] - x, goal[1] - y)
    if distance < arrival_tolerance:
        return 0.0, 0.0, True
    error = wrap(math.atan2(goal[1] - y, goal[0] - x) - yaw)
    reverse = abs(error) > math.pi / 2
    if reverse:
        error = wrap(error + math.pi)
    angular = max(-0.6, min(0.6, 1.5 * error))
    linear = min(speed, 0.7 * distance) if abs(error) < 0.18 else 0.0
    if reverse:
        linear = -linear
    frontal = [d for i, d in enumerate(scan.ranges)
               if abs(wrap(scan.angle_min + i * scan.angle_increment - (math.pi if reverse else 0))) < 0.55]
    if linear:
        if not frontal or any(math.isnan(d) or d < scan.range_min for d in frontal):
            raise RuntimeError('Invalid LiDAR readings in travel direction; robot stopped')
        if min(frontal) < 0.65:
            raise RuntimeError('Obstacle closer than 0.65 m; robot stopped')
    return linear, angular, False


class Driver(Node):
    def __init__(self, pose_topic=POSE_TOPIC, node_name='auto_map_driver'):
        super().__init__(node_name)
        self.pose = self.scan = None
        self.pose_time = self.scan_time = 0.0
        self.pose_stamp = self.scan_stamp = None
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(PoseArray, pose_topic, self.receive_pose, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.receive_scan, qos_profile_sensor_data)

    def receive_pose(self, msg):
        if not msg.poses:
            return
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if stamp == self.pose_stamp:
            return
        self.pose_stamp = stamp
        p, q = msg.poses[0].position, msg.poses[0].orientation
        self.pose = (p.x, p.y, math.atan2(2 * (q.w*q.z + q.x*q.y),
                                        1 - 2 * (q.y*q.y + q.z*q.z)))
        self.pose_time = time.monotonic()

    def receive_scan(self, msg):
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if stamp != self.scan_stamp:
            self.scan, self.scan_stamp = msg, stamp
            self.scan_time = time.monotonic()

    def command(self, linear=0.0, angular=0.0):
        msg = Twist()
        msg.linear.x, msg.angular.z = linear, angular
        self.pub.publish(msg)


def verify_world():
    prefix = subprocess.run(['ros2', 'pkg', 'prefix', 'robot_gazebo'],
                            check=True, capture_output=True, text=True, timeout=10).stdout.strip()
    world = Path(prefix) / 'share/robot_gazebo/worlds/obstacle_course.sdf'
    if ET.tostring(ET.parse(world).getroot()) != ET.tostring(ET.parse(ROOT / 'world_design/five_rooms.sdf').getroot()):
        raise RuntimeError('Installed world differs from five_rooms.sdf; route needs checking')
    result = subprocess.run(['gz', 'topic', '-e', '-t',
                             '/world/obstacle_course/dynamic_pose/info', '-n', '1'],
                            check=True, capture_output=True, text=True, timeout=10)
    # PoseArray loses names; check its first source pose before using index zero.
    first_name = re.search(r'\bpose\s*\{\s*name:\s*"([^"]+)"', result.stdout)
    if first_name is None or first_name.group(1) != 'research_robot':
        raise RuntimeError('Gazebo pose index 0 is not research_robot')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=900,
                        help='Maximum wall seconds of driving; save a partial map at the limit')
    parser.add_argument('--speed', type=float, default=0.35, help='Forward m/s, maximum 0.4')
    parser.add_argument('--resolution', type=float, default=0.05)
    parser.add_argument('--scan-stride', type=int, default=5)
    parser.add_argument('--live', action='store_true', help='Open a live occupancy grid window while driving')
    args = parser.parse_args()
    if not all(math.isfinite(v) and v > 0 for v in (args.duration, args.speed, args.resolution)) or args.speed > 0.4 or args.scan_stride < 1:
        parser.error('Positive duration/resolution, 0 < speed <= 0.4, scan-stride >= 1 required')
    verify_world()
    # Keep ROS alive during Ctrl+C cleanup so zero velocity can be published.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    def interrupted(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    node = Driver()
    bridge = recorder = None
    directory = ROOT / 'maps' / ('auto_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    data = directory / 'data.csv'
    reached, reason, failure = 0, 'duration limit', None
    try:
        # Discovery must settle before checking for teleop or another driver.
        discovery = time.monotonic() + 2
        while time.monotonic() < discovery:
            rclpy.spin_once(node, timeout_sec=0.1)
        others = [p.node_name for p in node.get_publishers_info_by_topic('/cmd_vel')
                  if p.node_name != node.get_name() or p.node_namespace != node.get_namespace()]
        if others:
            raise RuntimeError('Close other /cmd_vel publishers first: ' + ', '.join(others))
        directory.mkdir(parents=True, exist_ok=False)
        bridge = subprocess.Popen(['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/world/obstacle_course/dynamic_pose/info@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V',
            '--ros-args', '-r', '/world/obstacle_course/dynamic_pose/info:=' + POSE_TOPIC], start_new_session=True)
        recorder = subprocess.Popen([ROS_PYTHON, str(ROOT / 'save_laser_show_pointcloud.py'),
            '--output', str(data), '--world-x', '0', '--world-y', '0', '--world-yaw', '0',
            '--world-pose-topic', POSE_TOPIC], start_new_session=True)
        ready_deadline = time.monotonic() + 15
        while not data.with_suffix('.meta.json').exists():
            rclpy.spin_once(node, timeout_sec=0.05)
            if time.monotonic() > ready_deadline or recorder.poll() is not None or bridge.poll() is not None:
                raise RuntimeError('Recorder/sensors did not become ready')
        if args.live:
            viewer = subprocess.Popen([ROS_PYTHON, str(ROOT / 'live_gridmap.py'),
                '--input', str(data), '--resolution', str(args.resolution),
                '--scan-stride', str(args.scan_stride)], start_new_session=True)
            ready_deadline = time.monotonic() + 20
            while not data.with_suffix('.live_ready').exists():
                rclpy.spin_once(node, timeout_sec=0.05)
                if viewer.poll() is not None or time.monotonic() > ready_deadline:
                    raise RuntimeError('Live Gridmap window could not open')
        if node.pose is None or not all(math.isfinite(v) for v in node.pose) or abs(node.pose[1]) > 0.35 or abs(node.pose[0]) > 9:
            raise RuntimeError('Start robot in the central corridor: |y| <= 0.35 and |x| <= 9')
        start = time.monotonic()
        waypoint_start = start
        progress_time, progress_pose = start, node.pose
        print(f'Automatic mapping started: {directory}', flush=True)
        while reached < len(TARGETS) and time.monotonic() - start < args.duration:
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            if recorder.poll() is not None or bridge.poll() is not None:
                raise RuntimeError('Recorder or pose bridge stopped')
            if node.pose is None or node.scan is None or now - node.pose_time > 1 or now - node.scan_time > 1:
                raise RuntimeError('Pose or LiDAR stale for over one second')
            if any(p.node_name != node.get_name() or p.node_namespace != node.get_namespace()
                   for p in node.get_publishers_info_by_topic('/cmd_vel')):
                raise RuntimeError('Another /cmd_vel publisher appeared')
            if now - waypoint_start > 150:
                raise RuntimeError('Waypoint timeout; robot stopped')
            if math.hypot(node.pose[0] - progress_pose[0], node.pose[1] - progress_pose[1]) > 0.08:
                progress_time, progress_pose = now, node.pose
            elif now - progress_time > 15:
                raise RuntimeError('Robot made no position progress for 15 seconds; check drive/heading')
            linear, angular, arrived = velocity(node.pose, TARGETS[reached], node.scan, args.speed)
            node.command(linear, angular)
            if arrived:
                print(f'Reached {reached + 1}/{len(TARGETS)}: {TARGETS[reached]}', flush=True)
                reached += 1
                waypoint_start = now
                progress_time, progress_pose = now, node.pose
        if reached == len(TARGETS):
            reason = 'route complete'
    except KeyboardInterrupt:
        reason = 'Ctrl+C'
    except Exception as error:
        reason, failure = str(error), error
    finally:
        for _ in range(5):
            node.command()
            rclpy.spin_once(node, timeout_sec=0.05)
        stop(recorder)
        stop(bridge)
        node.destroy_node()
        rclpy.shutdown()
    if directory.exists():
        (directory / 'survey.json').write_text(json.dumps(dict(
            reason=reason, reached=reached, total=len(TARGETS), targets=TARGETS,
            coverage='Measured LiDAR only; occluded areas remain unknown'), indent=2))
    print(f'Robot stopped: {reason}; reached {reached}/{len(TARGETS)} waypoints', flush=True)
    if data.exists() and data.stat().st_size and data.with_suffix('.meta.json').exists():
        build(data, directory, args.resolution, args.scan_stride)
    if failure:
        raise SystemExit(str(failure))


if __name__ == '__main__':
    main()
