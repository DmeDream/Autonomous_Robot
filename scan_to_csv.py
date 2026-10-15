#!/usr/bin/env python3
"""Record LaserScan and timestamped TF pose without changing the simulation."""
import argparse
import csv
import json
import math
import time
import signal
from collections import deque
from types import SimpleNamespace
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray
from tf2_ros import Buffer, TransformListener, TransformException


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


class Recorder(Node):
    def __init__(self, args):
        super().__init__('ocgm_scan_recorder', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.args = args
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.pending = deque(maxlen=100)
        self.rows = 0
        self.dropped = 0
        self.metadata = None
        self.world_samples = deque(maxlen=5000)
        if args.world_pose_topic:
            self.create_subscription(PoseArray, args.world_pose_topic, self.world_pose, qos_profile_sensor_data)
        self.file = Path(args.output).open('x', newline='')
        self.writer = csv.writer(self.file)
        self.create_subscription(LaserScan, args.scan, self.receive, qos_profile_sensor_data)
        self.create_timer(0.05, self.drain)

    def receive(self, scan):
        if len(self.pending) == self.pending.maxlen:
            self.dropped += 1
        self.pending.append((time.monotonic(), scan))

    def world_pose(self, message):
        # PoseArray has no names: index 0 was checked against research_robot in Pose_V.
        if not message.poses:
            return
        stamp = Time.from_msg(message.header.stamp).nanoseconds
        if self.world_samples and stamp <= self.world_samples[-1][0]:
            if stamp < self.world_samples[-1][0]:
                self.world_samples.clear()
            else:
                return
        self.world_samples.append((stamp, message.poses[0]))

    def synchronized_world_pose(self, stamp):
        for (ta, a), (tb, b) in zip(self.world_samples, list(self.world_samples)[1:]):
            if ta <= stamp <= tb and tb - ta <= 100_000_000:
                fraction = (stamp - ta) / (tb - ta)
                heading = yaw(a.orientation)
                difference = math.atan2(math.sin(yaw(b.orientation) - heading), math.cos(yaw(b.orientation) - heading))
                heading += fraction * difference
                return SimpleNamespace(translation=SimpleNamespace(
                    x=a.position.x + fraction * (b.position.x - a.position.x),
                    y=a.position.y + fraction * (b.position.y - a.position.y)),
                    rotation=SimpleNamespace(x=0, y=0, z=math.sin(heading / 2), w=math.cos(heading / 2)))
        raise TransformException('No bracketing Gazebo world poses for scan timestamp')

    def drain(self):
        while self.pending:
            arrival, scan = self.pending[0]
            try:
                stamp = Time.from_msg(scan.header.stamp)
                pose = (self.synchronized_world_pose(stamp.nanoseconds) if self.args.world_pose_topic else
                        self.buffer.lookup_transform(self.args.pose_frame, self.args.base_frame, stamp).transform)
                mount = self.buffer.lookup_transform(self.args.base_frame, scan.header.frame_id, stamp).transform
            except TransformException:
                if time.monotonic() - arrival > 2:
                    self.pending.popleft()
                    self.dropped += 1
                    continue
                return
            self.pending.popleft()
            count = len(scan.ranges)
            signature = (count, scan.header.frame_id, scan.angle_min, scan.angle_increment,
                         scan.range_min, scan.range_max, mount.translation.x,
                         mount.translation.y, yaw(mount.rotation))
            if self.metadata is None:
                self.signature = signature
                self.metadata = dict(rays=count, scan_topic=self.args.scan,
                    pose_frame=self.args.pose_frame, base_frame=self.args.base_frame,
                    sensor_frame=scan.header.frame_id, angle_min=scan.angle_min,
                    angle_increment=scan.angle_increment, range_min=scan.range_min,
                    range_max=scan.range_max, sensor_x=mount.translation.x,
                    sensor_y=mount.translation.y, sensor_yaw=yaw(mount.rotation),
                    world_from_pose=[self.args.world_x, self.args.world_y, self.args.world_yaw],
                    csv_header=not self.args.no_header,
                    pose_reference='sensor' if self.args.sensor_pose else 'vehicle',
                    pose_policy=('Gazebo PoseArray index 0, verified research_robot; linear position/shortest-yaw interpolation at scan stamp'
                                 if self.args.world_pose_topic else 'timestamped TF at scan stamp'),
                    world_pose_topic=self.args.world_pose_topic, angles='sensor-relative radians')
                Path(self.args.output).with_suffix('.meta.json').write_text(json.dumps(self.metadata, indent=2))
                if self.args.sensor_pose:
                    self.metadata.update(sensor_x=0.0, sensor_y=0.0, sensor_yaw=0.0)
                    Path(self.args.output).with_suffix('.meta.json').write_text(json.dumps(self.metadata, indent=2))
                if not self.args.no_header:
                    self.writer.writerow(['x', 'y', 'yaw'] + [v for i in range(count)
                                         for v in (f'range_{i}', f'angle_{i}')])
                self.get_logger().info(f'{count} rays, {3 + 2 * count} columns; recording {self.args.output}')
            if signature != self.signature:
                self.get_logger().error('Scan geometry/mount changed; dropping incompatible scan')
                self.dropped += 1
                continue
            c, s = math.cos(self.args.world_yaw), math.sin(self.args.world_yaw)
            x = self.args.world_x + c * pose.translation.x - s * pose.translation.y
            y = self.args.world_y + s * pose.translation.x + c * pose.translation.y
            heading = yaw(pose.rotation) + self.args.world_yaw
            if self.args.sensor_pose:
                c, s = math.cos(heading), math.sin(heading)
                x, y = (x + c * mount.translation.x - s * mount.translation.y,
                        y + s * mount.translation.x + c * mount.translation.y)
                heading += yaw(mount.rotation)
            row = [x, y, math.atan2(math.sin(heading), math.cos(heading))]
            for i, distance in enumerate(scan.ranges):
                row.extend((distance, scan.angle_min + i * scan.angle_increment))
            self.writer.writerow(row)
            self.file.flush()
            self.rows += 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default='data.csv')
    parser.add_argument('--scan', default='/scan')
    parser.add_argument('--pose-frame', default='odom')
    parser.add_argument('--base-frame', default='base_footprint')
    parser.add_argument('--world-x', type=float, required=True, help='Gazebo world <- pose frame translation X')
    parser.add_argument('--world-y', type=float, required=True)
    parser.add_argument('--world-yaw', type=float, required=True)
    parser.add_argument('--duration', type=float, default=0, help='Wall seconds, 0 until Ctrl-C')
    parser.add_argument('--no-header', action='store_true', help='Numeric-only CSV compatible with np.loadtxt')
    parser.add_argument('--sensor-pose', action='store_true', help='Record LiDAR base pose instead of vehicle pose')
    parser.add_argument('--world-pose-topic', default='', help='Timestamped Gazebo PoseArray; first pose must be the robot base')
    args = parser.parse_args()
    if not all(math.isfinite(v) for v in (args.world_x, args.world_y, args.world_yaw, args.duration)) or args.duration < 0:
        parser.error('Transform must be finite and duration nonnegative')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    def interrupted(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    node = Recorder(args)
    start = time.monotonic()
    try:
        while rclpy.ok() and (args.duration == 0 or time.monotonic() - start < args.duration):
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.file.close()
        print(f'Saved {node.rows} scans; dropped {node.dropped}; pending {len(node.pending)}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if not node.rows:
        raise SystemExit('No synchronized scans saved; check /scan, /clock and TF')


if __name__ == '__main__':
    main()
