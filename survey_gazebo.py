#!/usr/bin/env python3
"""Drive a bounded survey of the existing simulated obstacle course via /cmd_vel."""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock


class Survey(Node):
    def __init__(self):
        super().__init__('ocgm_gazebo_survey')
        self.pose = None
        self.scan = None
        self.last_pose = 0
        self.last_scan = 0
        self.clock = None
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(PoseArray, '/ocgm/world_pose_array', self.world_pose, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.laser, qos_profile_sensor_data)
        self.create_subscription(Clock, '/clock', self.tick, 10)

    def odom(self, message):
        p, q = message.pose.pose.position, message.pose.pose.orientation
        self.pose = (p.x - 1, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        self.last_pose = time.monotonic()

    def world_pose(self, message):
        if not message.poses:
            return
        p, q = message.poses[0].position, message.poses[0].orientation
        self.pose = (p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        self.last_pose = time.monotonic()

    def laser(self, message):
        self.scan = message
        self.last_scan = time.monotonic()

    def tick(self, message):
        self.clock = message.clock.sec + message.clock.nanosec * 1e-9

    def command(self, linear=0.0, angular=0.0):
        message = Twist()
        message.linear.x, message.angular.z = linear, angular
        self.pub.publish(message)


def main():
    rclpy.init()
    node = Survey()
    # The existing skid-steer does not track turning odometry reliably.
    # Survey the clear central corridor in both directions without modifying it.
    targets = [(-3.5, 0), (3.5, 0), (-1, 0)]
    deadline = time.monotonic() + 240
    index = 0
    try:
        while index < len(targets) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            if node.pose is None or node.scan is None or node.clock is None or now - node.last_pose > 2 or now - node.last_scan > 2:
                node.command()
                continue
            x, y, yaw = node.pose
            gx, gy = targets[index]
            distance = math.hypot(gx - x, gy - y)
            if distance < 0.15:
                print(f'Reached {targets[index]} at ({x:.2f}, {y:.2f})', flush=True)
                index += 1
                node.command()
                continue
            error = math.atan2(math.sin(math.atan2(gy - y, gx - x) - yaw), math.cos(math.atan2(gy - y, gx - x) - yaw))
            reverse = abs(error) > math.pi / 2
            if reverse:
                error = math.atan2(math.sin(error + math.pi), math.cos(error + math.pi))
            angular = max(-0.7, min(0.7, 1.8 * error))
            linear = min(0.4, 0.7 * distance) if abs(error) < 0.3 else 0.0
            if reverse:
                linear = -linear
            if linear:
                front = [d for i, d in enumerate(node.scan.ranges)
                         if (abs(node.scan.angle_min + i * node.scan.angle_increment) < 0.3 if linear > 0 else
                             abs(node.scan.angle_min + i * node.scan.angle_increment) > math.pi - 0.3) and math.isfinite(d)]
                if front and min(front) < 0.6:
                    raise RuntimeError('Obstacle in front; stopped survey')
            node.command(linear, angular)
        if index != len(targets):
            raise RuntimeError('Survey timed out before completing all waypoints')
    finally:
        for _ in range(5):
            node.command()
            rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
