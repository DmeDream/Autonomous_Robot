#!/usr/bin/env python3
"""Display a measured occupancy grid as the scan recorder appends CSV rows."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

from myworld_ocgm_660610823 import OccupancyGrid


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--resolution', type=float, default=0.05)
    parser.add_argument('--scan-stride', type=int, default=5)
    parser.add_argument('--ros-map', action='store_true', help='Also publish a ROS OccupancyGrid on /map')
    args = parser.parse_args()
    if args.scan_stride < 1:
        parser.error('scan-stride must be positive')
    metadata = json.loads(args.input.with_suffix('.meta.json').read_text())
    grid = OccupancyGrid((-10.5, 10.5, -8.5, 8.5), args.resolution)
    fig, ax = plt.subplots(figsize=(8, 7))
    fig.canvas.manager.set_window_title('Live Gridmap - Gazebo robot')
    raster = ax.imshow(grid.probabilities(), origin='lower',
        extent=(grid.xmin, grid.xmax, grid.ymin, grid.ymax),
        cmap='gray_r', vmin=0, vmax=1)
    path, = ax.plot([], [], color='red', linewidth=1, label='LiDAR path')
    robot, = ax.plot([], [], 'o', color='limegreen', markersize=7, label='LiDAR now')
    ax.set(xlabel='World X (m)', ylabel='World Y (m)', title='Live Gridmap: waiting for scans')
    ax.set_aspect('equal')
    ax.legend(loc='upper left')
    fig.colorbar(raster, ax=ax, label='Occupancy probability (gray = unknown)')
    fig.tight_layout()
    source = args.input.open()
    if metadata.get('csv_header', True):
        source.readline()
    xs, ys = [], []
    count = 0
    ros_node = publisher = None
    if args.ros_map:
        import rclpy
        from rclpy.signals import SignalHandlerOptions
        from rclpy.qos import QoSProfile, DurabilityPolicy
        from nav_msgs.msg import OccupancyGrid as MapMessage
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        ros_node = rclpy.create_node('live_gridmap_publisher')
        publisher = ros_node.create_publisher(MapMessage, '/map',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def refresh():
        nonlocal count
        for _ in range(20):
            offset = source.tell()
            line = source.readline()
            if not line or not line.endswith('\n'):
                source.seek(offset)
                break
            row = [float(v) for v in line.strip().split(',')]
            if len(row) != 3 + 2 * metadata['rays']:
                raise ValueError('Recorded row does not match sensor metadata')
            if count % args.scan_stride == 0:
                grid.scan(row, metadata)
            xs.append(row[0])
            ys.append(row[1])
            count += 1
        raster.set_data(grid.probabilities())
        path.set_data(xs, ys)
        if xs:
            robot.set_data(xs[-1:], ys[-1:])
        status_file = args.input.parent / 'survey.json'
        status = json.loads(status_file.read_text())['reason'] if status_file.exists() else 'recording'
        ax.set_title(f'Live Gridmap | {count} scans | {status}')
        if publisher is not None:
            import numpy as np
            message = MapMessage()
            message.header.frame_id = 'world'
            message.header.stamp = ros_node.get_clock().now().to_msg()
            message.info.resolution = args.resolution
            message.info.height, message.info.width = grid.log_odds.shape
            message.info.origin.position.x = grid.xmin
            message.info.origin.position.y = grid.ymin
            message.info.origin.orientation.w = 1.0
            values = np.rint(grid.probabilities() * 100).astype('int8')
            values[grid.log_odds == 0] = -1
            message.data = values.ravel().tolist()
            publisher.publish(message)
            rclpy.spin_once(ros_node, timeout_sec=0)
        fig.canvas.draw_idle()

    timer = fig.canvas.new_timer(interval=500)
    timer.add_callback(refresh)
    timer.start()
    fig.canvas.draw()
    args.input.with_suffix('.live_ready').touch()
    print('Live Gridmap window ready', flush=True)
    try:
        plt.show()
    finally:
        timer.stop()
        source.close()
        if ros_node is not None:
            ros_node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
