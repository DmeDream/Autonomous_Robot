#!/usr/bin/env python3
"""Load and plot the measured Gazebo point cloud using the assignment variables."""
import argparse
import json
from pathlib import Path

import numpy as np


def load_data(filename='data.csv'):
    metadata = json.loads(Path(filename).with_suffix('.meta.json').read_text())
    data = np.loadtxt(filename, delimiter=',', ndmin=2,
                      skiprows=1 if metadata.get('csv_header', False) else 0)
    if data.shape[1] < 5 or (data.shape[1] - 3) % 2:
        raise ValueError('Expected x,y,yaw followed by range/angle pairs')
    if data.shape[1] != 3 + 2 * metadata['rays']:
        raise ValueError('CSV width differs from metadata')
    X, Y, TH = data[:, 0:1], data[:, 1:2], data[:, 2:3]
    sensor_length, sensor_angle = data[:, 3::2], data[:, 4::2]
    return X, Y, TH, sensor_length, sensor_angle, metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', default='data.csv')
    parser.add_argument('--output', default='pointcloud_660610823.png')
    parser.add_argument('--show', action='store_true')
    args = parser.parse_args()
    X, Y, TH, sensor_length, sensor_angle, metadata = load_data(args.input)
    import matplotlib
    if not args.show:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    # Both the original vehicle-pose format and new sensor-pose format are supported.
    sx = X + np.cos(TH) * metadata['sensor_x'] - np.sin(TH) * metadata['sensor_y']
    sy = Y + np.sin(TH) * metadata['sensor_x'] + np.cos(TH) * metadata['sensor_y']
    direction = TH + metadata['sensor_yaw'] + sensor_angle
    valid = np.isfinite(sensor_length) & (sensor_length >= metadata['range_min']) & (sensor_length < metadata['range_max'])
    safe_length = np.where(valid, sensor_length, 0)
    px = sx + safe_length * np.cos(direction)
    py = sy + safe_length * np.sin(direction)
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.scatter(px[valid], py[valid], s=0.2, c='black', label='Measured LiDAR hits')
    ax.plot(sx[:, 0], sy[:, 0], 'r-', label='Sensor trajectory')
    ax.set(xlabel='Gazebo world X (m)', ylabel='Gazebo world Y (m)', title='Measured point cloud')
    ax.set_aspect('equal')
    ax.legend()
    fig.tight_layout()
    fig.savefig(args.output, dpi=160)
    print(f'X,Y,TH: {X.shape}; sensor_length,sensor_angle: {sensor_length.shape}')
    if args.show:
        plt.show()
    plt.close(fig)


if __name__ == '__main__':
    main()
