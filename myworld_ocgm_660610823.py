#!/usr/bin/env python3
"""Build measured occupancy probabilities with log odds and Bresenham rays."""
import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np


def bresenham(x0, y0, x1, y1):
    dx, dy = abs(x1 - x0), -abs(y1 - y0)
    sx, sy = (1 if x0 < x1 else -1), (1 if y0 < y1 else -1)
    error = dx + dy
    while True:
        yield x0, y0
        if (x0, y0) == (x1, y1):
            break
        twice = 2 * error
        if twice >= dy:
            error += dy
            x0 += sx
        if twice <= dx:
            error += dx
            y0 += sy


class OccupancyGrid:
    def __init__(self, bounds, resolution):
        self.xmin, self.xmax, self.ymin, self.ymax = bounds
        self.resolution = resolution
        if not all(math.isfinite(v) for v in (*bounds, resolution)) or resolution <= 0:
            raise ValueError('Bounds/resolution must be finite, resolution positive')
        if self.xmax <= self.xmin or self.ymax <= self.ymin:
            raise ValueError('Bounds must have positive area')
        nx = math.ceil((self.xmax - self.xmin) / resolution)
        ny = math.ceil((self.ymax - self.ymin) / resolution)
        if nx * ny > 20_000_000:
            raise ValueError('Grid exceeds 20 million cells; increase resolution')
        self.log_odds = np.zeros((ny, nx))
        self.xmax = self.xmin + nx * resolution
        self.ymax = self.ymin + ny * resolution

    def cell(self, x, y):
        return math.floor((x - self.xmin) / self.resolution), math.floor((y - self.ymin) / self.resolution)

    def inside(self, cell):
        return 0 <= cell[0] < self.log_odds.shape[1] and 0 <= cell[1] < self.log_odds.shape[0]

    def ray(self, x, y, angle, distance, hit):
        start = self.cell(x, y)
        if not self.inside(start):
            raise ValueError('Sensor origin outside map; expand --bounds')
        # Limit traversal to the grid boundary, preserving whether the true hit is inside.
        dx, dy = distance * math.cos(angle), distance * math.sin(angle)
        endpoint = self.cell(x + dx, y + dy)
        true_hit = hit and self.inside(endpoint)
        fraction = 1.0
        for origin, delta, lo, hi in ((x, dx, self.xmin, self.xmax), (y, dy, self.ymin, self.ymax)):
            if delta > 0:
                fraction = min(fraction, (hi - origin) / delta)
            elif delta < 0:
                fraction = min(fraction, (lo - origin) / delta)
        if fraction < 1:
            fraction = max(0, fraction - 1e-10)
            endpoint = self.cell(x + fraction * dx, y + fraction * dy)
        for cell in bresenham(*start, *endpoint):
            if not self.inside(cell):
                break
            update = math.log(0.7 / 0.3) if true_hit and cell == endpoint else math.log(0.3 / 0.7)
            ix, iy = cell
            self.log_odds[iy, ix] = max(-10, min(10, self.log_odds[iy, ix] + update))

    def scan(self, row, metadata):
        x, y, yaw = row[:3]
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            raise ValueError('Invalid vehicle pose')
        c, s = math.cos(yaw), math.sin(yaw)
        sx = x + c * metadata['sensor_x'] - s * metadata['sensor_y']
        sy = y + s * metadata['sensor_x'] + c * metadata['sensor_y']
        for distance, angle in zip(row[3::2], row[4::2]):
            if not math.isfinite(angle) or math.isnan(distance) or distance < metadata['range_min']:
                continue
            hit = math.isfinite(distance) and distance < metadata['range_max']
            self.ray(sx, sy, yaw + metadata['sensor_yaw'] + angle,
                     min(distance, metadata['range_max']), hit)

    def probabilities(self):
        return 1 / (1 + np.exp(-self.log_odds))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', default='data.csv')
    parser.add_argument('--output', default='ocgm_660610823.csv')
    parser.add_argument('--resolution', type=float, default=0.05)
    parser.add_argument('--bounds', type=float, nargs=4, default=[-10.5, 10.5, -8.5, 8.5],
                        metavar=('XMIN', 'XMAX', 'YMIN', 'YMAX'))
    parser.add_argument('--show', action='store_true')
    parser.add_argument('--scan-stride', type=int, default=1, help='Use every Nth recorded scan; full CSV remains unchanged')
    args = parser.parse_args()
    if args.scan_stride < 1:
        parser.error('--scan-stride must be positive')
    metadata = json.loads(Path(args.input).with_suffix('.meta.json').read_text())
    for key in ('sensor_x', 'sensor_y', 'sensor_yaw', 'range_min', 'range_max'):
        if not math.isfinite(metadata[key]):
            raise ValueError(f'Invalid metadata: {key}')
    if not 0 <= metadata['range_min'] < metadata['range_max']:
        raise ValueError('Invalid range limits')
    grid = OccupancyGrid(args.bounds, args.resolution)
    trajectory = []
    with open(args.input, newline='') as source:
        reader = csv.reader(source)
        expected = ['x', 'y', 'yaw'] + [v for i in range(metadata['rays'])
                                      for v in (f'range_{i}', f'angle_{i}')]
        has_header = metadata.get('csv_header', True)
        if has_header and next(reader) != expected:
            raise ValueError('CSV header does not match ray metadata')
        recorded = 0
        for index, fields in enumerate(reader):
            line = index + (2 if has_header else 1)
            if len(fields) != len(expected):
                raise ValueError(f'Row {line}: expected {len(expected)} columns')
            row = [float(v) for v in fields]
            recorded += 1
            if index % args.scan_stride == 0:
                grid.scan(row, metadata)
            trajectory.append(row[:2])
    if not trajectory:
        raise ValueError('CSV contains no scans')
    probability = grid.probabilities()
    assert np.isfinite(probability).all() and ((probability >= 0) & (probability <= 1)).all()
    np.savetxt(args.output, probability, delimiter=',', fmt='%.8f')
    extent = [grid.xmin, grid.xmax, grid.ymin, grid.ymax]
    Path(args.output).with_suffix('.meta.json').write_text(json.dumps(dict(
        resolution=args.resolution, extent=extent, shape=list(probability.shape),
        layout='rows increase world Y; columns increase world X; cell-center coordinates',
        scans=len(trajectory), scans_used=math.ceil(recorded / args.scan_stride), scan_stride=args.scan_stride), indent=2))
    import matplotlib
    if not args.show:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(10, 8))
    plot = ax.imshow(probability, origin='lower', extent=extent, cmap='gray_r', vmin=0, vmax=1)
    path = np.asarray(trajectory)
    ax.plot(path[:, 0], path[:, 1], 'r-', linewidth=1,
            label='Sensor trajectory' if metadata.get('pose_reference') == 'sensor' else 'Vehicle trajectory')
    ax.scatter(*path[0], c='red', s=15)
    ax.set(xlabel='Gazebo world X (m)', ylabel='Gazebo world Y (m)', title='OCGM 660610823')
    ax.set_aspect('equal')
    ax.legend()
    fig.colorbar(plot, ax=ax, label='Occupancy probability')
    fig.tight_layout()
    fig.savefig(Path(args.output).with_suffix('.png'), dpi=160)
    if args.show:
        plt.show()
    plt.close(fig)
    print(f'{len(trajectory)} scans -> {probability.shape}; probability [{probability.min():.6f}, {probability.max():.6f}]')


if __name__ == '__main__':
    main()
