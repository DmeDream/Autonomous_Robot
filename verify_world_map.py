#!/usr/bin/env python3
"""Validate this flat obstacle-course SDF against measured occupied cell centers.

Ground truth is read only for comparison and never used to generate the map.
"""
import csv
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


def pose(element):
    values = [float(v) for v in element.findtext('pose', '0 0 0 0 0 0').split()]
    if abs(values[3]) > 1e-6 or abs(values[4]) > 1e-6:
        raise ValueError('This validator supports planar world geometry only')
    return values


def compose(a, b):
    c, s = math.cos(a[5]), math.sin(a[5])
    return [a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1],
            a[2] + b[2], 0, 0, a[5] + b[5]]


def main():
    data_meta = json.loads(Path('data.meta.json').read_text())
    map_meta = json.loads(Path('ocgm_660610823.meta.json').read_text())
    with open('data.csv', newline='') as file:
        rows = list(csv.reader(file))
    width = 3 + 2 * data_meta['rays']
    assert all(len(row) == width for row in rows)
    grid = np.loadtxt('ocgm_660610823.csv', delimiter=',')
    assert list(grid.shape) == map_meta['shape']
    assert np.isfinite(grid).all() and ((grid >= 0) & (grid <= 1)).all()
    iy, ix = np.where(grid > 0.65)
    resolution = map_meta['resolution']
    x = map_meta['extent'][0] + (ix + 0.5) * resolution
    y = map_meta['extent'][2] + (iy + 0.5) * resolution
    nearest = np.full(len(x), np.inf)
    world_path = '/home/d-dream/robot_ws/src/robot_gazebo/worlds/obstacle_course.sdf'
    world = ET.parse(world_path).getroot().find('world')
    if world.findall('include'):
        raise ValueError('Resolve included models before validating this world')
    surfaces = 0
    # Actual sensor plane is approximately z=0.35 in this simulation.
    for model in world.findall('model'):
        if model.findall('model'):
            raise ValueError('Nested models need explicit resolution')
        for link in model.findall('link'):
            for collision in link.findall('collision'):
                transform = compose(compose(pose(model), pose(link)), pose(collision))
                geometry = collision.find('geometry')
                box, cylinder = geometry.find('box'), geometry.find('cylinder')
                if box is not None:
                    length, breadth, height = map(float, box.findtext('size').split())
                elif cylinder is not None:
                    height = float(cylinder.findtext('length'))
                else:
                    raise ValueError('Unsupported collision shape')
                if not transform[2] - height / 2 <= 0.35 <= transform[2] + height / 2:
                    continue
                surfaces += 1
                dx, dy = x - transform[0], y - transform[1]
                c, s = math.cos(transform[5]), math.sin(transform[5])
                lx, ly = c * dx + s * dy, -s * dx + c * dy
                if box is not None:
                    qx, qy = np.abs(lx) - length / 2, np.abs(ly) - breadth / 2
                    signed = np.hypot(np.maximum(qx, 0), np.maximum(qy, 0)) + np.minimum(np.maximum(qx, qy), 0)
                    distance = np.abs(signed)
                else:
                    distance = np.abs(np.hypot(lx, ly) - float(cylinder.findtext('radius')))
                nearest = np.minimum(nearest, distance)
    assert len(nearest) > 0
    report = dict(scans=len(rows) - (1 if data_meta.get('csv_header', True) else 0), rays=data_meta['rays'], columns=width,
                  grid_shape=list(grid.shape), probability_min=float(grid.min()),
                  probability_max=float(grid.max()), nan_count=int(np.isnan(grid).sum()),
                  occupied_cells=len(ix), world_collision_surfaces=surfaces,
                  occupied_surface_distance_median_m=float(np.median(nearest)),
                  occupied_surface_distance_p95_m=float(np.percentile(nearest, 95)),
                  fraction_occupied_within_0_10m=float(np.mean(nearest <= 0.1)),
                  unknown_fraction=float(np.mean(grid == 0.5)),
                  coverage='Visible surfaces from recorded trajectory; unseen regions remain unknown')
    assert report['fraction_occupied_within_0_10m'] >= 0.95, report
    Path('verification_report.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
