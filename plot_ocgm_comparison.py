#!/usr/bin/env python3
"""Compare the measured OCGM with an actual aligned Gazebo camera screenshot.

Default crop is calibrated against obstacle_course walls in the 581x797 screenshot
captured at camera world position (0,0,17), quaternion (-0.5,0.5,0.5,0.5).
It spans approximately x=[-10.5,10.5], y=[-8.5,8.5] at the ground plane.
Elevated obstacle surfaces and shadows differ from the 2D LiDAR plane.
"""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from load_laser_show_pointcloud import load_data


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--screenshot', default='gazebo_top_view.png')
    parser.add_argument('--crop', nargs=4, type=int, default=[96, 241, 486, 557],
                        metavar=('LEFT', 'TOP', 'RIGHT', 'BOTTOM'))
    args = parser.parse_args()
    metadata = json.loads(Path('ocgm_660610823.meta.json').read_text())
    probability = np.loadtxt('ocgm_660610823.csv', delimiter=',')
    screenshot = plt.imread(args.screenshot)
    left, top, right, bottom = args.crop
    if not (0 <= left < right <= screenshot.shape[1] and 0 <= top < bottom <= screenshot.shape[0]):
        raise ValueError('Crop outside screenshot')
    extent = metadata['extent']
    screenshot_extent = [-10.5, 10.5, -8.5, 8.5]
    X, Y, TH, lengths, angles, data_meta = load_data()
    fig, axes = plt.subplots(1, 2, figsize=(14, 7), sharex=True, sharey=True)
    axes[0].imshow(probability, cmap='gray_r', vmin=0, vmax=1, origin='lower', extent=extent)
    axes[0].plot(X[:, 0], Y[:, 0], 'r-', linewidth=1, label='Sensor trajectory')
    axes[0].set_title('Custom Python OCGM 660610823')
    axes[0].legend(loc='upper right')
    # Preserve the screenshot itself; this is a scientific comparison figure.
    axes[1].imshow(screenshot[top:bottom, left:right], origin='upper', extent=screenshot_extent)
    axes[1].set_title('Actual Gazebo top view (ground-plane scale)')
    for ax in axes:
        ax.set(xlim=extent[:2], ylim=extent[2:], xlabel='World X (m)', ylabel='World Y (m)')
        ax.set_aspect('equal')
    fig.text(0.5, 0.04, 'Same world axes and aspect. Screenshot scale calibrated from boundary walls; elevated surfaces/shadows are not LiDAR returns.',
             ha='center', fontsize=9)
    fig.tight_layout(rect=[0, 0.07, 1, 1])
    fig.savefig('ocgm_vs_gazebo_660610823.png', dpi=180)
    plt.close(fig)
    print('Saved ocgm_vs_gazebo_660610823.png')


if __name__ == '__main__':
    main()
