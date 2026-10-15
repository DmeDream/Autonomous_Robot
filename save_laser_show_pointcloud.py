#!/usr/bin/env python3
"""Gazebo counterpart of the supplied external CoppeliaSim recorder.

Writes numeric-only CSV with sensor-base world x,y,yaw and measured range/angle pairs.
No CoppeliaSim dependency; use the existing Gazebo launch and teleop separately.
"""
import sys

from scan_to_csv import main


if __name__ == '__main__':
    sys.argv.extend(['--no-header', '--sensor-pose'])
    main()
