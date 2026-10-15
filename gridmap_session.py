#!/usr/bin/env python3
"""Record and display scans; finish and save when the separate room driver ends."""
import argparse
from datetime import datetime
import fcntl
import json
import os
import signal
import subprocess
import time

from collect_map import ROOT, ROS_PYTHON, build, stop


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--resolution', type=float, default=0.05)
    parser.add_argument('--scan-stride', type=int, default=5)
    args = parser.parse_args()
    if not 0 < args.resolution < 1 or args.scan_stride < 1:
        parser.error('0 < resolution < 1 and scan-stride >= 1 required')
    maps = ROOT / 'maps'
    maps.mkdir(exist_ok=True)
    lock = (maps / '.mapping.lock').open('w')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise SystemExit('A Gridmap session is already running')
    directory = maps / ('rooms_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    directory.mkdir()
    data = directory / 'data.csv'
    recorder = viewer = None
    def interrupted(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    try:
        recorder = subprocess.Popen([ROS_PYTHON, str(ROOT / 'save_laser_show_pointcloud.py'),
            '--output', str(data), '--world-x', '0', '--world-y', '0', '--world-yaw', '0',
            '--world-pose-topic', '/ocgm/world_pose_array'], start_new_session=True)
        deadline = time.monotonic() + 20
        while not data.with_suffix('.meta.json').exists():
            if recorder.poll() is not None or time.monotonic() > deadline:
                raise RuntimeError('No synchronized scans; start run_ros.sh first')
            time.sleep(0.1)
        viewer = subprocess.Popen([ROS_PYTHON, str(ROOT / 'live_gridmap.py'), '--input', str(data),
            '--resolution', str(args.resolution), '--scan-stride', str(args.scan_stride), '--ros-map'], start_new_session=True)
        deadline = time.monotonic() + 20
        while not data.with_suffix('.live_ready').exists():
            if viewer.poll() is not None or time.monotonic() > deadline:
                raise RuntimeError('Gridmap window did not open')
            time.sleep(0.1)
        session = dict(directory=str(directory), pid=os.getpid(), ready=True)
        temp = maps / 'active_session.tmp'
        temp.write_text(json.dumps(session))
        temp.replace(maps / 'active_session.json')
        print('Gridmap ready. In another terminal: bash run_rooms.sh', flush=True)
        print(f'Recording: {directory}', flush=True)
        while not (directory / 'survey.json').exists():
            if recorder.poll() is not None:
                raise RuntimeError('Scan recorder stopped unexpectedly')
            time.sleep(0.2)
    except KeyboardInterrupt:
        print('Stopping recording and saving Gridmap...', flush=True)
    finally:
        stop(recorder)
    if data.exists() and data.stat().st_size and data.with_suffix('.meta.json').exists():
        build(data, directory, args.resolution, args.scan_stride)
    # Leave the viewer open so the completed map remains visible.


if __name__ == '__main__':
    main()
