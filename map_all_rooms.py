#!/usr/bin/env python3
"""Drive an [x, y, r] array through all five rooms and save a measured map."""
import argparse
from datetime import datetime
import fcntl
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

from collect_map import ROOT, ROS_PYTHON, build, load_ros_environment, stop


# World coordinates: x/y in metres; r is yaw in radians.
# Edit this array to change the route. The controller visits 0 ... t-1.
WAYPOINTS = [
    [-6.6,  0.0,  math.pi/2],
    [-6.6,  2.8,  math.pi/2],  # Room 1
    [-8.0,  3.0,  0.0],
    [-4.5,  4.8, -math.pi/2],
    [-5.0,  3.0,  math.pi],
    [-6.6,  2.8, -math.pi/2],
    [-6.6,  0.0,  0.0],
    [ 0.0,  0.0,  math.pi/2],
    [ 0.0,  3.0,  math.pi/2],  # Room 2
    [ 0.0,  5.0,  math.pi],
    [ 0.0,  3.0, -math.pi/2],
    [ 0.0,  0.0,  0.0],
    [ 6.6,  0.0,  math.pi/2],
    [ 6.6,  3.0,  math.pi/2],  # Room 3
    [ 4.6,  3.2,  math.pi/2],
    [ 4.6,  6.8,  0.0],
    [ 8.6,  3.2,  math.pi],
    [ 6.6,  3.0, -math.pi/2],
    [ 6.6,  0.0,  math.pi],
    [ 5.0,  0.0, -math.pi/2],
    [ 5.0, -3.0, -math.pi/2],  # Room 5
    [ 2.7, -6.4,  0.0],
    [ 7.3, -6.5,  math.pi],
    [ 5.0, -3.0,  math.pi/2],
    [ 5.0,  0.0,  math.pi],
    [-5.0,  0.0, -math.pi/2],
    [-5.0, -3.0, -math.pi/2],  # Room 4
    [-5.0, -7.0,  0.0],
    [-1.4, -5.0,  math.pi],
    [-5.0, -3.0,  math.pi/2],
    [-5.0,  0.0,  0.0],
    [-1.0,  0.0,  0.0],      # Return home
]


def goals_from_array(poses):
    goals = []
    t = len(poses)
    if t == 0:
        raise ValueError('WAYPOINTS must contain at least one [x, y, r] row')
    for i in range(t):
        if len(poses[i]) != 3 or not all(math.isfinite(float(v)) for v in poses[i]):
            raise ValueError(f'WAYPOINTS[{i}] must contain three finite numbers')
        x, y, r = map(float, poses[i])
        room = None
        if y > 1.4:
            room = 1 if x < -3.3 else 3 if x > 3.3 else 2
        elif y < -1.4:
            room = 4 if x < 0 else 5
        goal = dict(name=f'pose[{i}] / room {room}' if room else f'pose[{i}] / corridor',
                    x=x, y=y, yaw=math.atan2(math.sin(r), math.cos(r)))
        if room:
            goal['room'] = room
        goals.append(goal)
    return goals


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--speed', type=float, default=0.30)
    parser.add_argument('--duration', type=float, default=1200)
    parser.add_argument('--resolution', type=float, default=0.05)
    parser.add_argument('--scan-stride', type=int, default=5)
    parser.add_argument('--limit', type=int, help='Try only the first N waypoints')
    parser.add_argument('--check-route', action='store_true', help='Check the array without driving or recording')
    parser.add_argument('--live', action='store_true', help='Also show the measured map as it grows')
    parser.add_argument('--output-dir', type=Path, help='New output directory; leave the latest root map untouched')
    args = parser.parse_args()
    if (not math.isfinite(args.speed) or not 0 < args.speed <= .4 or
            not math.isfinite(args.duration) or args.duration <= 0 or
            not math.isfinite(args.resolution) or not 0 < args.resolution < 1 or args.scan_stride < 1):
        parser.error('0 < speed <= 0.4, positive duration, 0 < resolution < 1 and scan-stride >= 1 required')
    if args.limit is not None and not 1 <= args.limit <= len(WAYPOINTS):
        parser.error(f'limit must be between 1 and {len(WAYPOINTS)}')
    try:
        goals = goals_from_array(WAYPOINTS[:args.limit])
    except (ValueError, TypeError) as error:
        parser.error(str(error))
    load_ros_environment()
    # Validate every point and planned segment before starting a recording.
    with tempfile.TemporaryDirectory(prefix='room-array-') as temporary:
        checked = Path(temporary) / 'waypoints.json'
        checked.write_text(json.dumps(goals))
        checked_route = subprocess.run([ROS_PYTHON, str(ROOT / 'room_route.py'), '--poses', str(checked), '--check-route'],
                                       capture_output=True, text=True)
        if checked_route.returncode:
            raise SystemExit(checked_route.stderr or checked_route.stdout)
    if args.check_route:
        print(checked_route.stdout, end='')
        print(f'Array checked: t={len(goals)}, indices 0...{len(goals)-1}', flush=True)
        return
    print(f'Array checked: t={len(goals)}, indices 0...{len(goals)-1}', flush=True)
    maps = ROOT / 'maps'
    maps.mkdir(exist_ok=True)
    lock = (maps / '.mapping.lock').open('w')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise SystemExit('Another Gridmap session is running; stop it first')
    directory = args.output_dir.resolve() if args.output_dir else maps / (
        'array_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    directory.mkdir(parents=True, exist_ok=False)
    data = directory / 'data.csv'
    poses_file = directory / 'waypoints.json'
    poses_file.write_text(json.dumps(goals, indent=2))
    (directory / 'waypoints_xyr.json').write_text(json.dumps(
        [[g['x'], g['y'], g['yaw']] for g in goals], indent=2))
    pose_topic = f'/ocgm/array_world_pose_{os.getpid()}'
    bridge = recorder = driver = viewer = None
    failure = None

    def interrupted(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    print(f'Output: {directory}', flush=True)
    try:
        # Verify the exact existing world and named robot before using PoseArray index 0.
        subprocess.run([ROS_PYTHON, '-c', 'from auto_map import verify_world; verify_world()'],
                       cwd=ROOT, check=True)
        bridge = subprocess.Popen(['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/world/obstacle_course/dynamic_pose/info@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V',
            '--ros-args', '-r', '/world/obstacle_course/dynamic_pose/info:=' + pose_topic], start_new_session=True)
        recorder = subprocess.Popen([ROS_PYTHON, str(ROOT / 'save_laser_show_pointcloud.py'),
            '--output', str(data), '--world-x', '0', '--world-y', '0', '--world-yaw', '0',
            '--world-pose-topic', pose_topic], start_new_session=True)
        deadline = time.monotonic() + 20
        while not data.with_suffix('.meta.json').exists() or not data.stat().st_size:
            if time.monotonic() > deadline or recorder.poll() is not None or bridge.poll() is not None:
                raise RuntimeError('No synchronized scans; start ROS bringup first')
            time.sleep(.1)
        if args.live:
            viewer = subprocess.Popen([ROS_PYTHON, str(ROOT / 'live_gridmap.py'), '--input', str(data),
                '--resolution', str(args.resolution), '--scan-stride', str(args.scan_stride)], start_new_session=True)
            deadline = time.monotonic() + 20
            while not data.with_suffix('.live_ready').exists():
                if viewer.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError('Live Gridmap did not open')
                time.sleep(.1)
        session = directory / 'session.json'
        session.write_text(json.dumps(dict(directory=str(directory), pid=os.getpid(), ready=True)))
        driver = subprocess.Popen([ROS_PYTHON, str(ROOT / 'room_route.py'), '--poses', str(poses_file),
            '--session', str(session), '--pose-topic', pose_topic,
            '--speed', str(args.speed), '--duration', str(args.duration)], start_new_session=True)
        driver.wait()
        if driver.returncode:
            failure = f'Room driver stopped (exit {driver.returncode}); see survey.json'
    except KeyboardInterrupt:
        print('Stopping the robot and saving the measured map…', flush=True)
    except Exception as error:
        failure = str(error)
    finally:
        stop(driver)
        stop(recorder)
        stop(bridge)
    if data.is_file() and data.stat().st_size and data.with_suffix('.meta.json').exists():
        build(data, directory, args.resolution, args.scan_stride, update_latest=args.output_dir is None)
        print(f'Actual pose[t] array: {directory / "poses_t.npy"}\n'
              f'Pose/delta table: {directory / "poses_t.csv"}', flush=True)
    if failure:
        raise SystemExit(failure)


if __name__ == '__main__':
    main()
