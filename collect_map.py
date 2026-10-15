#!/usr/bin/env python3
"""Manual Gazebo mapping: collect until Ctrl+C, then build a custom Python OCGM."""
import argparse
from datetime import datetime
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys


ROOT = Path(__file__).resolve().parent
ROS_PYTHON = '/usr/bin/python3'


def load_ros_environment():
    """Source ROS for VS Code runs even when the editor was opened from the desktop."""
    keys = ('PATH', 'PYTHONPATH', 'LD_LIBRARY_PATH', 'AMENT_PREFIX_PATH',
            'CMAKE_PREFIX_PATH', 'COLCON_PREFIX_PATH', 'ROS_DISTRO',
            'ROS_VERSION', 'ROS_PYTHON_VERSION', 'GZ_CONFIG_PATH',
            'GZ_SIM_SYSTEM_PLUGIN_PATH', 'GZ_SIM_RESOURCE_PATH')
    export = 'import json,os; keys=' + repr(keys) + '; print(json.dumps({k:os.environ[k] for k in keys if k in os.environ}))'
    setup = ('source /opt/ros/jazzy/setup.bash && '
             'source /home/d-dream/robot_ws/install/setup.bash && '
             '/usr/bin/python3 -c "$1"')
    result = subprocess.run(['/bin/bash', '-c', setup, 'ocgm-ros-setup', export],
                            capture_output=True, text=True, timeout=20)
    if result.returncode:
        raise SystemExit('Could not load ROS Jazzy / robot_ws environment: ' + result.stderr.strip())
    os.environ.update(json.loads(result.stdout))


def stop(process):
    if process is not None and process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5)


def build(data, directory, resolution, stride, update_latest=True):
    result = directory / 'ocgm_660610823.csv'
    subprocess.run([ROS_PYTHON, str(ROOT / 'myworld_ocgm_660610823.py'),
                    '--input', str(data), '--output', str(result),
                    '--resolution', str(resolution), '--scan-stride', str(stride)], check=True)
    # Keep the latest map where occupancyongcheck.py reads it; retain each session.
    if update_latest:
        for suffix in ('.csv', '.meta.json', '.png'):
            source = result.with_suffix(suffix)
            shutil.copyfile(source, ROOT / source.name)
        print(f'แผนที่ล่าสุด / Latest map: {ROOT / result.name}', flush=True)
    subprocess.run([ROS_PYTHON, str(ROOT / 'load_laser_show_pointcloud.py'),
                    '--input', str(data), '--output', str(directory / 'pointcloud_660610823.png')], check=True)
    print(f'\nเสร็จแล้ว / Saved map: {result}\nภาพ / Plot: {result.with_suffix(".png")}\n', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=0, help='Optional wall seconds; default collect until Ctrl+C')
    parser.add_argument('--resolution', type=float, default=0.05)
    parser.add_argument('--scan-stride', type=int, default=5)
    parser.add_argument('--from-data', type=Path, help='Build a new map from existing CSV without starting ROS nodes')
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration < 0 or not math.isfinite(args.resolution) or args.resolution <= 0 or args.scan_stride < 1:
        parser.error('duration >= 0, resolution > 0, scan-stride >= 1 required')
    if not args.from_data:
        load_ros_environment()
        if not shutil.which('ros2'):
            parser.error('ROS 2 executable was not found after loading ROS Jazzy')
        try:
            check = subprocess.run(['ros2', 'topic', 'info', '/scan'], capture_output=True, text=True, timeout=10)
        except subprocess.TimeoutExpired:
            parser.error('ROS discovery timed out; start Gazebo first')
        if check.returncode or 'Publisher count: 0' in check.stdout:
            parser.error('No /scan publisher. Start Gazebo with rviz:=false first')
    else:
        args.from_data = args.from_data.resolve()
        if not args.from_data.is_file() or not args.from_data.with_suffix('.meta.json').is_file():
            parser.error('--from-data needs CSV and its .meta.json sidecar')
    # A fresh folder each time; never overwrite another session.
    directory = ROOT / 'maps' / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    directory.mkdir(parents=True, exist_ok=False)
    data = directory / 'data.csv'
    if args.from_data:
        shutil.copyfile(args.from_data, data)
        shutil.copyfile(args.from_data.with_suffix('.meta.json'), data.with_suffix('.meta.json'))
    else:
        bridge = recorder = None
        manually_stopped = False
        try:
            bridge = subprocess.Popen([
                'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                '/world/obstacle_course/dynamic_pose/info@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V',
                '--ros-args', '-r', '/world/obstacle_course/dynamic_pose/info:=/ocgm/manual_world_pose_array'],
                start_new_session=True)
            recorder = subprocess.Popen([
                ROS_PYTHON, str(ROOT / 'save_laser_show_pointcloud.py'),
                '--output', str(data), '--world-x', '0', '--world-y', '0', '--world-yaw', '0',
                '--world-pose-topic', '/ocgm/manual_world_pose_array', '--duration', str(args.duration)],
                start_new_session=True)
            print(f'\nกำลังเก็บข้อมูล / Recording: {data}\n'
                  'ขับรถด้วย teleop ในอีก Terminal / Drive using teleop in another terminal.\n'
                  'หยุดรถด้วย k ก่อน แล้วกด Ctrl+C ที่นี่เพื่อสร้างแมพ / Stop the robot, then Ctrl+C here to build the map.\n', flush=True)
            recorder.wait()
        except KeyboardInterrupt:
            manually_stopped = True
            print('\nหยุดบันทึก กำลังสร้างแมพ / Stopping recording and building map...', flush=True)
            stop(recorder)
        finally:
            stop(recorder)
            stop(bridge)
        if (recorder is None or not data.is_file() or not data.with_suffix('.meta.json').is_file()
                or (recorder.returncode != 0 and not manually_stopped)):
            raise SystemExit(f'No usable scans saved. Check Gazebo /scan and world pose. Data folder: {directory}')
        if recorder.returncode != 0:
            print(f'Recorder exited with {recorder.returncode} after stopping; validating saved scans while building map.', flush=True)
    build(data, directory, args.resolution, args.scan_stride)


if __name__ == '__main__':
    try:
        main()
    except subprocess.CalledProcessError as error:
        raise SystemExit(f'Mapping command failed (exit {error.returncode}); recorded data is preserved')
