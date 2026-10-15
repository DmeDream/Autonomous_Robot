"""One launch for the five-room mapping simulation and ROS interfaces."""
from pathlib import Path
import xml.etree.ElementTree as ET
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    description = Path(get_package_share_directory('robot_description'))
    gazebo = Path(get_package_share_directory('robot_gazebo'))
    root = ET.fromstring(xacro.process_file(str(description / 'urdf/robot.urdf.xacro'),
        mappings={'controllers_file': str(description / 'config/controllers.yaml')}).toxml())
    # This simulation uses the native DiffDrive interface already bridged to
    # /cmd_vel. Give that system exclusive ownership of wheel commands.
    for control in root.findall('ros2_control'):
        root.remove(control)
    for element in root.findall('gazebo'):
        if element.get('reference', '').endswith('_wheel'):
            # The axle is invariant as the wheel spins: low lateral friction,
            # normal rolling traction. Four wheels must skid sideways to turn.
            element.find('mu1').text = '0.02'
            element.find('mu2').text = '1.0'
            direction = element.find('fdir1')
            if direction is None:
                direction = ET.SubElement(element, 'fdir1')
            direction.text = '0 0 1'
        for plugin in element.findall('plugin'):
            if 'gz_ros2_control' in plugin.get('name', ''):
                element.remove(plugin)
    urdf = ET.tostring(root, encoding='unicode')
    runtime = Path(__file__).resolve().parent / '.runtime'
    runtime.mkdir(exist_ok=True)
    (runtime / 'mapping_robot.urdf').write_text(urdf)
    rviz_config = yaml.safe_load((Path(get_package_share_directory('robot_bringup')) / 'rviz/robot.rviz').read_text())
    rviz_config['Visualization Manager']['Global Options']['Fixed Frame'] = 'world'
    rviz_config['Visualization Manager']['Displays'].append({
        'Class': 'rviz_default_plugins/Map', 'Name': 'Measured Gridmap', 'Enabled': True,
        'Topic': {'Value': '/map', 'Durability Policy': 'Transient Local', 'Reliability Policy': 'Reliable'},
        'Alpha': 0.7, 'Color Scheme': 'map'})
    (runtime / 'rooms.rviz').write_text(yaml.safe_dump(rviz_config))
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
            Path(get_package_share_directory('ros_gz_sim')) / 'launch/gz_sim.launch.py')),
            launch_arguments={'gz_args': '-s -r -v 2 ' + str(gazebo / 'worlds/obstacle_course.sdf')}.items()),
        ExecuteProcess(cmd=['gz', 'sim', '-g', '--render-engine', 'ogre', '--force-version', '8'], output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': urdf, 'use_sim_time': True}]),
        Node(package='ros_gz_sim', executable='create', arguments=[
            '-name', 'research_robot', '-topic', 'robot_description', '-x', '-1', '-y', '0', '-z', '0.08']),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             parameters=[{'config_file': str(gazebo / 'config/bridge.yaml'), 'use_sim_time': True}]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', arguments=[
            '/world/obstacle_course/dynamic_pose/info@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V'],
             remappings=[('/world/obstacle_course/dynamic_pose/info', '/ocgm/world_pose_array')]),
        Node(package='tf2_ros', executable='static_transform_publisher', arguments=[
            '--x', '-1', '--y', '0', '--z', '0', '--yaw', '0',
            '--frame-id', 'world', '--child-frame-id', 'odom']),
        Node(package='rviz2', executable='rviz2', condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', str(runtime / 'rooms.rviz')],
             parameters=[{'use_sim_time': True}]),
    ])
