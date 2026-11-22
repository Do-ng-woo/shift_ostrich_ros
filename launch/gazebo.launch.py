"""Preview Fusion geometry in Gazebo Sim; start paused to inspect the CAD pose."""
import json
import os
import shlex
import xml.etree.ElementTree as ET
import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_preview(context):
    share = Path(get_package_share_directory('shift_ostrich'))
    geometry = json.loads((share / 'urdf/spawn_geometry.json').read_text())
    controlled = LaunchConfiguration('controlled').perform(context) == 'true'
    fixed_base = LaunchConfiguration('fixed_base').perform(context) == 'true'
    filename = 'shift_ostrich_controlled.urdf' if controlled else 'shift_ostrich.urdf'
    description = (share / 'urdf' / filename).read_text().replace('$(find shift_ostrich)', str(share))
    initial_posture = LaunchConfiguration('initial_posture').perform(context)
    posture_spawn_z = geometry['spawn_z_m']
    if initial_posture == 'ready':
        if not controlled:
            raise ValueError('initial_posture:=ready requires controlled:=true')
        sys.path.insert(0, str(share))
        from core.trajectory import load_motion
        from core.model_geometry import posture_bounds
        saved = load_motion(share, 'transform')
        positions = dict(zip(saved['joint_names'], saved['stages'][0]['positions']))
        robot = ET.fromstring(description)
        for entry in robot.findall('ros2_control/joint'):
            entry.find("state_interface[@name='position']/param[@name='initial_value']").text = str(positions[entry.get('name')])
        lower, _ = posture_bounds(robot, share, positions)
        posture_spawn_z = float(geometry['ground_clearance_m'] - lower[2])
        description = ET.tostring(robot, encoding='unicode')
    if fixed_base:
        robot = ET.fromstring(description)
        ET.SubElement(robot, 'link', name='world')
        joint = ET.SubElement(robot, 'joint', name='preview_world_fixed', type='fixed')
        ET.SubElement(joint, 'parent', link='world')
        ET.SubElement(joint, 'child', link='base_link')
        description = ET.tostring(robot, encoding='unicode')
    spawn_description = description
    if LaunchConfiguration('observe_collisions').perform(context) == 'true':
        sys.path.insert(0, str(share))
        from core.simulation_model import prepare_observed_model
        spawn_description = prepare_observed_model(description)
    z = LaunchConfiguration('z').perform(context)
    if z == 'auto':
        z = str(posture_spawn_z)
    args = ['-v', '3']
    if LaunchConfiguration('paused').perform(context).lower() == 'false':
        args.append('-r')
    if LaunchConfiguration('gui').perform(context).lower() == 'false':
        args.append('-s')
    args.append(str(share / 'worlds/preview.sdf'))
    resources = os.pathsep.join(filter(None, [str(share.parent), os.environ.get('GZ_SIM_RESOURCE_PATH', '')]))
    actions = [
        SetEnvironmentVariable('GZ_PARTITION', LaunchConfiguration('partition').perform(context)),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resources),
        LogInfo(msg=f'Shift Ostrich: spawn z={z} m; controlled={controlled}, fixed_base={fixed_base}.'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(Path(get_package_share_directory('ros_gz_sim')) / 'launch/gz_sim.launch.py')),
            launch_arguments={'gz_args': shlex.join(args), 'on_exit_shutdown': 'true'}.items()),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description, 'use_sim_time': True}], output='screen'),
        Node(package='ros_gz_sim', executable='create',
             arguments=['-world', 'shift_preview', '-name', 'shift_ostrich', '-string', spawn_description, '-z', z], output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             arguments=['/world/shift_preview/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'] + ([] if controlled else [
                        '/model/shift_ostrich/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model']),
             remappings=[('/model/shift_ostrich/joint_state', '/joint_states'),
                         ('/world/shift_preview/clock', '/clock')], output='screen'),
    ]
    if controlled:
        for controller in ('joint_state_broadcaster', 'joint_trajectory_controller'):
            actions.append(Node(package='controller_manager', executable='spawner',
                                arguments=[controller, '--controller-manager-timeout', '60'], output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('paused', default_value='false', choices=['true', 'false'],
                              description='Start paused for CAD inspection. Unpaused joints are passive.'),
        DeclareLaunchArgument('z', default_value='auto', description='Root height in metres; auto uses mesh bounds + 5 mm.'),
        DeclareLaunchArgument('controlled', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('initial_posture', default_value='ready', choices=['zero', 'ready']),
        DeclareLaunchArgument('partition', default_value='shift_ostrich_preview'),
        DeclareLaunchArgument('observe_collisions', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('fixed_base', default_value='false', choices=['true', 'false'],
                              description='Anchor base_link to world for joint-only inspection.'),
        OpaqueFunction(function=launch_preview),
    ])
