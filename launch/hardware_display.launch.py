"""Display real Shift Ostrich joints; optionally enable manual direction tests."""
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('shift_ostrich'))
    robot = ET.fromstring((share/'urdf/shift_ostrich.urdf').read_text())
    # CAD has no gripper geometry. Add its actuator to the hardware GUI only.
    ET.SubElement(robot, 'link', name='hardware_gripper_link')
    gripper = ET.SubElement(robot, 'joint', name='gripper_pitch', type='continuous')
    ET.SubElement(gripper, 'parent', link=robot.find("joint[@name='Arm_waist']/child").get('link'))
    ET.SubElement(gripper, 'child', link='hardware_gripper_link')
    ET.SubElement(gripper, 'axis', xyz='0 1 0')
    boolean = lambda name: ParameterValue(LaunchConfiguration(name), value_type=bool)
    text = lambda name: ParameterValue(LaunchConfiguration(name), value_type=str)
    number = lambda name: ParameterValue(LaunchConfiguration(name), value_type=float)
    return LaunchDescription([
        DeclareLaunchArgument('config_path', default_value=str(share/'config/hardware.yaml')),
        DeclareLaunchArgument('torque_on_start', default_value='true'),
        DeclareLaunchArgument('center_on_start', default_value='false'),
        DeclareLaunchArgument('torque_off_on_shutdown', default_value='true'),
        DeclareLaunchArgument('motor_ids', default_value=''),
        DeclareLaunchArgument('torque_motor_ids', default_value=''),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_joint_state_gui', default_value='false'),
        DeclareLaunchArgument('enable_joint_state_commands', default_value=LaunchConfiguration('use_joint_state_gui')),
        DeclareLaunchArgument('enable_trajectory_commands', default_value='true'),
        DeclareLaunchArgument('enable_velocity_commands', default_value='false'),
        DeclareLaunchArgument('passive_wheels', default_value='false'),
        DeclareLaunchArgument('enable_imu_tf', default_value='true',
                              description='Follow IMU attitude in RViz.'),
        DeclareLaunchArgument('sync_policy_feedback', default_value='false'),
        DeclareLaunchArgument('present_joint_state_rate_hz', default_value='15.0'),
        DeclareLaunchArgument('enable_opencr_imu', default_value=LaunchConfiguration('enable_imu_tf')),
        DeclareLaunchArgument('imu_topic', default_value='/opencr/imu'),
        DeclareLaunchArgument('opencr_imu_id', default_value='200'),
        DeclareLaunchArgument('opencr_imu_rate_hz', default_value='30.0'),
        DeclareLaunchArgument('imu_mount_roll_deg', default_value='-90.0'),
        DeclareLaunchArgument('imu_mount_pitch_deg', default_value='0.0'),
        DeclareLaunchArgument('imu_mount_yaw_deg', default_value='0.0'),
        DeclareLaunchArgument('imu_rpy_remap', default_value='pRy'),
        DeclareLaunchArgument('imu_pivot_x', default_value='0.0'),
        DeclareLaunchArgument('imu_pivot_y', default_value='0.0'),
        DeclareLaunchArgument('imu_pivot_z', default_value='0.0'),
        DeclareLaunchArgument('imu_zero_yaw_on_start', default_value='true'),
        DeclareLaunchArgument('imu_yaw_zero_samples', default_value='10'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': ET.tostring(robot, encoding='unicode'),
                          'use_sim_time': False}], output='screen'),
        Node(package='shift_ostrich', executable='dxl_joint_state_bridge.py', output='screen',
             parameters=[{**{name: boolean(name) for name in (
                 'center_on_start', 'torque_on_start', 'torque_off_on_shutdown',
                 'enable_joint_state_commands', 'enable_trajectory_commands', 'enable_velocity_commands',
                 'enable_opencr_imu', 'sync_policy_feedback', 'passive_wheels')},
                 **{name: text(name) for name in ('config_path', 'motor_ids', 'torque_motor_ids')},
                 'joint_command_topic': '/hardware/joint_commands',
                 'opencr_imu_topic': text('imu_topic'),
                 'opencr_imu_id': ParameterValue(LaunchConfiguration('opencr_imu_id'), value_type=int),
                 'opencr_imu_rate_hz': number('opencr_imu_rate_hz'),
                 'present_joint_state_rate_hz': number('present_joint_state_rate_hz'),
                 'publish_present_joint_states': True, 'present_joint_state_topic': '/hardware/joint_states'}]),
        Node(package='shift_ostrich', executable='imu_base_tf.py', output='screen',
             condition=IfCondition(LaunchConfiguration('enable_imu_tf')),
             parameters=[{'imu_topic': text('imu_topic'), 'parent_frame': 'world', 'child_frame': 'base_link',
                          'rpy_remap': text('imu_rpy_remap'),
                          'zero_yaw_on_start': boolean('imu_zero_yaw_on_start'),
                          'yaw_zero_samples': ParameterValue(LaunchConfiguration('imu_yaw_zero_samples'), value_type=int),
                          **{name: number('imu_'+name) for name in (
                              'mount_roll_deg','mount_pitch_deg','mount_yaw_deg','pivot_x','pivot_y','pivot_z')}}]),
        Node(package='tf2_ros', executable='static_transform_publisher',
             condition=UnlessCondition(LaunchConfiguration('enable_imu_tf')),
             arguments=['--frame-id','world','--child-frame-id','base_link']),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             name='hardware_state_aggregator',
             parameters=[{'source_list': ['/hardware/joint_states'], 'use_sim_time': False}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             name='hardware_command_gui',
             condition=IfCondition(LaunchConfiguration('use_joint_state_gui')),
             parameters=[{'use_sim_time': False}],
             remappings=[('/joint_states', '/hardware/joint_commands')]),
        Node(package='rviz2', executable='rviz2', condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', str(share/'config/hardware_rviz.rviz')]),
    ])
