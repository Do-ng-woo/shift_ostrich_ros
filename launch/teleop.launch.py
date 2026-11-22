"""Start the passive leader and teleop adapter; reuse the running OpenCR bridge."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression


def generate_launch_description():
    share = Path(get_package_share_directory('shift_ostrich'))
    config = ParameterValue(LaunchConfiguration('config_path'), value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument('config_path', default_value=str(share/'config/teleop.yaml')),
        DeclareLaunchArgument('preview_only', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('enable_on_start', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('leader_spring', default_value=PythonExpression([
            "'false' if '", LaunchConfiguration('preview_only'), "' == 'true' else 'true'"
        ]), choices=['true', 'false']),
        Node(package='shift_ostrich', executable='leader_reader.py', output='screen',
             parameters=[{'config_path': config,
                          'enable_gripper_spring': ParameterValue(LaunchConfiguration('leader_spring'), value_type=bool)}]),
        Node(package='shift_ostrich', executable='arm_teleop.py', output='screen',
             parameters=[{'config_path': config,
                          'preview_only': ParameterValue(LaunchConfiguration('preview_only'), value_type=bool),
                          'enable_on_start': ParameterValue(LaunchConfiguration('enable_on_start'), value_type=bool)}]),
    ])
