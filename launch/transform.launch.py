"""Run the Biped transformation on the updated CAD model on the ground."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('shift_ostrich'))
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('autoplay', default_value='true', description='Execute once after controller is ready.'),
        DeclareLaunchArgument('fixed_base', default_value='false'),
        DeclareLaunchArgument('z', default_value='auto'),
        DeclareLaunchArgument('result_file', default_value=''),
        DeclareLaunchArgument('partition', default_value='shift_ostrich_transform'),
        DeclareLaunchArgument('observe_collisions', default_value='true'),
        DeclareLaunchArgument('metrics_file', default_value=''),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share / 'launch/gazebo.launch.py')),
            launch_arguments={'gui': LaunchConfiguration('gui'), 'paused': 'false', 'controlled': 'true',
                              'initial_posture': 'ready',
                              'fixed_base': LaunchConfiguration('fixed_base'), 'z': LaunchConfiguration('z'),
                              'partition': LaunchConfiguration('partition'),
                              'observe_collisions': LaunchConfiguration('observe_collisions')}.items()),
        Node(package='shift_ostrich', executable='transform.py', output='screen',
             additional_env={'GZ_PARTITION': LaunchConfiguration('partition')},
             condition=IfCondition(LaunchConfiguration('autoplay')),
             parameters=[{'use_sim_time': True,
                          'result_file': ParameterValue(LaunchConfiguration('result_file'), value_type=str),
                          'metrics_file': ParameterValue(LaunchConfiguration('metrics_file'), value_type=str)}]),
    ])
