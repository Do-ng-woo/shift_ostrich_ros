"""CH V2 exclusive hardware interface; load the policy in a separate terminal."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
  share = Path(get_package_share_directory("shift_ostrich"))
  return LaunchDescription(
    [
      DeclareLaunchArgument("rviz", default_value="true"),
      DeclareLaunchArgument(
        "config_path", default_value=str(share / "config/hardware.yaml")
      ),
      IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / "launch/hardware_display.launch.py")),
        launch_arguments={
          "config_path": LaunchConfiguration("config_path"),
          "rviz": LaunchConfiguration("rviz"),
          "sync_policy_feedback": "true",
          "present_joint_state_rate_hz": "50.0",
          "opencr_imu_rate_hz": "50.0",
          "enable_opencr_imu": "true",
          "enable_joint_state_commands": "false",
          "enable_trajectory_commands": "true",
          "enable_velocity_commands": "false",
          "passive_wheels": "true",
          "use_joint_state_gui": "false",
          "center_on_start": "false",
          "torque_on_start": "true",
        }.items(),
      ),
    ]
  )
