#!/usr/bin/env python3
"""Send the current Shift Ostrich READY pose to a trajectory subscriber once."""
import math
from pathlib import Path
import sys
import time

source_root = Path(__file__).resolve().parents[1]
if (source_root / 'core').is_dir():
    sys.path.insert(0, str(source_root))

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from core.ready_posture import ready_targets


def ready_trajectory(share, move_duration_sec):
    """Build the standalone READY command using the verified model and joint order."""
    if not math.isfinite(move_duration_sec) or move_duration_sec <= 0:
        raise ValueError('move_duration_sec must be finite and positive')
    names, positions = ready_targets(share)
    msg = JointTrajectory()
    msg.joint_names = names
    point = JointTrajectoryPoint()
    point.positions = positions
    point.velocities = [0.0] * len(msg.joint_names)
    nanoseconds = round(move_duration_sec * 1_000_000_000)
    if not 0 < nanoseconds < 2**31 * 1_000_000_000:
        raise ValueError('move_duration_sec is outside the ROS Duration range')
    point.time_from_start.sec, point.time_from_start.nanosec = divmod(
        nanoseconds, 1_000_000_000
    )
    msg.points = [point]
    return msg


class ReadyPosturePublisher(Node):
    def __init__(self):
        super().__init__('ready_posture_publisher')
        self.declare_parameter('move_duration_sec', 3.0)
        self.declare_parameter('subscriber_timeout_sec', 10.0)
        self.declare_parameter('dry_run', False)
        self.message = ready_trajectory(
            Path(get_package_share_directory('shift_ostrich')),
            float(self.get_parameter('move_duration_sec').value),
        )
        self.publisher_ = self.create_publisher(
            JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10
        )

    def publish_posture(self):
        if self.get_parameter('dry_run').value:
            angles = dict(zip(self.message.joint_names,
                              map(math.degrees, self.message.points[0].positions)))
            self.get_logger().info(f'READY joint angles (deg): {angles}')
            return
        timeout = float(self.get_parameter('subscriber_timeout_sec').value)
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('subscriber_timeout_sec must be finite and positive')
        deadline = time.monotonic() + timeout
        while self.publisher_.get_subscription_count() == 0:
            if not rclpy.ok() or time.monotonic() >= deadline:
                raise RuntimeError(
                    'No trajectory subscriber; start Gazebo or enable the hardware '
                    'bridge enable_trajectory_commands parameter'
                )
            rclpy.spin_once(self, timeout_sec=0.05)
        self.publisher_.publish(self.message)
        if not self.publisher_.wait_for_all_acked(Duration(seconds=2.0)):
            raise RuntimeError('READY publication was not acknowledged')
        self.get_logger().info(
            'Sent READY: hip -50°, knee +60°, ankle -30° on both legs '
            '(pelvis 20° forward from the saved READY); '
            'shoulder +15° (2219 ticks), elbow -10° (upward), wrist 0° (straight). '
            'Topic publication does not confirm motion completion.'
        )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ReadyPosturePublisher()
        node.publish_posture()
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
