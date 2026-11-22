#!/usr/bin/env python3
"""J146 hardware execution: settled READY, live feedback and bounded targets."""

import argparse
import csv
import json
import math
import subprocess
import time
from pathlib import Path
from tempfile import TemporaryDirectory

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .contract import ARMS, DT, LOWER
from .imu import ImuTransform, quaternion_to_matrix
from .policy import READY, History, Policy, nominal_index, phase

TOPIC = "/joint_trajectory_controller/joint_trajectory"
AUX_JOINTS = (*ARMS, "L_wheel", "R_wheel")


def playback_times(control_step, speed):
  """Keep real execution limits separate from slowed reference phase time."""
  if not math.isfinite(speed) or not 0 < speed <= 1:
    raise ValueError("playback speed must be finite and in (0, 1]")
  real_seconds = control_step * DT
  return real_seconds, real_seconds * speed


def prepare_policy(path, mjlab_root, output_dir):
  source = path.expanduser().resolve()
  if not source.is_file():
    raise ValueError(f"J146 checkpoint not found: {source}")
  if source.suffix == ".npz":
    Policy(source)
    return source
  if source.suffix != ".pt":
    raise ValueError("Choose a J146 .pt checkpoint or exported .npz")
  project = mjlab_root.expanduser().resolve()
  if not (project / "pyproject.toml").is_file():
    raise ValueError(f"mjlab project not found: {project}")
  output = output_dir / "j146_policy.npz"
  try:
    subprocess.run(
      [
        "uv",
        "run",
        "--project",
        str(project),
        "--no-sync",
        "python",
        str(Path(__file__).with_name("export_policy.py")),
        "--checkpoint",
        str(source),
        "--output",
        str(output),
      ],
      check=True,
    )
  except (FileNotFoundError, subprocess.CalledProcessError) as exc:
    raise ValueError("J146 export failed; ROS/hardware were not started") from exc
  Policy(output)
  return output


class PolicyNode(Node):
  def __init__(self, args):
    super().__init__("shift_j146_policy")
    self.yaw = None
    self.initial_yaw = None
    self.args = args
    self.policy = Policy(args.policy)
    self.q = self.dq = self.gravity = self.gyro = None
    self.joint_time = self.imu_time = 0.0
    self.aux_positions = {}
    self.bias = None
    self.gyro_samples = []
    self.initial = None
    self.ready_started = None
    self.ready = False
    self.ready_within_since = None
    self.running = False
    self.episode_step = 0
    self.last_action = np.zeros(10, np.float32)
    self.history = History()
    self.last_target = None
    self.last_cycle = None
    self.stopped = False
    self.hardware_fault = {}
    self.last_wait_warning = -math.inf
    self.imu_transform = ImuTransform(-90.0, 0.0, 0.0, "pRy")
    self.pub = self.create_publisher(JointTrajectory, TOPIC, 1) if args.arm else None
    self.create_subscription(JointState, "/hardware/joint_states", self.on_joints, 1)
    self.create_subscription(Imu, "/opencr/imu", self.on_imu, qos_profile_sensor_data)
    self.create_subscription(
      String,
      "/hardware/feedback_fault",
      self.on_hardware_fault,
      QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )
    self.create_service(Trigger, "/ch_policy/start", self.start_policy)
    self.create_service(Trigger, "/ch_policy/stop", self.stop_policy)
    self.client = AsyncParameterClient(self, "/dxl_joint_state_bridge")
    self.interface_valid = False
    self.interface_checked = 0.0
    self.pending = None
    self.limits = {}
    config = yaml.safe_load(args.hardware_config.read_text())
    factor = config["conversion"]["tick_per_rad"]
    for motor in config["joints"]:
      if motor["joint_name"] not in LOWER:
        continue
      center = 2048 if motor.get("center_tick") is None else motor["center_tick"]
      direction = 1 if motor.get("direction") is None else motor["direction"]
      values = [
        (tick - center) / (direction * factor)
        for tick in (
          motor.get("min_position_tick", 0),
          motor.get("max_position_tick", 4095),
        )
      ]
      self.limits[motor["joint_name"]] = (min(values), max(values))
    if set(self.limits) != set(LOWER):
      raise ValueError("Hardware mapping missing policy lower joints")
    self.check_limits(READY)
    args.log_dir.mkdir(parents=True, exist_ok=True)
    prefix = getattr(self.policy, "log_prefix", "ch_v2")
    self.log = (args.log_dir / f"{prefix}_{time.time_ns()}.csv").open("w", newline="")
    self.writer = csv.writer(self.log)
    self.writer.writerow(
      [
        "time",
        "state",
        "phase_time",
        "tilt_deg",
        "obs_zmax",
        *[f"q/{j}" for j in LOWER],
        *[f"target/{j}" for j in LOWER],
        "gravity_roll_deg",
        "gravity_pitch_deg",
        *[f"gravity/{a}" for a in "xyz"],
        *[f"gyro/{a}" for a in "xyz"],
        *[f"dq/{j}" for j in LOWER],
        *[f"measured/{j}" for j in AUX_JOINTS],
      ]
    )
    self.timer = self.create_timer(DT, self.cycle)
    self.get_logger().info(
      f"{getattr(self.policy, 'label', 'J146')} width={self.policy.width:.3f} rad; "
      f"curriculum={self.policy.progress:.2f}; "
      f"command={self.policy.speed:.3f} m/s; playback={args.playback_speed:.2f}x; "
      f"arm={args.arm}. Waiting for live feedback."
    )

  def on_hardware_fault(self, msg):
    try:
      value = json.loads(msg.data)
    except (ValueError, TypeError):
      return
    if isinstance(value, dict) and value.get("fault"):
      self.hardware_fault = value
      self.stop(
        "Hardware bridge latched fault; READY/policy blocked: " + json.dumps(value)
      )

  def startup_wait_reason(self, now, interface):
    reasons = []
    if self.q is None:
      reasons.append("no complete lower-body position/velocity feedback")
    elif now - self.joint_time > self.args.sensor_timeout:
      reasons.append("joint feedback is stale")
    if self.gravity is None:
      reasons.append("no valid IMU orientation")
    elif now - self.imu_time > self.args.sensor_timeout:
      reasons.append("IMU feedback is stale")
    if self.bias is None:
      reasons.append("waiting for stationary gyro calibration")
    if not interface:
      reasons.append(
        "bridge configuration/parameter response or exclusive topic ownership not ready"
      )
    return "; ".join(reasons)

  def check_limits(self, target):
    if not np.isfinite(target).all():
      raise ValueError("Non-finite joint target")
    for name, value in zip(LOWER, target, strict=True):
      lo, hi = self.limits[name]
      if not lo <= value <= hi:
        raise ValueError(f"{name}: target {value:.4f} outside [{lo:.4f},{hi:.4f}]")

  def on_joints(self, msg):
    if len(msg.name) != len(msg.position) or len(msg.velocity) != len(msg.name):
      return
    index = {name: i for i, name in enumerate(msg.name)}
    if not all(j in index for j in LOWER):
      return
    q = np.array([msg.position[index[j]] for j in LOWER], np.float32)
    dq = np.array([msg.velocity[index[j]] for j in LOWER], np.float32)
    age = (
      self.get_clock().now().nanoseconds
      - (msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec)
    ) / 1e9
    if (
      not np.isfinite(q).all()
      or not np.isfinite(dq).all()
      or not -0.02 <= age <= self.args.sensor_timeout
    ):
      return
    self.aux_positions = {
      j: float(msg.position[index[j]]) for j in AUX_JOINTS if j in index
    }
    self.q, self.dq = q, dq
    self.joint_time = time.monotonic()

  def on_imu(self, msg):
    if msg.orientation_covariance[0] < 0 or msg.angular_velocity_covariance[0] < 0:
      return
    quat = np.array(
      [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
    )
    gyro = np.array(
      [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
    )
    age = (
      self.get_clock().now().nanoseconds
      - (msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec)
    ) / 1e9
    if (
      not np.isfinite(quat).all()
      or not np.isfinite(gyro).all()
      or not 0.9 <= np.linalg.norm(quat) <= 1.1
      or not -0.02 <= age <= self.args.sensor_timeout
    ):
      return
    self.gravity = self.imu_transform.projected_gravity(quat)
    self.gyro = self.imu_transform.angular_velocity(gyro)
    self.imu_time = time.monotonic()
    rotation = quaternion_to_matrix(self.imu_transform.orientation(quat))
    self.yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    if self.bias is None:
      if np.linalg.norm(self.gyro) > 0.35 or (
        self.dq is not None and np.max(np.abs(self.dq)) > 0.15
      ):
        self.gyro_samples.clear()
        return
      self.gyro_samples.append(self.gyro.copy())
      if len(self.gyro_samples) >= 100:
        self.bias = np.mean(self.gyro_samples, axis=0)
        self.get_logger().info("Stationary gyro calibration complete")

  def check_interface(self, now):
    if not self.args.arm:
      return True
    if self.pending is not None and self.pending.done():
      try:
        values = self.pending.result().values
        valid = (
          len(values) == 8
          and values[0].bool_value
          and values[1].bool_value
          and values[2].bool_value
          and not values[3].bool_value
          and not values[4].bool_value
          and values[5].double_value >= 40
          and values[6].double_value >= 40
          and values[7].bool_value
        )
        self.interface_valid = valid
        self.interface_checked = now
      except Exception:
        self.interface_valid = False
      self.pending = None
    if (
      self.pending is None
      and now - self.interface_checked > 1
      and self.client.services_are_ready()
    ):
      self.pending = self.client.get_parameters(
        [
          "torque_on_start",
          "enable_trajectory_commands",
          "sync_policy_feedback",
          "enable_joint_state_commands",
          "enable_velocity_commands",
          "present_joint_state_rate_hz",
          "opencr_imu_rate_hz",
          "passive_wheels",
        ]
      )
    return (
      self.interface_valid
      and now - self.interface_checked < 2.5
      and self.count_subscribers(TOPIC) == 1
      and self.count_publishers(TOPIC) == 1
      and self.count_publishers("/hardware/joint_states") == 1
      and self.count_publishers("/opencr/imu") == 1
    )

  def begin_policy(self):
    if self.yaw is None:
      self.stop("No valid IMU heading; J146 start blocked")
      return
    self.initial_yaw = self.yaw
    self.running = True
    self.episode_step = 0
    self.last_action.fill(0)
    self.history = History()
    self.last_cycle = None
    self.get_logger().warn("policy READY verified; starting policy at phase time 0")

  def start_policy(self, request, response):
    if self.ready and not self.stopped and not self.running:
      if (
        np.max(np.abs(self.q - READY)) > np.deg2rad(8) or np.max(np.abs(self.dq)) > 0.15
      ):
        response.success = False
        response.message = "Robot is no longer settled at policy READY"
        return response
      self.begin_policy()
      response.success = True
      response.message = "J146 forward policy started"
    else:
      response.success = False
      response.message = (
        "Not READY, already running, or stopped; restart node after a stop"
      )
    return response

  def update_ready(self, now, elapsed):
    if elapsed < self.args.ready_time:
      return
    settled = (
      np.max(np.abs(self.q - READY)) <= np.deg2rad(8)
      and np.max(np.abs(self.dq)) <= 0.15
    )
    if not settled:
      self.ready_within_since = None
    elif self.ready_within_since is None:
      self.ready_within_since = now
    if (
      self.ready_within_since is not None
      and now - self.ready_within_since >= self.args.ready_settle_time
    ):
      self.ready = True
      if self.args.start_after_ready:
        self.begin_policy()
      else:
        self.get_logger().warn("READY verified. Call /ch_policy/start to walk.")
    elif elapsed > self.args.ready_time + 10:
      self.stop("policy READY did not settle within 10 seconds; policy not started")

  def stop_policy(self, request, response):
    self.stop("Operator stop; holding last target")
    response.success = True
    response.message = "Policy stopped; bridge holds last target. Support robot before hardware shutdown."
    return response

  def stop(self, reason):
    if not self.stopped:
      self.get_logger().error(reason)
    self.running = False
    self.stopped = True
    # Do not issue a blind READY move with stale sensors. The position motors
    # retain their most recent goal; stopping the hardware launch releases them.

  def publish(self, target):
    self.check_limits(target)
    if self.pub is not None:
      msg = JointTrajectory()
      msg.joint_names = list(LOWER)
      point = JointTrajectoryPoint()
      point.positions = [float(x) for x in target]
      msg.points = [point]  # zero time: direct stream, no second interpolation
      self.pub.publish(msg)
    self.last_target = target.copy()

  def walking_target(self, t):
    if self.yaw is None or self.initial_yaw is None:
      raise ValueError("J146 heading not initialized")
    nominal = self.policy.nominal[nominal_index(round(t / DT))]
    relative_yaw = self.yaw - self.initial_yaw
    obs = self.history.append(
      self.q,
      self.dq,
      self.gravity,
      self.gyro - self.bias,
      self.last_action,
      [0.03, 0.0, 0.0],
      nominal,
      phase(t),
      [math.sin(relative_yaw), math.cos(relative_yaw)],
    )
    zmax = float(np.max(np.abs(self.policy.normalized(obs))))
    if zmax > 15:
      raise ValueError(f"J146 observation outlier: zmax={zmax:.1f}")
    residual = self.policy(obs)
    self.last_action = np.clip(residual, -1, 1)
    return self.policy.target(nominal, residual), zmax

  def cycle(self):
    now = time.monotonic()
    interface = self.check_interface(now)
    if self.stopped:
      return
    if self.q is None or self.gravity is None or self.bias is None or not interface:
      if self.ready_started is not None:
        self.stop("Hardware interface or sensor stream lost")
      elif now - self.last_wait_warning >= 5.0:
        self.last_wait_warning = now
        self.get_logger().warn(
          "Waiting before READY: " + self.startup_wait_reason(now, interface)
        )
      return
    if (
      now - self.joint_time > self.args.sensor_timeout
      or now - self.imu_time > self.args.sensor_timeout
    ):
      self.stop("Sensor timeout; holding last target")
      return
    tilt = math.degrees(math.acos(float(np.clip(-self.gravity[2], -1, 1))))
    if tilt > self.args.max_tilt:
      self.stop(f"Tilt stop: {tilt:.1f} deg")
      return
    if self.running and self.last_cycle is not None and now - self.last_cycle > 0.06:
      self.stop("Policy timing overrun; holding last target")
      return
    self.last_cycle = now
    if self.ready_started is None:
      self.initial = self.q.copy()
      self.last_target = self.q.copy()
      self.ready_started = now
      self.get_logger().warn(
        "Moving to policy READY over "
        f"{self.args.ready_time:.1f}s: hip -20, knee +40, ankle -20 deg; "
        "arms hold measured startup pose"
      )
    elapsed = now - self.ready_started
    zmax = 0.0
    if not self.ready:
      u = np.clip(elapsed / self.args.ready_time, 0, 1)
      target = self.initial + (READY - self.initial) * (u * u * (3 - 2 * u))
      self.update_ready(now, elapsed)
      if self.stopped:
        return
    elif not self.running:
      target = READY.copy()
    else:
      real_seconds, t = playback_times(self.episode_step, self.args.playback_speed)
      try:
        target, zmax = self.walking_target(t)
      except ValueError as exc:
        self.stop(str(exc))
        return
      self.episode_step += 1
      if self.args.duration > 0 and real_seconds >= self.args.duration:
        self.stop("Requested duration complete")
        return
    maximum_step = 0.12 * (self.args.playback_speed if self.running else 1.0)
    target = self.last_target + np.clip(
      target - self.last_target, -maximum_step, maximum_step
    )
    try:
      self.publish(target)
    except ValueError as exc:
      self.stop(str(exc))
      return
    self.writer.writerow(
      [
        now,
        "walking" if self.running else "ready",
        self.episode_step * DT * self.args.playback_speed,
        tilt,
        zmax,
        *self.q,
        *target,
        math.degrees(math.atan2(-self.gravity[1], -self.gravity[2])),
        math.degrees(math.atan2(self.gravity[0], np.linalg.norm(self.gravity[1:]))),
        *self.gravity,
        *(self.gyro - self.bias),
        *self.dq,
        *[self.aux_positions.get(j, math.nan) for j in AUX_JOINTS],
      ]
    )
    if self.episode_step % 50 == 0:
      self.log.flush()


def main():
  root = Path(__file__).resolve().parent
  default_config = root.parents[1] / "config/hardware.yaml"
  if not default_config.is_file():
    default_config = (
      Path(get_package_share_directory("shift_ostrich")) / "config/hardware.yaml"
    )
  parser = argparse.ArgumentParser(description=__doc__)
  policy_source = parser.add_mutually_exclusive_group(required=True)
  policy_source.add_argument("--policy", type=Path, help="Policy .pt or .npz file")
  policy_source.add_argument(
    "--checkpoint-file",
    dest="policy",
    type=Path,
    help="Policy .pt checkpoint, same path syntax as play",
  )
  parser.add_argument("--mjlab-root", type=Path, default=Path.home() / "mjlab")
  parser.add_argument(
    "--check-only",
    action="store_true",
    help="Convert and validate the model, then exit without ROS initialization",
  )
  parser.add_argument(
    "--hardware-config",
    type=Path,
    default=default_config,
  )
  parser.add_argument(
    "--arm",
    action="store_true",
    help="Publish motor targets; omitted means read-only preview",
  )
  parser.add_argument(
    "--duration",
    type=float,
    default=8.0,
    help="Real walking seconds, independent of playback speed; 0 means continuous",
  )
  parser.add_argument(
    "--playback-speed",
    "--speed-scale",
    type=float,
    default=1.0,
    help="Reference/phase playback rate in (0,1]; 0.5 is half speed, control stays 50Hz",
  )
  parser.add_argument("--ready-time", type=float, default=4.0)
  parser.add_argument(
    "--ready-settle-time",
    type=float,
    default=1.0,
    help="Continuous settled READY seconds before policy may start",
  )
  parser.add_argument(
    "--start-after-ready",
    action="store_true",
    help="Move to policy READY, verify settling, then start policy automatically",
  )
  parser.add_argument("--sensor-timeout", type=float, default=0.15)
  parser.add_argument("--max-tilt", type=float, default=30.0)
  parser.add_argument("--log-dir", type=Path, default=None)
  args = parser.parse_args()
  if args.log_dir is None:
    args.log_dir = args.mjlab_root / "logs/shift_ostrich_hardware"
  if not (
    math.isfinite(args.duration)
    and args.duration >= 0
    and math.isfinite(args.playback_speed)
    and 0 < args.playback_speed <= 1
    and math.isfinite(args.ready_time)
    and args.ready_time >= 3
    and math.isfinite(args.ready_settle_time)
    and args.ready_settle_time >= 0.5
    and 0 < args.sensor_timeout <= 0.25
    and 0 < args.max_tilt <= 35
  ):
    parser.error(
      "Invalid playback speed, duration, READY time, sensor timeout or tilt limit"
    )
  with TemporaryDirectory(prefix="shift_j146_policy_") as export_dir:
    try:
      args.policy = prepare_policy(args.policy, args.mjlab_root, Path(export_dir))
    except ValueError as exc:
      parser.error(str(exc))
    if args.check_only:
      print("Policy validation passed; ROS and hardware were not started")
      return
    rclpy.init()
    node = None
    try:
      node = PolicyNode(args)
      rclpy.spin(node)
    except KeyboardInterrupt:
      pass
    finally:
      if node is not None:
        node.log.close()
        node.destroy_node()
      if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
  main()
