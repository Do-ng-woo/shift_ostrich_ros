"""Execute saved transformation waypoints through the real trajectory topic."""

import json
import math
import time
from copy import deepcopy
from itertools import combinations
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_prefix
from rcl_interfaces.srv import GetParameters
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .hardware_limits import tick_to_joint_position
from .mesh_interference import MeshInterference, first_sequence_contact
from .ready_posture import ready_targets

TOPIC = "/joint_trajectory_controller/joint_trajectory"


def hardware_waypoints(share, motion, saved):
  data = deepcopy(saved)
  ready_names, ready_values = ready_targets(share)
  ready = dict(zip(ready_names, ready_values, strict=True))
  shoulder = tick_to_joint_position(share, "Arm_shoulder", 2200)
  if motion == "transform":
    data["stages"][0]["positions"] = [ready[n] for n in data["joint_names"]]
  else:
    data["stages"][-1]["positions"] = [ready[n] for n in data["joint_names"]]
  indices = [
    i for i, n in enumerate(data["joint_names"]) if n not in ("L_wheel", "R_wheel")
  ]
  names = [data["joint_names"][i] for i in indices]
  fold_index = next(
    (i for i, s in enumerate(data["stages"]) if s["name"] == "folded_supported"), None
  )
  for stage_index, stage in enumerate(data["stages"]):
    stage["positions"] = [stage["positions"][i] for i in indices]
    stage["positions"][names.index("Arm_shoulder")] = shoulder
    # Shoulder 2200 changes arm clearance: -50 deg wrist now intersects
    # the upper arm. Keep the wrist within the checked -35 deg envelope.
    wrist = names.index("Arm_waist")
    stage["positions"][wrist] = max(stage["positions"][wrist], math.radians(-35))
    if motion == "transform" and fold_index is not None:
      # Release the foot/shank fold BEFORE moving the hip at folded_supported.
      # Reducing knee flexion by the same amount preserves their pitch sum.
      floor_deg = (
        -30 if fold_index is not None and stage_index >= fold_index - 1 else -60
      )
      for side in ("L", "R"):
        ankle = names.index(side + "_ankle_pitch")
        knee = names.index(side + "_knee_pitch")
        old = stage["positions"][ankle]
        released = max(old, math.radians(floor_deg))
        stage["positions"][ankle] = released
        stage["positions"][knee] = max(0.0, stage["positions"][knee] - (released - old))
  data["joint_names"] = names
  config = yaml.safe_load((Path(share) / "config/hardware.yaml").read_text())
  factor = config["conversion"]["tick_per_rad"]
  for motor in config["joints"]:
    if motor["joint_name"] not in names:
      continue
    idx = names.index(motor["joint_name"])
    center = 2048 if motor.get("center_tick") is None else motor["center_tick"]
    direction = 1 if motor.get("direction") is None else motor["direction"]
    for stage in data["stages"]:
      tick = round(center + direction * stage["positions"][idx] * factor)
      if (
        not motor.get("min_position_tick", 0)
        <= tick
        <= motor.get("max_position_tick", 4095)
      ):
        raise ValueError(
          f"{stage['name']}: {motor['joint_name']} target {tick} outside configured bounds"
        )
  return data


class FoldReleaseGate:
  """Hold the preceding waypoint until both ankles and knees are unfolded."""

  def __init__(self, data):
    self.paused_sec = 0.0
    self.started = None
    self.released = False
    names = data["joint_names"]
    fold = next(
      (i for i, s in enumerate(data["stages"]) if s["name"] == "folded_supported"), None
    )
    self.time = None if fold is None else data["stages"][fold - 1]["time_sec"]
    self.targets = (
      {}
      if fold is None
      else {
        name: data["stages"][fold - 1]["positions"][names.index(name)]
        for name in names
        if name.endswith(("_ankle_pitch", "_knee_pitch"))
      }
    )

  def phase_time(self, wall_elapsed, measured):
    phase = wall_elapsed - self.paused_sec
    if self.time is None or self.released or phase <= self.time:
      return phase
    self.paused_sec += phase - self.time
    if all(
      abs(measured[n] - value) <= math.radians(5) for n, value in self.targets.items()
    ):
      self.released = True
    elif self.started is None:
      self.started = wall_elapsed
    elif wall_elapsed - self.started > 2.0:
      raise RuntimeError(
        "Ankle/knee release not reached before stage 6; holding previous pose"
      )
    return self.time


def bridge_parameters(node, timeout=2.0):
  client = node.create_client(GetParameters, "/dxl_joint_state_bridge/get_parameters")
  try:
    if not client.wait_for_service(timeout_sec=timeout):
      return None
    req = GetParameters.Request()
    req.names = [
      "enable_trajectory_commands",
      "enable_joint_state_commands",
      "torque_on_start",
    ]
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done() or future.result() is None:
      raise RuntimeError("Hardware bridge parameter check timed out")
    values = future.result().values
    if len(values) != 3 or not all(v.type == 1 for v in values):
      raise RuntimeError("Invalid bridge interface parameters")
    if not values[0].bool_value or values[1].bool_value or not values[2].bool_value:
      raise RuntimeError(
        "Hardware motion requires trajectory=true, joint_state_commands=false, torque_on_start=true"
      )
    return values
  finally:
    node.destroy_client(client)


def run_hardware_motion(node, share, motion, saved):
  data = hardware_waypoints(share, motion, saved)
  names = data["joint_names"]
  feedback = {"positions": None, "time": 0.0}
  fault = {}

  def on_fault(msg):
    try:
      value = json.loads(msg.data)
    except (ValueError, TypeError):
      return
    if isinstance(value, dict) and value.get("fault"):
      fault.update(value)

  def on_state(msg):
    if len(msg.name) != len(msg.position):
      return
    age = (
      node.get_clock().now().nanoseconds
      - (msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec)
    ) / 1e9
    if not -0.02 <= age <= 0.3:
      return
    values = dict(zip(msg.name, msg.position, strict=True))
    if all(n in values and math.isfinite(values[n]) for n in names):
      feedback.update(positions={n: values[n] for n in names}, time=time.monotonic())

  sub = node.create_subscription(JointState, "/hardware/joint_states", on_state, 1)
  fault_sub = node.create_subscription(
    String,
    "/hardware/feedback_fault",
    on_fault,
    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
  )
  pub = node.create_publisher(JointTrajectory, TOPIC, 1)

  def publish(positions):
    msg = JointTrajectory()
    msg.joint_names = names
    point = JointTrajectoryPoint()
    point.positions = [float(positions[n]) for n in names]
    msg.points = [point]
    pub.publish(msg)

  try:
    deadline = time.monotonic() + 5.0
    while rclpy.ok() and time.monotonic() < deadline:
      rclpy.spin_once(node, timeout_sec=0.02)
      if fault:
        raise RuntimeError("Hardware bridge fault: " + json.dumps(fault))
      if feedback["positions"] is not None and pub.get_subscription_count() == 1:
        break
    else:
      raise RuntimeError("No complete hardware feedback or trajectory subscriber")
    other = [
      p
      for p in node.get_publishers_info_by_topic(TOPIC)
      if p.node_name != node.get_name() or p.node_namespace != node.get_namespace()
    ]
    if other:
      raise RuntimeError(
        "Stop policy, teleop and READY publishers before transformation"
      )
    initial = feedback["positions"].copy()
    library = (
      Path(get_package_prefix("shift_ostrich")) / "lib/libshift_mesh_interference.so"
    )
    checker = MeshInterference(share, library)
    pairs = [
      p for p in combinations(checker.meshes, 2) if frozenset(p) not in checker.adjacent
    ]

    class Nonadjacent:
      def contacts(self, positions):
        return checker.contacts(positions, pairs=pairs)

    try:
      path = [("measured_start", [initial[n] for n in names])]
      path += [(s["name"], s["positions"]) for s in data["stages"]]
      contact = first_sequence_contact(Nonadjacent(), names, path, max_step_deg=0.5)
      if contact:
        raise RuntimeError(f"Hardware path mesh interference: {contact}")
    finally:
      checker.close()
    # Refresh feedback after the potentially lengthy mesh check.
    deadline = time.monotonic() + 1.0
    while feedback["time"] < time.monotonic() - 0.1 and time.monotonic() < deadline:
      rclpy.spin_once(node, timeout_sec=0.02)
    if fault:
      raise RuntimeError("Hardware bridge fault: " + json.dumps(fault))
    if time.monotonic() - feedback["time"] > 0.3:
      raise RuntimeError("Stale hardware feedback after path check")
    if max(abs(initial[n] - feedback["positions"][n]) for n in names) > math.radians(3):
      raise RuntimeError("Robot moved during path check; run command again")
    node.get_logger().warn(
      f"HARDWARE {motion}: shoulder fixed 2200 tick; {data['stages'][-1]['time_sec']:.1f}s; wheels uncommanded"
    )
    gate = FoldReleaseGate(data)
    started = time.monotonic()
    last = initial.copy()
    seen = set()
    while rclpy.ok():
      now = time.monotonic()
      wall_elapsed = now - started
      rclpy.spin_once(node, timeout_sec=0.005)
      if fault:
        raise RuntimeError(
          "Hardware bridge fault; motion stopped: " + json.dumps(fault)
        )
      if now - feedback["time"] > 0.3:
        publish(last)
        raise RuntimeError("Hardware feedback timeout; holding last commanded pose")
      if node.count_publishers(TOPIC) != 1:
        raise RuntimeError(
          "Another motion publisher appeared; stopping hardware motion"
        )
      was_waiting, was_released = gate.started is not None, gate.released
      elapsed = gate.phase_time(wall_elapsed, feedback["positions"])
      if not was_waiting and gate.started is not None:
        node.get_logger().warn(
          "Holding before stage 6: waiting for ankle -30 / knee 70 deg"
        )
      if not was_released and gate.released:
        node.get_logger().info("Ankle/knee release verified; entering stage 6")
      previous_time = 0.0
      previous = initial
      target = None
      for idx, stage in enumerate(data["stages"]):
        end = stage["time_sec"]
        values = dict(zip(names, stage["positions"], strict=True))
        if elapsed <= end:
          alpha = max(0.0, min(1.0, (elapsed - previous_time) / (end - previous_time)))
          target = {n: previous[n] + alpha * (values[n] - previous[n]) for n in names}
          if idx not in seen:
            seen.add(idx)
            node.get_logger().info(f"Stage {idx + 1}: {stage['name']}")
          break
        previous_time, previous = end, values
      if target is None:
        target = previous
      # Rate limit streaming targets without a second bridge interpolation.
      target = {n: last[n] + max(-0.12, min(0.12, target[n] - last[n])) for n in names}
      publish(target)
      last = target
      if elapsed >= data["stages"][-1]["time_sec"]:
        error = max(abs(feedback["positions"][n] - target[n]) for n in names)
        if error < math.radians(8):
          node.get_logger().info(
            f"HARDWARE {motion} complete; measured maximum error {math.degrees(error):.1f} deg"
          )
          return {
            "motion": motion,
            "backend": "hardware",
            "wall_duration_sec": wall_elapsed,
            "release_wait_sec": gate.paused_sec,
            "maximum_error_deg": math.degrees(error),
          }
        if elapsed > data["stages"][-1]["time_sec"] + 5:
          raise RuntimeError(
            f"Final hardware tracking error {math.degrees(error):.1f} deg exceeds 8 deg"
          )
      time.sleep(0.015)
  except KeyboardInterrupt:
    raise
  finally:
    if rclpy.ok():
      pub.wait_for_all_acked(Duration(seconds=0.3))
    node.destroy_publisher(pub)
    node.destroy_subscription(sub)
    node.destroy_subscription(fault_sub)
