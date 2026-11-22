#!/usr/bin/env python3
"""Shift Ostrich real-motor bridge, measured state display and direction tests."""
import csv
import datetime as dt
import math
import fcntl
import json
from pathlib import Path
import os
import struct
import time
from typing import Dict, List, Optional, Tuple

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray, String
from rclpy.qos import QoSProfile, DurabilityPolicy
from trajectory_msgs.msg import JointTrajectory
from std_srvs.srv import Trigger

try:
    from dynamixel_sdk import COMM_SUCCESS, GroupSyncRead, GroupSyncWrite, PacketHandler, PortHandler
except ImportError:
    COMM_SUCCESS = None
    GroupSyncRead = None
    GroupSyncWrite = None
    PacketHandler = None
    PortHandler = None


ADDR_OPERATING_MODE = 11
ADDR_PWM_LIMIT = 36
ADDR_TORQUE_ENABLE = 64
ADDR_HARDWARE_ERROR_STATUS = 70
ADDR_POSITION_D_GAIN = 80
ADDR_POSITION_I_GAIN = 82
ADDR_POSITION_P_GAIN = 84
ADDR_GOAL_VELOCITY = 104
ADDR_GOAL_CURRENT = 102
ADDR_PROFILE_ACCELERATION = 108
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_PWM = 124
ADDR_PRESENT_LOAD = 126
ADDR_PRESENT_VELOCITY = 128
ADDR_PRESENT_POSITION = 132
ADDR_PRESENT_INPUT_VOLTAGE = 144
ADDR_PRESENT_TEMPERATURE = 146
LEN_GOAL_POSITION = 4
LEN_GOAL_VELOCITY = 4
LEN_TELEMETRY_BLOCK = 23
ADDR_OPENCR_IMU_BLOCK = 100
LEN_OPENCR_IMU_BLOCK = 68
STANDARD_GRAVITY = 9.80665
TORQUE_DISABLE = 0
TORQUE_ENABLE = 1
HARDWARE_ERROR_BITS = {
    0: "input_voltage",
    2: "overheating",
    3: "motor_encoder",
    4: "electrical_shock",
    5: "overload",
}


Trajectory = List[Tuple[float, Dict[str, float]]]


def interpolate_trajectory(
    start_positions: Dict[str, float],
    trajectory: Trajectory,
    elapsed: float,
) -> Tuple[Dict[str, float], bool]:
    """Linearly sample a trajectory, using the current pose as the t=0 point."""
    if not trajectory:
        return dict(start_positions), True

    if elapsed >= trajectory[-1][0]:
        return dict(trajectory[-1][1]), True

    previous_time = 0.0
    previous_positions = start_positions
    for next_time, next_positions in trajectory:
        if elapsed <= next_time:
            duration = next_time - previous_time
            alpha = 1.0 if duration <= 0.0 else (elapsed - previous_time) / duration
            alpha = max(0.0, min(1.0, alpha))
            sampled = {
                name: previous_positions[name]
                + alpha * (next_positions[name] - previous_positions[name])
                for name in next_positions
            }
            return sampled, False
        previous_time = next_time
        previous_positions = next_positions

    return dict(trajectory[-1][1]), True


class MotorConfig:
    def __init__(self, data: dict, tick_per_rad: float):
        self.id = int(data["id"])
        self.joint_name = data["joint_name"]
        self.control_mode = data["control_mode"]
        self.operating_mode = int(data["operating_mode"])
        if (self.control_mode not in ('position', 'velocity') or
                self.control_mode == 'position' and self.operating_mode not in (3, 5) or
                self.control_mode == 'velocity' and self.operating_mode != 1):
            raise ValueError(f'ID {self.id}: expected position mode 3/5 or wheel velocity mode 1')
        self.direction_checked = bool(data.get("direction_checked", False))
        self.zero_checked = bool(data.get("zero_checked", False))
        direction, center = data.get("direction"), data.get("center_tick")
        if direction is not None and direction not in (-1, 1):
            raise ValueError(f"ID {self.id}: direction must be +1 or -1")
        self.center_tick = 2048 if center is None else int(center)
        self.direction = 1 if direction is None else int(direction)
        self.configured_min_tick = int(data.get('min_position_tick', 0))
        self.configured_max_tick = int(data.get('max_position_tick', 4095))
        if not 0 <= self.configured_min_tick <= self.configured_max_tick <= 4095:
            raise ValueError(f'ID {self.id}: invalid configured position limits')
        self.min_tick = self.configured_min_tick
        self.max_tick = self.configured_max_tick
        self.active = bool(data.get("active", True))
        self.tick_per_rad = tick_per_rad
        self.pwm_limit = data.get("pwm_limit")
        self.position_p_gain = data.get("position_p_gain")
        self.position_i_gain = data.get("position_i_gain")
        self.position_d_gain = data.get("position_d_gain")
        self.profile_acceleration = data.get("profile_acceleration")
        self.profile_velocity = data.get("profile_velocity")
        self.goal_current = data.get('goal_current')
        if self.operating_mode == 5 and (self.goal_current is None or
                not math.isfinite(self.goal_current) or self.goal_current <= 0):
            raise ValueError(f'ID {self.id}: current-based position mode needs a positive goal_current cap')
        self.model = str(data.get("model", "unknown"))
        self.effort_feedback = str(data.get("effort_feedback", "load"))
        self.stall_torque_nm = float(data.get("stall_torque_nm", 1.4))
        self.current_unit_a = float(data.get("current_unit_a", 0.00269))
        self.stall_current_a = float(data.get("stall_current_a", 1.0))

    def effort_telemetry(self, raw: int):
        if self.effort_feedback == "current":
            current_a = self.direction * raw * self.current_unit_a
            torque_nm = current_a / self.stall_current_a * self.stall_torque_nm
            return "", current_a, torque_nm
        load_percent = self.direction * raw * 0.1
        torque_nm = load_percent * 0.01 * self.stall_torque_nm
        return load_percent, "", torque_nm

    def position_to_tick(self, position_rad: float) -> int:
        tick = self.center_tick + self.direction * position_rad * self.tick_per_rad
        return max(0, min(4095, int(round(tick))))

    def tick_to_position(self, tick: int) -> float:
        return (tick - self.center_tick) / (self.direction * self.tick_per_rad)

    def velocity_to_raw(self, velocity_rad_s: float, raw_per_rad_s: float) -> int:
        raw = round(self.direction * velocity_rad_s * raw_per_rad_s)
        return max(-(2**31), min(2**31 - 1, raw))


class DxlJointStateBridge(Node):
    def __init__(self):
        super().__init__("dxl_joint_state_bridge")

        self.declare_parameter("config_path", "")
        self.declare_parameter("usb_settle_sec", 1.0)
        self.declare_parameter("startup_ping_attempts", 5)
        self.declare_parameter("startup_retry_delay_sec", 0.3)
        self.declare_parameter("torque_on_start", True)
        self.declare_parameter("motor_ids", "")
        self.declare_parameter("torque_motor_ids", "")
        self.declare_parameter("joint_command_topic", "/hardware/joint_commands")
        self.declare_parameter("torque_off_on_shutdown", True)
        self.declare_parameter('preserve_existing_torque_on_failure', False)
        self.declare_parameter("center_on_start", False)
        self.declare_parameter("startup_ready_posture_on_start", False)
        self.declare_parameter("min_tick_change", 2)
        self.declare_parameter("log_joint_states", True)
        self.declare_parameter("enable_joint_state_commands", True)
        self.declare_parameter("enable_trajectory_commands", False)
        self.declare_parameter("enable_velocity_commands", False)
        self.declare_parameter("passive_wheels", False)
        self.declare_parameter(
            "trajectory_topic", "/joint_trajectory_controller/joint_trajectory"
        )
        self.declare_parameter(
            "velocity_topic", "/wheel_velocity_controller/commands"
        )
        self.declare_parameter("max_wheel_velocity_rad_s", 2.0)
        self.declare_parameter("wheel_command_timeout_sec", 0.5)
        self.declare_parameter("log_telemetry", False)
        self.declare_parameter("telemetry_log_path", "")
        self.declare_parameter("telemetry_rate_hz", 5.0)
        self.declare_parameter("telemetry_duration_sec", 10.0)
        self.declare_parameter("telemetry_motor_ids", "")
        self.declare_parameter("publish_present_joint_states", True)
        self.declare_parameter("present_joint_state_topic", "/joint_states")
        self.declare_parameter("present_joint_state_rate_hz", 15.0)
        self.declare_parameter("sync_policy_feedback", False)
        self.declare_parameter("enable_opencr_imu", False)
        self.declare_parameter("opencr_imu_id", 200)
        self.declare_parameter("opencr_imu_topic", "/opencr/imu")
        self.declare_parameter("opencr_imu_frame_id", "base_link")
        self.declare_parameter("opencr_imu_rate_hz", 30.0)

        if PacketHandler is None or PortHandler is None:
            raise RuntimeError(
                "dynamixel_sdk is not installed. Install the ROS/Python Dynamixel SDK first."
            )

        config_path = self.get_parameter("config_path").get_parameter_value().string_value
        if not config_path:
            pkg_share = get_package_share_directory("shift_ostrich")
            config_path = os.path.join(pkg_share, "config", "hardware.yaml")

        self.config = self._load_config(config_path)
        self.port_handler = PortHandler(self.config["bus"]["device"])
        self.packet_handler = PacketHandler(float(self.config["bus"]["protocol_version"]))
        self.group_sync_write = GroupSyncWrite(
            self.port_handler,
            self.packet_handler,
            ADDR_GOAL_POSITION,
            LEN_GOAL_POSITION,
        )
        self.group_sync_write_velocity = GroupSyncWrite(
            self.port_handler,
            self.packet_handler,
            ADDR_GOAL_VELOCITY,
            LEN_GOAL_VELOCITY,
        )
        self.group_sync_read = None
        self.enabled_by_bridge = set()
        self.preexisting_torque_ids = set()
        self.startup_complete = False
        self.last_goal_ticks: Dict[int, int] = {}
        self.last_goal_velocities: Dict[int, int] = {}
        self.last_velocity_command_time = None
        self.velocity_watchdog_stopped = True
        self.received_joint_states = False
        self.active_trajectory: Trajectory = []
        self.trajectory_start_positions: Dict[str, float] = {}
        self.trajectory_start_time = None
        self.telemetry_file = None
        self.telemetry_writer = None
        self.telemetry_start_time = None
        self.telemetry_duration_sec = 0.0
        self.telemetry_motors = []
        self.opencr_imu_warned = False

        self.min_tick_change = (
            self.get_parameter("min_tick_change").get_parameter_value().integer_value
        )
        self.max_wheel_velocity_rad_s = float(
            self.get_parameter("max_wheel_velocity_rad_s").value
        )
        self.wheel_command_timeout_sec = float(
            self.get_parameter("wheel_command_timeout_sec").value
        )
        self.velocity_raw_per_rad_s = float(
            self.config["conversion"]["velocity_raw_per_rad_per_sec"]
        )

        self.all_motors = self._load_active_motors()
        self.position_motors = [
            motor for motor in self.all_motors if motor.control_mode == "position"
        ]
        self.velocity_motors = [
            motor for motor in self.all_motors if motor.control_mode == "velocity"
        ]
        self.joint_to_motor = {motor.joint_name: motor for motor in self.position_motors}

        if (self.get_parameter("publish_present_joint_states").value and
                self.get_parameter("joint_command_topic").value == self.get_parameter("present_joint_state_topic").value):
            raise ValueError("Command and measured-state topics must differ")
        if not self.get_parameter("torque_on_start").value and any(self.get_parameter(p).value for p in
                ("enable_joint_state_commands", "enable_trajectory_commands", "enable_velocity_commands",
                 "center_on_start", "startup_ready_posture_on_start")):
            raise ValueError("Command interfaces require torque_on_start:=true")

        self._open_bus()
        try:
            self._configure_motors(torque_on_start=self.get_parameter("torque_on_start").value)
        except Exception:
            self._release_bus(force_off=True)
            raise
        if self.get_parameter("center_on_start").value:
            self._center_position_motors()
        if self.get_parameter("startup_ready_posture_on_start").value:
            self._send_startup_ready_posture()

        publish_present_joint_states = bool(
            self.get_parameter("publish_present_joint_states").value
        )
        present_joint_state_topic = self.get_parameter("present_joint_state_topic").value
        if publish_present_joint_states:
            self._setup_present_joint_state_publisher(present_joint_state_topic)

        joint_state_commands_enabled = bool(
            self.get_parameter("enable_joint_state_commands").value
        )
        command_topic = self.get_parameter("joint_command_topic").value
        if publish_present_joint_states and command_topic == present_joint_state_topic:
            raise ValueError("Command and measured-state topics must differ")

        if joint_state_commands_enabled:
            self.subscription = self.create_subscription(
                JointState,
                command_topic,
                self.joint_state_callback,
                10,
            )

        if self.get_parameter("enable_trajectory_commands").value:
            trajectory_topic = (
                self.get_parameter("trajectory_topic").get_parameter_value().string_value
            )
            self.trajectory_subscription = self.create_subscription(
                JointTrajectory,
                trajectory_topic,
                self.trajectory_callback,
                10,
            )
            self.trajectory_timer = self.create_timer(0.008, self.trajectory_timer_callback)

        if self.get_parameter("enable_velocity_commands").value:
            velocity_topic = self.get_parameter("velocity_topic").value
            self.velocity_subscription = self.create_subscription(
                Float64MultiArray,
                velocity_topic,
                self.velocity_command_callback,
                10,
            )
            self.velocity_watchdog_timer = self.create_timer(
                0.1,
                self.velocity_watchdog_callback,
            )

        if self.get_parameter("log_telemetry").value:
            self._setup_telemetry_logger()

        if self.get_parameter("enable_opencr_imu").value:
            self._setup_opencr_imu()

        names = ", ".join(m.joint_name for m in self.position_motors)
        self.get_logger().info(f"Configured Dynamixel position motors: {names}")
        self.startup_complete = True
        self.create_service(Trigger, '~/gripper_status', self.gripper_status_callback)

    def _release_bus(self, force_off=False):
        self.port_handler.is_using = False
        for motor in self.all_motors:
            if motor.id not in self.enabled_by_bridge:
                continue
            try:
                if motor.control_mode == "velocity":
                    self._write4(motor.id, ADDR_GOAL_VELOCITY, 0)
                if force_off or self.get_parameter("torque_off_on_shutdown").value:
                    if (force_off and self.get_parameter('preserve_existing_torque_on_failure').value
                            and motor.id in self.preexisting_torque_ids
                            and motor.id not in getattr(self, 'faulted_motor_ids', set())):
                        continue
                    self._write1(motor.id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            except Exception as exc:
                self.get_logger().error(f"ID {motor.id}: shutdown command failed: {exc}")
                self.port_handler.is_using = False
        if getattr(self.port_handler, 'ser', None) is not None:
            self.port_handler.closePort()
        self.enabled_by_bridge.clear()
        if hasattr(self, "bus_lock"):
            self.bus_lock.close()

    def destroy_node(self):
        self._release_bus(force_off=not self.startup_complete)
        if self.telemetry_file is not None:
            self.telemetry_file.close()
        super().destroy_node()

    def _load_config(self, config_path: str) -> dict:
        with open(config_path, "r", encoding="utf-8") as stream:
            config = yaml.safe_load(stream)
        self.get_logger().info(f"Loaded Dynamixel config: {config_path}")
        return config

    def _load_active_motors(self) -> List[MotorConfig]:
        passive_wheels = bool(self.get_parameter("passive_wheels").value)
        if passive_wheels and self.get_parameter("enable_velocity_commands").value:
            raise ValueError("passive_wheels and velocity commands cannot both be enabled")
        tick_per_rad = float(self.config["conversion"]["tick_per_rad"])
        defaults = self.config.get("defaults", {})
        motors = []
        requested = self.get_parameter("motor_ids").value.strip()
        selected = {int(i) for i in requested.split(',')} if requested else None
        for joint in self.config["joints"]:
            joint = dict(joint)
            if not joint.get("active", True):
                continue
            if selected is not None and joint['id'] not in selected:
                continue
            # Manual sliders test every joint, including both wheel axes.
            # Explicit velocity-command mode keeps the configured wheel modes.
            if (joint['control_mode'] == 'velocity' and not passive_wheels
                    and not self.get_parameter('enable_velocity_commands').value):
                joint.update(control_mode='position', operating_mode=3)
            for key in (
                "model",
                "effort_feedback",
                "stall_torque_nm",
                "current_unit_a",
                "stall_current_a",
                "pwm_limit",
                "position_p_gain",
                "position_i_gain",
                "position_d_gain",
                "profile_acceleration",
                "profile_velocity",
                "goal_current",
            ):
                if key not in joint:
                    joint[key] = defaults.get(key)
            motors.append(MotorConfig(joint, tick_per_rad))
        if not motors or len({m.id for m in motors}) != len(motors):
            raise ValueError("Empty motor selection or duplicate motor IDs")
        if len({m.joint_name for m in motors}) != len(motors):
            raise ValueError("Duplicate joint names")
        if selected is not None and selected != {m.id for m in motors}:
            raise ValueError("Unknown or inactive motor selected")
        return motors

    def _open_bus(self):
        self.bus_lock = open('/tmp/shift_ostrich_calibration.lock', 'w')
        fcntl.flock(self.bus_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        device = self.config["bus"]["device"]
        baudrate = int(self.config["bus"]["baudrate"])
        if not self.port_handler.openPort():
            raise RuntimeError(f"Failed to open Dynamixel port: {device}")
        if not self.port_handler.setBaudRate(baudrate):
            self.port_handler.closePort()
            raise RuntimeError(f"Failed to set Dynamixel baudrate: {baudrate}")
        self.get_logger().info(f"Opened Dynamixel bus {device} @ {baudrate}")
        time.sleep(max(0., float(self.get_parameter('usb_settle_sec').value)))

    def _ping_motors(self):
        """Retry startup discovery and report all missing IDs before any writes."""
        pending = {motor.id for motor in self.all_motors}
        models = {}
        attempts = max(1, int(self.get_parameter('startup_ping_attempts').value))
        delay = max(0., float(self.get_parameter('startup_retry_delay_sec').value))
        details = {}
        for attempt in range(1, attempts+1):
            for mid in sorted(pending):
                model, result, error = self.packet_handler.ping(self.port_handler, mid)
                if result == COMM_SUCCESS and error == 0:
                    models[mid] = model
                    pending.remove(mid)
                    details.pop(mid, None)
                else:
                    details[mid] = (self.packet_handler.getTxRxResult(result) if result != COMM_SUCCESS
                                    else self.packet_handler.getRxPacketError(error))
            if not pending:
                self.get_logger().info(f"Motor connection OK: IDs {sorted(models)}")
                return models
            self.get_logger().warn(f"Motor ping {attempt}/{attempts}: no valid response from IDs {sorted(pending)}")
            if attempt < attempts:
                time.sleep(delay)
        if models or self.get_parameter('enable_opencr_imu').value:
            self.get_logger().warn(
                f"Continuing with connected motor IDs {sorted(models)}; "
                f"offline IDs {sorted(pending)} will not receive commands. "
                "Check their power/wiring/IDs/baudrate and restart to include them.")
            if not models:
                self.get_logger().info('No motor responses; continuing OpenCR IMU reading without motor commands')
            return models
        bus = self.config['bus']
        raise RuntimeError(
            f"Motor ping failed after {attempts} attempts: IDs {sorted(pending)}. "
            f"Port {bus['device']}, baud {bus['baudrate']}, Protocol 2.0. "
            f"Check motor power, OpenCR TTL bridge firmware/wiring, and bus baudrate; "
            f"close other programs using the port. Details: {details}")

    def _configure_motors(self, torque_on_start: bool):
        selected_text = self.get_parameter("torque_motor_ids").value.strip()
        selected = {int(i) for i in selected_text.split(',')} if selected_text else {m.id for m in self.all_motors}
        if not selected <= {m.id for m in self.all_motors}:
            raise ValueError("Unknown torque_motor_ids")
        if not self.get_parameter("enable_velocity_commands").value:
            selected -= {m.id for m in self.velocity_motors}
        states = {}
        models = self._ping_motors()
        self.all_motors = [motor for motor in self.all_motors if motor.id in models]
        self.position_motors = [motor for motor in self.all_motors if motor.control_mode == 'position']
        self.velocity_motors = [motor for motor in self.all_motors if motor.control_mode == 'velocity']
        self.joint_to_motor = {motor.joint_name: motor for motor in self.position_motors}
        selected.intersection_update(models)
        # Preflight every selected motor before any register writes.
        for motor in self.all_motors:
            model = models[motor.id]
            if model not in (1020, 1030, 1060, 1200):
                raise ValueError(f"ID {motor.id}: unsupported model number {model}")
            position = self._read_register(motor.id, ADDR_PRESENT_POSITION, 4)
            states[motor.id] = dict(position=position, mode=self._read_register(motor.id, 11, 1),
                torque=self._read_register(motor.id, 64, 1), drive=self._read_register(motor.id, 10, 1),
                error=self._read_register(motor.id, 70, 1), voltage=self._read_register(motor.id, 144, 2)/10)
            motor.min_tick = max(motor.configured_min_tick, self._read_register(motor.id, 52, 4))
            motor.max_tick = min(motor.configured_max_tick, self._read_register(motor.id, 48, 4))
            if not motor.direction_checked or (motor.control_mode == "position" and not motor.zero_checked):
                self.get_logger().info(f"ID {motor.id}: manual mapping center={motor.center_tick}, "
                                       f"direction={motor.direction}; confirm against RViz")
            if not torque_on_start or motor.id not in selected:
                continue
            state = states[motor.id]
            if motor.operating_mode == 5:
                if model not in (1020, 1030, 1200):
                    raise RuntimeError(f'ID {motor.id}: mode 5 is unsupported for verified model {model}')
                current_limit = self._read_register(motor.id, 38, 2)
                if current_limit <= 0:
                    raise RuntimeError(f'ID {motor.id}: Current Limit is zero')
                motor.goal_current = min(int(motor.goal_current), current_limit)
            if state['error']:
                raise RuntimeError(f"ID {motor.id}: motor hardware error: {state['error']}")
            if motor.control_mode == 'position':
                current = state['position'] % 4096
                if not motor.min_tick <= current <= motor.max_tick:
                    raise RuntimeError(
                        f'ID {motor.id} {motor.joint_name}: current position {current} tick '
                        f'is outside allowed [{motor.min_tick}, {motor.max_tick}]. '
                        'Startup stopped before any motor writes; support the robot and '
                        'place this joint inside its Wizard limits. No automatic boundary move.')
        if not torque_on_start:
            self.get_logger().info("Read-only startup: no mode, tuning, goal or torque writes")
            return
        self.preexisting_torque_ids = {m.id for m in self.all_motors
                                      if states[m.id]['torque'] and states[m.id]['mode'] == m.operating_mode}
        if self.get_parameter("passive_wheels").value:
            for motor in self.velocity_motors:
                # Release an existing position hold as well as velocity torque.
                # Do not change wheel mode or move its shaft on the CH path.
                self._write1(motor.id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
                if self._read_register(motor.id, ADDR_TORQUE_ENABLE, 1) != 0:
                    raise RuntimeError(f"ID {motor.id}: passive wheel torque-off failed")
            self.get_logger().info("CH passive wheels: torque OFF; no wheel position targets")
        for motor in self.all_motors:
            if motor.id not in selected:
                continue
            if states[motor.id]['mode'] != motor.operating_mode:
                if states[motor.id]['torque']:
                    self._write1(motor.id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
                self._write1(motor.id, ADDR_OPERATING_MODE, motor.operating_mode)
                if self._read_register(motor.id, ADDR_OPERATING_MODE, 1) != motor.operating_mode:
                    raise RuntimeError(f"ID {motor.id}: operating mode readback failed")
            current = self._read_register(motor.id, ADDR_PRESENT_POSITION, 4) % 4096
            if motor.control_mode == 'position' and not motor.min_tick <= current <= motor.max_tick:
                raise RuntimeError(f'ID {motor.id}: current position changed outside allowed '
                                   f'[{motor.min_tick}, {motor.max_tick}] before torque enable')
            self._apply_motor_tuning(motor)
            if motor.control_mode == "velocity":
                self._write4(motor.id, ADDR_GOAL_VELOCITY, 0)
            else:
                self._write4(motor.id, ADDR_GOAL_POSITION, current)
                self.last_goal_ticks[motor.id] = current
            self.enabled_by_bridge.add(motor.id)
            self._write1(motor.id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
            if self._read_register(motor.id, ADDR_TORQUE_ENABLE, 1) != 1:
                raise RuntimeError(f"ID {motor.id}: torque enable readback failed")
        self.get_logger().info(f"Torque enabled: {sorted(self.enabled_by_bridge)}")

    def _read_register(self, mid, address, size):
        value, result, error = getattr(self.packet_handler, f'read{size}ByteTxRx')(
            self.port_handler, mid, address)
        self._check_result(mid, address, result, error, operation="read")
        return value

    def _checked_goal_tick(self, motor, position):
        if motor.id not in self.enabled_by_bridge or not math.isfinite(position):
            return None
        requested = round(motor.center_tick + motor.direction * position * motor.tick_per_rad)
        if not motor.min_tick <= requested <= motor.max_tick:
            self.get_logger().warn(
                f'ID {motor.id} {motor.joint_name}: rejected {requested} tick; '
                f'allowed [{motor.min_tick}, {motor.max_tick}]', throttle_duration_sec=2.0)
            return None
        return requested

    def _apply_motor_tuning(self, motor: MotorConfig):
        writes_2byte = (
            ('goal_current', ADDR_GOAL_CURRENT, motor.goal_current if motor.operating_mode == 5 else None),
            ("pwm_limit", ADDR_PWM_LIMIT, motor.pwm_limit),
            ("position_d_gain", ADDR_POSITION_D_GAIN, motor.position_d_gain),
            ("position_i_gain", ADDR_POSITION_I_GAIN, motor.position_i_gain),
            ("position_p_gain", ADDR_POSITION_P_GAIN, motor.position_p_gain),
        )
        writes_4byte = (
            ("profile_acceleration", ADDR_PROFILE_ACCELERATION, motor.profile_acceleration),
            ("profile_velocity", ADDR_PROFILE_VELOCITY, motor.profile_velocity),
        )

        applied = []
        for name, address, value in writes_2byte:
            if value is None:
                continue
            self._write2(motor.id, address, int(value))
            applied.append(f"{name}={int(value)}")

        for name, address, value in writes_4byte:
            if value is None:
                continue
            self._write4(motor.id, address, int(value))
            applied.append(f"{name}={int(value)}")

        if applied:
            self.get_logger().info(
                f"DXL {motor.id} {motor.joint_name} tuning: {', '.join(applied)}"
            )

    def gripper_status_callback(self, request, response):
        motor = self.joint_to_motor.get('gripper_pitch')
        if motor is None:
            response.success = False
            response.message = 'ID 17 gripper is not connected'
            return response
        try:
            values = {name: self._read_register(motor.id, address, size)
                      for name, address, size in (
                          ('mode',11,1), ('torque',64,1), ('hardware_error',70,1),
                          ('p_gain',84,2), ('i_gain',82,2), ('d_gain',80,2),
                          ('current_limit',38,2), ('goal_current',102,2),
                          ('goal_tick',116,4), ('present_tick',132,4),
                          ('present_current_raw',126,2), ('present_velocity_raw',128,4),
                          ('voltage_raw',144,2), ('temperature',146,1),
                          ('drive_mode',10,1), ('watchdog',98,1))}
            response.success = True
            response.message = json.dumps(values)
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        return response

    def _center_position_motors(self):
        self._send_position_map({motor.joint_name: 0. for motor in self.position_motors},
                                source="explicit_center")

    def _send_startup_ready_posture(self):
        import sys
        share = Path(get_package_share_directory('shift_ostrich'))
        sys.path.insert(0, str(share))
        from core.ready_posture import ready_targets
        names, positions = ready_targets(share)
        self._send_position_map(dict(zip(names, positions)),
                                source="startup_ready")

    def joint_state_callback(self, msg: JointState):
        if not self.received_joint_states:
            self.received_joint_states = True
            matched = [name for name in msg.name if name in self.joint_to_motor]
            self.get_logger().info(
                f"Received joint commands with {len(msg.name)} joints; "
                f"{len(matched)} mapped to Dynamixel motors"
            )

        for joint_name, position in zip(msg.name, msg.position):
            motor = self.joint_to_motor.get(joint_name)
            if motor is None:
                continue

            if not math.isfinite(position):
                continue
            goal_tick = self._checked_goal_tick(motor, position)
            if goal_tick is None:
                continue
            prev_tick = self.last_goal_ticks.get(motor.id)
            if prev_tick is not None and abs(goal_tick - prev_tick) < self.min_tick_change:
                continue

            self._write_goal_tick(motor, goal_tick)
            if self.get_parameter("log_joint_states").value:
                self.get_logger().info(
                    f"{joint_name} -> ID {motor.id}: {position:.4f} rad, tick {goal_tick}",
                    throttle_duration_sec=0.5,
                )

    def trajectory_callback(self, msg: JointTrajectory):
        trajectory: Trajectory = []
        mapped_names = [name for name in msg.joint_names if name in self.joint_to_motor]
        commanded_names = []
        previous_time = -1.0

        for point in msg.points:
            positions = {}
            for joint_name, position in zip(msg.joint_names, point.positions):
                if joint_name in self.joint_to_motor:
                    positions[joint_name] = position

            if not positions:
                continue

            time_from_start = point.time_from_start.sec + (
                point.time_from_start.nanosec * 1e-9
            )
            if time_from_start < previous_time:
                self.get_logger().error(
                    "Rejected trajectory: time_from_start must be nondecreasing"
                )
                return
            trajectory.append((time_from_start, positions))
            previous_time = time_from_start
            for joint_name in positions:
                if joint_name not in commanded_names:
                    commanded_names.append(joint_name)

        if not trajectory:
            self.get_logger().warn("Ignored trajectory with no executable points")
            return

        if len(trajectory) == 1 and trajectory[0][0] == 0.0:
            # Teleop sends a rate-limited streaming setpoint. Reanchoring this
            # at measured position each update continually pulls a lagging
            # motor's target back and restarts another interpolation.
            self.active_trajectory = []
            self.trajectory_start_positions = {}
            self.trajectory_start_time = None
            self._send_position_map(trajectory[0][1], source='streaming_setpoint')
            return

        start_positions = self._read_current_positions(commanded_names)
        carried_positions = dict(start_positions)
        normalized_trajectory: Trajectory = []
        for time_from_start, positions in trajectory:
            carried_positions.update(positions)
            normalized_trajectory.append((time_from_start, dict(carried_positions)))

        self.active_trajectory = normalized_trajectory
        self.trajectory_start_positions = start_positions
        self.trajectory_start_time = self.get_clock().now()
        self.get_logger().warn(
            f"Received trajectory: {len(msg.points)} points, "
            f"{len(mapped_names)} mapped joints, {len(trajectory)} executable points; "
            "linear interpolation enabled"
        )

    def trajectory_timer_callback(self):
        if not self.active_trajectory or self.trajectory_start_time is None:
            return

        elapsed = (
            self.get_clock().now() - self.trajectory_start_time
        ).nanoseconds * 1e-9

        positions, finished = interpolate_trajectory(
            self.trajectory_start_positions,
            self.active_trajectory,
            elapsed,
        )
        self._send_position_map(positions, source="trajectory")

        if finished:
            self.active_trajectory = []
            self.trajectory_start_positions = {}
            self.trajectory_start_time = None
            self.get_logger().info("Finished trajectory playback")

    def velocity_command_callback(self, msg: Float64MultiArray):
        if len(msg.data) < len(self.velocity_motors):
            self.get_logger().error(
                f"Wheel command has {len(msg.data)} values, but "
                f"{len(self.velocity_motors)} velocity motors are configured"
            )
            return

        goal_velocities = {}
        clamped = False
        for motor, requested_velocity in zip(self.velocity_motors, msg.data):
            if motor.id not in self.enabled_by_bridge:
                continue
            velocity = float(requested_velocity)
            if not math.isfinite(velocity):
                velocity = 0.0
            limited_velocity = max(
                -self.max_wheel_velocity_rad_s,
                min(self.max_wheel_velocity_rad_s, velocity),
            )
            clamped = clamped or limited_velocity != velocity
            goal_velocities[motor] = motor.velocity_to_raw(
                limited_velocity,
                self.velocity_raw_per_rad_s,
            )

        self._sync_write_goal_velocities(goal_velocities)
        self.last_velocity_command_time = self.get_clock().now()
        self.velocity_watchdog_stopped = all(
            raw_velocity == 0 for raw_velocity in goal_velocities.values()
        )
        if clamped:
            self.get_logger().warn(
                f"Clamped wheel command to ±{self.max_wheel_velocity_rad_s:.2f} rad/s",
                throttle_duration_sec=1.0,
            )

    def velocity_watchdog_callback(self):
        if self.last_velocity_command_time is None or self.velocity_watchdog_stopped:
            return
        elapsed = (
            self.get_clock().now() - self.last_velocity_command_time
        ).nanoseconds * 1e-9
        if elapsed <= self.wheel_command_timeout_sec:
            return

        self._sync_write_goal_velocities(
            {motor: 0 for motor in self.velocity_motors}
        )
        self.velocity_watchdog_stopped = True
        self.get_logger().warn(
            f"Wheel command timeout after {elapsed:.2f}s; stopped all wheels"
        )

    def _read_current_positions(self, joint_names: List[str]) -> Dict[str, float]:
        positions = {}
        for joint_name in joint_names:
            motor = self.joint_to_motor[joint_name]
            tick = self._read_present_position_tick(motor)
            if tick is None:
                raise RuntimeError(f"Cannot read actual trajectory start: {joint_name}")
            positions[joint_name] = motor.tick_to_position(tick)
        return positions

    def _read_present_position_tick(self, motor: MotorConfig) -> Optional[int]:
        tick, result, error = self.packet_handler.read4ByteTxRx(
            self.port_handler,
            motor.id,
            ADDR_PRESENT_POSITION,
        )
        if result != COMM_SUCCESS:
            self.get_logger().error(
                f"DXL {motor.id} position read failed: "
                f"{self.packet_handler.getTxRxResult(result)}"
            )
            return None
        if error != 0:
            self._log_motor_fault(motor.id)
            self.get_logger().error(
                f"DXL {motor.id} position packet error: "
                f"{self.packet_handler.getRxPacketError(error)}"
            )
            return None
        return int(tick)

    def _setup_present_joint_state_publisher(self, topic: str):
        self.present_joint_state_pub = self.create_publisher(JointState, topic, 10)
        rate_hz = max(
            1.0,
            float(self.get_parameter("present_joint_state_rate_hz").value),
        )
        if self.get_parameter("sync_policy_feedback").value:
            self.policy_feedback_fault = False
            self.policy_fault_snapshot = None
            self.policy_fault_pub = self.create_publisher(String, '/hardware/feedback_fault',
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.policy_fault_pub.publish(String(data=json.dumps({'fault': False})))
            self.policy_state_reader = GroupSyncRead(
                self.port_handler, self.packet_handler, ADDR_PRESENT_VELOCITY, 8)
            for motor in self.all_motors:
                if not self.policy_state_reader.addParam(motor.id):
                    raise RuntimeError(f"Cannot add feedback ID {motor.id}")
            self.policy_fault_timer = self.create_timer(1.0, self.policy_fault_callback)
        self.present_joint_state_timer = self.create_timer(
            1.0 / rate_hz,
            self.present_joint_state_callback,
        )
        self.get_logger().warn(
            f"Publishing present Dynamixel joint states to {topic} at {rate_hz:.1f} Hz"
        )

    def policy_fault_callback(self):
        # Latch a fault by withdrawing complete feedback. The policy then stops
        # issuing targets; the bridge holds its last goal until operator action.
        if self.policy_feedback_fault:
            return
        for motor in self.position_motors:
            error, comm, packet_error = self.packet_handler.read1ByteTxRx(
                self.port_handler, motor.id, ADDR_HARDWARE_ERROR_STATUS)
            torque, torque_comm, torque_error = self.packet_handler.read1ByteTxRx(
                self.port_handler, motor.id, ADDR_TORQUE_ENABLE)
            if (comm != COMM_SUCCESS or packet_error or error or
                    torque_comm != COMM_SUCCESS or torque_error or torque != 1):
                self.policy_feedback_fault = True
                snapshot = {
                    'fault': True, 'motor_id': motor.id,
                    'joint': getattr(motor, 'joint_name', 'unknown'),
                    'hardware_error': int(error) if comm == COMM_SUCCESS else None,
                    'error_flags': [name for bit,name in HARDWARE_ERROR_BITS.items()
                                    if comm == COMM_SUCCESS and error & (1 << bit)],
                    'torque': int(torque) if torque_comm == COMM_SUCCESS else None,
                    'error_read_comm': int(comm), 'error_read_packet': int(packet_error),
                    'torque_read_comm': int(torque_comm), 'torque_read_packet': int(torque_error),
                }
                # Read-only evidence; never reboot, clear errors or re-enable torque.
                for name,address,size in (('voltage_raw',144,2), ('temperature_c',146,1),
                                           ('position_tick',132,4), ('goal_tick',116,4)):
                    try:
                        value,result,alert = getattr(self.packet_handler, f'read{size}ByteTxRx')(
                            self.port_handler,motor.id,address)
                        snapshot[name] = int(value) if result == COMM_SUCCESS else None
                    except Exception:
                        snapshot[name] = None
                self.policy_fault_snapshot = snapshot
                message = json.dumps(snapshot)
                self.get_logger().error('Hardware feedback latched OFF: ' + message)
                pub = getattr(self, 'policy_fault_pub', None)
                if pub is not None:
                    pub.publish(String(data=message))
                return

    def present_joint_state_callback(self):
        if hasattr(self, "policy_state_reader"):
            self.policy_sync_state_callback()
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        for motor in self.all_motors:
            tick = self._read_present_position_tick(motor)
            if tick is None:
                continue
            msg.name.append(motor.joint_name)
            msg.position.append(motor.tick_to_position(tick))
            msg.velocity.append(0.0)
            msg.effort.append(0.0)

        if msg.name:
            self.present_joint_state_pub.publish(msg)

    def policy_sync_state_callback(self):
        if self.policy_feedback_fault:
            return
        reader = self.policy_state_reader
        if reader.txRxPacket() != COMM_SUCCESS:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        for motor in self.all_motors:
            if not reader.isAvailable(motor.id, ADDR_PRESENT_VELOCITY, 8):
                return  # Never refresh the timestamp with incomplete feedback.
            velocity = reader.getData(motor.id, ADDR_PRESENT_VELOCITY, 4)
            tick = reader.getData(motor.id, ADDR_PRESENT_POSITION, 4)
            if velocity & (1 << 31):
                velocity -= 1 << 32
            if tick & (1 << 31):
                tick -= 1 << 32
            msg.name.append(motor.joint_name)
            msg.position.append(motor.tick_to_position(tick))
            msg.velocity.append(motor.direction * velocity / self.velocity_raw_per_rad_s)
            msg.effort.append(0.0)
        self.present_joint_state_pub.publish(msg)

    def _setup_telemetry_logger(self):
        if GroupSyncRead is None:
            raise RuntimeError(
                "dynamixel_sdk GroupSyncRead is unavailable; cannot log telemetry."
            )

        log_path = self.get_parameter("telemetry_log_path").get_parameter_value().string_value
        if not log_path:
            stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
            log_path = os.path.join(
                os.getcwd(),
                "src",
                "shift_ostrich",
                "motor_logs",
                f"dxl_telemetry_{stamp}.csv",
            )

        os.makedirs(os.path.dirname(os.path.abspath(log_path)), exist_ok=True)
        self.telemetry_file = open(log_path, "w", newline="", encoding="utf-8")
        self.telemetry_writer = csv.writer(self.telemetry_file)
        self.telemetry_writer.writerow(
            [
                "time_sec",
                "id",
                "joint_name",
                "model",
                "control_mode",
                "goal_tick",
                "present_position_tick",
                "position_error_tick",
                "present_velocity_raw",
                "present_pwm_raw",
                "present_pwm_percent",
                "effort_feedback_type",
                "effort_feedback_raw",
                "present_load_percent",
                "present_current_a",
                "torque_est_nm",
                "voltage_v",
                "temperature_c",
                "hardware_error_status",
                "hardware_error_flags",
            ]
        )
        self.telemetry_file.flush()

        motor_ids_text = (
            self.get_parameter("telemetry_motor_ids").get_parameter_value().string_value
        )
        if motor_ids_text.strip():
            wanted_ids = {
                int(item.strip())
                for item in motor_ids_text.split(",")
                if item.strip()
            }
            self.telemetry_motors = [
                motor for motor in self.all_motors if motor.id in wanted_ids
            ]
        else:
            self.telemetry_motors = list(self.all_motors)

        self.group_sync_read = GroupSyncRead(
            self.port_handler,
            self.packet_handler,
            ADDR_PRESENT_PWM,
            LEN_TELEMETRY_BLOCK,
        )
        for motor in self.telemetry_motors:
            if not self.group_sync_read.addParam(motor.id):
                self.get_logger().error(f"Failed to add DXL {motor.id} to telemetry sync read")

        rate_hz = self.get_parameter("telemetry_rate_hz").get_parameter_value().double_value
        rate_hz = max(1.0, rate_hz)
        self.telemetry_duration_sec = (
            self.get_parameter("telemetry_duration_sec").get_parameter_value().double_value
        )
        self.telemetry_start_time = self.get_clock().now()
        self.telemetry_timer = self.create_timer(1.0 / rate_hz, self.telemetry_callback)
        duration_text = (
            "until shutdown"
            if self.telemetry_duration_sec <= 0.0
            else f"for {self.telemetry_duration_sec:.1f}s"
        )
        self.get_logger().warn(
            f"Logging Dynamixel telemetry for {len(self.telemetry_motors)} motors "
            f"at {rate_hz:.1f} Hz {duration_text}: {log_path}"
        )

    def telemetry_callback(self):
        if self.group_sync_read is None or self.telemetry_writer is None:
            return

        now = self.get_clock().now()
        time_sec = (now - self.telemetry_start_time).nanoseconds * 1e-9
        if self.telemetry_duration_sec > 0.0 and time_sec > self.telemetry_duration_sec:
            self._stop_telemetry_logger()
            return

        result = self.group_sync_read.txRxPacket()
        if result != COMM_SUCCESS:
            self.get_logger().warn(
                f"Telemetry sync read failed: {self.packet_handler.getTxRxResult(result)}",
                throttle_duration_sec=1.0,
            )
            return

        for motor in self.telemetry_motors:
            if not self.group_sync_read.isAvailable(
                motor.id, ADDR_PRESENT_PWM, LEN_TELEMETRY_BLOCK
            ):
                self.get_logger().warn(
                    f"Telemetry unavailable for DXL {motor.id}",
                    throttle_duration_sec=1.0,
                )
                continue

            present_pwm = self._sync_read_signed(motor.id, ADDR_PRESENT_PWM, 2)
            effort_raw = self._sync_read_signed(motor.id, ADDR_PRESENT_LOAD, 2)
            load_percent, current_a, torque_nm = motor.effort_telemetry(effort_raw)
            present_velocity = self._sync_read_signed(motor.id, ADDR_PRESENT_VELOCITY, 4)
            present_position = self._sync_read_signed(motor.id, ADDR_PRESENT_POSITION, 4)
            voltage_raw = self.group_sync_read.getData(
                motor.id, ADDR_PRESENT_INPUT_VOLTAGE, 2
            )
            temperature = self.group_sync_read.getData(
                motor.id, ADDR_PRESENT_TEMPERATURE, 1
            )
            hardware_error_status = self._read_hardware_error_status(motor.id)
            hardware_error_flags = self._format_hardware_error_flags(
                hardware_error_status
            )
            if hardware_error_status:
                self.get_logger().error(
                    f"DXL {motor.id} {motor.joint_name} hardware error "
                    f"{hardware_error_status}: {hardware_error_flags}",
                    throttle_duration_sec=1.0,
                )

            goal_tick = self.last_goal_ticks.get(motor.id, "")
            position_error = ""
            if isinstance(goal_tick, int):
                position_error = goal_tick - present_position

            self.telemetry_writer.writerow(
                [
                    f"{time_sec:.4f}",
                    motor.id,
                    motor.joint_name,
                    motor.model,
                    motor.control_mode,
                    goal_tick,
                    present_position,
                    position_error,
                    present_velocity,
                    present_pwm,
                    f"{present_pwm * 0.113:.3f}",
                    motor.effort_feedback,
                    effort_raw,
                    "" if load_percent == "" else f"{load_percent:.3f}",
                    "" if current_a == "" else f"{current_a:.6f}",
                    f"{torque_nm:.6f}",
                    f"{voltage_raw * 0.1:.2f}",
                    temperature,
                    hardware_error_status,
                    hardware_error_flags,
                ]
            )
        self.telemetry_file.flush()

    def _stop_telemetry_logger(self):
        if hasattr(self, "telemetry_timer"):
            self.telemetry_timer.cancel()
        if self.telemetry_file is not None:
            self.telemetry_file.flush()
            self.telemetry_file.close()
            self.telemetry_file = None
        self.telemetry_writer = None
        self.group_sync_read = None
        self.get_logger().info("Stopped Dynamixel telemetry logging")

    def _setup_opencr_imu(self):
        self.opencr_imu_id = int(self.get_parameter("opencr_imu_id").value)
        self.opencr_imu_frame_id = self.get_parameter("opencr_imu_frame_id").value
        topic = self.get_parameter("opencr_imu_topic").value
        self.opencr_imu_pub = self.create_publisher(Imu, topic, 10)
        rate_hz = max(1.0, float(self.get_parameter("opencr_imu_rate_hz").value))
        self.opencr_imu_timer = self.create_timer(
            1.0 / rate_hz,
            self.opencr_imu_callback,
        )
        self.get_logger().warn(
            f"OpenCR IMU enabled: reading virtual DXL ID {self.opencr_imu_id} "
            f"at {rate_hz:.1f} Hz and publishing {topic}"
        )

    def opencr_imu_callback(self):
        data, result, error = self.packet_handler.readTxRx(
            self.port_handler,
            self.opencr_imu_id,
            ADDR_OPENCR_IMU_BLOCK,
            LEN_OPENCR_IMU_BLOCK,
        )
        if result != COMM_SUCCESS or error != 0 or len(data) != LEN_OPENCR_IMU_BLOCK:
            if not self.opencr_imu_warned:
                result_text = (
                    self.packet_handler.getTxRxResult(result)
                    if result != COMM_SUCCESS
                    else self.packet_handler.getRxPacketError(error)
                )
                self.get_logger().warn(
                    "OpenCR IMU read failed. Upload "
                    "arduino/opencr_dxl_bridge_with_imu/opencr_dxl_bridge_with_imu.ino "
                    f"and keep enable_opencr_imu:=true. Detail: {result_text}"
                )
                self.opencr_imu_warned = True
            return

        (
            _time_ms,
            qw,
            qx,
            qy,
            qz,
            _roll_deg,
            _pitch_deg,
            _yaw_deg,
            gyro_x_dps,
            gyro_y_dps,
            gyro_z_dps,
            acc_x_g,
            acc_y_g,
            acc_z_g,
            _gyro_x_adc,
            _gyro_y_adc,
            _gyro_z_adc,
            _acc_x_adc,
            _acc_y_adc,
            _acc_z_adc,
        ) = struct.unpack("<I13f6h", bytes(data))

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.opencr_imu_frame_id
        msg.orientation.w = float(qw)
        msg.orientation.x = float(qx)
        msg.orientation.y = float(qy)
        msg.orientation.z = float(qz)
        msg.orientation_covariance[0] = 0.0025
        msg.orientation_covariance[4] = 0.0025
        msg.orientation_covariance[8] = 0.0025

        deg_to_rad = math.pi / 180.0
        msg.angular_velocity.x = float(gyro_x_dps) * deg_to_rad
        msg.angular_velocity.y = float(gyro_y_dps) * deg_to_rad
        msg.angular_velocity.z = float(gyro_z_dps) * deg_to_rad
        msg.angular_velocity_covariance[0] = 0.02
        msg.angular_velocity_covariance[4] = 0.02
        msg.angular_velocity_covariance[8] = 0.02

        msg.linear_acceleration.x = float(acc_x_g) * STANDARD_GRAVITY
        msg.linear_acceleration.y = float(acc_y_g) * STANDARD_GRAVITY
        msg.linear_acceleration.z = float(acc_z_g) * STANDARD_GRAVITY
        msg.linear_acceleration_covariance[0] = 0.04
        msg.linear_acceleration_covariance[4] = 0.04
        msg.linear_acceleration_covariance[8] = 0.04

        self.opencr_imu_pub.publish(msg)
        self.opencr_imu_warned = False

    def _sync_read_signed(self, dxl_id: int, address: int, length: int) -> int:
        value = self.group_sync_read.getData(dxl_id, address, length)
        bits = length * 8
        sign_bit = 1 << (bits - 1)
        if value & sign_bit:
            value -= 1 << bits
        return value

    def _read_hardware_error_status(self, dxl_id: int) -> int:
        value, result, error = self.packet_handler.read1ByteTxRx(
            self.port_handler,
            dxl_id,
            ADDR_HARDWARE_ERROR_STATUS,
        )
        if result != COMM_SUCCESS or error != 0:
            return -1
        return int(value)

    @staticmethod
    def _format_hardware_error_flags(status: int) -> str:
        if status == 0:
            return ""
        if status < 0:
            return "read_failed"
        flags = [
            name for bit, name in HARDWARE_ERROR_BITS.items() if status & (1 << bit)
        ]
        return "|".join(flags) if flags else f"unknown_bits_{status}"

    def _send_position_map(self, positions: Dict[str, float], source: str):
        if getattr(self, "policy_feedback_fault", False):
            return
        goal_ticks = {}

        for joint_name, position in positions.items():
            motor = self.joint_to_motor.get(joint_name)
            if motor is None or not math.isfinite(position):
                continue
            goal_tick = self._checked_goal_tick(motor, position)
            if goal_tick is None:
                continue
            prev_tick = self.last_goal_ticks.get(motor.id)
            if prev_tick is not None and abs(goal_tick - prev_tick) < self.min_tick_change:
                continue
            goal_ticks[motor] = goal_tick

        if goal_ticks:
            # Use the same acknowledged ID-17 write as the working sliders.
            # SyncWrite has no status reply, so it cannot report a gripper
            # hardware Alert at command delivery time.
            gripper = next((m for m in goal_ticks if m.id == 17), None)
            gripper_tick = goal_ticks.pop(gripper) if gripper is not None else None
            if goal_ticks:
                self._sync_write_goal_ticks(goal_ticks)
            if gripper is not None:
                self._write_goal_tick(gripper, gripper_tick)

    def _write_goal_tick(self, motor: MotorConfig, goal_tick: int):
        self._write4(motor.id, ADDR_GOAL_POSITION, goal_tick)
        self.last_goal_ticks[motor.id] = goal_tick

    def _sync_write_goal_ticks(self, goal_ticks: Dict[MotorConfig, int]):
        self.group_sync_write.clearParam()

        for motor, goal_tick in goal_ticks.items():
            param = int(goal_tick).to_bytes(4, byteorder="little", signed=False)
            if not self.group_sync_write.addParam(motor.id, list(param)):
                raise RuntimeError(f"Failed to add DXL {motor.id} to sync write")

        result = self.group_sync_write.txPacket()
        if result != COMM_SUCCESS:
            raise RuntimeError(
                f"Sync write failed: {self.packet_handler.getTxRxResult(result)}"
            )

        for motor, goal_tick in goal_ticks.items():
            self.last_goal_ticks[motor.id] = goal_tick

    def _sync_write_goal_velocities(self, goal_velocities: Dict[MotorConfig, int]):
        self.group_sync_write_velocity.clearParam()

        for motor, goal_velocity in goal_velocities.items():
            param = int(goal_velocity).to_bytes(4, byteorder="little", signed=True)
            if not self.group_sync_write_velocity.addParam(motor.id, list(param)):
                raise RuntimeError(
                    f"Failed to add DXL {motor.id} to velocity sync write"
                )

        result = self.group_sync_write_velocity.txPacket()
        if result != COMM_SUCCESS:
            raise RuntimeError(
                f"Velocity sync write failed: {self.packet_handler.getTxRxResult(result)}"
            )

        for motor, goal_velocity in goal_velocities.items():
            self.last_goal_velocities[motor.id] = goal_velocity

    def _write1(self, dxl_id: int, address: int, value: int):
        result, error = self.packet_handler.write1ByteTxRx(
            self.port_handler, dxl_id, address, value
        )
        self._check_result(dxl_id, address, result, error)

    def _write2(self, dxl_id: int, address: int, value: int):
        result, error = self.packet_handler.write2ByteTxRx(
            self.port_handler, dxl_id, address, value
        )
        self._check_result(dxl_id, address, result, error)

    def _write4(self, dxl_id: int, address: int, value: int):
        result, error = self.packet_handler.write4ByteTxRx(
            self.port_handler, dxl_id, address, value
        )
        self._check_result(dxl_id, address, result, error)

    def _log_motor_fault(self, mid):
        """Read fault evidence directly, including data returned with Alert set."""
        if not hasattr(self, 'faulted_motor_ids'):
            self.faulted_motor_ids = set()
        self.faulted_motor_ids.add(mid)
        snapshot = {'id': mid}
        for name, address, size in (('hardware_error', 70, 1), ('watchdog', 98, 1),
                                    ('voltage_raw', 144, 2), ('temperature', 146, 1),
                                    ('present_current_raw', 126, 2)):
            try:
                value, comm, error = getattr(self.packet_handler, f'read{size}ByteTxRx')(
                    self.port_handler, mid, address)
                snapshot[name] = {'value': value, 'comm': comm, 'packet_error': error}
            except Exception as exc:
                snapshot[name] = str(exc)
        self.get_logger().error('Motor fault snapshot: ' + json.dumps(snapshot))
        if mid == 17 and mid in self.enabled_by_bridge:
            self._stop_faulted_gripper()

    def _stop_faulted_gripper(self):
        # Alert (bit 7) accompanies otherwise valid status replies even to
        # torque-OFF commands. Accept that flag, but reject transport/instruction
        # failures, and attempt both stop operations independently.
        failures = []
        for address, size in ((ADDR_GOAL_CURRENT, 2), (ADDR_TORQUE_ENABLE, 1)):
            try:
                comm, error = getattr(self.packet_handler, f'write{size}ByteTxRx')(
                    self.port_handler, 17, address, 0)
                if comm != COMM_SUCCESS or error & 0x7f:
                    failures.append(f'{address}: comm={comm}, error={error}')
            except Exception as exc:
                failures.append(str(exc))
        if failures:
            raise RuntimeError('ID 17 fault stop failed: ' + '; '.join(failures))
        self.enabled_by_bridge.discard(17)
        self.get_logger().error('ID 17 fault: current zeroed, torque OFF; other motor torque retained')

    def _check_result(self, dxl_id: int, address: int, result: int, error: int, operation="write"):
        if result != COMM_SUCCESS:
            raise RuntimeError(
                f"DXL {dxl_id} {operation} failed at {address}: "
                f"{self.packet_handler.getTxRxResult(result)}"
            )
        elif error != 0:
            self._log_motor_fault(dxl_id)
            raise RuntimeError(
                f"DXL {dxl_id} packet error at {address}: "
                f"{self.packet_handler.getRxPacketError(error)}"
            )


def main(args: Optional[List[str]] = None):
    rclpy.init(args=args)
    node = DxlJointStateBridge.__new__(DxlJointStateBridge)
    exit_code = 0
    try:
        DxlJointStateBridge.__init__(node)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except (RuntimeError, ValueError, OSError) as exc:
        node.get_logger().error(str(exc))
        # A runtime fault must use the same guarded release as a failed handoff.
        # Otherwise a gripper fault would cut the preexisting leg holding torque.
        node.startup_complete = False
        exit_code = 1
    finally:
        try:
            if hasattr(node, 'all_motors') and hasattr(node, 'port_handler'):
                node.destroy_node()
            elif hasattr(node, 'handle'):
                Node.destroy_node(node)
        finally:
            if rclpy.ok():
                rclpy.shutdown()
    if exit_code:
        raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
