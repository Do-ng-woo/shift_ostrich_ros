"""ROS adapter for Shift teleop; uses the existing OpenCR bridge only."""
import json
import math
from pathlib import Path
import time

from ament_index_python.packages import get_package_prefix, get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import yaml

from .teleop import TeleopCollisionCheck, TeleopControl
from .hardware_limits import configured_gripper_drive, gripper_drive_matches


class ArmTeleop(Node):
    def __init__(self):
        super().__init__('shift_arm_teleop')
        share = Path(get_package_share_directory('shift_ostrich'))
        self.declare_parameter('config_path', str(share/'config/teleop.yaml'))
        self.declare_parameter('preview_only', True)
        self.declare_parameter('enable_on_start', False)
        self.preview = bool(self.get_parameter('preview_only').value)
        self.auto_enable = bool(self.get_parameter('enable_on_start').value)
        cfg = yaml.safe_load(Path(self.get_parameter('config_path').value).read_text())
        self.topics = cfg['topics']
        self.gripper_enabled = cfg.get('gripper', {}).get('enabled', True)
        self.expected_gripper_drive = configured_gripper_drive(share) if self.gripper_enabled else None
        library = Path(get_package_prefix('shift_ostrich'))/'lib/libshift_mesh_interference.so'
        self.collision = TeleopCollisionCheck(share, library)
        self.core = TeleopControl(share, cfg, self.collision)
        self.trajectory_pub = None if self.preview else self.create_publisher(
            JointTrajectory, self.topics['trajectory'], 1)
        self.preview_pub = self.create_publisher(JointTrajectory, self.topics['preview'], 1)
        self.wheel_pub = None if self.preview else self.create_publisher(
            Float64MultiArray, self.topics['wheel_velocity'], 1)
        self.status_pub = self.create_publisher(String, self.topics['status'], 1)
        self.create_subscription(JointState, self.topics['hardware_states'],
            lambda m: self.core.update_hardware(m.name, m.position, time.monotonic()), 1)
        self.create_subscription(JointState, self.topics['leader_states'],
            lambda m: self.core.update_leader(m.name, m.position, time.monotonic()), 1)
        self.create_service(SetBool, '/manipulator/enable_teleop', self.on_enable)
        self.parameter_client = self.create_client(GetParameters, '/dxl_joint_state_bridge/get_parameters')
        self.bridge_parameters = None
        self.parameter_future = None
        self.parameter_requested = -1e9
        self.gripper_client = self.create_client(Trigger, '/dxl_joint_state_bridge/gripper_status')
        self.gripper_future = None
        self.gripper_requested = -1e9
        self.gripper_drive = None
        self.last_status = ''
        self.startup_complete = False
        self.create_timer(1.0/cfg['control']['rate_hz'], self.tick)
        self.create_timer(0.5, self.publish_status)
        self.get_logger().info(
            f'Shift teleop: preview_only={self.preview}; starts aligned to measured pose; '
            'no OpenCR serial access or startup centering')

    def interface_error(self):
        if self.preview:
            return None
        if self.bridge_parameters is None or time.monotonic() - self.parameter_requested > 3.0:
            return 'Waiting for current hardware bridge parameter check'
        if not self.bridge_parameters[0] or self.bridge_parameters[1]:
            return 'Bridge requires enable_trajectory_commands:=true and enable_joint_state_commands:=false'
        drive = self.gripper_drive
        if self.gripper_enabled and (drive is None or time.monotonic()-self.gripper_requested > 10):
            return 'Waiting for ID 17 PD readback; restart the bridge after updating its configuration'
        if self.gripper_enabled and not gripper_drive_matches(drive, self.expected_gripper_drive):
            return 'ID 17 mode/PID/current readback must match config/hardware.yaml with torque ON and no hardware error'
        if self.trajectory_pub.get_subscription_count() != 1:
            return 'Exactly one hardware trajectory subscriber is required'
        others = [p for p in self.get_publishers_info_by_topic(self.topics['trajectory'])
                  if p.node_name != self.get_name() or p.node_namespace != self.get_namespace()]
        if others:
            return 'Another trajectory publisher is active: ' + ', '.join(p.node_name for p in others)
        return None

    def try_enable(self):
        error = self.interface_error()
        if error:
            return False, error
        return self.core.enable(time.monotonic())

    def on_enable(self, request, response):
        self.auto_enable = False
        if request.data:
            response.success, response.message = self.try_enable()
        else:
            self.core.disable()
            response.success, response.message = True, 'Teleop disabled; last hardware goals retained'
        self.get_logger().info(response.message)
        return response

    def update_bridge_parameters(self):
        now = time.monotonic()
        if self.preview or now - self.parameter_requested < 1.0:
            return
        if self.parameter_future is not None and not self.parameter_future.done():
            if now - self.parameter_requested < 3.0:
                return
            self.parameter_client.remove_pending_request(self.parameter_future)
        if not self.parameter_client.service_is_ready():
            return
        request = GetParameters.Request()
        request.names = ['enable_trajectory_commands', 'enable_joint_state_commands']
        self.parameter_requested = now
        self.parameter_future = self.parameter_client.call_async(request)
        self.parameter_future.add_done_callback(self.on_bridge_parameters)

    def on_bridge_parameters(self, future):
        try:
            values = future.result().values
            self.bridge_parameters = ([v.bool_value for v in values]
                                      if len(values) == 2 and all(v.type == 1 for v in values) else None)
        except Exception:
            self.bridge_parameters = None

    def update_gripper_diagnostics(self):
        if not self.gripper_enabled or self.preview or time.monotonic()-self.gripper_requested < 5:
            return
        if self.gripper_future is not None and not self.gripper_future.done():
            self.gripper_client.remove_pending_request(self.gripper_future)
        if not self.gripper_client.service_is_ready():
            return
        self.gripper_requested = time.monotonic()
        self.gripper_future = self.gripper_client.call_async(Trigger.Request())
        self.gripper_future.add_done_callback(self.on_gripper_diagnostics)

    def on_gripper_diagnostics(self, future):
        try:
            response = future.result()
            self.gripper_drive = json.loads(response.message) if response.success else None
        except Exception:
            self.gripper_drive = None

    def tick(self):
        self.update_bridge_parameters()
        self.update_gripper_diagnostics()
        if self.auto_enable and not self.startup_complete:
            ok, detail = self.try_enable()
            if ok:
                self.startup_complete = True
                self.get_logger().info(detail)
            else:
                self.core.detail = detail
        if self.core.enabled:
            error = self.interface_error()
            if error:
                self.core.disable(error)
        output = self.core.tick(time.monotonic())
        if output is None:
            return
        msg = JointTrajectory()
        msg.joint_names = self.core.names
        point = JointTrajectoryPoint()
        point.positions = [output[n] for n in msg.joint_names]
        # Core has already limited each step's speed and checked its path.
        # A zero-time single point is a streaming setpoint: do not interpolate
        # again from lagging encoder feedback on every 100 ms update.
        msg.points = [point]
        self.preview_pub.publish(msg)
        if self.trajectory_pub is not None:
            self.trajectory_pub.publish(msg)
            stop = Float64MultiArray()
            stop.data = [0.0, 0.0]
            self.wheel_pub.publish(stop)

    def publish_status(self):
        core = self.core
        now = time.monotonic()
        status = {'enabled': core.enabled, 'preview_only': self.preview,
                  'gripper_enabled': self.gripper_enabled,
                  'detail': core.detail, 'interface_error': self.interface_error(),
                  'hardware_age_sec': round(now-core.hardware_time, 3) if core.hardware else None,
                  'missing_hardware_joints': core.hardware_missing,
                  'leader_age_sec': round(now-core.input_time, 3) if core.input else None,
                  'lower_error_deg': {n: round(math.degrees(core.hardware[n]-v), 2)
                                      for n, v in core.lower.items()} if core.hardware else {},
                  'gripper': {'leader_rad': core.input.get('gripper_joint_1') if core.input else None,
                              'measured_rad': core.hardware.get('gripper_pitch') if core.hardware else None,
                              'goal_rad': core.output.get('gripper_pitch') if core.output else None},
                  'gripper_drive': self.gripper_drive,
                  'blocked_contacts': core.blocked_contacts}
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)
        detail = (core.detail, status['interface_error'], core.enabled)
        if detail != self.last_status:
            self.get_logger().info(msg.data)
            self.last_status = detail

    def destroy_node(self):
        self.collision.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ArmTeleop()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except _rclpy.RCLError:
        if rclpy.ok():
            raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
