"""OpenRB reader with optional ID-26 firmware-controlled OMX trigger."""
import fcntl
import math
import time
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from dynamixel_sdk import COMM_SUCCESS, GroupSyncRead, PacketHandler, PortHandler
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from sensor_msgs.msg import JointState
import yaml
from .gripper_spring import DynamixelTriggerSpring


def leader_joint_pairs(config):
    ids, names = config['ids'], config['names']
    if len(ids) != len(names) or len(set(ids)) != len(ids) or len(set(names)) != len(names):
        raise ValueError('Leader IDs and joint names must be unique and match in length')
    pairs = [(int(mid), name) for mid, name in zip(ids, names) if int(mid) != 25]
    if 26 in dict(pairs) and dict(pairs)[26] != 'gripper_joint_1':
        raise ValueError('Leader ID 26 must map to gripper_joint_1')
    return pairs


class LeaderReader(Node):
    def __init__(self):
        super().__init__('shift_leader_reader')
        default = Path(get_package_share_directory('shift_ostrich'))/'config/teleop.yaml'
        self.declare_parameter('config_path', str(default))
        self.declare_parameter('enable_gripper_spring', False)
        cfg = yaml.safe_load(Path(self.get_parameter('config_path').value).read_text())
        self.cfg = cfg['leader']
        self.joints = leader_joint_pairs(self.cfg)
        self.spring = None
        self.read_address, self.read_length = 132, 4
        self.port = PortHandler(self.cfg['device'])
        self.packet = PacketHandler(2.0)
        self.lock = open('/tmp/shift_ostrich_openrb_leader.lock', 'a')
        fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        try:
            if not self.port.openPort() or not self.port.setBaudRate(self.cfg['baudrate']):
                raise RuntimeError('Cannot open OpenRB leader port')
            spring_cfg = self.cfg.get('gripper_spring', {})
            spring_enabled = (self.get_parameter('enable_gripper_spring').value
                              and spring_cfg.get('enabled', False))
            if spring_enabled:
                self.read_address, self.read_length = 128, 8
            self.reader = GroupSyncRead(self.port, self.packet, self.read_address, self.read_length)
            for motor, _ in self.joints:
                if not self.reader.addParam(motor):
                    raise RuntimeError(f'Cannot register leader ID {motor}')
            if spring_enabled:
                self.spring = DynamixelTriggerSpring(self.port, self.packet, spring_cfg, time.monotonic())
                self.spring.start()
        except Exception:
            if self.spring is not None:
                self.spring.close()
            self.port.closePort()
            self.lock.close()
            raise
        self.publisher = self.create_publisher(JointState, cfg['topics']['leader_states'], 1)
        rate = spring_cfg.get('control_rate_hz', 100.0) if spring_enabled else self.cfg['rate_hz']
        self.create_timer(1.0/rate, self.read)
        self.get_logger().info(f'OpenRB reader: {self.cfg["device"]}; '
                               f'IDs {[mid for mid, _ in self.joints]}; ID 25 ignored; '
                               f'ID 26 OMX mode-5 trigger={spring_enabled}')

    def read(self):
        result = self.reader.txRxPacket()
        if result != COMM_SUCCESS:
            if getattr(self, 'spring', None) is not None:
                self.spring.zero()
            self.get_logger().warn(self.packet.getTxRxResult(result), throttle_duration_sec=2.0)
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        for motor, name in self.joints:
            if not self.reader.isAvailable(motor, getattr(self, 'read_address', 132),
                                           getattr(self, 'read_length', 4)):
                if getattr(self, 'spring', None) is not None:
                    self.spring.zero()
                return
            raw = self.reader.getData(motor, 132, 4)
            if raw >= 2**31:
                raw -= 2**32
            if motor == 26 and getattr(self, 'spring', None) is not None:
                velocity = self.reader.getData(motor, 128, 4)
                if velocity >= 2**31:
                    velocity -= 2**32
                self.spring.apply(raw, velocity*0.023980823922402087, time.monotonic())
            msg.name.append(name)
            msg.position.append((raw-2048)*2*math.pi/4096)
        self.publisher.publish(msg)

    def destroy_node(self):
        try:
            if self.spring is not None:
                self.spring.close()
        finally:
            self.reader.clearParam()
            self.port.closePort()
            self.lock.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LeaderReader()
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
