#!/usr/bin/env python3
"""Show Shift Ostrich IMU attitude as world -> base_link in RViz."""
import math
from typing import Optional

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


def normalize(q):
    norm = math.sqrt(sum(value * value for value in q))
    if norm <= 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(value / norm for value in q)


def multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def conjugate(q):
    x, y, z, w = q
    return (-x, -y, -z, w)


def rotate_vector(q, vector):
    vx, vy, vz = vector
    rotated = multiply(multiply(q, (vx, vy, vz, 0.0)), conjugate(q))
    return rotated[0], rotated[1], rotated[2]


def rpy_to_quaternion(roll: float, pitch: float, yaw: float):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return normalize(
        (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )
    )


def quaternion_to_matrix(q):
    x, y, z, w = normalize(q)
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z
    return (
        (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)),
        (2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
        (2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)),
    )


def matrix_to_quaternion(m):
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2][1] - m[1][2]) / s
        y = (m[0][2] - m[2][0]) / s
        z = (m[1][0] - m[0][1]) / s
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        w = (m[2][1] - m[1][2]) / s
        x = 0.25 * s
        y = (m[0][1] + m[1][0]) / s
        z = (m[0][2] + m[2][0]) / s
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        w = (m[0][2] - m[2][0]) / s
        x = (m[0][1] + m[1][0]) / s
        y = 0.25 * s
        z = (m[1][2] + m[2][1]) / s
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
        w = (m[1][0] - m[0][1]) / s
        x = (m[0][2] + m[2][0]) / s
        y = (m[1][2] + m[2][1]) / s
        z = 0.25 * s
    return normalize((x, y, z, w))


def matmul(a, b):
    return tuple(
        tuple(sum(a[row][k] * b[k][col] for k in range(3)) for col in range(3))
        for row in range(3)
    )


def transpose(m):
    return tuple(tuple(m[row][col] for row in range(3)) for col in range(3))


def remap_matrix(remap: str):
    values = {"r": 0, "p": 1, "y": 2}
    remap = remap.strip()
    if len(remap) != 3 or set(remap.lower()) != set('rpy'):
        raise ValueError('rpy_remap must contain r, p and y exactly once; uppercase reverses sign')

    rows = []
    for key in remap:
        sign = -1.0 if key.isupper() else 1.0
        axis = key.lower()
        if axis not in values:
            return ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
        row = [0.0, 0.0, 0.0]
        row[values[axis]] = sign
        rows.append(tuple(row))
    return tuple(rows)


def remap_quaternion(q, remap: str):
    s = remap_matrix(remap)
    r_in = quaternion_to_matrix(q)
    r_out = matmul(matmul(s, r_in), transpose(s))
    return matrix_to_quaternion(r_out)


def yaw_from_quaternion(q):
    m = quaternion_to_matrix(q)
    return math.atan2(m[1][0], m[0][0])


class ImuBaseTf(Node):
    def __init__(self):
        super().__init__("imu_base_tf")
        self.declare_parameter("imu_topic", "/opencr/imu")
        self.declare_parameter("parent_frame", "world")
        self.declare_parameter("child_frame", "base_link")
        self.declare_parameter("mount_roll_deg", -90.0)
        self.declare_parameter("mount_pitch_deg", 0.0)
        self.declare_parameter("mount_yaw_deg", 0.0)
        self.declare_parameter("invert_x", False)
        self.declare_parameter("invert_y", False)
        self.declare_parameter("invert_z", False)
        self.declare_parameter("invert_w", False)
        self.declare_parameter("rpy_remap", "pRy")
        self.declare_parameter("pivot_x", 0.0)
        self.declare_parameter("pivot_y", 0.0)
        self.declare_parameter("pivot_z", 0.0)
        self.declare_parameter("zero_yaw_on_start", True)
        self.declare_parameter("yaw_zero_samples", 10)

        self.parent_frame = self.get_parameter("parent_frame").value
        self.child_frame = self.get_parameter("child_frame").value
        self.mount_correction = rpy_to_quaternion(
            math.radians(float(self.get_parameter("mount_roll_deg").value)),
            math.radians(float(self.get_parameter("mount_pitch_deg").value)),
            math.radians(float(self.get_parameter("mount_yaw_deg").value)),
        )
        self.invert_x = bool(self.get_parameter("invert_x").value)
        self.invert_y = bool(self.get_parameter("invert_y").value)
        self.invert_z = bool(self.get_parameter("invert_z").value)
        self.invert_w = bool(self.get_parameter("invert_w").value)
        self.rpy_remap = self.get_parameter("rpy_remap").value
        if self.rpy_remap:
            remap_matrix(self.rpy_remap)
        self.pivot = (
            float(self.get_parameter("pivot_x").value),
            float(self.get_parameter("pivot_y").value),
            float(self.get_parameter("pivot_z").value),
        )
        self.zero_yaw_on_start = bool(self.get_parameter("zero_yaw_on_start").value)
        self.yaw_zero_samples = max(1, int(self.get_parameter("yaw_zero_samples").value))
        self.yaw_zero_sin_sum = 0.0
        self.yaw_zero_cos_sum = 0.0
        self.yaw_zero_count = 0
        self.yaw_zero_correction = (0.0, 0.0, 0.0, 1.0)

        self.broadcaster = TransformBroadcaster(self)
        self.latest_orientation = (0.0, 0.0, 0.0, 1.0)
        self.last_imu_time = None
        self.started_at = self.get_clock().now()
        imu_topic = self.get_parameter("imu_topic").value
        self.subscription = self.create_subscription(Imu, imu_topic, self.imu_callback, qos_profile_sensor_data)
        self.publish_timer = self.create_timer(1./30., self.publish_transform)
        self.get_logger().warn(
            f"Publishing IMU TF {self.parent_frame} -> {self.child_frame} from {imu_topic}"
        )

    def imu_callback(self, msg: Imu):
        raw = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        if (msg.orientation_covariance[0] == -1 or not all(math.isfinite(v) for v in raw)
                or sum(v*v for v in raw) < 1e-12):
            self.get_logger().warn('Invalid IMU orientation; retaining last valid pose', throttle_duration_sec=5.)
            return
        q = (
            -msg.orientation.x if self.invert_x else msg.orientation.x,
            -msg.orientation.y if self.invert_y else msg.orientation.y,
            -msg.orientation.z if self.invert_z else msg.orientation.z,
            -msg.orientation.w if self.invert_w else msg.orientation.w,
        )
        # q_world_base = q_world_sensor * q_sensor_base. Mounting is a
        # sensor/body extrinsic, not a rotation of the world coordinate frame.
        q = normalize(multiply(normalize(q), self.mount_correction))
        if self.rpy_remap:
            q = remap_quaternion(q, self.rpy_remap)
        if self.zero_yaw_on_start:
            q = self.apply_yaw_zero(q)
        if self.last_imu_time is None:
            self.get_logger().info('IMU orientation received; RViz attitude tracking active')
        self.latest_orientation = q
        self.last_imu_time = self.get_clock().now()

    def publish_transform(self):
        q = self.latest_orientation
        now = self.get_clock().now()
        age = (now-(self.last_imu_time if self.last_imu_time is not None else self.started_at)).nanoseconds*1e-9
        if age > 2.:
            self.get_logger().warn('No fresh IMU orientation; showing last pose (identity before first sample)',
                                   throttle_duration_sec=5.)

        transform = TransformStamped()
        transform.header.stamp = now.to_msg()
        transform.header.frame_id = self.parent_frame
        transform.child_frame_id = self.child_frame
        rotated_pivot = rotate_vector(q, self.pivot)
        transform.transform.translation.x = -rotated_pivot[0]
        transform.transform.translation.y = -rotated_pivot[1]
        transform.transform.translation.z = -rotated_pivot[2]
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]
        self.broadcaster.sendTransform(transform)

    def apply_yaw_zero(self, q):
        if self.yaw_zero_count < self.yaw_zero_samples:
            yaw = yaw_from_quaternion(q)
            self.yaw_zero_sin_sum += math.sin(yaw)
            self.yaw_zero_cos_sum += math.cos(yaw)
            self.yaw_zero_count += 1
            if self.yaw_zero_count == self.yaw_zero_samples:
                zero_yaw = math.atan2(self.yaw_zero_sin_sum, self.yaw_zero_cos_sum)
                self.yaw_zero_correction = rpy_to_quaternion(0.0, 0.0, -zero_yaw)
                self.get_logger().warn(
                    f"IMU yaw zeroed at startup: {math.degrees(zero_yaw):.1f} deg"
                )
        return normalize(multiply(self.yaw_zero_correction, q))


def main(args: Optional[list[str]] = None):
    rclpy.init(args=args)
    node = ImuBaseTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
