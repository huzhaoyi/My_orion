#!/usr/bin/env python3
"""
将 HoloOcean CableSensor（COM 系下 位置 + 轴向 + 欧拉角）转换为 MTC 所需的 object_pose / place_pose（base_link 下）。
单目标：3m 长、直径 5cm 缆绳，抓取与放置共用同一目标位姿。
"""

import math
from typing import Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from std_msgs.msg import Header
from holoocean_interfaces.msg import CableSensor
from orion_mtc_msgs.msg import PerceptionState
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


DEFAULT_LEFT_ARM_BASE_IN_ROV = (1.55, 0.5653, -0.283628)
DEFAULT_RIGHT_ARM_BASE_IN_ROV = (1.55, -0.5653, -0.283628)


def _euler_rad_to_quat(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """欧拉角 RPY（弧度，内旋 ZYX）转四元数 (x, y, z, w)。"""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return (float(qx), float(qy), float(qz), float(qw))


def _quat_to_rotation_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """四元数 (x,y,z,w) 转 3x3 旋转矩阵。"""
    return np.array([
        [
            1.0 - 2.0 * (qy * qy + qz * qz),
            2.0 * (qx * qy - qz * qw),
            2.0 * (qx * qz + qy * qw),
        ],
        [
            2.0 * (qx * qy + qz * qw),
            1.0 - 2.0 * (qx * qx + qz * qz),
            2.0 * (qy * qz - qx * qw),
        ],
        [
            2.0 * (qx * qz - qy * qw),
            2.0 * (qy * qz + qx * qw),
            1.0 - 2.0 * (qx * qx + qy * qy),
        ],
    ])


def _quat_from_rotation_matrix(R: np.ndarray) -> Tuple[float, float, float, float]:
    """3x3 旋转矩阵转四元数 (x, y, z, w)。"""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        qw = 0.25 / s
        qx = (R[2, 1] - R[1, 2]) * s
        qy = (R[0, 2] - R[2, 0]) * s
        qz = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s
    return (float(qx), float(qy), float(qz), float(qw))


def _quat_from_direction(dx: float, dy: float, dz: float) -> Tuple[float, float, float, float]:
    """
    由缆绳轴向（单位向量）得到四元数，使圆柱体局部 Z 轴与该方向一致。
    MoveIt/ROS 的 CYLINDER 沿局部 Z 轴，故用此姿态后圆柱会沿缆绳方向（平躺时方向为水平）。
    """
    z = np.array([float(dx), float(dy), float(dz)], dtype=float)
    n = np.linalg.norm(z)
    if n < 1e-9:
        z = np.array([1.0, 0.0, 0.0])
    else:
        z = z / n
    if z[2] < -0.9999:
        x = np.cross(np.array([0.0, 0.0, 1.0]), z)
    else:
        x = np.cross(np.array([0.0, 1.0, 0.0]), z)
    xn = np.linalg.norm(x)
    if xn < 1e-9:
        x = np.array([1.0, 0.0, 0.0]) if abs(z[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    else:
        x = x / xn
    y = np.cross(z, x)
    R = np.column_stack([x, y, z])
    return _quat_from_rotation_matrix(R)


def _rotation_matrix_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """RPY 弧度 -> 3x3 旋转矩阵（内旋 ZYX）。"""
    cx = math.cos(roll)
    sx = math.sin(roll)
    cy = math.cos(pitch)
    sy = math.sin(pitch)
    cz = math.cos(yaw)
    sz = math.sin(yaw)
    return np.array([
        [cy * cz, -cy * sz, sy],
        [sx * sy * cz + cx * sz, -sx * sy * sz + cx * cz, -sx * cy],
        [-cx * sy * cz + sx * sz, cx * sy * sz + sx * cz, cx * cy],
    ])


class CableSensorToObjectPoseNode(Node):
    """
    订阅 CableSensor（COM 系：位置 + 轴向 + 欧拉角）和 ROV 位姿，
    将缆绳目标变换到 base_link，发布 object_pose、place_pose（同源）与 perception_state。
    """

    def __init__(self) -> None:
        super().__init__("cable_sensor_to_object_pose")
        self.declare_parameter("cable_sensor_topic", "/holoocean/rov0/CableSensor")
        self.declare_parameter("rov_pose_topic", "/holoocean/rov0/PoseSensor")
        self.declare_parameter("object_pose_topic", "object_pose")
        self.declare_parameter("place_pose_topic", "place_pose")
        self.declare_parameter("world_frame_id", "map")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("perception_state_topic", "perception_state")
        self.declare_parameter("output_frame_id", "base_link")
        self.declare_parameter("use_left_arm", True)
        self.declare_parameter("left_arm_base_in_rov_x", DEFAULT_LEFT_ARM_BASE_IN_ROV[0])
        self.declare_parameter("left_arm_base_in_rov_y", DEFAULT_LEFT_ARM_BASE_IN_ROV[1])
        self.declare_parameter("left_arm_base_in_rov_z", DEFAULT_LEFT_ARM_BASE_IN_ROV[2])
        self.declare_parameter("right_arm_base_in_rov_x", DEFAULT_RIGHT_ARM_BASE_IN_ROV[0])
        self.declare_parameter("right_arm_base_in_rov_y", DEFAULT_RIGHT_ARM_BASE_IN_ROV[1])
        self.declare_parameter("right_arm_base_in_rov_z", DEFAULT_RIGHT_ARM_BASE_IN_ROV[2])
        self.declare_parameter("position_offset_x", 0.0)
        self.declare_parameter("position_offset_y", 0.0)
        self.declare_parameter("position_offset_z", 0.0)
        # CableSensor 的数据可能来自不同坐标系：
        # - positions：有的版本虽标注 COM，但实际给的是 world/map（可通过与 ROV(map) 接近性判断）
        # - directions / euler_angles：通常在 COM/机体系
        # 为避免误解读，分别提供 frame 参数；保留 cable_frame_is_com 作为兼容入口（同时影响三者）。
        self.declare_parameter("cable_frame_is_com", True)
        self.declare_parameter("cable_positions_frame", "")
        self.declare_parameter("cable_direction_frame", "")
        self.declare_parameter("cable_euler_frame", "")

        self._cable_sensor_topic = self.get_parameter("cable_sensor_topic").get_parameter_value().string_value
        self._rov_pose_topic = self.get_parameter("rov_pose_topic").get_parameter_value().string_value
        self._object_pose_topic = self.get_parameter("object_pose_topic").get_parameter_value().string_value
        self._place_pose_topic = self.get_parameter("place_pose_topic").get_parameter_value().string_value
        self._world_frame_id = self.get_parameter("world_frame_id").get_parameter_value().string_value
        self._publish_tf = self.get_parameter("publish_tf").get_parameter_value().bool_value
        self._perception_state_topic = self.get_parameter("perception_state_topic").get_parameter_value().string_value
        self._output_frame_id = self.get_parameter("output_frame_id").get_parameter_value().string_value
        self._use_left_arm = self.get_parameter("use_left_arm").get_parameter_value().bool_value
        if self._use_left_arm:
            tx = self.get_parameter("left_arm_base_in_rov_x").get_parameter_value().double_value
            ty = self.get_parameter("left_arm_base_in_rov_y").get_parameter_value().double_value
            tz = self.get_parameter("left_arm_base_in_rov_z").get_parameter_value().double_value
        else:
            tx = self.get_parameter("right_arm_base_in_rov_x").get_parameter_value().double_value
            ty = self.get_parameter("right_arm_base_in_rov_y").get_parameter_value().double_value
            tz = self.get_parameter("right_arm_base_in_rov_z").get_parameter_value().double_value
        self._t_arm_in_rov = np.array([tx, ty, tz], dtype=float)
        self._offset_x = self.get_parameter("position_offset_x").get_parameter_value().double_value
        self._offset_y = self.get_parameter("position_offset_y").get_parameter_value().double_value
        self._offset_z = self.get_parameter("position_offset_z").get_parameter_value().double_value
        self._cable_frame_is_com = self.get_parameter("cable_frame_is_com").get_parameter_value().bool_value
        self._cable_positions_frame = self.get_parameter("cable_positions_frame").get_parameter_value().string_value.strip()
        self._cable_direction_frame = self.get_parameter("cable_direction_frame").get_parameter_value().string_value.strip()
        self._cable_euler_frame = self.get_parameter("cable_euler_frame").get_parameter_value().string_value.strip()
        if not self._cable_positions_frame:
            self._cable_positions_frame = "com" if self._cable_frame_is_com else "world"
        if not self._cable_direction_frame:
            self._cable_direction_frame = "com" if self._cable_frame_is_com else "world"
        if not self._cable_euler_frame:
            self._cable_euler_frame = "com" if self._cable_frame_is_com else "world"

        self._rov_position: Optional[np.ndarray] = None
        self._rov_orientation_xyzw: Optional[Tuple[float, float, float, float]] = None
        self._last_rov_in_base: Optional[PoseStamped] = None
        self._last_rov_pose_in_world: Optional[PoseStamped] = None
        self._last_object_pose: Optional[PoseStamped] = None

        self._sub_cable = self.create_subscription(
            CableSensor,
            self._cable_sensor_topic,
            self._on_cable_sensor,
            10,
        )
        self._sub_rov_pose = self.create_subscription(
            PoseWithCovarianceStamped,
            self._rov_pose_topic,
            self._on_rov_pose,
            10,
        )
        self._pub_object_pose = self.create_publisher(PoseStamped, self._object_pose_topic, 10)
        self._pub_place_pose = self.create_publisher(PoseStamped, self._place_pose_topic, 10)
        self._pub_perception_state = self.create_publisher(PerceptionState, self._perception_state_topic, 10)
        if self._publish_tf:
            self._tf_broadcaster = TransformBroadcaster(self)
            self._tf_static_broadcaster = StaticTransformBroadcaster(self)
            self._publish_static_rov_to_base_link()
        else:
            self._tf_broadcaster = None
            self._tf_static_broadcaster = None

    def _publish_static_rov_to_base_link(self) -> None:
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "rov0"
        t.child_frame_id = self._output_frame_id
        t.transform.translation.x = float(self._t_arm_in_rov[0])
        t.transform.translation.y = float(self._t_arm_in_rov[1])
        t.transform.translation.z = float(self._t_arm_in_rov[2])
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self._tf_static_broadcaster.sendTransform(t)

    def _on_rov_pose(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        self._rov_position = np.array([p.x, p.y, p.z], dtype=float)
        self._rov_orientation_xyzw = (o.x, o.y, o.z, o.w)
        if self._publish_tf and self._tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = msg.header.stamp
            t.header.frame_id = self._world_frame_id
            t.child_frame_id = "rov0"
            t.transform.translation.x = p.x
            t.transform.translation.y = p.y
            t.transform.translation.z = p.z
            t.transform.rotation.x = o.x
            t.transform.rotation.y = o.y
            t.transform.rotation.z = o.z
            t.transform.rotation.w = o.w
            self._tf_broadcaster.sendTransform(t)
        rov_in_base = PoseStamped()
        rov_in_base.header.stamp = msg.header.stamp
        rov_in_base.header.frame_id = self._output_frame_id
        rov_in_base.pose.position.x = float(-self._t_arm_in_rov[0])
        rov_in_base.pose.position.y = float(-self._t_arm_in_rov[1])
        rov_in_base.pose.position.z = float(-self._t_arm_in_rov[2])
        rov_in_base.pose.orientation.x = 0.0
        rov_in_base.pose.orientation.y = 0.0
        rov_in_base.pose.orientation.z = 0.0
        rov_in_base.pose.orientation.w = 1.0
        self._last_rov_in_base = rov_in_base
        rov_in_world = PoseStamped()
        rov_in_world.header.stamp = msg.header.stamp
        rov_in_world.header.frame_id = self._world_frame_id
        rov_in_world.pose.position.x = p.x
        rov_in_world.pose.position.y = p.y
        rov_in_world.pose.position.z = p.z
        rov_in_world.pose.orientation.x = o.x
        rov_in_world.pose.orientation.y = o.y
        rov_in_world.pose.orientation.z = o.z
        rov_in_world.pose.orientation.w = o.w
        self._last_rov_pose_in_world = rov_in_world
        if self._last_object_pose is None:
            return
        self._publish_perception_state(msg.header.stamp)

    def _publish_perception_state(self, stamp) -> None:
        ps = PerceptionState()
        ps.header.stamp = stamp
        ps.header.frame_id = self._output_frame_id
        ps.object_pose = self._last_object_pose
        ps.rov_pose_in_base_link = self._last_rov_in_base if self._last_rov_in_base else PoseStamped()
        if ps.rov_pose_in_base_link.header.stamp.sec == 0 and ps.rov_pose_in_base_link.header.stamp.nanosec == 0:
            ps.rov_pose_in_base_link.header.stamp = stamp
            ps.rov_pose_in_base_link.header.frame_id = self._output_frame_id
        ps.rov_pose_in_world = self._last_rov_pose_in_world if self._last_rov_pose_in_world else PoseStamped()
        if ps.rov_pose_in_world.header.stamp.sec == 0 and ps.rov_pose_in_world.header.stamp.nanosec == 0:
            ps.rov_pose_in_world.header.stamp = stamp
            ps.rov_pose_in_world.header.frame_id = self._world_frame_id
        self._pub_perception_state.publish(ps)

    def _on_cable_sensor(self, msg: CableSensor) -> None:
        if msg.num_cables <= 0:
            return
        if self._rov_position is None or self._rov_orientation_xyzw is None:
            self.get_logger().warn(
                "cable_sensor_to_object_pose: 尚无 ROV 位姿，跳过本帧（需订阅 " + self._rov_pose_topic + "）",
                throttle_duration_sec=2.0,
            )
            return

        if msg.num_cables != 1:
            self.get_logger().warn(
                "cable_sensor_to_object_pose: 仅支持单缆绳 num_cables=1，当前=%u 跳过"
                % (msg.num_cables,),
                throttle_duration_sec=2.0,
            )
            return
        need = 3
        if len(msg.positions) < need or len(msg.directions) < need or len(msg.euler_angles) < need:
            return

        stamp = self.get_clock().now().to_msg()
        R_rov = _quat_to_rotation_matrix(*self._rov_orientation_xyzw) if self._rov_orientation_xyzw else np.eye(3)
        t_rov = self._rov_position if self._rov_position is not None else np.zeros(3)

        px = msg.positions[0]
        py = msg.positions[1]
        pz = msg.positions[2]
        dx = msg.directions[0]
        dy = msg.directions[1]
        dz = msg.directions[2]
        p_raw = np.array([px, py, pz], dtype=float)
        d_raw = np.array([dx, dy, dz], dtype=float)

        # positions -> world
        if self._cable_positions_frame == "world":
            p_world = p_raw
        elif self._cable_positions_frame == "rov" or self._cable_positions_frame == "com":
            p_world = R_rov @ p_raw + t_rov
        else:
            self.get_logger().warn(
                "cable_sensor_to_object_pose: cable_positions_frame=%s 不支持，使用 world 处理"
                % (self._cable_positions_frame,),
                throttle_duration_sec=2.0,
            )
            p_world = p_raw

        # direction -> world（用于归一化与可选用途；当前仅用于校验/未来扩展）
        if self._cable_direction_frame == "world":
            d_world = d_raw
        elif self._cable_direction_frame == "rov" or self._cable_direction_frame == "com":
            d_world = R_rov @ d_raw
        else:
            d_world = d_raw
        dn = np.linalg.norm(d_world)
        if dn < 1e-9:
            d_world = np.array([0.0, 0.0, 1.0])
        else:
            d_world = d_world / dn
        p_rov = R_rov.T @ (p_world - t_rov)
        p_base = p_rov - self._t_arm_in_rov + np.array(
            [self._offset_x, self._offset_y, self._offset_z], dtype=float
        )

        # 缆绳姿态：用轴向 direction 得到四元数，使圆柱体沿缆绳方向（与网页“平躺”一致）
        # MoveIt CYLINDER 沿局部 Z 轴，故 orientation 需满足 local Z = 缆绳方向
        d_base = R_rov.T @ d_world
        dn_base = np.linalg.norm(d_base)
        if dn_base < 1e-9:
            d_base = np.array([1.0, 0.0, 0.0])
        else:
            d_base = d_base / dn_base
        q_base = _quat_from_direction(float(d_base[0]), float(d_base[1]), float(d_base[2]))

        out = PoseStamped()
        out.header.stamp = stamp
        out.header.frame_id = self._output_frame_id
        out.pose.position.x = float(p_base[0])
        out.pose.position.y = float(p_base[1])
        out.pose.position.z = float(p_base[2])
        out.pose.orientation.x = q_base[0]
        out.pose.orientation.y = q_base[1]
        out.pose.orientation.z = q_base[2]
        out.pose.orientation.w = q_base[3]

        self._pub_object_pose.publish(out)
        self._pub_place_pose.publish(out)
        self._last_object_pose = out

        if self._publish_tf and self._tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self._output_frame_id
            t.child_frame_id = "cable"
            t.transform.translation.x = out.pose.position.x
            t.transform.translation.y = out.pose.position.y
            t.transform.translation.z = out.pose.position.z
            t.transform.rotation.x = out.pose.orientation.x
            t.transform.rotation.y = out.pose.orientation.y
            t.transform.rotation.z = out.pose.orientation.z
            t.transform.rotation.w = out.pose.orientation.w
            self._tf_broadcaster.sendTransform(t)

        self._publish_perception_state(stamp)
        if not hasattr(self, "_cable_logged"):
            self._cable_logged = True
            self.get_logger().info(
                "cable_sensor_to_object_pose: 已收到 CableSensor，发布 object_pose / place_pose（缆绳单目标）"
            )
        # self.get_logger().info(
        #     "cable_sensor_to_object_pose: 缆绳 base_link 位姿 x=%.3f y=%.3f z=%.3f（供 MTC 抓取目标）"
        #     % (float(p_base[0]), float(p_base[1]), float(p_base[2])),
        #     throttle_duration_sec=2.0,
        # )


def main(args=None):
    rclpy.init(args=args)
    node = CableSensorToObjectPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
