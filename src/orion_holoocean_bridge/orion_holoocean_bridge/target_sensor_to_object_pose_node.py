#!/usr/bin/env python3
"""
将 HoloOcean TargetSensor（世界系下 xyz + 圆柱轴方向）转换为 MTC 所需的 /object_pose（base_link 下）。
订阅 ROV 位姿（PoseSensor，世界系），将目标点从世界系变换到机械臂 base_link，并用 direction 构造物体姿态（无欧拉角时由方向向量推导）。
"""

import math
from typing import Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped, Vector3Stamped
from std_msgs.msg import Header
from holoocean_interfaces.msg import TargetSensor
from orion_mtc_msgs.msg import PerceptionState
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


# 机械臂基座在 ROV 系下平移 [m] 的默认值，仅平移无旋转（与 docs/tf_conversion.md 一致）；可由参数覆盖
DEFAULT_LEFT_ARM_BASE_IN_ROV = (1.55, 0.5653, -0.283628)
DEFAULT_RIGHT_ARM_BASE_IN_ROV = (1.55, -0.5653, -0.283628)


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


def _rotation_matrix_from_direction(direction: np.ndarray) -> np.ndarray:
    """
    由圆柱轴方向向量构造物体在世界系下的旋转矩阵（物体 Z 轴 = direction）。
    direction 应为单位向量或会被归一化。
    """
    z = np.asarray(direction, dtype=float).ravel()[:3]
    n = np.linalg.norm(z)
    if n < 1e-9:
        z = np.array([0.0, 0.0, 1.0])
    else:
        z = z / n
    world_up = np.array([0.0, 0.0, 1.0])
    x = np.cross(world_up, z)
    nx = np.linalg.norm(x)
    if nx < 1e-9:
        x = np.array([1.0, 0.0, 0.0]) if abs(z[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
        x = x - np.dot(x, z) * z
        x = x / np.linalg.norm(x)
    else:
        x = x / nx
    y = np.cross(z, x)
    R = np.column_stack((x, y, z))
    return R


def _rotation_matrix_side_grasp_from_direction(direction: np.ndarray) -> np.ndarray:
    """
    由圆柱轴方向构造侧向抓取坐标系（base_link 下）的旋转矩阵。
    约定：y = 夹爪闭合方向（垂直于圆柱轴），z = 末端接近方向（垂直于圆柱轴，从外侧指向物体）。
    这样末端从 z 方向接近、夹爪沿 y 方向闭合，从圆柱侧面包夹。
    direction 为圆柱轴单位向量（在 base_link 下）；会做归一化与符号统一（a.z >= 0）。
    """
    a = np.asarray(direction, dtype=float).ravel()[:3]
    n = np.linalg.norm(a)
    if n < 1e-9:
        a = np.array([0.0, 0.0, 1.0])
    else:
        a = a / n
    if a[2] < 0.0:
        a = -a
    ref = np.array([0.0, 0.0, 1.0])
    if abs(float(np.dot(a, ref))) > 0.95:
        ref = np.array([1.0, 0.0, 0.0])
    y = np.cross(a, ref)
    ny = np.linalg.norm(y)
    if ny < 1e-9:
        y = np.array([1.0, 0.0, 0.0]) if abs(a[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
        y = y - np.dot(y, a) * a
        y = y / np.linalg.norm(y)
    else:
        y = y / ny
    z = np.cross(y, a)
    z = z / np.linalg.norm(z)
    x = np.cross(y, z)
    x = x / np.linalg.norm(x)
    R = np.column_stack((x, y, z))
    return R


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


class TargetSensorToObjectPoseNode(Node):
    """
    订阅 TargetSensor（世界系 xyz + direction）和 ROV 位姿，将选定目标变换到 base_link 并发布 /object_pose。
    姿态由 direction（圆柱轴）推导，无欧拉角输入。
    """

    def __init__(self) -> None:
        super().__init__("target_sensor_to_object_pose")
        self.declare_parameter("target_sensor_topic", "/holoocean/rov0/TargetSensor")
        self.declare_parameter("rov_pose_topic", "/holoocean/rov0/PoseSensor")
        self.declare_parameter("object_pose_topic", "object_pose")
        self.declare_parameter("world_frame_id", "map")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("perception_state_topic", "perception_state")
        self.declare_parameter("output_frame_id", "base_link")
        self.declare_parameter("object_axis_topic", "object_axis")
        self.declare_parameter("target_index", 1)
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
        self.declare_parameter("use_pose_sensor_stamp_for_rov_tf", False)

        self._target_sensor_topic = self.get_parameter("target_sensor_topic").get_parameter_value().string_value
        self._rov_pose_topic = self.get_parameter("rov_pose_topic").get_parameter_value().string_value
        self._object_pose_topic = self.get_parameter("object_pose_topic").get_parameter_value().string_value
        self._world_frame_id = self.get_parameter("world_frame_id").get_parameter_value().string_value
        self._publish_tf = self.get_parameter("publish_tf").get_parameter_value().bool_value
        self._perception_state_topic = self.get_parameter("perception_state_topic").get_parameter_value().string_value
        self._output_frame_id = self.get_parameter("output_frame_id").get_parameter_value().string_value
        self._object_axis_topic = self.get_parameter("object_axis_topic").get_parameter_value().string_value
        self._target_index = self.get_parameter("target_index").get_parameter_value().integer_value
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
        self._use_pose_sensor_stamp_for_rov_tf = (
            self.get_parameter("use_pose_sensor_stamp_for_rov_tf").get_parameter_value().bool_value
        )
        self._rov_position: Optional[np.ndarray] = None
        self._rov_orientation_xyzw: Optional[Tuple[float, float, float, float]] = None
        self._last_rov_in_base: Optional[PoseStamped] = None
        self._last_rov_pose_in_world: Optional[PoseStamped] = None
        self._last_object_pose: Optional[PoseStamped] = None

        self._sub_target = self.create_subscription(
            TargetSensor,
            self._target_sensor_topic,
            self._on_target_sensor,
            10,
        )
        self._sub_rov_pose = self.create_subscription(
            PoseWithCovarianceStamped,
            self._rov_pose_topic,
            self._on_rov_pose,
            10,
        )
        self._pub_pose = self.create_publisher(PoseStamped, self._object_pose_topic, 10)
        self._pub_axis = self.create_publisher(Vector3Stamped, self._object_axis_topic, 10)
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
        """ROV 世界系位姿（PoseSensor），用于 world→ROV→base_link 变换。"""
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        self._rov_position = np.array([p.x, p.y, p.z], dtype=float)
        self._rov_orientation_xyzw = (o.x, o.y, o.z, o.w)
        if self._use_pose_sensor_stamp_for_rov_tf:
            stamp = msg.header.stamp
        else:
            stamp = self.get_clock().now().to_msg()
        if self._publish_tf and self._tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = stamp
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
        rov_in_base.header.stamp = stamp
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
        rov_in_world.header.stamp = stamp
        rov_in_world.header.frame_id = self._world_frame_id
        rov_in_world.pose.position.x = p.x
        rov_in_world.pose.position.y = p.y
        rov_in_world.pose.position.z = p.z
        rov_in_world.pose.orientation.x = o.x
        rov_in_world.pose.orientation.y = o.y
        rov_in_world.pose.orientation.z = o.z
        rov_in_world.pose.orientation.w = o.w
        self._last_rov_pose_in_world = rov_in_world
        # 仅在已有物体/目标数据时发布感知状态，避免网页收到“空 object_pose”与“有数据”交替导致 0↔有数据 闪烁
        if self._last_object_pose is None:
            return
        ps = PerceptionState()
        ps.header.stamp = stamp
        ps.header.frame_id = self._output_frame_id
        ps.object_pose = self._last_object_pose
        ps.rov_pose_in_base_link = rov_in_base
        ps.rov_pose_in_world = rov_in_world
        self._pub_perception_state.publish(ps)
        if not hasattr(self, "_rov_pose_logged"):
            self._rov_pose_logged = True
            self.get_logger().info(
                "target_sensor_to_object_pose: 已收到 ROV 位姿（" + self._rov_pose_topic + "），开始发布 object_pose 与 TF"
            )

    def _on_target_sensor(self, msg: TargetSensor) -> None:
        if self._rov_position is None or self._rov_orientation_xyzw is None:
            self.get_logger().warn(
                "target_sensor_to_object_pose: 尚无 ROV 位姿，跳过本帧（需订阅 "
                + self._rov_pose_topic + " 且 HoloOcean 有发布）",
                throttle_duration_sec=2.0,
            )
            return
        n = msg.num_targets
        if n <= 0:
            return

        R_rov = _quat_to_rotation_matrix(*self._rov_orientation_xyzw)
        t_rov = self._rov_position
        stamp = self.get_clock().now().to_msg()

        if self._publish_tf and self._tf_broadcaster is not None:
            for k in range(n):
                i = k * 3
                px = msg.positions[i]
                py = msg.positions[i + 1]
                pz = msg.positions[i + 2]
                t = TransformStamped()
                t.header.stamp = stamp
                t.header.frame_id = self._world_frame_id
                t.child_frame_id = "target_{}".format(k)
                t.transform.translation.x = float(px)
                t.transform.translation.y = float(py)
                t.transform.translation.z = float(pz)
                t.transform.rotation.x = 0.0
                t.transform.rotation.y = 0.0
                t.transform.rotation.z = 0.0
                t.transform.rotation.w = 1.0
                self._tf_broadcaster.sendTransform(t)

        # 发布多目标集合（base_link），供 MTC 目标选择 + 抓取候选
        positions_base = []
        directions_base = []
        for k in range(n):
            i = k * 3
            px = msg.positions[i]
            py = msg.positions[i + 1]
            pz = msg.positions[i + 2]
            dx = msg.directions[i]
            dy = msg.directions[i + 1]
            dz = msg.directions[i + 2]
            p_world = np.array([px, py, pz], dtype=float)
            d_world = np.array([dx, dy, dz], dtype=float)
            dn = np.linalg.norm(d_world)
            if dn < 1e-9:
                d_world = np.array([0.0, 0.0, 1.0])
            else:
                d_world = d_world / dn
            p_rov = R_rov.T @ (p_world - t_rov)
            p_base = p_rov - self._t_arm_in_rov + np.array(
                [self._offset_x, self._offset_y, self._offset_z], dtype=float
            )
            d_base = R_rov.T @ d_world
            d_base = d_base / np.linalg.norm(d_base)
            positions_base.extend([float(p_base[0]), float(p_base[1]), float(p_base[2])])
            directions_base.extend([float(d_base[0]), float(d_base[1]), float(d_base[2])])

        # 单目标 object_pose（选定 target_index）：位置为物体中心，姿态为侧向抓取系（y=闭合方向，z=接近方向，均垂直于圆柱轴）
        idx = max(0, min(self._target_index, n - 1))
        i = idx * 3
        px = msg.positions[i]
        py = msg.positions[i + 1]
        pz = msg.positions[i + 2]
        dx = msg.directions[i]
        dy = msg.directions[i + 1]
        dz = msg.directions[i + 2]
        p_world = np.array([px, py, pz], dtype=float)
        d_world = np.array([dx, dy, dz], dtype=float)
        dn = np.linalg.norm(d_world)
        if dn < 1e-9:
            d_world = np.array([0.0, 0.0, 1.0])
        else:
            d_world = d_world / dn
        p_rov = R_rov.T @ (p_world - t_rov)
        p_base = p_rov - self._t_arm_in_rov + np.array(
            [self._offset_x, self._offset_y, self._offset_z], dtype=float
        )
        d_base = R_rov.T @ d_world
        d_base = d_base / np.linalg.norm(d_base)
        R_grasp_base = _rotation_matrix_side_grasp_from_direction(d_base)
        q_grasp = _quat_from_rotation_matrix(R_grasp_base)
        out = PoseStamped()
        out.header.stamp = stamp
        out.header.frame_id = self._output_frame_id
        out.pose.position.x = float(p_base[0])
        out.pose.position.y = float(p_base[1])
        out.pose.position.z = float(p_base[2])
        out.pose.orientation.x = q_grasp[0]
        out.pose.orientation.y = q_grasp[1]
        out.pose.orientation.z = q_grasp[2]
        out.pose.orientation.w = q_grasp[3]
        self._pub_pose.publish(out)
        self._last_object_pose = out
        axis_msg = Vector3Stamped()
        axis_msg.header.stamp = stamp
        axis_msg.header.frame_id = self._output_frame_id
        axis_msg.vector.x = float(d_base[0])
        axis_msg.vector.y = float(d_base[1])
        axis_msg.vector.z = float(d_base[2])
        self._pub_axis.publish(axis_msg)

        # 感知状态：物体位姿 + ROV 世界系/基座系位姿，单话题供网页显示
        ps = PerceptionState()
        ps.header.stamp = stamp
        ps.header.frame_id = self._output_frame_id
        ps.object_pose = out
        ps.object_axis_direction.x = float(d_base[0])
        ps.object_axis_direction.y = float(d_base[1])
        ps.object_axis_direction.z = float(d_base[2])
        ps.object_confidence = 1.0
        ps.rov_pose_in_base_link = self._last_rov_in_base if self._last_rov_in_base is not None else PoseStamped()
        if ps.rov_pose_in_base_link.header.stamp.sec == 0 and ps.rov_pose_in_base_link.header.stamp.nanosec == 0:
            ps.rov_pose_in_base_link.header.stamp = stamp
            ps.rov_pose_in_base_link.header.frame_id = self._output_frame_id
        ps.rov_pose_in_world = self._last_rov_pose_in_world if self._last_rov_pose_in_world is not None else PoseStamped()
        if ps.rov_pose_in_world.header.stamp.sec == 0 and ps.rov_pose_in_world.header.stamp.nanosec == 0:
            ps.rov_pose_in_world.header.stamp = stamp
            ps.rov_pose_in_world.header.frame_id = self._world_frame_id
        self._pub_perception_state.publish(ps)


def main(args=None):
    rclpy.init(args=args)
    node = TargetSensorToObjectPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
