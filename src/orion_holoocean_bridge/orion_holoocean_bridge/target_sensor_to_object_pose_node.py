#!/usr/bin/env python3
"""
将 HoloOcean TargetSensor（世界系下 xyz + 圆柱轴方向）转换为 MTC 所需的 /object_pose（base_link 下）。
订阅 ROV 位姿（DynamicsSensorOdom），将目标点从世界系变换到机械臂 base_link，并用 direction 构造物体姿态（无欧拉角时由方向向量推导）。
"""

import math
from typing import Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from holoocean_interfaces.msg import TargetSensor


# 机械臂基座在 ROV 系下平移 [m]，仅平移无旋转（与 docs/tf_conversion.md 一致）
LEFT_ARM_BASE_IN_ROV = (1.55, 0.5653, -0.283628)
RIGHT_ARM_BASE_IN_ROV = (1.55, -0.5653, -0.283628)


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
        self.declare_parameter("rov_odom_topic", "/holoocean/rov0/DynamicsSensorOdom")
        self.declare_parameter("object_pose_topic", "object_pose")
        self.declare_parameter("output_frame_id", "base_link")
        self.declare_parameter("target_index", 2)
        self.declare_parameter("use_left_arm", True)

        self._target_sensor_topic = self.get_parameter("target_sensor_topic").get_parameter_value().string_value
        self._rov_odom_topic = self.get_parameter("rov_odom_topic").get_parameter_value().string_value
        self._object_pose_topic = self.get_parameter("object_pose_topic").get_parameter_value().string_value
        self._output_frame_id = self.get_parameter("output_frame_id").get_parameter_value().string_value
        self._target_index = self.get_parameter("target_index").get_parameter_value().integer_value
        self._use_left_arm = self.get_parameter("use_left_arm").get_parameter_value().bool_value

        self._t_arm_in_rov = np.array(
            LEFT_ARM_BASE_IN_ROV if self._use_left_arm else RIGHT_ARM_BASE_IN_ROV,
            dtype=float,
        )
        self._rov_position: Optional[np.ndarray] = None
        self._rov_orientation_xyzw: Optional[Tuple[float, float, float, float]] = None

        self._sub_target = self.create_subscription(
            TargetSensor,
            self._target_sensor_topic,
            self._on_target_sensor,
            10,
        )
        self._sub_odom = self.create_subscription(
            Odometry,
            self._rov_odom_topic,
            self._on_rov_odom,
            10,
        )
        self._pub_pose = self.create_publisher(PoseStamped, self._object_pose_topic, 10)

        self.get_logger().info(
            "target_sensor_to_object_pose: target_topic=%s rov_odom=%s object_pose=%s frame=%s target_index=%d arm=%s"
            % (
                self._target_sensor_topic,
                self._rov_odom_topic,
                self._object_pose_topic,
                self._output_frame_id,
                self._target_index,
                "left" if self._use_left_arm else "right",
            )
        )

    def _on_rov_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        self._rov_position = np.array([p.x, p.y, p.z], dtype=float)
        self._rov_orientation_xyzw = (o.x, o.y, o.z, o.w)

    def _on_target_sensor(self, msg: TargetSensor) -> None:
        if self._rov_position is None or self._rov_orientation_xyzw is None:
            self.get_logger().debug(
                "target_sensor_to_object_pose: 尚无 ROV 位姿，跳过本帧",
                throttle_duration_sec=2.0,
            )
            return
        n = msg.num_targets
        if n <= 0:
            return
        idx = max(0, min(self._target_index, n - 1))
        i = idx * 3
        px = msg.positions[i]
        py = msg.positions[i + 1]
        pz = msg.positions[i + 2]
        dx = msg.directions[i]
        dy = msg.directions[i + 1]
        dz = msg.directions[i + 2]

        p_world = np.array([px, py, pz], dtype=float)
        direction = np.array([dx, dy, dz], dtype=float)

        R_rov = _quat_to_rotation_matrix(*self._rov_orientation_xyzw)
        t_rov = self._rov_position
        p_rov = R_rov.T @ (p_world - t_rov)
        p_base = p_rov - self._t_arm_in_rov

        R_obj_world = _rotation_matrix_from_direction(direction)
        R_obj_rov = R_rov.T @ R_obj_world
        q_obj_rov = _quat_from_rotation_matrix(R_obj_rov)

        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self._output_frame_id
        out.pose.position.x = float(p_base[0])
        out.pose.position.y = float(p_base[1])
        out.pose.position.z = float(p_base[2])
        out.pose.orientation.x = q_obj_rov[0]
        out.pose.orientation.y = q_obj_rov[1]
        out.pose.orientation.z = q_obj_rov[2]
        out.pose.orientation.w = q_obj_rov[3]
        self._pub_pose.publish(out)
        self.get_logger().info(
            "target_sensor_to_object_pose: 已发布 target[%d] 在 base_link 下 (%.3f, %.3f, %.3f)"
            % (idx, p_base[0], p_base[1], p_base[2]),
            throttle_duration_sec=1.0,
        )


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
