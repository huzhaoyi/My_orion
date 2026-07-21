#!/usr/bin/env python3
"""
发布插孔段「插头相对孔」位姿，供 world model 构造 s_t / s_{t+1}。

激活窗口：TARGET_INSERT 的 move to pre-insert ENTER 起，至 open hand ENTER 前。
孔系来自 target_insert_holes；插头系来自 /manipulator/tf 的 peg_tip。
"""

from __future__ import annotations

import math
from typing import Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sealien_ctrlpilot_manipulator_orion_mtc_msgs.msg import InsertRelativeState, TargetSet, TaskStage
from tf2_ros import Buffer, TransformListener

DEG_TO_RAD = math.pi / 180.0


def _normalize(vec: np.ndarray, fallback: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(vec))
    if not math.isfinite(n) or n < 1e-9:
        return fallback.copy()
    return vec / n


def _quat_to_rot(x: float, y: float, z: float, w: float) -> np.ndarray:
    q = np.array([x, y, z, w], dtype=float)
    n = float(np.linalg.norm(q))
    if n < 1e-9:
        return np.eye(3, dtype=float)
    q /= n
    rot = np.array(
        [
            [1.0 - 2.0 * (q[1] ** 2 + q[2] ** 2), 2.0 * (q[0] * q[1] - q[2] * q[3]), 2.0 * (q[0] * q[2] + q[1] * q[3])],
            [2.0 * (q[0] * q[1] + q[2] * q[3]), 1.0 - 2.0 * (q[0] ** 2 + q[2] ** 2), 2.0 * (q[1] * q[2] - q[0] * q[3])],
            [2.0 * (q[0] * q[2] - q[1] * q[3]), 2.0 * (q[1] * q[2] + q[0] * q[3]), 1.0 - 2.0 * (q[0] ** 2 + q[1] ** 2)],
        ],
        dtype=float,
    )
    return rot


def _rot_to_quat(rot: np.ndarray) -> Tuple[float, float, float, float]:
    trace = float(rot[0, 0] + rot[1, 1] + rot[2, 2])
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (rot[2, 1] - rot[1, 2]) / s
        y = (rot[0, 2] - rot[2, 0]) / s
        z = (rot[1, 0] - rot[0, 1]) / s
    elif rot[0, 0] > rot[1, 1] and rot[0, 0] > rot[2, 2]:
        s = math.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2]) * 2.0
        w = (rot[2, 1] - rot[1, 2]) / s
        x = 0.25 * s
        y = (rot[0, 1] + rot[1, 0]) / s
        z = (rot[0, 2] + rot[2, 0]) / s
    elif rot[1, 1] > rot[2, 2]:
        s = math.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2]) * 2.0
        w = (rot[0, 2] - rot[2, 0]) / s
        x = (rot[0, 1] + rot[1, 0]) / s
        y = 0.25 * s
        z = (rot[1, 2] + rot[2, 1]) / s
    else:
        s = math.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1]) * 2.0
        w = (rot[1, 0] - rot[0, 1]) / s
        x = (rot[0, 2] + rot[2, 0]) / s
        y = (rot[1, 2] + rot[2, 1]) / s
        z = 0.25 * s
    return float(x), float(y), float(z), float(w)


def insert_axis_from_hole_pose(pose: Pose, axis_local: np.ndarray) -> np.ndarray:
    rot = _quat_to_rot(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    )
    axis = rot @ axis_local
    return _normalize(axis, axis_local)


def compute_relative_state(
    hole_pose: Pose,
    peg_pos: np.ndarray,
    peg_rot: np.ndarray,
    axis_local: np.ndarray,
    rod_axis_tcp: np.ndarray,
) -> Tuple[float, float, float, Pose]:
    hole_pos = np.array(
        [hole_pose.position.x, hole_pose.position.y, hole_pose.position.z],
        dtype=float,
    )
    hole_rot = _quat_to_rot(
        hole_pose.orientation.x,
        hole_pose.orientation.y,
        hole_pose.orientation.z,
        hole_pose.orientation.w,
    )
    insert_axis = insert_axis_from_hole_pose(hole_pose, axis_local)
    delta = peg_pos - hole_pos
    e_ax = float(delta.dot(insert_axis))
    e_lat = float(np.linalg.norm(delta - e_ax * insert_axis))

    rod_axis = _normalize(peg_rot @ rod_axis_tcp, insert_axis)
    dot_val = float(np.clip(rod_axis.dot(insert_axis), -1.0, 1.0))
    angle_err = float(math.acos(dot_val))

    hole_rot_t = hole_rot.T
    rel_pos = hole_rot_t @ (peg_pos - hole_pos)
    rel_rot = hole_rot_t @ peg_rot
    qx, qy, qz, qw = _rot_to_quat(rel_rot)

    rel = Pose()
    rel.position.x = float(rel_pos[0])
    rel.position.y = float(rel_pos[1])
    rel.position.z = float(rel_pos[2])
    rel.orientation.x = qx
    rel.orientation.y = qy
    rel.orientation.z = qz
    rel.orientation.w = qw
    return e_lat, e_ax, angle_err, rel


class InsertRelativeStateNode(Node):
    def __init__(self) -> None:
        super().__init__("insert_relative_state")

        self.declare_parameter("output_topic", "/manipulator/insert_rel_state")
        self.declare_parameter("target_insert_holes_topic", "/manipulator/target_insert_holes")
        self.declare_parameter("target_set_topic", "/manipulator/target_set")
        self.declare_parameter("task_stage_topic", "/manipulator/task_stage")
        self.declare_parameter("tf_topic", "/manipulator/tf")
        self.declare_parameter("tf_static_topic", "/manipulator/tf_static")
        self.declare_parameter("base_frame", "arm_base_link")
        self.declare_parameter("peg_frame", "peg_tip")
        self.declare_parameter("hole_index", 0)
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("start_stage_name", "move to pre-insert")
        self.declare_parameter("end_stage_name", "open hand")
        self.declare_parameter(
            "insert_axis_local_xyz",
            [0.0, -1.0, 0.0],
        )
        self.declare_parameter(
            "peg_rod_axis_tcp_xyz",
            [0.0, 1.0, 0.0],
        )

        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        holes_topic = self.get_parameter("target_insert_holes_topic").get_parameter_value().string_value
        target_set_topic = self.get_parameter("target_set_topic").get_parameter_value().string_value
        task_stage_topic = self.get_parameter("task_stage_topic").get_parameter_value().string_value
        tf_topic = self.get_parameter("tf_topic").get_parameter_value().string_value
        tf_static_topic = self.get_parameter("tf_static_topic").get_parameter_value().string_value

        self._base_frame = self.get_parameter("base_frame").get_parameter_value().string_value
        self._peg_frame = self.get_parameter("peg_frame").get_parameter_value().string_value
        self._hole_index = int(self.get_parameter("hole_index").get_parameter_value().integer_value)
        self._start_stage = self.get_parameter("start_stage_name").get_parameter_value().string_value
        self._end_stage = self.get_parameter("end_stage_name").get_parameter_value().string_value

        axis_param = self.get_parameter("insert_axis_local_xyz").value
        rod_param = self.get_parameter("peg_rod_axis_tcp_xyz").value
        self._axis_local = _normalize(np.array(axis_param, dtype=float), np.array([0.0, -1.0, 0.0]))
        self._rod_axis_tcp = _normalize(np.array(rod_param, dtype=float), np.array([0.0, 1.0, 0.0]))

        rate_hz = self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        if rate_hz < 1.0:
            rate_hz = 50.0

        self._holes: Optional[PoseArray] = None
        self._latch_value = 0.0
        self._insert_window_active = False
        self._current_stage_name = ""

        self._tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_subscription(PoseArray, holes_topic, self._on_holes, qos_profile_sensor_data)
        self.create_subscription(TargetSet, target_set_topic, self._on_target_set, qos_profile_sensor_data)
        self.create_subscription(TaskStage, task_stage_topic, self._on_task_stage, 10)
        self._pub = self.create_publisher(InsertRelativeState, output_topic, 10)
        self._timer = self.create_timer(1.0 / float(rate_hz), self._on_timer)

        self.get_logger().info(
            "insert_relative_state: pub=%s holes=%s peg=%s start_stage=%s"
            % (output_topic, holes_topic, self._peg_frame, self._start_stage)
        )

    def _on_holes(self, msg: PoseArray) -> None:
        self._holes = msg

    def _on_target_set(self, msg: TargetSet) -> None:
        if self._hole_index < 0 or self._hole_index >= len(msg.latches):
            self._latch_value = 0.0
            return
        self._latch_value = float(msg.latches[self._hole_index])

    def _on_task_stage(self, msg: TaskStage) -> None:
        if msg.task_type != "TARGET_INSERT":
            return
        self._current_stage_name = msg.stage_name
        if msg.stage_name == self._start_stage and msg.stage_state == "ENTER":
            self._insert_window_active = True
        if self._insert_window_active and msg.stage_name == self._end_stage and msg.stage_state == "ENTER":
            self._insert_window_active = False

    def _lookup_peg_in_base(self) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        try:
            tf = self._tf_buffer.lookup_transform(
                self._base_frame,
                self._peg_frame,
                rclpy.time.Time(),
            )
        except Exception:
            return None
        pos = np.array(
            [
                tf.transform.translation.x,
                tf.transform.translation.y,
                tf.transform.translation.z,
            ],
            dtype=float,
        )
        rot = _quat_to_rot(
            tf.transform.rotation.x,
            tf.transform.rotation.y,
            tf.transform.rotation.z,
            tf.transform.rotation.w,
        )
        return pos, rot

    def _on_timer(self) -> None:
        msg = InsertRelativeState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.active = self._insert_window_active
        msg.task_stage = self._current_stage_name
        msg.hole_index = int(self._hole_index)
        msg.latch = float(self._latch_value)
        msg.hole_frame_id = "hole_slot_%d" % self._hole_index

        if (
            not self._insert_window_active
            or self._holes is None
            or len(self._holes.poses) <= self._hole_index
        ):
            msg.lateral_error_m = float("nan")
            msg.axial_error_m = float("nan")
            msg.angle_error_rad = float("nan")
            self._pub.publish(msg)
            return

        peg = self._lookup_peg_in_base()
        if peg is None:
            msg.lateral_error_m = float("nan")
            msg.axial_error_m = float("nan")
            msg.angle_error_rad = float("nan")
            self._pub.publish(msg)
            return

        peg_pos, peg_rot = peg
        hole_pose = self._holes.poses[self._hole_index]
        e_lat, e_ax, angle_err, rel_pose = compute_relative_state(
            hole_pose,
            peg_pos,
            peg_rot,
            self._axis_local,
            self._rod_axis_tcp,
        )
        msg.lateral_error_m = e_lat
        msg.axial_error_m = e_ax
        msg.angle_error_rad = angle_err
        msg.relative_pose = rel_pose
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InsertRelativeStateNode()
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
