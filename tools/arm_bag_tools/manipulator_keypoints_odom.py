#!/usr/bin/env python3
"""
odom 系任务关键点：与 holoocean TargetSensor 桥接及 manipulator_task_keypoints_odom.yaml 同源。

供 run_episode_record.py 与同事路径（robotic_arm_cmd + frame_id=odom）一致地下发 keypoint。
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

try:
    import yaml
except ImportError as exc:
    raise ImportError("需要 PyYAML：pip install pyyaml 或通过 ROS 依赖安装") from exc

from geometry_msgs.msg import Pose

DEFAULT_KEYPOINT_FRAME = "odom"


def _quat_from_rotation_matrix(R: np.ndarray) -> Tuple[float, float, float, float]:
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0.0:
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


def _quat_to_rotation_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    return np.array(
        [
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
        ],
        dtype=float,
    )


def _rotation_matrix_side_grasp_from_direction(
    direction: np.ndarray,
    y_axis_hint: Optional[np.ndarray] = None,
) -> np.ndarray:
    a = np.asarray(direction, dtype=float).ravel()[:3]
    n = np.linalg.norm(a)
    if n < 1.0e-9:
        a = np.array([0.0, 0.0, 1.0], dtype=float)
    else:
        a = a / n
    if a[2] < 0.0:
        a = -a
    y = None
    if y_axis_hint is not None:
        y_hint = np.asarray(y_axis_hint, dtype=float).ravel()[:3]
        if np.linalg.norm(y_hint) > 1.0e-9:
            y_hint = y_hint / np.linalg.norm(y_hint)
            y_proj = y_hint - np.dot(y_hint, a) * a
            ny_proj = np.linalg.norm(y_proj)
            if ny_proj > 1.0e-9:
                y = y_proj / ny_proj
                if float(np.dot(y, y_hint)) < 0.0:
                    y = -y
    if y is None:
        ref = np.array([0.0, 0.0, 1.0], dtype=float)
        if abs(float(np.dot(a, ref))) > 0.95:
            ref = np.array([1.0, 0.0, 0.0], dtype=float)
        y = np.cross(a, ref)
        ny = np.linalg.norm(y)
        if ny < 1.0e-9:
            y = np.array([1.0, 0.0, 0.0], dtype=float) if abs(a[0]) < 0.9 else np.array([0.0, 1.0, 0.0], dtype=float)
            y = y - np.dot(y, a) * a
            y = y / np.linalg.norm(y)
        else:
            y = y / ny
    z = np.cross(y, a)
    z = z / np.linalg.norm(z)
    x = np.cross(y, z)
    x = x / np.linalg.norm(x)
    return np.column_stack((x, y, z))


def _pose_from_position_quat(
    position_xyz: Sequence[float],
    orientation_xyzw: Sequence[float],
) -> Pose:
    pose = Pose()
    pose.position.x = float(position_xyz[0])
    pose.position.y = float(position_xyz[1])
    pose.position.z = float(position_xyz[2])
    pose.orientation.x = float(orientation_xyzw[0])
    pose.orientation.y = float(orientation_xyzw[1])
    pose.orientation.z = float(orientation_xyzw[2])
    pose.orientation.w = float(orientation_xyzw[3])
    return pose


def load_keypoints_catalog(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError("catalog 根节点须为 mapping: %s" % path)
    frame = str(data.get("keypoint_frame", DEFAULT_KEYPOINT_FRAME)).strip() or DEFAULT_KEYPOINT_FRAME
    grasp_cfg = data.get("grasp") or {}
    offset = float(grasp_cfg.get("offset_along_direction_m", 0.122671))
    slots_raw = data.get("insert_slots") or []
    slots: List[Dict[str, Any]] = []
    for entry in slots_raw:
        if not isinstance(entry, dict):
            continue
        pos = entry.get("position_xyz")
        ori = entry.get("orientation_xyzw")
        if not pos or not ori or len(pos) != 3 or len(ori) != 4:
            continue
        slots.append(
            {
                "id": str(entry.get("id", "slot_%d" % (len(slots) + 1))),
                "position_xyz": [float(x) for x in pos],
                "orientation_xyzw": [float(x) for x in ori],
            }
        )
    if not slots:
        raise ValueError("catalog 未包含有效 insert_slots: %s" % path)
    return {
        "keypoint_frame": frame,
        "grasp_offset_along_direction_m": offset,
        "insert_slots": slots,
        "path": str(path),
    }


def get_insert_keypoint_odom(catalog: Dict[str, Any], insert_index: int) -> Tuple[Pose, str, str]:
    slots = catalog["insert_slots"]
    if insert_index < 0 or insert_index >= len(slots):
        raise IndexError("insert_index=%d 越界（共 %d 孔）" % (insert_index, len(slots)))
    slot = slots[insert_index]
    pose = _pose_from_position_quat(slot["position_xyz"], slot["orientation_xyzw"])
    return pose, catalog["keypoint_frame"], str(slot["id"])


def _target_sensor_vectors_at_index(
    positions: Sequence[float],
    directions: Sequence[float],
    grasp_index: int,
) -> Tuple[np.ndarray, np.ndarray]:
    i = grasp_index * 3
    if len(positions) < i + 3 or len(directions) < i + 3:
        raise ValueError("TargetSensor positions/directions 长度不足")
    p_world = np.array([positions[i], positions[i + 1], positions[i + 2]], dtype=float)
    d_world = np.array([directions[i], directions[i + 1], directions[i + 2]], dtype=float)
    dn = np.linalg.norm(d_world)
    if dn < 1.0e-9:
        d_world = np.array([0.0, 0.0, 1.0], dtype=float)
    else:
        d_world = d_world / dn
    return p_world, d_world


def compute_grasp_keypoint_arm_base_link_from_target_sensor(
    num_targets: int,
    positions: Sequence[float],
    directions: Sequence[float],
    grasp_index: int,
    grasp_offset_along_direction_m: float,
    rov_position_xyz: Sequence[float],
    rov_orientation_xyzw: Sequence[float],
    t_arm_in_rov: Sequence[float],
    position_offset_xyz: Sequence[float] = (0.0, 0.0, 0.0),
    last_side_grasp_y: Optional[np.ndarray] = None,
) -> Tuple[Pose, np.ndarray]:
    """
    与 holoocean target_sensor_to_object_pose 一致：侧向抓取系在 arm_base_link 下构造。
    """
    if num_targets <= 0:
        raise ValueError("TargetSensor num_targets<=0")
    if grasp_index < 0 or grasp_index >= num_targets:
        raise IndexError("grasp_index=%d 越界（num_targets=%d）" % (grasp_index, num_targets))
    p_world, d_world = _target_sensor_vectors_at_index(positions, directions, grasp_index)
    R_rov = _quat_to_rotation_matrix(
        float(rov_orientation_xyzw[0]),
        float(rov_orientation_xyzw[1]),
        float(rov_orientation_xyzw[2]),
        float(rov_orientation_xyzw[3]),
    )
    t_rov = np.array(rov_position_xyz, dtype=float)
    t_arm = np.array(t_arm_in_rov, dtype=float)
    offset = np.array(position_offset_xyz, dtype=float)
    p_rov = R_rov.T @ (p_world - t_rov)
    p_base = p_rov - t_arm + offset
    d_base = R_rov.T @ d_world
    d_base = d_base / max(np.linalg.norm(d_base), 1.0e-9)
    p_grasp_base = p_base + d_base * float(grasp_offset_along_direction_m)
    R_grasp_base = _rotation_matrix_side_grasp_from_direction(d_base, last_side_grasp_y)
    y_out = np.array(R_grasp_base[:, 1], dtype=float)
    qx, qy, qz, qw = _quat_from_rotation_matrix(R_grasp_base)
    pose = Pose()
    pose.position.x = float(p_grasp_base[0])
    pose.position.y = float(p_grasp_base[1])
    pose.position.z = float(p_grasp_base[2])
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose, y_out


def compute_grasp_keypoint_odom_from_target_sensor(
    num_targets: int,
    positions: Sequence[float],
    directions: Sequence[float],
    grasp_index: int,
    grasp_offset_along_direction_m: float,
    rov_position_xyz: Sequence[float],
    rov_orientation_xyzw: Sequence[float],
    t_arm_in_rov: Sequence[float],
    position_offset_xyz: Sequence[float] = (0.0, 0.0, 0.0),
    last_side_grasp_y: Optional[np.ndarray] = None,
) -> Tuple[Pose, str, np.ndarray]:
    """
    先在 arm_base_link 构造侧向抓取系，再经 ROV 位姿变到 odom/world（与 MTC odom→arm TF 互逆）。
    """
    pose_base, y_out = compute_grasp_keypoint_arm_base_link_from_target_sensor(
        num_targets,
        positions,
        directions,
        grasp_index,
        grasp_offset_along_direction_m,
        rov_position_xyz,
        rov_orientation_xyzw,
        t_arm_in_rov,
        position_offset_xyz,
        last_side_grasp_y,
    )
    R_rov = _quat_to_rotation_matrix(
        float(rov_orientation_xyzw[0]),
        float(rov_orientation_xyzw[1]),
        float(rov_orientation_xyzw[2]),
        float(rov_orientation_xyzw[3]),
    )
    t_rov = np.array(rov_position_xyz, dtype=float)
    t_arm = np.array(t_arm_in_rov, dtype=float)
    R_grasp_base = _quat_to_rotation_matrix(
        float(pose_base.orientation.x),
        float(pose_base.orientation.y),
        float(pose_base.orientation.z),
        float(pose_base.orientation.w),
    )
    p_rov = np.array(
        [
            float(pose_base.position.x) + t_arm[0],
            float(pose_base.position.y) + t_arm[1],
            float(pose_base.position.z) + t_arm[2],
        ],
        dtype=float,
    )
    p_odom = R_rov @ p_rov + t_rov
    R_odom = R_rov @ R_grasp_base
    qx, qy, qz, qw = _quat_from_rotation_matrix(R_odom)
    pose = Pose()
    pose.position.x = float(p_odom[0])
    pose.position.y = float(p_odom[1])
    pose.position.z = float(p_odom[2])
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose, DEFAULT_KEYPOINT_FRAME, y_out


def transform_pose_arm_base_link_to_odom(
    pose_base: Pose,
    rov_position_xyz: Sequence[float],
    rov_orientation_xyzw: Sequence[float],
    t_arm_in_rov: Sequence[float],
) -> Pose:
    """arm_base_link 位姿经 ROV 链变到 odom/world（与 MTC odom→arm TF 互逆）。"""
    R_rov = _quat_to_rotation_matrix(
        float(rov_orientation_xyzw[0]),
        float(rov_orientation_xyzw[1]),
        float(rov_orientation_xyzw[2]),
        float(rov_orientation_xyzw[3]),
    )
    t_rov = np.array(rov_position_xyz, dtype=float)
    t_arm = np.array(t_arm_in_rov, dtype=float)
    R_base = _quat_to_rotation_matrix(
        float(pose_base.orientation.x),
        float(pose_base.orientation.y),
        float(pose_base.orientation.z),
        float(pose_base.orientation.w),
    )
    p_rov = np.array(
        [
            float(pose_base.position.x) + t_arm[0],
            float(pose_base.position.y) + t_arm[1],
            float(pose_base.position.z) + t_arm[2],
        ],
        dtype=float,
    )
    p_odom = R_rov @ p_rov + t_rov
    R_odom = R_rov @ R_base
    qx, qy, qz, qw = _quat_from_rotation_matrix(R_odom)
    out = Pose()
    out.position.x = float(p_odom[0])
    out.position.y = float(p_odom[1])
    out.position.z = float(p_odom[2])
    out.orientation.x = qx
    out.orientation.y = qy
    out.orientation.z = qz
    out.orientation.w = qw
    return out


def compute_grasp_reach_arm_base_link_m(
    num_targets: int,
    positions: Sequence[float],
    directions: Sequence[float],
    grasp_index: int,
    grasp_offset_along_direction_m: float,
    rov_position_xyz: Sequence[float],
    rov_orientation_xyzw: Sequence[float],
    t_arm_in_rov: Sequence[float],
    position_offset_xyz: Sequence[float] = (0.0, 0.0, 0.0),
) -> float:
    """与 holoocean 桥一致：校验用 arm_base_link 下抓取点距离。"""
    if grasp_index < 0 or grasp_index >= num_targets:
        return float("inf")
    i = grasp_index * 3
    p_world = np.array([positions[i], positions[i + 1], positions[i + 2]], dtype=float)
    d_world = np.array([directions[i], directions[i + 1], directions[i + 2]], dtype=float)
    dn = np.linalg.norm(d_world)
    if dn < 1.0e-9:
        d_world = np.array([0.0, 0.0, 1.0], dtype=float)
    else:
        d_world = d_world / dn
    R_rov = _quat_to_rotation_matrix(
        float(rov_orientation_xyzw[0]),
        float(rov_orientation_xyzw[1]),
        float(rov_orientation_xyzw[2]),
        float(rov_orientation_xyzw[3]),
    )
    t_rov = np.array(rov_position_xyz, dtype=float)
    t_arm = np.array(t_arm_in_rov, dtype=float)
    offset = np.array(position_offset_xyz, dtype=float)
    p_rov = R_rov.T @ (p_world - t_rov)
    p_base = p_rov - t_arm + offset
    d_base = R_rov.T @ d_world
    d_base = d_base / max(np.linalg.norm(d_base), 1.0e-9)
    p_base = p_base + d_base * float(grasp_offset_along_direction_m)
    return float(np.linalg.norm(p_base))
