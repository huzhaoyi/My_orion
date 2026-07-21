#!/usr/bin/env python3
"""
从 rosbag2 抽取插孔段 (s_t, a_t, s_{t+1}) 转移数据，供 world model 训练使用。

状态 s：InsertRelativeState（几何误差 + 孔系相对位姿），或从 target_insert_holes + TF 离线重算。
动作 a：/holoocean/command/agent/arm（AgentCommand，默认左臂 7 维 deg）。
窗口：insert_rel_state.active=true；若无该话题则按 task_stage 回退
      （move to pre-insert ENTER → open hand ENTER 前）。

依赖（需 source ROS2 + 工作空间）:
  holoocean_interfaces, sealien_ctrlpilot_manipulator_orion_mtc_msgs, rosbag2_py, rclpy, numpy

用法:
  ./extract_insert_trajectories.py bags/ep_target_001
  ./extract_insert_trajectories.py --jobs-jsonl jobs.jsonl --bags-root bags
  ./extract_insert_trajectories.py bags/ep_target_001 --merge-out dataset/all_insert.npz
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

try:
    import numpy as np
except ImportError:
    print("请安装 numpy: pip install numpy", file=sys.stderr)
    sys.exit(1)

try:
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
except ImportError:
    print("请 source ROS2（需要 rosbag2_py）", file=sys.stderr)
    sys.exit(1)

try:
    from rclpy.serialization import deserialize_message
except ImportError:
    print("请 source ROS2（需要 rclpy）", file=sys.stderr)
    sys.exit(1)

try:
    from geometry_msgs.msg import Pose, PoseArray, TransformStamped
    from holoocean_interfaces.msg import AgentCommand
    from sealien_ctrlpilot_manipulator_orion_mtc_msgs.msg import InsertRelativeState, TaskStage
    from tf2_msgs.msg import TFMessage
except ImportError as exc:
    print(
        "无法导入消息类型: %s\n请 source ROS2 与工作空间（holoocean_interfaces、"
        "sealien_ctrlpilot_manipulator_orion_mtc_msgs）。" % exc,
        file=sys.stderr,
    )
    sys.exit(1)

try:
    from sealien_ctrlpilot_manipulator_orion_holoocean_bridge.insert_relative_state_node import (
        compute_relative_state,
    )
except ImportError:
    compute_relative_state = None  # type: ignore


TOPIC_INSERT_STATE = "/manipulator/insert_rel_state"
TOPIC_AGENT_CMD = "/holoocean/command/agent/arm"
TOPIC_TASK_STAGE = "/manipulator/task_stage"
TOPIC_INSERT_HOLES = "/manipulator/target_insert_holes"
TOPIC_TF = "/manipulator/tf"

STAGE_START = "move to pre-insert"
STAGE_END = "open hand"
TASK_TYPE_INSERT = "TARGET_INSERT"

DEFAULT_AXIS_LOCAL = np.array([0.0, -1.0, 0.0], dtype=float)
DEFAULT_ROD_AXIS_TCP = np.array([0.0, 1.0, 0.0], dtype=float)

STATE_NAMES_ERRORS = ["lateral_error_m", "axial_error_m", "angle_error_rad"]
STATE_NAMES_POSE = [
    "rel_x",
    "rel_y",
    "rel_z",
    "rel_qx",
    "rel_qy",
    "rel_qz",
    "rel_qw",
]
ACTION_NAMES_LEFT7 = [
    "cmd_j1_deg",
    "cmd_j2_deg",
    "cmd_j3_deg",
    "cmd_j4_deg",
    "cmd_j5_deg",
    "cmd_j6_deg",
    "cmd_grip_deg",
]


@dataclass
class InsertSample:
    t_abs_s: float
    active: bool
    task_stage: str
    lateral_error_m: float
    axial_error_m: float
    angle_error_rad: float
    rel_pose: np.ndarray
    hole_index: int
    latch: float
    source: str

    @property
    def errors(self) -> np.ndarray:
        return np.array(
            [self.lateral_error_m, self.axial_error_m, self.angle_error_rad],
            dtype=float,
        )

    @property
    def pose7(self) -> np.ndarray:
        return self.rel_pose.copy()

    def is_valid_geometry(self) -> bool:
        vals = np.concatenate([self.errors, self.rel_pose])
        return bool(np.all(np.isfinite(vals)))


@dataclass
class EpisodeExtractResult:
    episode_id: str
    bag_dir: str
    outcome: str
    window_source: str
    num_state_samples: int
    num_transitions: int
    latch_final: float
    duration_s: float
    max_align_dt_s: float
    dropped_align: int
    output_dir: str
    extra: Dict[str, Any] = field(default_factory=dict)


def _open_reader(bag_dir: Path) -> SequentialReader:
    storage_options = StorageOptions(uri=str(bag_dir.resolve()), storage_id="sqlite3")
    converter_options = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    return reader


def _stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _header_stamp_sec(msg) -> float:
    if hasattr(msg, "header") and msg.header is not None:
        return _stamp_to_sec(msg.header.stamp)
    return float("nan")


def _pose_to_array(pose: Pose) -> np.ndarray:
    return np.array(
        [
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ],
        dtype=float,
    )


def _parse_agent_command(msg: AgentCommand, action_dims: str) -> np.ndarray:
    cmd = list(msg.command) if msg.command else []
    while len(cmd) < 14:
        cmd.append(0.0)
    if action_dims == "full14":
        return np.array([float(x) for x in cmd[:14]], dtype=float)
    return np.array([float(x) for x in cmd[0:7]], dtype=float)


def _insert_state_from_msg(msg: InsertRelativeState, source: str) -> InsertSample:
    return InsertSample(
        t_abs_s=_header_stamp_sec(msg),
        active=bool(msg.active),
        task_stage=str(msg.task_stage),
        lateral_error_m=float(msg.lateral_error_m),
        axial_error_m=float(msg.axial_error_m),
        angle_error_rad=float(msg.angle_error_rad),
        rel_pose=_pose_to_array(msg.relative_pose),
        hole_index=int(msg.hole_index),
        latch=float(msg.latch),
        source=source,
    )


@dataclass
class TfRecord:
    t_abs_s: float
    parent: str
    child: str
    translation: np.ndarray
    rotation_quat: np.ndarray


class OfflineTfLookup:
    """按时间查询 arm_base_link -> peg_tip（取 stamp 之前最新一条）。"""

    def __init__(self, records: Sequence[TfRecord]) -> None:
        self._records = sorted(records, key=lambda r: r.t_abs_s)

    def lookup(self, t_query_s: float, parent: str, child: str) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        best: Optional[TfRecord] = None
        for rec in self._records:
            if rec.parent != parent or rec.child != child:
                continue
            if rec.t_abs_s > t_query_s + 1e-6:
                break
            best = rec
        if best is None:
            return None
        rot = _quat_to_rot(
            best.rotation_quat[0],
            best.rotation_quat[1],
            best.rotation_quat[2],
            best.rotation_quat[3],
        )
        return best.translation.copy(), rot


def _quat_to_rot(x: float, y: float, z: float, w: float) -> np.ndarray:
    q = np.array([x, y, z, w], dtype=float)
    n = float(np.linalg.norm(q))
    if n < 1e-9:
        return np.eye(3, dtype=float)
    q /= n
    return np.array(
        [
            [1.0 - 2.0 * (q[1] ** 2 + q[2] ** 2), 2.0 * (q[0] * q[1] - q[2] * q[3]), 2.0 * (q[0] * q[2] + q[1] * q[3])],
            [2.0 * (q[0] * q[1] + q[2] * q[3]), 1.0 - 2.0 * (q[0] ** 2 + q[2] ** 2), 2.0 * (q[1] * q[2] - q[0] * q[3])],
            [2.0 * (q[0] * q[2] - q[1] * q[3]), 2.0 * (q[1] * q[2] + q[0] * q[3]), 1.0 - 2.0 * (q[0] ** 2 + q[1] ** 2)],
        ],
        dtype=float,
    )


def _nearest_before(
    times: np.ndarray,
    values: List[Any],
    t_query: float,
) -> Optional[Any]:
    if times.size == 0:
        return None
    idx = int(np.searchsorted(times, t_query, side="right") - 1)
    if idx < 0:
        return None
    return values[idx]


def _read_bag_topics(bag_dir: Path) -> Dict[str, Any]:
    insert_samples: List[InsertSample] = []
    cmd_times: List[float] = []
    cmd_values: List[np.ndarray] = []
    stage_events: List[Tuple[float, str, str, str]] = []
    holes_times: List[float] = []
    holes_values: List[PoseArray] = []
    tf_records: List[TfRecord] = []

    reader = _open_reader(bag_dir)
    try:
        while reader.has_next():
            topic, data, t_ns = reader.read_next()
            t_bag_s = float(t_ns) * 1e-9

            if topic == TOPIC_INSERT_STATE:
                msg = deserialize_message(data, InsertRelativeState)
                insert_samples.append(_insert_state_from_msg(msg, "topic"))

            elif topic == TOPIC_AGENT_CMD:
                msg = deserialize_message(data, AgentCommand)
                cmd_times.append(t_bag_s)
                cmd_values.append(_parse_agent_command(msg, "left7"))

            elif topic == TOPIC_TASK_STAGE:
                msg = deserialize_message(data, TaskStage)
                stage_events.append(
                    (
                        _header_stamp_sec(msg),
                        str(msg.task_type),
                        str(msg.stage_name),
                        str(msg.stage_state),
                    )
                )

            elif topic == TOPIC_INSERT_HOLES:
                msg = deserialize_message(data, PoseArray)
                holes_times.append(_header_stamp_sec(msg))
                holes_values.append(msg)

            elif topic == TOPIC_TF:
                msg = deserialize_message(data, TFMessage)
                for tf in msg.transforms:
                    rec = _tf_to_record(tf, t_bag_s)
                    if rec is not None:
                        tf_records.append(rec)
    finally:
        del reader

    return {
        "insert_samples": insert_samples,
        "cmd_times": np.array(cmd_times, dtype=float),
        "cmd_values": cmd_values,
        "stage_events": stage_events,
        "holes_times": np.array(holes_times, dtype=float),
        "holes_values": holes_values,
        "tf_records": tf_records,
    }


def _tf_to_record(tf: TransformStamped, fallback_t_s: float) -> Optional[TfRecord]:
    parent = tf.header.frame_id.lstrip("/")
    child = tf.child_frame_id.lstrip("/")
    t_s = _header_stamp_sec(tf)
    if not math.isfinite(t_s) or t_s <= 0.0:
        t_s = fallback_t_s
    tr = tf.transform.translation
    rot = tf.transform.rotation
    return TfRecord(
        t_abs_s=t_s,
        parent=parent,
        child=child,
        translation=np.array([tr.x, tr.y, tr.z], dtype=float),
        rotation_quat=np.array([rot.x, rot.y, rot.z, rot.w], dtype=float),
    )


def _task_stage_window(stage_events: Sequence[Tuple[float, str, str, str]]) -> Optional[Tuple[float, float]]:
    start_t: Optional[float] = None
    end_t: Optional[float] = None
    for t_s, task_type, stage_name, stage_state in stage_events:
        if task_type != TASK_TYPE_INSERT:
            continue
        if stage_name == STAGE_START and stage_state == "ENTER":
            start_t = t_s if start_t is None else min(start_t, t_s)
        if start_t is not None and stage_name == STAGE_END and stage_state == "ENTER":
            end_t = t_s if end_t is None else min(end_t, t_s)
    if start_t is None:
        return None
    if end_t is None:
        return start_t, float("inf")
    return start_t, end_t


def _in_window(t_s: float, start_t: float, end_t: float) -> bool:
    if t_s < start_t - 1e-6:
        return False
    if math.isfinite(end_t) and t_s >= end_t - 1e-6:
        return False
    return True


def _build_samples_from_topic(
    insert_samples: Sequence[InsertSample],
) -> Tuple[List[InsertSample], str]:
    active = [s for s in insert_samples if s.active and s.is_valid_geometry()]
    if active:
        return active, "insert_rel_state.active"
    return [], "insert_rel_state.active"


def _build_samples_recomputed(
    bag_data: Dict[str, Any],
    window: Tuple[float, float],
    hole_index: int,
    axis_local: np.ndarray,
    rod_axis_tcp: np.ndarray,
    base_frame: str,
    peg_frame: str,
    rate_hz: float,
) -> Tuple[List[InsertSample], str]:
    if compute_relative_state is None:
        return [], "recompute_unavailable"

    holes_times: np.ndarray = bag_data["holes_times"]
    holes_values: List[PoseArray] = bag_data["holes_values"]
    tf_lookup = OfflineTfLookup(bag_data["tf_records"])
    if holes_times.size == 0 or not bag_data["tf_records"]:
        return [], "recompute_missing_tf_or_holes"

    start_t, end_t = window
    dt = 1.0 / max(rate_hz, 1.0)
    samples: List[InsertSample] = []
    t = start_t
    while t < end_t:
        holes_msg = _nearest_before(holes_times, holes_values, t)
        peg = tf_lookup.lookup(t, base_frame, peg_frame)
        if holes_msg is not None and peg is not None and hole_index < len(holes_msg.poses):
            peg_pos, peg_rot = peg
            hole_pose = holes_msg.poses[hole_index]
            e_lat, e_ax, angle_err, rel_pose = compute_relative_state(
                hole_pose,
                peg_pos,
                peg_rot,
                axis_local,
                rod_axis_tcp,
            )
            samples.append(
                InsertSample(
                    t_abs_s=t,
                    active=True,
                    task_stage=STAGE_START,
                    lateral_error_m=float(e_lat),
                    axial_error_m=float(e_ax),
                    angle_error_rad=float(angle_err),
                    rel_pose=_pose_to_array(rel_pose),
                    hole_index=hole_index,
                    latch=float("nan"),
                    source="recompute",
                )
            )
        t += dt
    return samples, "recompute_tf_holes"


def _select_state_samples(
    bag_data: Dict[str, Any],
    mode: str,
    hole_index: int,
    axis_local: np.ndarray,
    rod_axis_tcp: np.ndarray,
    base_frame: str,
    peg_frame: str,
    rate_hz: float,
) -> Tuple[List[InsertSample], str, Optional[Tuple[float, float]]]:
    stage_window = _task_stage_window(bag_data["stage_events"])
    topic_samples, topic_src = _build_samples_from_topic(bag_data["insert_samples"])

    if mode == "topic":
        return topic_samples, topic_src, stage_window

    if topic_samples:
        return topic_samples, topic_src, stage_window

    if mode == "recompute" or mode == "auto":
        if stage_window is None:
            return [], "no_window", stage_window
        rec_samples, rec_src = _build_samples_recomputed(
            bag_data,
            stage_window,
            hole_index,
            axis_local,
            rod_axis_tcp,
            base_frame,
            peg_frame,
            rate_hz,
        )
        return rec_samples, rec_src, stage_window

    return [], "unknown_mode", stage_window


def _state_vector(sample: InsertSample, representation: str) -> np.ndarray:
    if representation == "errors":
        return sample.errors
    if representation == "pose7":
        return sample.pose7
    if representation == "full10":
        return np.concatenate([sample.errors, sample.pose7])
    raise ValueError("未知 state representation: %s" % representation)


def _nearest_action(
    cmd_times: np.ndarray,
    cmd_values: List[np.ndarray],
    t_query: float,
    max_dt_s: float,
) -> Tuple[Optional[np.ndarray], float]:
    if cmd_times.size == 0:
        return None, float("inf")
    idx = int(np.argmin(np.abs(cmd_times - t_query)))
    dt = float(abs(cmd_times[idx] - t_query))
    if dt > max_dt_s:
        return None, dt
    return cmd_values[idx].copy(), dt


def _build_transitions(
    samples: Sequence[InsertSample],
    cmd_times: np.ndarray,
    cmd_values: List[np.ndarray],
    representation: str,
    max_align_dt_s: float,
    action_dims: str,
) -> Tuple[Dict[str, np.ndarray], Dict[str, Any]]:
    if len(samples) < 2:
        empty_f = np.zeros(0, dtype=float)
        dim_s = len(_state_vector(samples[0], representation)) if samples else 0
        dim_a = 14 if action_dims == "full14" else 7
        return (
            {
                "t_abs_s": empty_f,
                "t_rel_s": empty_f,
                "s": np.zeros((0, dim_s), dtype=float),
                "a": np.zeros((0, dim_a), dtype=float),
                "s_next": np.zeros((0, dim_s), dtype=float),
                "dt_s": empty_f,
                "align_dt_s": empty_f,
                "latch": empty_f,
                "task_stage": np.array([], dtype=object),
            },
            {"dropped_align": 0, "max_align_dt_s": 0.0},
        )

    t0 = samples[0].t_abs_s
    s_list: List[np.ndarray] = []
    s_next_list: List[np.ndarray] = []
    a_list: List[np.ndarray] = []
    t_list: List[float] = []
    dt_list: List[float] = []
    align_dt_list: List[float] = []
    latch_list: List[float] = []
    stage_list: List[str] = []

    dropped_align = 0
    max_seen_align = 0.0

    if action_dims == "full14":
        for i in range(len(cmd_values)):
            cmd_values[i] = _resize_action(cmd_values[i], 14)
    else:
        for i in range(len(cmd_values)):
            cmd_values[i] = _resize_action(cmd_values[i], 7)

    for i in range(len(samples) - 1):
        cur = samples[i]
        nxt = samples[i + 1]
        if not cur.is_valid_geometry() or not nxt.is_valid_geometry():
            continue
        action, align_dt = _nearest_action(cmd_times, cmd_values, cur.t_abs_s, max_align_dt_s)
        if action is None:
            dropped_align += 1
            continue
        max_seen_align = max(max_seen_align, align_dt)
        s_list.append(_state_vector(cur, representation))
        s_next_list.append(_state_vector(nxt, representation))
        a_list.append(action)
        t_list.append(cur.t_abs_s)
        dt_list.append(nxt.t_abs_s - cur.t_abs_s)
        align_dt_list.append(align_dt)
        latch_list.append(cur.latch)
        stage_list.append(cur.task_stage)

    s_arr = np.array(s_list, dtype=float) if s_list else np.zeros((0, 0), dtype=float)
    s_next_arr = np.array(s_next_list, dtype=float) if s_next_list else np.zeros((0, 0), dtype=float)
    a_arr = np.array(a_list, dtype=float) if a_list else np.zeros((0, 0), dtype=float)
    t_abs = np.array(t_list, dtype=float)
    t_rel = t_abs - t0 if t_list else np.array([], dtype=float)

    return (
        {
            "t_abs_s": t_abs,
            "t_rel_s": t_rel,
            "s": s_arr,
            "a": a_arr,
            "s_next": s_next_arr,
            "dt_s": np.array(dt_list, dtype=float),
            "align_dt_s": np.array(align_dt_list, dtype=float),
            "latch": np.array(latch_list, dtype=float),
            "task_stage": np.array(stage_list, dtype=object),
        },
        {"dropped_align": dropped_align, "max_align_dt_s": max_seen_align},
    )


def _resize_action(action: np.ndarray, dim: int) -> np.ndarray:
    out = np.zeros(dim, dtype=float)
    n = min(dim, action.size)
    out[:n] = action[:n]
    return out


def _state_names(representation: str) -> List[str]:
    if representation == "errors":
        return STATE_NAMES_ERRORS.copy()
    if representation == "pose7":
        return STATE_NAMES_POSE.copy()
    if representation == "full10":
        return STATE_NAMES_ERRORS + STATE_NAMES_POSE
    raise ValueError(representation)


def _action_names(action_dims: str) -> List[str]:
    if action_dims == "full14":
        return ["cmd_%d_deg" % i for i in range(14)]
    return ACTION_NAMES_LEFT7.copy()


def _write_transitions_csv(path: Path, arrays: Dict[str, np.ndarray], state_names: List[str], action_names: List[str]) -> None:
    n = arrays["s"].shape[0]
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        header = ["t_abs_s", "t_rel_s", "dt_s", "align_dt_s", "latch", "task_stage"]
        header.extend(state_names)
        header.extend(action_names)
        header.extend(["%s_next" % n for n in state_names])
        w.writerow(header)
        for i in range(n):
            row = [
                "%.9f" % arrays["t_abs_s"][i],
                "%.9f" % arrays["t_rel_s"][i],
                "%.9f" % arrays["dt_s"][i],
                "%.9f" % arrays["align_dt_s"][i],
                "%.6f" % arrays["latch"][i] if math.isfinite(arrays["latch"][i]) else "",
                str(arrays["task_stage"][i]),
            ]
            row.extend(["%.9f" % v if math.isfinite(v) else "" for v in arrays["s"][i]])
            row.extend(["%.9f" % v if math.isfinite(v) else "" for v in arrays["a"][i]])
            row.extend(["%.9f" % v if math.isfinite(v) else "" for v in arrays["s_next"][i]])
            w.writerow(row)


def _write_states_csv(path: Path, samples: Sequence[InsertSample]) -> None:
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(
            [
                "t_abs_s",
                "active",
                "source",
                "task_stage",
                "lateral_error_m",
                "axial_error_m",
                "angle_error_rad",
                "rel_x",
                "rel_y",
                "rel_z",
                "rel_qx",
                "rel_qy",
                "rel_qz",
                "rel_qw",
                "hole_index",
                "latch",
            ]
        )
        for s in samples:
            w.writerow(
                [
                    "%.9f" % s.t_abs_s,
                    int(s.active),
                    s.source,
                    s.task_stage,
                    "%.9f" % s.lateral_error_m if math.isfinite(s.lateral_error_m) else "",
                    "%.9f" % s.axial_error_m if math.isfinite(s.axial_error_m) else "",
                    "%.9f" % s.angle_error_rad if math.isfinite(s.angle_error_rad) else "",
                    *["%.9f" % v if math.isfinite(v) else "" for v in s.rel_pose],
                    s.hole_index,
                    "%.6f" % s.latch if math.isfinite(s.latch) else "",
                ]
            )


def _write_metadata(path: Path, meta: Dict[str, Any]) -> None:
    path.write_text(json.dumps(meta, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def _maybe_plot(out_dir: Path, arrays: Dict[str, np.ndarray], state_names: List[str]) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return

    if arrays["s"].shape[0] == 0:
        return

    plot_dir = out_dir / "plots"
    plot_dir.mkdir(parents=True, exist_ok=True)
    t = arrays["t_rel_s"]

    if "lateral_error_m" in state_names:
        fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        for ax, name in zip(axes, STATE_NAMES_ERRORS):
            idx = state_names.index(name)
            ax.plot(t, arrays["s"][:, idx], label="s[%s]" % name)
            ax.plot(t, arrays["s_next"][:, idx], label="s_next", alpha=0.6)
            ax.set_ylabel(name)
            ax.grid(True, alpha=0.3)
            ax.legend(fontsize=8)
        axes[-1].set_xlabel("t_rel_s")
        fig.suptitle("Insert errors")
        fig.tight_layout()
        fig.savefig(plot_dir / "errors.png", dpi=140)
        plt.close(fig)

    action_norm = np.linalg.norm(arrays["a"], axis=1)
    fig, ax = plt.subplots(figsize=(10, 3))
    ax.plot(t, action_norm)
    ax.set_xlabel("t_rel_s")
    ax.set_ylabel("||a|| (deg)")
    ax.set_title("Action norm")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(plot_dir / "action_norm.png", dpi=140)
    plt.close(fig)


def extract_single_bag(
    bag_dir: Path,
    out_dir: Path,
    episode_id: str,
    outcome: str,
    args: argparse.Namespace,
) -> EpisodeExtractResult:
    bag_data = _read_bag_topics(bag_dir)
    samples, window_source, stage_window = _select_state_samples(
        bag_data,
        args.mode,
        args.insert_index,
        np.array(args.insert_axis_local_xyz, dtype=float),
        np.array(args.peg_rod_axis_tcp_xyz, dtype=float),
        args.base_frame,
        args.peg_frame,
        args.resample_hz,
    )

    if args.min_samples > 0 and len(samples) < args.min_samples:
        raise RuntimeError(
            "有效状态样本不足: %d < %d (window=%s)"
            % (len(samples), args.min_samples, window_source)
        )

    cmd_times = bag_data["cmd_times"]
    if args.action_dims == "full14":
        reader_cmd = _open_reader(bag_dir)
        cmd_times = []
        cmd_values_full: List[np.ndarray] = []
        try:
            while reader_cmd.has_next():
                topic, data, t_ns = reader_cmd.read_next()
                if topic != TOPIC_AGENT_CMD:
                    continue
                msg = deserialize_message(data, AgentCommand)
                cmd_times.append(float(t_ns) * 1e-9)
                cmd_values_full.append(_parse_agent_command(msg, "full14"))
        finally:
            del reader_cmd
        cmd_values = cmd_values_full
        cmd_times_arr = np.array(cmd_times, dtype=float)
    else:
        cmd_values = bag_data["cmd_values"]
        cmd_times_arr = cmd_times

    arrays, align_meta = _build_transitions(
        samples,
        cmd_times_arr,
        cmd_values,
        args.state_representation,
        args.max_align_dt_s,
        args.action_dims,
    )

    if args.min_transitions > 0 and arrays["s"].shape[0] < args.min_transitions:
        raise RuntimeError(
            "转移样本不足: %d < %d (dropped_align=%d)"
            % (arrays["s"].shape[0], args.min_transitions, align_meta["dropped_align"])
        )

    out_dir.mkdir(parents=True, exist_ok=True)
    state_names = _state_names(args.state_representation)
    action_names = _action_names(args.action_dims)

    npz_path = out_dir / "transitions.npz"
    np.savez_compressed(
        npz_path,
        episode_id=np.array(episode_id),
        bag_dir=np.array(str(bag_dir)),
        state_names=np.array(state_names),
        action_names=np.array(action_names),
        **arrays,
    )

    _write_transitions_csv(out_dir / "transitions.csv", arrays, state_names, action_names)
    _write_states_csv(out_dir / "states.csv", samples)

    duration_s = 0.0
    if len(samples) >= 2:
        duration_s = float(samples[-1].t_abs_s - samples[0].t_abs_s)
    latch_final = float(samples[-1].latch) if samples and math.isfinite(samples[-1].latch) else float("nan")

    meta = {
        "episode_id": episode_id,
        "bag_dir": str(bag_dir),
        "episode_outcome": outcome,
        "window_source": window_source,
        "stage_window": list(stage_window) if stage_window else None,
        "state_representation": args.state_representation,
        "action_dims": args.action_dims,
        "num_state_samples": len(samples),
        "num_transitions": int(arrays["s"].shape[0]),
        "duration_s": duration_s,
        "latch_final": latch_final,
        "max_align_dt_s": align_meta["max_align_dt_s"],
        "dropped_align": align_meta["dropped_align"],
        "topics": {
            "state": TOPIC_INSERT_STATE,
            "action": TOPIC_AGENT_CMD,
            "holes": TOPIC_INSERT_HOLES,
            "tf": TOPIC_TF,
            "task_stage": TOPIC_TASK_STAGE,
        },
    }
    _write_metadata(out_dir / "metadata.json", meta)

    if args.plot:
        _maybe_plot(out_dir, arrays, state_names)

    return EpisodeExtractResult(
        episode_id=episode_id,
        bag_dir=str(bag_dir),
        outcome=outcome,
        window_source=window_source,
        num_state_samples=len(samples),
        num_transitions=int(arrays["s"].shape[0]),
        latch_final=latch_final,
        duration_s=duration_s,
        max_align_dt_s=float(align_meta["max_align_dt_s"]),
        dropped_align=int(align_meta["dropped_align"]),
        output_dir=str(out_dir),
        extra=meta,
    )


def _load_jobs_jsonl(path: Path) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            rows.append(json.loads(line))
    return rows


def _episodes_from_jobs(
    jobs_path: Path,
    bags_root: Path,
    task_filter: str,
) -> List[Tuple[str, Path, str]]:
    rows = _load_jobs_jsonl(jobs_path)
    episodes: Dict[str, Dict[str, Any]] = {}
    for row in rows:
        eid = str(row.get("episode_id", ""))
        if not eid:
            continue
        episodes.setdefault(
            eid,
            {
                "outcome": row.get("episode_outcome", ""),
                "bag_dir": row.get("bag_dir", ""),
                "has_insert": False,
            },
        )
        if row.get("task_name") == task_filter:
            episodes[eid]["has_insert"] = True
            episodes[eid]["insert_row"] = row
        if row.get("episode_outcome"):
            episodes[eid]["outcome"] = row.get("episode_outcome")

    result: List[Tuple[str, Path, str]] = []
    for eid, info in sorted(episodes.items()):
        if not info.get("has_insert"):
            continue
        bag_rel = str(info.get("bag_dir", ""))
        if not bag_rel:
            continue
        bag_path = Path(bag_rel)
        if not bag_path.is_absolute():
            bag_path = (bags_root / bag_rel).resolve()
            if not bag_path.exists():
                bag_path = (jobs_path.parent / bag_rel).resolve()
        if not bag_path.is_dir():
            continue
        outcome = str(info.get("outcome", ""))
        result.append((eid, bag_path, outcome))
    return result


def _merge_npz(results: Sequence[EpisodeExtractResult], merge_out: Path, representation: str, action_dims: str) -> None:
    episode_ids: List[str] = []
    s_parts: List[np.ndarray] = []
    a_parts: List[np.ndarray] = []
    s_next_parts: List[np.ndarray] = []
    t_rel_parts: List[np.ndarray] = []

    for res in results:
        npz = np.load(Path(res.output_dir) / "transitions.npz", allow_pickle=True)
        if npz["s"].shape[0] == 0:
            continue
        n = npz["s"].shape[0]
        episode_ids.extend([res.episode_id] * n)
        s_parts.append(npz["s"])
        a_parts.append(npz["a"])
        s_next_parts.append(npz["s_next"])
        t_rel_parts.append(npz["t_rel_s"])

    if not s_parts:
        raise RuntimeError("无可合并的转移样本")

    merge_out.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        merge_out,
        episode_id=np.array(episode_ids, dtype=object),
        state_names=np.array(_state_names(representation)),
        action_names=np.array(_action_names(action_dims)),
        s=np.vstack(s_parts),
        a=np.vstack(a_parts),
        s_next=np.vstack(s_next_parts),
        t_rel_s=np.concatenate(t_rel_parts),
    )


def build_arg_parser() -> argparse.ArgumentParser:
    script_dir = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(
        description="从 rosbag2 抽取插孔段 (s_t, a_t, s_{t+1})",
    )
    parser.add_argument(
        "bag_dirs",
        nargs="*",
        type=Path,
        help="bag 目录；省略时配合 --jobs-jsonl 批量处理",
    )
    parser.add_argument(
        "--jobs-jsonl",
        type=Path,
        default=None,
        help="从 jobs.jsonl 读取 episode 列表（使用 TARGET_INSERT 所在 episode）",
    )
    parser.add_argument(
        "--bags-root",
        type=Path,
        default=script_dir / "bags",
        help="jobs.jsonl 中 bag_dir 相对路径根目录",
    )
    parser.add_argument(
        "--out-root",
        type=Path,
        default=script_dir / "analysis" / "insert_extract",
        help="输出根目录",
    )
    parser.add_argument(
        "--merge-out",
        type=Path,
        default=None,
        help="将全部 episode 合并为一个 npz",
    )
    parser.add_argument(
        "--mode",
        choices=["auto", "topic", "recompute"],
        default="auto",
        help="auto: 优先 insert_rel_state，否则 TF+孔位重算",
    )
    parser.add_argument(
        "--state-representation",
        choices=["errors", "pose7", "full10"],
        default="full10",
        help="状态向量：3 维误差 / 7 维位姿 / 10 维合并",
    )
    parser.add_argument(
        "--action-dims",
        choices=["left7", "full14"],
        default="left7",
        help="动作维度：左臂 7 或 AgentCommand 全 14",
    )
    parser.add_argument("--insert-index", type=int, default=0, help="孔位下标（重算模式）")
    parser.add_argument("--max-align-dt-s", type=float, default=0.05, help="状态-动作最大对齐误差 [s]")
    parser.add_argument("--min-samples", type=int, default=10, help="最少状态样本数")
    parser.add_argument("--min-transitions", type=int, default=5, help="最少转移条数")
    parser.add_argument("--resample-hz", type=float, default=50.0, help="重算模式重采样频率")
    parser.add_argument("--base-frame", default="arm_base_link")
    parser.add_argument("--peg-frame", default="peg_tip")
    parser.add_argument(
        "--insert-axis-local-xyz",
        nargs=3,
        type=float,
        default=DEFAULT_AXIS_LOCAL.tolist(),
    )
    parser.add_argument(
        "--peg-rod-axis-tcp-xyz",
        nargs=3,
        type=float,
        default=DEFAULT_ROD_AXIS_TCP.tolist(),
    )
    parser.add_argument("--plot", action="store_true", help="输出误差/动作范数图")
    parser.add_argument(
        "--episode-outcome",
        default="",
        help="单 bag 模式写入 metadata 的 episode_outcome",
    )
    return parser


def main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()

    jobs: List[Tuple[str, Path, str]] = []
    if args.bag_dirs:
        for bag_dir in args.bag_dirs:
            eid = bag_dir.name
            jobs.append((eid, bag_dir.resolve(), args.episode_outcome))
    elif args.jobs_jsonl is not None:
        jobs = _episodes_from_jobs(args.jobs_jsonl.resolve(), args.bags_root.resolve(), "TARGET_INSERT")
    else:
        parser.error("请指定 bag 目录或 --jobs-jsonl")

    if not jobs:
        print("没有可处理的 episode", file=sys.stderr)
        return 1

    results: List[EpisodeExtractResult] = []
    manifest: List[Dict[str, Any]] = []
    failures: List[str] = []

    for episode_id, bag_dir, outcome in jobs:
        if not bag_dir.is_dir():
            failures.append("%s: bag 不存在 %s" % (episode_id, bag_dir))
            continue
        out_dir = args.out_root / episode_id
        try:
            res = extract_single_bag(bag_dir, out_dir, episode_id, outcome, args)
            results.append(res)
            manifest.append(
                {
                    "episode_id": res.episode_id,
                    "bag_dir": res.bag_dir,
                    "outcome": res.outcome,
                    "window_source": res.window_source,
                    "num_transitions": res.num_transitions,
                    "output_dir": res.output_dir,
                }
            )
            print(
                "[extract] %s: transitions=%d window=%s -> %s"
                % (episode_id, res.num_transitions, res.window_source, out_dir)
            )
        except Exception as exc:
            failures.append("%s: %s" % (episode_id, exc))
            print("[extract] %s: 失败: %s" % (episode_id, exc), file=sys.stderr)

    args.out_root.mkdir(parents=True, exist_ok=True)
    _write_metadata(
        args.out_root / "manifest.json",
        {"episodes": manifest, "failures": failures},
    )

    if args.merge_out is not None and results:
        try:
            _merge_npz(results, args.merge_out, args.state_representation, args.action_dims)
            print("[extract] 已合并: %s" % args.merge_out)
        except Exception as exc:
            print("[extract] 合并失败: %s" % exc, file=sys.stderr)
            return 1

    if failures and not results:
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
