#!/usr/bin/env python3
"""
从 rosbag2 目录读取臂相关话题，画「AgentCommand vs ArmSensor vs joint_states」曲线并输出 CSV 统计表。
依赖: 已 source 的 ROS2 环境 + 工作空间（holoocean_interfaces、sensor_msgs）。
可选: pip install -r requirements.txt（numpy、matplotlib）

一键分析（推荐）: 在 tools/arm_bag_tools/ 下已 source ROS+工作空间后执行 ./analyze_arm_bag.sh

手动:
  source /opt/ros/$ROS_DISTRO/setup.bash
  source <ws>/install/setup.bash
  python3 plot_arm_bag.py <bag 目录> [--out 目录]
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

try:
    import numpy as np
except ImportError:
    print("请安装 numpy: pip install numpy", file=sys.stderr)
    sys.exit(1)

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError:
    print("请安装 matplotlib: pip install matplotlib", file=sys.stderr)
    sys.exit(1)

try:
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
except ImportError:
    print("请安装 rosbag2_py（source ROS2 后通常可用 ros2 自带）", file=sys.stderr)
    sys.exit(1)

try:
    from rclpy.serialization import deserialize_message
except ImportError:
    print("请安装 rclpy（source ROS2）", file=sys.stderr)
    sys.exit(1)

try:
    from sensor_msgs.msg import JointState
    from holoocean_interfaces.msg import AgentCommand, WorkingClassROVArmSensor
except ImportError as e:
    print(
        "无法导入消息类型: %s\n请先 source /opt/ros/$ROS_DISTRO/setup.bash "
        "与包含 holoocean_interfaces、sensor_msgs 的 install/setup.bash" % e,
        file=sys.stderr,
    )
    sys.exit(1)


RAD_TO_DEG = 180.0 / math.pi
ARM_JOINT_NAMES = [
    "joint_base_link_Link1",
    "joint_Link1_Link2",
    "joint_Link2_Link3",
    "joint_LinkVirtual_Link4",
    "joint_Link4_Link5",
    "joint_Link5_Link6",
]
HAND_JOINT_NAMES = [
    "joint_Link6_Link7",
    "joint_Link6_Link8",
]
LABELS_ARM = ["J1", "J2", "J3", "J4", "J5", "J6", "Grip"]


def _open_reader(bag_dir: Path) -> SequentialReader:
    storage_options = StorageOptions(uri=str(bag_dir.resolve()), storage_id="sqlite3")
    converter_options = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    return reader


def _read_bag(bag_dir: Path):
    cmd_t: list[float] = []
    cmd_data: list[list[float]] = []

    sens_t: list[float] = []
    sens_data: list[list[float]] = []

    js_t: list[float] = []
    js_arm_rad: list[list[float]] = []
    js_grip_deg: list[float] = []

    reader = _open_reader(bag_dir)
    try:
        while reader.has_next():
            topic, data, t_ns = reader.read_next()
            t_sec = float(t_ns) * 1e-9

            if topic == "/holoocean/command/agent/arm":
                msg = deserialize_message(data, AgentCommand)
                c = list(msg.command) if msg.command else []
                while len(c) < 14:
                    c.append(0.0)
                left = c[0:7]
                cmd_t.append(t_sec)
                cmd_data.append(left)

            elif topic == "/holoocean/rov0/ArmSensor":
                msg = deserialize_message(data, WorkingClassROVArmSensor)
                raw = getattr(msg, "left_arm_joints", None)
                if raw is None:
                    continue
                left = list(raw[:7]) if len(raw) >= 7 else list(raw) + [0.0] * (7 - len(raw))
                while len(left) < 7:
                    left.append(0.0)
                sens_t.append(t_sec)
                sens_data.append([float(x) for x in left])

            elif topic == "/joint_states":
                msg = deserialize_message(data, JointState)
                name_to_p: dict[str, float] = {}
                for i, n in enumerate(msg.name):
                    if i < len(msg.position):
                        name_to_p[n] = float(msg.position[i])
                arm_rad = []
                for jn in ARM_JOINT_NAMES:
                    arm_rad.append(name_to_p.get(jn, float("nan")))
                g7 = name_to_p.get(HAND_JOINT_NAMES[0], float("nan"))
                g8 = name_to_p.get(HAND_JOINT_NAMES[1], float("nan"))
                opening_rad = 0.5 * (g7 - g8) if math.isfinite(g7) and math.isfinite(g8) else float("nan")
                grip_deg = opening_rad * RAD_TO_DEG

                js_t.append(t_sec)
                js_arm_rad.append(arm_rad)
                js_grip_deg.append(grip_deg)

    finally:
        del reader

    return (
        np.array(cmd_t, dtype=float),
        np.array(cmd_data, dtype=float),
        np.array(sens_t, dtype=float),
        np.array(sens_data, dtype=float),
        np.array(js_t, dtype=float),
        np.array(js_arm_rad, dtype=float),
        np.array(js_grip_deg, dtype=float),
    )


def _interp_rows(t_src: np.ndarray, y_src: np.ndarray, t_q: np.ndarray) -> np.ndarray:
    """y_src: (n, k), 对每列在 t_src 上插值到 t_q。"""
    if t_src.size == 0 or t_q.size == 0:
        return np.full((t_q.size, y_src.shape[1] if y_src.ndim == 2 else 0), np.nan)
    if y_src.ndim == 1:
        y_src = y_src.reshape(-1, 1)
    out = np.empty((t_q.size, y_src.shape[1]), dtype=float)
    for k in range(y_src.shape[1]):
        out[:, k] = np.interp(t_q, t_src, y_src[:, k], left=np.nan, right=np.nan)
    return out


def _stats_table(
    names: list[str],
    err: np.ndarray,
) -> list[tuple[str, float, float]]:
    """每列 rmse、max_abs。"""
    rows = []
    for i, name in enumerate(names):
        col = err[:, i]
        col = col[np.isfinite(col)]
        if col.size == 0:
            rows.append((name, float("nan"), float("nan")))
            continue
        rmse = float(np.sqrt(np.mean(col**2)))
        mx = float(np.max(np.abs(col)))
        rows.append((name, rmse, mx))
    return rows


def _wrap_deg_delta(delta_deg: np.ndarray) -> np.ndarray:
    """将角度差折叠到 (-180, 180]°（与最短弧一致），非有限值保持 nan。"""
    out = np.full_like(delta_deg, np.nan, dtype=float)
    m = np.isfinite(delta_deg)
    if not np.any(m):
        return out
    rad = np.deg2rad(delta_deg[m])
    out[m] = np.rad2deg(np.arctan2(np.sin(rad), np.cos(rad)))
    return out


def _first_spike_rows(
    t_s: np.ndarray,
    err_raw: np.ndarray,
    err_wrap: np.ndarray,
    cmd: np.ndarray,
    sens: np.ndarray,
    labels: list[str],
    n_arm_wrap: int,
    threshold_deg: float,
) -> list[tuple[str, float, float, float, float, float]]:
    """各关节首次 |err_wrap|>阈值（J1–Jn_arm）；夹爪用 |err_raw|>阈值。返回 (name,t,raw,wrap,cmd,sens)。"""
    rows = []
    n = err_raw.shape[1]
    for j in range(n):
        if j < n_arm_wrap:
            e = err_wrap[:, j]
            use = np.isfinite(e) & (np.abs(e) > threshold_deg)
        else:
            e = err_raw[:, j]
            use = np.isfinite(e) & (np.abs(e) > threshold_deg)
        if not np.any(use):
            continue
        k = int(np.flatnonzero(use)[0])
        wrap_at = float(err_wrap[k, j]) if j < n_arm_wrap else float(err_raw[k, j])
        rows.append(
            (
                labels[j],
                float(t_s[k]),
                float(err_raw[k, j]),
                wrap_at,
                float(cmd[k, j]),
                float(sens[k, j]),
            )
        )
    return rows


def main() -> int:
    parser = argparse.ArgumentParser(description="画臂 rosbag 对比曲线并导出 CSV")
    parser.add_argument("bag_dir", type=Path, help="ros2 bag 目录（内含 .db3）")
    parser.add_argument(
        "--out",
        type=Path,
        default=Path("arm_bag_analysis"),
        help="输出目录（图 + CSV）",
    )
    parser.add_argument(
        "--spike-deg",
        type=float,
        default=60.0,
        help="anomaly_report.txt：J1–J6 用 |折叠误差|>此值(°)、夹爪用 |原始误差|>此值，记首次时刻",
    )
    args = parser.parse_args()

    bag_dir = args.bag_dir
    if not bag_dir.is_dir():
        print("bag 目录不存在: %s" % bag_dir, file=sys.stderr)
        return 1

    out_dir = args.out
    out_dir.mkdir(parents=True, exist_ok=True)

    (
        cmd_t,
        cmd_data,
        sens_t,
        sens_data,
        js_t,
        js_arm_rad,
        js_grip_deg,
    ) = _read_bag(bag_dir)

    if cmd_t.size == 0 and sens_t.size == 0:
        print("bag 中无 AgentCommand 与 ArmSensor", file=sys.stderr)
        return 1

    t0 = float("inf")
    for arr in (cmd_t, sens_t, js_t):
        if arr.size > 0:
            t0 = min(t0, float(np.min(arr)))
    if t0 == float("inf"):
        t0 = 0.0

    cmd_t_rel = cmd_t - t0 if cmd_t.size else cmd_t
    sens_t_rel = sens_t - t0 if sens_t.size else sens_t
    js_t_rel = js_t - t0 if js_t.size else js_t

    if cmd_t.size > 0:
        t_plot = cmd_t_rel
        sens_i = _interp_rows(sens_t_rel, sens_data, cmd_t_rel) if sens_t.size else None
        js_deg = np.hstack(
            [js_arm_rad * RAD_TO_DEG, js_grip_deg.reshape(-1, 1)],
        )
        js_i = _interp_rows(js_t_rel, js_deg, cmd_t_rel) if js_t.size else None
        cmd_plot = cmd_data
    elif sens_t.size > 0:
        t_plot = sens_t_rel
        cmd_plot = _interp_rows(cmd_t_rel, cmd_data, sens_t_rel) if cmd_t.size else None
        sens_i = sens_data
        js_deg = np.hstack(
            [js_arm_rad * RAD_TO_DEG, js_grip_deg.reshape(-1, 1)],
        )
        js_i = _interp_rows(js_t_rel, js_deg, sens_t_rel) if js_t.size else None
    else:
        print("无可用时间序列", file=sys.stderr)
        return 1

    n = len(LABELS_ARM)
    fig, axes = plt.subplots(n, 1, figsize=(12, 10), sharex=True)
    if n == 1:
        axes = [axes]

    err_for_csv = np.full((t_plot.size, n), np.nan)
    err_wrap_csv = np.full((t_plot.size, n), np.nan)

    for i in range(6):
        ax = axes[i]
        if cmd_plot is not None and cmd_plot.shape[0] == t_plot.size:
            ax.plot(t_plot, cmd_plot[:, i], label="cmd (deg)", linewidth=1.0, color="C0")
        if sens_i is not None and sens_i.shape[0] == t_plot.size:
            ax.plot(t_plot, sens_i[:, i], label="sensor (deg)", linewidth=1.0, color="C1", alpha=0.85)
        if js_i is not None and js_i.shape[0] == t_plot.size:
            ax.plot(
                t_plot,
                js_i[:, i],
                label="joint_states (deg)",
                linewidth=1.0,
                color="C2",
                alpha=0.75,
            )
        if (
            cmd_plot is not None
            and sens_i is not None
            and cmd_plot.shape[0] == t_plot.size
            and sens_i.shape[0] == t_plot.size
        ):
            err_for_csv[:, i] = cmd_plot[:, i] - sens_i[:, i]
            err_wrap_csv[:, i] = _wrap_deg_delta(err_for_csv[:, i])
            ax.plot(
                t_plot,
                err_for_csv[:, i],
                label="cmd-sensor raw",
                linewidth=0.7,
                color="C3",
                alpha=0.55,
                linestyle="--",
            )
            ax.plot(
                t_plot,
                err_wrap_csv[:, i],
                label="cmd-sensor wrap ±180°",
                linewidth=0.9,
                color="C4",
                alpha=0.85,
            )
        ax.set_ylabel(LABELS_ARM[i])
        ax.legend(loc="upper right", fontsize=7)
        ax.grid(True, alpha=0.3)

    ax_g = axes[6]
    if cmd_plot is not None and cmd_plot.shape[0] == t_plot.size:
        ax_g.plot(t_plot, cmd_plot[:, 6], label="cmd grip (deg)", linewidth=1.0, color="C0")
    if sens_i is not None and sens_i.shape[0] == t_plot.size:
        ax_g.plot(t_plot, sens_i[:, 6], label="sensor grip (deg)", linewidth=1.0, color="C1", alpha=0.85)
    if js_i is not None and js_i.shape[0] == t_plot.size:
        ax_g.plot(
            t_plot,
            js_i[:, 6],
            label="js grip (opening rad→deg 近似)",
            linewidth=1.0,
            color="C2",
            alpha=0.75,
        )
    if (
        cmd_plot is not None
        and sens_i is not None
        and cmd_plot.shape[0] == t_plot.size
        and sens_i.shape[0] == t_plot.size
    ):
        err_for_csv[:, 6] = cmd_plot[:, 6] - sens_i[:, 6]
        err_wrap_csv[:, 6] = err_for_csv[:, 6]
        ax_g.plot(
            t_plot,
            err_for_csv[:, 6],
            label="cmd-sensor raw (grip)",
            linewidth=0.7,
            color="C3",
            alpha=0.55,
            linestyle="--",
        )
    ax_g.set_ylabel(LABELS_ARM[6])
    ax_g.set_xlabel("t (s) from bag start")
    ax_g.legend(loc="upper right", fontsize=7)
    ax_g.grid(True, alpha=0.3)

    fig.suptitle("Arm chain: %s" % bag_dir.name)
    fig.tight_layout()
    png_path = out_dir / "arm_comparison.png"
    fig.savefig(png_path, dpi=150)
    plt.close(fig)

    stats = _stats_table(LABELS_ARM, err_for_csv)
    summary_csv = out_dir / "summary_stats.csv"
    with summary_csv.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["joint", "rmse_cmd_minus_sensor_deg", "max_abs_cmd_minus_sensor_deg"])
        for name, rmse, mx in stats:
            w.writerow([name, "%.6f" % rmse if math.isfinite(rmse) else "", "%.6f" % mx if math.isfinite(mx) else ""])

    stats_wrapped = _stats_table(LABELS_ARM, err_wrap_csv)
    summary_wrapped_csv = out_dir / "summary_stats_wrapped.csv"
    with summary_wrapped_csv.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(
            [
                "joint",
                "rmse_wrapped_deg",
                "max_abs_wrapped_deg",
                "note",
            ]
        )
        for idx, (name, rmse, mx) in enumerate(stats_wrapped):
            note = ""
            if idx < 6:
                note = "J1–J6: shortest-angle cmd−sensor"
            elif idx == 6:
                note = "Grip: same as raw (非周期，不折叠)"
            w.writerow(
                [
                    name,
                    "%.6f" % rmse if math.isfinite(rmse) else "",
                    "%.6f" % mx if math.isfinite(mx) else "",
                    note,
                ]
            )

    def _fmt_cell(v: float) -> str:
        return "%.6f" % v if math.isfinite(v) else ""

    err_sidecar = out_dir / "cmd_sensor_error.csv"
    with err_sidecar.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        h = ["t_s"]
        for j in range(6):
            h.extend(["j%d_err_raw_deg" % (j + 1), "j%d_err_wrap_deg" % (j + 1)])
        h.append("grip_err_raw_deg")
        w.writerow(h)
        for k in range(int(t_plot.size)):
            row = ["%.6f" % t_plot[k]]
            for j in range(6):
                row.extend(
                    [
                        _fmt_cell(float(err_for_csv[k, j])),
                        _fmt_cell(float(err_wrap_csv[k, j])),
                    ]
                )
            row.append(_fmt_cell(float(err_for_csv[k, 6])))
            w.writerow(row)

    anomaly_path = out_dir / "anomaly_report.txt"
    if (
        cmd_plot is not None
        and sens_i is not None
        and cmd_plot.shape[0] == t_plot.size
        and sens_i.shape[0] == t_plot.size
    ):
        spikes = _first_spike_rows(
            t_plot,
            err_for_csv,
            err_wrap_csv,
            cmd_plot,
            sens_i,
            LABELS_ARM,
            6,
            float(args.spike_deg),
        )
        lines = [
            "cmd−sensor 首次超过阈值（J1–J6: |折叠误差|>%.3f°；Grip: |原始误差|>%.3f°）"
            % (args.spike_deg, args.spike_deg),
            "时间 t_s 为与 aligned_series / png 一致的原点相对秒。",
            "",
        ]
        j4_row = next((s for s in spikes if s[0] == "J4"), None)
        if j4_row is not None:
            _n, _ts, _rw, _wp, _c, _sn = j4_row
            lines.append(
                "J4 首次超阈值: t=%.6f s, err_raw=%.4f°, err_wrap=%.4f°, cmd=%.4f°, sensor=%.4f°"
                % (_ts, _rw, _wp, _c, _sn)
            )
            lines.append("")
        if not spikes:
            lines.append("全程无超过阈值的采样点。")
        else:
            lines.append(
                "joint\tt_s\terr_raw_deg\terr_wrap_or_grip_deg\tcmd_deg\tsensor_deg"
            )
            for name, ts, raw_d, wrap_d, cmd_d, sens_d in spikes:
                lines.append(
                    "%s\t%.6f\t%.4f\t%.4f\t%.4f\t%.4f"
                    % (name, ts, raw_d, wrap_d, cmd_d, sens_d)
                )
        anomaly_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    else:
        anomaly_path.write_text("无 cmd/sensor 对齐数据，跳过 spike 检测。\n", encoding="utf-8")

    aligned_csv = out_dir / "aligned_series.csv"
    with aligned_csv.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        header = ["t_s"]
        for j in range(7):
            header.extend(["cmd_%d_deg" % j, "sensor_%d_deg" % j, "joint_states_%d_deg" % j])
        w.writerow(header)
        n_row = int(t_plot.size)
        for k in range(n_row):
            row = ["%.6f" % t_plot[k]]
            for j in range(7):
                c = float(cmd_plot[k, j]) if cmd_plot is not None and k < cmd_plot.shape[0] else float("nan")
                s = float(sens_i[k, j]) if sens_i is not None and k < sens_i.shape[0] else float("nan")
                p = float(js_i[k, j]) if js_i is not None and k < js_i.shape[0] else float("nan")
                row.extend([_fmt_cell(c), _fmt_cell(s), _fmt_cell(p)])
            w.writerow(row)

    print("已写入: %s" % png_path)
    print("已写入: %s" % summary_csv)
    print("已写入: %s" % summary_wrapped_csv)
    print("已写入: %s" % err_sidecar)
    print("已写入: %s" % anomaly_path)
    print("已写入: %s" % aligned_csv)
    return 0


if __name__ == "__main__":
    sys.exit(main())
