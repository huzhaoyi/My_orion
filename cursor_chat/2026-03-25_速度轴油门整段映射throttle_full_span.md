# 2026-03-25 速度轴油门整段映射（throttle_full_span）

## 需求

手柄 `axes` 为 -1…1，作油门用；不希望按中位对称截断（旧逻辑 `abs(axis)` 两侧均满速、中位 idle）。

## 实现

- 新参数 `speed_axis_map_mode`：`symmetric_abs`（默认，旧行为）、`throttle_full_span`（`[-1,1]` 线性 → 比例 `[0,1]`，`-1`=0、`+1`=1）。
- `axis_deadband` 在 `throttle_full_span` 下仅作用在映射后低端；`axis_deadband: 0` 时为严格线性 0%～100%。
- `db <= 1e-9` 时代码直接返回 `u`，避免数值问题。
- `speed_axis_invert` 仍在读取后先对轴取反，再映射。
- `joy_manipulator.yaml`：`speed_axis_map_mode: throttle_full_span`，`axis_deadband: 0.0`。
