# 2026-03-25 手动夹爪独立速度倍率

## 参数

- `manual_gripper_speed_scale`（默认代码 1.0）：仅作用手动 SW5-6 夹爪积分，`vg = _grip_max_speed * scale_grip * dt * scale`；与 6 臂无关。
- YAML 示例 `2.5`：在相同油门与 `manual_gripper_max_speed_deg_s` 下开合约 2.5 倍；负值按 0 处理。
