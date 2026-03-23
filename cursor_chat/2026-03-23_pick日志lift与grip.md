# pick 日志：lift 笛卡尔与夹持话题

## 日志摘要

1. **candidate 1 — `lift object` 规划失败**  
   - `CartesianPath: min_fraction not met`  
   - `min_distance not reached (0.0481 < 0.05)`  
   - 抬升段在关节/限位附近只能走约 4.8cm，而 `lift_object_min_dist` 为 5cm，整段被拒。

2. **candidate 3 — 执行到闭合手后 `wait gripped` 超时**  
   - `waitForGripped: timeout (expect_gripped=1, last=0.000)`  
   - `orion_mtc` 用 `/manipulator/left_arm_gripped`（Float32）与阈值 0.5 判断夹紧；仿真侧若未发布或始终为 0，会一直超时。

## 已做配置

- `lift_object_min_dist`：**0.05 → 0.04**（`pick_params.yaml` + `MTCConfig` 默认 + `declareParameters`），与常见 4.8cm 级短距兼容。

## 夹持仍为 0 时

- 查 `arm_sensor_to_joint_state` / Holoocean 是否在夹爪闭合后把对应量映射到 `left_arm_gripped`；  
- 或临时调低 `orion_mtc_node` 中 wait 阈值（仅调试用）。
