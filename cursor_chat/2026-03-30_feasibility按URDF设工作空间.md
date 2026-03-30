# feasibility 参数按 URDF 工作空间设置

## 依据

`orion_description/urdf/orion.urdf`：按各关节 `origin`/`axis` 实现 6 轴（臂）+`gripper_tcp` 固定段的正运动学，关节在 `[-π,π]` 内蒙特卡洛采样。

统计量级：`gripper_tcp` 相对 `base_link` 的 `‖p‖_max≈1.85 m`、`‖p‖_min≈0.02 m`，`z` 约 `[-1.82, 1.83] m`。

## 写入配置

- `orion_mtc_params.yaml` 的 `feasibility`：`max_reach_hard/soft`、`min_reach_safe`、`z_min/z_max` 在统计极值基础上留工程余量，并在块注释中说明；目标为物体/缆绳中心而非 TCP。
- `feasibility_checker.hpp` 内 `FeasibilityParams` 默认值与 yaml 对齐（无参加载时的回退）。
