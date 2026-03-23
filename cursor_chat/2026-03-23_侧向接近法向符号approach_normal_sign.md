# 侧向接近法向符号 `approach_normal_sign`

## 现象

仿真中表现为从缆绳**下方**向上夹，与期望的接近侧相反（+Z/-Z 对调感）。

## 原因简述

侧向法向 `n` 由 `UnitZ × cable_axis` 等构造，在垂直于缆绳轴的平面内只固定了**一侧**；`pregrasp = grasp - n * offset`，沿 `+n` 接近缆绳。若 `n` 在世界系中竖直分量与期望相反，则会出现自下而上接近。

## 改动

- **`CableGraspConfig::approach_normal_sign`**：负值时对每条候选的 `n_raw` 取反（翻转接近侧）。
- **参数**：`cable_side_grasp.approach_normal_sign`，`pick_params.yaml` 中默认 **` -1.0`** 以适配当前 Holoocean 流程。
- **`cable_side_grasp.cpp`**：在 `enumerateCableSideGrasps` 内 `n = n_raw * approach_sign`。

若实机/别场景方向正确，可改回 **`1.0`**。
