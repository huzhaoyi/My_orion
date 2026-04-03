# Orion 机械臂工作空间限值

本文档用于**配置抓取目标与安全边界**时参考；**与执行端硬拒绝一致**的标量以 `src/orion_mtc/config/orion_mtc_params.yaml` 的 **`feasibility`** 为准。

## 与后端一致的硬限（base_link，物体/缆绳中心）

| 项 | 含义 | 典型值 (m) | 参数名 |
|----|------|------------|--------|
| **‖p‖ 下限** | 距基座原点过近拒绝 | 0.14 | `min_reach_safe` |
| **‖p‖ 硬上限** | 距基座原点过远拒绝 | 1.8 | `max_reach_hard` |
| **‖p‖ 软上限** | 审批/诊断告警 | 1.66 | `max_reach_soft` |
| **Z 下限** | 高度过低拒绝 | -0.55 | `z_min` |
| **Z 上限** | 过高拒绝 | 1.55 | `z_max` |

坐标系：**base_link**（`orion.urdf`），单位 m。URDF 或策略变更时请同时改 yaml、**`web/js/data/feasibilityWorkspace.js`**（与前端展示同步）。

## Web 与 3D 示意

- **任务栏**（`RightPanel.js`）：**两行**——① **工作空间**：`gripper_tcp` 采样∩硬限后的 X/Y/Z 示意 AABB（与 3D 线框同源）；② **目标（缆绳）**：`object_pose` 用的 ‖p‖、Z、软 ‖p‖（与 `feasibilityWorkspace.js` / yaml 一致）。
- **3D 线框**：`RobotScene` 使用同源 `getWorkspaceBoundsScene()`。`RobotModelLoader.sampleWorkspace()` 按 **各关节 URDF 标称限位** 粗采样（与 `orion_description/urdf/orion.urdf` 一致），再与 ‖p‖、Z 硬限求交。示意框角点未必全部可达；以 ‖p‖ 与 Z 硬限为准。

## 配置提醒

- **object_pose**：应在 ‖p‖ 与 Z 硬限内；角点外包络可能略宽，勿仅按 X/Y 最大值理解可达性。
- **URDF 变更**：重跑前端逻辑或更新 yaml / `feasibilityWorkspace.js` 后核对 Web 与 `check_pick`。
