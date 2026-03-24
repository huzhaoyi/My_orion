# 按 URDF 更新 feasibility 可达距离

## 操作

- 运行 `tools/compute_max_reach_from_urdf.py`（`base_link`→`gripper_tcp`，margin 1.08，soft=hard×0.92）。
- 当前 `orion.urdf`：`kinematic_ub ≈ 1.888 m` → `max_reach_hard: 2.039`，`max_reach_soft: 1.876`。
- 已写入 `orion_mtc/config/orion_mtc_params.yaml` 的 `feasibility` 段，并加注释说明重算命令。

## 说明

- 该值为**链长几何上界 × 余量**，用于 CheckPick / 预检距离阈值；**不等于**任意姿态下 TCP 可达。若仍报 NO_IK，需查关节限位、姿态与 IK 种子。
- URDF 或连杆变更后应重跑脚本并更新 yaml。
