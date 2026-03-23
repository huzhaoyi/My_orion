# 2026-03-23 handlePick 可达性预检改为候选 TCP 距离

## 变更
- `task_manager.cpp`：`handlePick` 不再用缆绳中心到 `base_link` 原点的球半径做硬门限；改为对每个 `CableGraspCandidate` 计算 `grasp_pose`、`pregrasp_pose` 平移范数（即 `gripper_tcp` 在 `base_link` 下位置，与 `setFromIK(..., hand_frame)` 一致），取 `max(r_grasp, r_pre)`，再对所有候选取 **min**；若该值仍大于 `feasibility.max_reach_hard` 则提前返回 `TARGET_UNREACHABLE`。
- 无候选时回退为缆绳中心距离并打 WARN（与旧逻辑兼容）。
- `pick_params.yaml`：补充注释说明 CheckPick 与 handlePick 预检差异。

## 验证
- `colcon build --packages-select orion_mtc` 通过。
