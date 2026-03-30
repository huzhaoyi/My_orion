# 工作空间硬限接入 handlePick

## 目标

在真正跑 MTC 规划前，对与 `check_pick` 一致的几何硬限自动拒绝：`max_reach_hard`、`min_reach_safe`、`z_min`、`z_max`。

## 实现

- **FeasibilityChecker**：抽取 `workspaceHasHardLimitViolation`，供 `checkPick` 与外部共用；新增 `objectPoseWithinWorkspaceHardLimits(Pose, reject_reason)`。`max_reach_soft` 仍为仅警告，在 `checkPick` 中在 `r ∈ (soft, hard]` 时追加（不再使用与原 `if/else if` 完全相同的分支结构，行为等价）。
- **TaskManager**：`setFeasibilityChecker` + 在 `object_pose` 已变换到 **`base_link`** 后调用硬限校验；失败 `setStateError("WORKSPACE: " + ...)` 并返回。非 `base_link` 时告警并跳过（与此前无 TF 时行为兼容）。
- **OrionMTCNode**：创建 `FeasibilityChecker` 后对 `task_manager_` `setFeasibilityChecker`。
- **README**：抓取小节补充说明。
