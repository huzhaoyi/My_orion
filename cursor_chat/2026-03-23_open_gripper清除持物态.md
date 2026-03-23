# 持物态改为由 /left_arm_gripped 驱动（2026-03-23 修正）

## 更正

先前在 `open_gripper` 服务成功回调里清持物态不符合设计。**持物是否与物理一致应以话题 `/manipulator/left_arm_gripped` 为准**，不应由 open/close 服务推断。

## 实现

- 撤销 `handleOpenGripper()` 末尾的清持物/scene/发布逻辑。
- 新增 `TaskManager::applyGripperFeedbackFromTopic(double)`：若 `gripped_value < 0.5`（与 `isGripperLocked` 阈值一致）且非 `PICKING`，且当前为持物语义，则 `clearHeldObject`、`IDLE`、清 attach、发布 `held_object_state`。
- `ManipulatorRosInterface` 在订阅 `left_arm_gripped` 时除更新 atomic 外调用 `applyGripperFeedbackFromTopic`。
- README：`open_gripper` 行恢复为仅描述运动；新增 `left_arm_gripped` 行说明持物同步规则。
