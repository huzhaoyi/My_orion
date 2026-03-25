# 2026-03-25 手柄切回自动时解除急停闭锁

## 问题

进手动触发的 `emergency_stop` 会置位 `estop_requested_`，`open_gripper` 等在 `executeSolution` 段 0 前即中止。

## 改动

- **orion_mtc**：`TaskManager::clearEmergencyStopLatch()`；服务 `/manipulator/clear_estop`（Trigger）。
- **joy_manipulator_node**：`on_auto_enter_call_clear_estop`（默认 true），切回自动时 **先** `emergency_stop`（异步完成后再 **clear_estop**）；急停未就绪则仅尝试 clear。
- YAML / README 已补说明。
