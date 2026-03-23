# 急停 / 回 ready 改为与 open_gripper 相同的 Trigger 服务

## 背景

rosbridge 对 `std_msgs/Empty` 话题发布不可靠；开夹爪使用 `std_srvs/srv/Trigger` 的 `call_service` 正常。

## 后端（orion_mtc）

- 新增服务：`/manipulator/emergency_stop`、`/manipulator/go_to_ready`，类型均为 `std_srvs/srv/Trigger`。
- 为避免与同名 **服务** 冲突，已 **移除** 原 `std_msgs/Empty` **话题** 订阅（ROS2 中同名 topic/service 不宜并存）。
- 急停：回调内 `requestEmergencyStop()`，`success=true`，`message=emergency_stop`。
- 回 ready：回调内同步 `tryGoToReady()`（节点使用 `MultiThreadedExecutor`，可阻塞等待真实结果），`success`/`message` 与执行结果一致。

## 前端

- `wsClient.js`：删除 `advertiseEmptyPublishTopics`；新增 `callEmergencyStop`、`callGoToReady`（`go_to_ready` 超时 120s）。
- `TopBar.js`：按钮改为 `callService` 风格，与 `main.js` 中开夹爪一致；提示文案改为「调用服务」。

## 验证

- `colcon build --packages-select orion_mtc` 通过。
