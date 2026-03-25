## 主题：自动模式右手 ready 键松手调用 go_to_ready

### 问题

`buttons[4]` 仅有 Joy 变化日志与「回 ready 位」文案，未调 MTC 接口。

### 实现

- 新增客户端：`/manipulator/go_to_ready`（`std_srvs/Trigger`，与 Web 一致）。
- `_control_timer` 自动分支：当 `log_right_joy_ready_button_index ≥ 0` 且 `on_auto_right_ready_release_call_go_to_ready` 为 true 时，检测该键 **按下→松开**（与日志「非0→0」一致），异步调用 `go_to_ready`；手动模式下只同步内部边沿状态，不发服务。
- `joy_manipulator.yaml` / `README` orion_joy_arm_bridge 一行已说明；关闭服务调用可设 `on_auto_right_ready_release_call_go_to_ready: false`。
