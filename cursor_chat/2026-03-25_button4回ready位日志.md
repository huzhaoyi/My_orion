## 主题：右手柄 buttons[4] 松手日志标注「回 ready 位」

### 需求

Joy 变化日志中 `buttons[4] 1→0` 表示硬件回中/ready，需在文案上标明。

### 实现

- 参数 `log_right_joy_ready_button_index`（默认 4，`-1` 关闭）：仅 **右手柄**、该下标 **非 0 → 0** 时，在对应 diff 片段后追加 `（回 ready 位）`。
- `joy_manipulator.yaml` 调试段已增加同名键。
