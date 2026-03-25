## 主题：log_right_joy_ready_button_index 归入 YAML 自动模式右手段

### 原因

与 `right_pick_button_index`、`auto_gripper_*` 同属右手自动侧键位说明，应写在同一节，避免只在「调试日志」里出现。

### 改动

- `joy_manipulator.yaml`：在「自动模式：右手抓取、右手夹爪服务」下增加 `log_right_joy_ready_button_index`（仍为 button4 默认），注释标明自动模式不绑服务、仅日志标注；调试段删除重复键。
- `joy_manipulator_node.py`：`declare_parameter` 注释与 YAML 位置对齐（参数名未改）。
