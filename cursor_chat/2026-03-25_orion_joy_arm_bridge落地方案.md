# 2026-03-25 orion_joy_arm_bridge 落地方案

## 当日进展

推进 **Sealien_CtrlPilot_Payload_Orion** 的 **手柄接入 MTC/HoloOcean 执行链** 工作，完成 **orion_joy_arm_bridge** 新包：`joy_manipulator_node`（双路 Joy → `pick_trigger`/夹爪服务；手动 → `FollowJointTrajectory`）、`config/joy_manipulator.yaml`、`launch/joy_manipulator.launch.py`；**pick_holoocean.launch.py** 增加 **`use_joy_manipulator`**；**README** 补充包说明与启动示例，**orion_mtc/package.xml** 增加 **exec_depend**；将 README 中 **`/manipulator/emergency_stop`** 更正为 **Trigger 服务**（与代码一致）。

## 完成情况

当日任务 **已完成**（包已 `colcon build --packages-select orion_joy_arm_bridge` 通过）。

## 问题/需求

- 手柄 **mode 键位 33** 若驱动 `buttons` 长度不足，需在 `joy_manipulator.yaml` 改为 **`mode_source: axis`** 并调阈值。
- 网页侧若仍对 `emergency_stop` 发 **Empty**，需与 **Trigger 服务** 对齐（历史文档曾写为话题）。
- 实机需确认 **`/left_joy`、`/right_joy`** 由 `joy_linux` 或自有节点发布。
