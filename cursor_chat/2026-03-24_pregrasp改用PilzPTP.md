# pregrasp 规划由 OMPL 改为 Pilz PTP

## 变更

- 文件：`src/orion_mtc/src/planning/pick_task_builder.cpp`
- `MoveTo("move to pregrasp")` 由 `PipelinePlanner(..., "move_group")` 改为与 `move to ready` 相同的 `ptp_planner`（`pilz` + `PTP`）。
- 删除未再使用的 `ompl_planner` 实例。

## 目的

减少 OMPL 采样随机性导致的同条件下轨迹形态不一致（拧、抖、抓偏）。

## 验证

- `colcon build --packages-select orion_mtc` 通过。
