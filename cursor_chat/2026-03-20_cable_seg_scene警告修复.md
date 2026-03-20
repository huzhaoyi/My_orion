# cable_seg 规划场景重复删除告警修复

- **现象**：`move_group` 报 `Tried to remove world object 'cable_seg_*', but it does not exist`。
- **原因**：`remove_cable_segments` 的 `scene_diff` 已在 `executeSubTrajectory` 中应用；`executePickSolution` 又对同 id 调用 `removeWorldObject`，造成重复 REMOVE。
- **修改**：去掉 `executePickSolution` 中的缆绳段 `removeWorldObject` 循环；删除 `cable_world_object_ids` 参数及 `task_manager` 中的拼装与传入；头文件注释说明。
- **验证**：`colcon build --packages-select orion_mtc` 通过。
