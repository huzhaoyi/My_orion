# executePickSolution 诊断与当前代码状态

## 结论（与对话一致）

- **DONE 早于整次 pick 收口**：`stage_report(..., DONE)` 在 `executeSubTrajectory` 成功后立即发出，段后处理（scene 清理、FK、`wait_for_gripped`）仍在本段循环内执行，因此会出现「前端已 DONE、后端仍未回到 `handlePick` 成功分支」的时间窗。

## 本仓库当前分支上的三项修复（已存在，非本次新增）

1. **`hand_frame`**：`solution_executor.cpp` 中 `executePickSolution` 已使用 `"gripper_tcp"`（非 `"Link6"`）。
2. **`remove_cable_segments`**：已按调用方传入的 `cable_world_object_ids` 逐项 `removeWorldObject`，无硬编码 16 段。
3. **阶段名对齐**：`task_manager.cpp` 中 `PICK_STAGE_NAMES_CABLE_SIDE` 已含首项 `"current"`，注释说明与 MTC 子轨迹顺序对齐。

## 本次改动：段后处理诊断日志

在 `solution_executor.cpp` 的 `executePickSolution` 中增加：

- 每段在发出 DONE 之后：`segment %zu DONE, entering post-process name=%s`
- 每段整段后处理结束后：`segment %zu post-process finished name=%s`
- 全部段结束后、`return true` 前：`all segments finished, return true`

**解读**：若某段后出现第一条日志而无对应 `post-process finished`，则卡在该段的后处理（scene 删除、FK、`wait_for_gripped` 等）；若所有段均有 `post-process finished` 却无 `all segments finished`，则逻辑上不应发生（需再查）；若已有 `all segments finished` 而 job 仍未收口，则需查 `handlePick` 之后链路。

## 构建

`colcon build --packages-select orion_mtc` 已通过。
