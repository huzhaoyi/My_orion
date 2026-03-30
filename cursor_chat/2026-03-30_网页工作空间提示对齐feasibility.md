# 网页工作空间提示对齐 feasibility

## 内容

- 新增 **`web/js/data/feasibilityWorkspace.js`**：与 `orion_mtc_params.yaml` 的 `feasibility` 数值一致，供前端单点维护。
- **`RightPanel.js`**：任务栏「工作空间」展示 ‖p‖ 硬限、Z 硬限、软 ‖p‖、示意框 X/Y；说明与 `orion_mtc_params` 一致。
- **`RobotModelLoader.js`**：FK 延伸至 **gripper_tcp**（+0.108 m）；`i===3` 继续使用 Link3→Virtual 固定 rpy，TCP 段为纯平移；粗采样点按 feasibility 过滤后求 AABB；`getWorkspaceBoundsForDoc` 增加 `radial_m` / `soft_reach_m`。
- **`RobotScene.js` / `theme.css`**：注释与样式说明更新。
- **`docs/workspace_limits.md`、README**：与硬限表及前端数据源对齐。
