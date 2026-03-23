# 2026-03-23 测试结论

## 已执行

- `colcon build --packages-select orion_mtc orion_moveit_config`：成功。
- `src/orion_moveit_config/scripts/moveit_smoke_test.sh`：通过（`You can start planning now!`，无 Pilz 减速度失败）。

## 未执行（本环境）

- `pick_holoocean.launch.py` + 真实 PICK，故 **`pick_side_summary` 的 grasp/pregrasp 计数无法在 CI 中验证**，需在 HoloOcean 联调后看一行日志。

## 结论

- **编译与 MoveIt 冒烟**：正常，可作为发布前基线。
- **侧抓统计逻辑**：源码已含 `pick_side_summary`，联调时用 `grep pick_side_summary` 即可。
