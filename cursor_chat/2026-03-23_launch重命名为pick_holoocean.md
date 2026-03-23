# 2026-03-23 launch 重命名为 pick_holoocean

## 当日进展

将 `src/orion_mtc/launch/pick_place_holoocean.launch.py` 重命名为 **`pick_holoocean.launch.py`**（与仅抓取能力一致），并更新 **README** 中全部 `ros2 launch orion_mtc ...` 引用。

## 完成情况

**已完成**：仓库内 `grep` 无 `pick_place_holoocean` 残留；`orion_mtc/launch` 下仅保留 `pick_holoocean.launch.py`；launch 文件头注释由「Pick-and-place」改为「抓取（MTC）与 HoloOcean 联调」。

## 问题/需求

无。旧脚本若写死 `pick_place_holoocean.launch.py` 需人工改为 `pick_holoocean.launch.py`。
