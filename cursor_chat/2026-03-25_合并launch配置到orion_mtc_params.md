# 2026-03-25 合并 launch 配置到 orion_mtc_params

## 进展

删除 **`pick_holoocean_launch.yaml`**，将 **`use_joy_manipulator`** 并入 **`orion_mtc/config/orion_mtc_params.yaml`**。`pick_holoocean.launch.py` 读入后 **从传入 mtc_node 的参数字典中剔除** 该键，仅用于 **`DeclareLaunchArgument` 默认值**；README 已同步。

## 使用

在 `orion_mtc_params.yaml` 末尾编辑 `use_joy_manipulator: true/false`，`colcon build` + `source` 后生效（开发与 install 路径一致时亦可直接改 `share` 下副本）。
