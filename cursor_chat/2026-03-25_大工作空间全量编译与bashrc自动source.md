# 2026-03-25 大工作空间全量编译与 bashrc 自动 source

## 完成内容

- 在 `~/sealien_ws` 根目录执行：`colcon build --symlink-install`（已先 `source /opt/ros/humble/setup.bash`）。
- 9 个包均编译成功：`holoocean_interfaces`、`holoocean_main`、`holoocean_examples`、`orion_description`、`orion_moveit_config`、`orion_mtc_msgs`、`orion_holoocean_bridge`、`orion_mtc`、`sealien_ctrlpilot_msgmanagement`。
- 在 `~/.bashrc` 中于 fishros 的 Humble `setup.bash` 之后增加：若存在 `~/sealien_ws/install/setup.bash` 则自动 `source`，新 bash 终端无需再手动 source 工作空间。

## 说明

- 修改 `.bashrc` 后需 **新开终端** 或执行 `source ~/.bashrc` 才生效。
- 重新编译后 overlay 仍为同一 `install/setup.bash` 路径，一般无需改 `.bashrc`。
