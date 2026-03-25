# 2026-03-25 colcon build 后自动 source

## 进展

新增 **`scripts/colcon_build_source.sh`**：在推断的 colcon 工作空间根执行 `colcon build`，完成后 **`source <ws>/install/setup.bash`** 并打印提示；支持环境变量 **`COLCON_WS`** 覆盖工作空间路径。**README「构建」** 改为以工作空间为基准，并说明未 source 会导致 `PackageNotFoundError`（如 `orion_joy_arm_bridge`）。

## 使用

自仓库目录：`./scripts/colcon_build_source.sh --symlink-install`  
或：`COLCON_WS=/path/to/sealien_ws /path/to/.../colcon_build_source.sh`
