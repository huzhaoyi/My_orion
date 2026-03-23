# 合并 pick + runtime yaml；命名与多 yaml 说明

## 历史「放置」与 build 产物

- **`src/` 下已无文件名含 `place`**（放置能力已移除）。
- **`build/`** 若仍见 `place_*.hpp` 等，为历史编译残留；整仓 `rm -rf build install log` 后重编可清。
- **`orion_mtc_msgs/package.xml`**：`pick/place` → `pick / manipulation`。
- **`orion_mtc/package.xml`**：`pick-and-place` → `cable pick and manipulation`。

## 配置合并

- 新增 **`src/orion_mtc/config/orion_mtc_params.yaml`**：`grasp_offset` / `cable_side_grasp` / `feasibility` + **`runtime_policy`** 同文件顶层键。
- 删除 **`pick_params.yaml`**、**`runtime_policy.yaml`**。
- **`pick_holoocean.launch.py`**：只加载 `orion_mtc_params.yaml` 并 `move_group_params.update(mtc_app)`。
- **README**、**`tools/compute_max_reach_from_urdf.py`**：路径与说明已更新。

## 其他 yaml 未合并原因

- **`orion_moveit_config/config/`**（joint_limits、ompl、pilz、kinematics、controllers 等）：MoveIt 惯例为多文件，合并会难维护且易与工具链假设冲突。
- **`orion_holoocean_bridge/config/holoocean_bridge_params.yaml`**：属另一包、另一节点，与 MTC 应用参数职责不同，保持独立。
