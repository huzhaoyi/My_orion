# keypoint_arm_tf：base_link 与感知 TF 树断开

## 现象

单独 `ros2 launch orion_mtc keypoint_arm_tf.launch.py` 时出现：

`Could not find a connection between 'base_link' and 'left_arm_base' because they are not part of the same tree`

## 原因

离线模式仅发布 `camera→sensor_link→left_arm_base`，没有任何变换把 `base_link` 接到该链上；若节点或下游要把左臂系下的位姿变换到 `base_link`，TF 必失败。

## 处理

在 `keypoint_arm_tf.launch.py` 的非平台分支增加 `base_link→camera` 单位静态 TF，使链路为：

`base_link → camera → sensor_link → left_arm_base`（及右臂）

**假设**：单机调试时认为 `base_link` 与 `camera` 重合；若实际安装有外参，应把该 static 的平移/四元数改成标定值，或改用 `use_platform_tf:=true` 接入整机 URDF。

## 文件

- `src/orion_mtc/launch/keypoint_arm_tf.launch.py`
