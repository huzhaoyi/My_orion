# 方案1：base_link→camera 外参（与 left_arm_base 对齐）

## 问题

`tf_under` 下原 **`base_link→camera` 单位**，Keypoints 变到 `base_link` 后呈 **y≈2.4** 的声呐系，与 **`/object_pose` 桥接** (~1.0, -0.57, 0.28) 不一致。

## 做法

令 **独立调试** 中 **`sensor_link→left_arm_base`** 平移 **(1.55, 0.5653, -0.283628)**、`**camera→sensor_link**` 四元数 **(0.5,0.5,0.5,0.5)** 不变；在 **`tf_under`** 分支将 **`base_link→camera`** 设为

**inv(camera→sensor→left_arm_base)**：

- 平移 **(-1.55, -0.5653, 0.283628)**
- 四元数 **(qx,qy,qz,qw) = (0.5, 0.5, 0.5, -0.5)**

则在数学上 **URDF `base_link` 与调试用 `left_arm_base` 原点/轴向对齐**，camera 下典型点变到 `base_link` 后量级与桥接一致。

## 维护

若修改 **`sensor_link→left_arm_base`** 或 **`camera→sensor_link`**，须按同一几何重算 **`base_link→camera`**。

## 文件

`src/orion_mtc/launch/keypoint_arm_tf.launch.py`（`static_base_link_to_camera`）；`README.md` 一句说明。
