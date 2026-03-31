# tf2 Buffer 带 timeout 查询报错修复

## 现象

`tf2_buffer` ERROR：在非专用线程填充 buffer 时对 `canTransform`/`lookupTransform` 使用 timeout。

## 原因

`TransformListener(..., spin_thread=false)` 时 `/tf` 仅随主 executor 回调，与 `transform(..., timeout)` 同线程阻塞等待冲突。

## 修改

`keypoint_to_arm_tf_node` 创建 `TransformListener` 时改为 **`spin_thread=true`**（与 ROS2 默认一致），由专用线程收 TF 并自动 `setUsingDedicatedThread(true)`。

## 构建

`colcon build --packages-select orion_mtc` 已通过。
