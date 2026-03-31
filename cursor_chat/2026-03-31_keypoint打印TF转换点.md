# keypoint 打印 TF 转换后坐标

## 需求

在终端打印每个关键点经 TF 变换到左右臂坐标系后的坐标。

## 实现

- **`keypoint_to_arm_tf_node.cpp`**：`log_each_keypoint_tf` 为 true 时，将原先的 `RCLCPP_DEBUG` 改为单行 **`RCLCPP_INFO`**：`kp[i] src=<frame> (x,y,z) | <left> (...) | <right> (...)`。
- **`keypoint_arm_tf.launch.py`**：新增参数 **`log_each_keypoint_tf`**（默认 **true**），写入三档 `keypoint_params`。
- **`pick_holoocean.launch.py`**：include keypoint 时传入 **`log_each_keypoint_tf:=false`**，避免与 MTC 联调刷屏。

单机调试直接 `ros2 launch orion_mtc keypoint_arm_tf.launch.py` 即可逐点打印；关闭打印：`log_each_keypoint_tf:=false`。
