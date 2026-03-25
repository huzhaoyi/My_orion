# /keypoints 类型与 cable_detect 对齐

**原因**：同一 topic 上 `cable_detect` 发布 `sealien_ctrlpilot_msgmanagement/msg/Keypoints`，原节点订阅 `orion_mtc_msgs/msg/Keypoints`，ROS2 中类型名不同即无法匹配。

**修改**：`keypoint_to_arm_tf_node` 改为订阅 `sealien_ctrlpilot_msgmanagement::msg::Keypoints`；`package.xml`/`CMakeLists.txt` 增加对 `sealien_ctrlpilot_msgmanagement` 的依赖；默认 `qos_best_effort=false` 与 Reliable 发布一致；README 与 launch 参数已更新。

**构建**：需先 `source` sealien 的 `install/setup.bash` 再编译 `orion_mtc`。
