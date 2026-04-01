# Keypoints QoS：按 topic info 与发布端对齐

## 现场

`node_camera2sonar_keypoints_publisher` 发布 **RELIABLE**；此前节点默认改为 **BEST_EFFORT** 后，`keypoint_to_arm_tf` 与发布端不一致（仅该订阅为 BE）。

## 当前策略

- `qos_best_effort` 默认 **false**（**RELIABLE** 订阅），与 **RELIABLE 发布**一致。
- 若某环境发布为 **BEST_EFFORT**，再设 **`qos_best_effort:=true`**。
- README：`ros2 topic echo` 须用**完整话题名**，不可用字面 `...`。

## 命令示例

```bash
ros2 topic echo /perception/sonar/keypoints
# 发布为 BEST_EFFORT 时：
# ros2 topic echo /perception/sonar/keypoints --qos-reliability best_effort
```
