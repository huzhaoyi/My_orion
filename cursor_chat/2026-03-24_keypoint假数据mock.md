# keypoint_to_arm_tf 假数据模式

- 参数 **`use_mock_keypoints`**：为 true 时不创建 `/keypoints` 订阅，用 **`mock_period_sec`** 定时器注入一条与 echo 样例一致的 Keypoints（默认 `frame_id:=camera`，点 (0, 2.905448…, ~0)）。
- `keypoint_arm_tf.launch.py` 增加启动参数 **`use_mock_keypoints`**（默认 false），示例：`use_mock_keypoints:=true`。
- README 已补充说明。
