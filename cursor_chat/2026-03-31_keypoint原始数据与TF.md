# Keypoints：取消强制 2.9/0，走原始数据 + TF

## 需求

不要通过 launch/默认值把 keypoint 的 y、z（及 mock）钉死在 2.9、0 等；真机应以**消息里的原始坐标**和 **header.frame_id**，只做 TF 变换。

## 修改

1. **`keypoint_arm_tf.launch.py`**：三处 `keypoint_params` 删除 `mock_kp_x/y/z` 传递；mock 时由节点默认参数或运行时显式传入。
2. **`keypoint_to_arm_tf_node.cpp`**：
   - `mock_kp_y` 默认由 2.9 改为 **0**（与 x、z 一致）。
   - `override_keypoint_y_value` 默认由 2.9 改为 **0**（仅当 `override_all_keypoints_y:=true` 才改写）。
   - `MOCK_KP_SONAR_CABLE_9`：**保留 x 向采样间距**，y/z 改为 **0**（离线最小占位链），注释说明真机应订阅原始 Keypoints。
3. **`README.md`**：Keypoint 调试段改为强调默认使用消息内坐标 + TF，删除“典型 (0,2.9,0)”与强制常数表述。

## 行为说明

- **订阅模式**（`use_mock_keypoints:=false`）：逻辑本就只对 `msg->keypoints` 做 TF；`override_all_keypoints_*` 默认 false，不受影响。
- **Mock 模式**：`legacy_single` 默认点为原点；`sonar_cable_9` 为沿 x 的测试链；需要特定离线点时请在 launch/参数中显式设 `mock_kp_*`。
