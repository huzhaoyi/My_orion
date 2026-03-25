# keypoint 与 camera TF

- `keypoint_arm_tf.launch.py` 增加 `camera`→`sensor_link` 静态单位变换，使融合 `header.frame_id:=camera` 时能连到 `sensor_link`→`left_arm_base`/`right_arm_base`。
- `keypoint_to_arm_tf_node` 增加参数 `force_source_frame`（非空则忽略 header 帧名）。
- README 对应小节已更新。
