# camera→sensor_link 旋转标定

- 按实测：sensor 三轴在 camera 中为列向量 (-1,0,0)、(0,0,1)、(0,1,0)，欧拉约 yaw=-90°、pitch=-180°、roll=0。
- `keypoint_arm_tf.launch.py` 中 `static_camera_to_sensor_link` 由单位四元数改为 **(qx,qy,qz,qw)=(0, √2/2, √2/2, 0)**，平移仍为 0。
- README 对应说明已更新。
