# HoloOcean 重启后 TF_OLD_DATA：ROV TF 改用节点时钟

## 现象

HoloOcean 重启后仿真时间回退，`map→rov0` 仍用 PoseSensor 的 `header.stamp`，tf2 拒绝“时间倒流”的变换，出现 `TF_OLD_DATA ignoring data from the past for frame rov0`，表现为侧似“未重新连接”。

## 原因

- `joint_states` 桥接已用 `get_clock().now()`；
- `cable` 的 `object_pose` 已用节点时钟；
- 唯独 `cable_sensor_to_object_pose` / `target_sensor_to_object_pose` 中 `map→rov0` 曾用 `msg.header.stamp`（仿真时间）。

## 修改

- 两节点增加参数 `use_pose_sensor_stamp_for_rov_tf`（默认 `false`）：为 `false` 时 ROV TF 与 perception 中 ROV 相关 stamp 使用 `get_clock().now()`；`true` 时保持旧行为。
- `holoocean_bridge_params.yaml`、README 已同步说明。

## 文件

- `orion_holoocean_bridge/.../cable_sensor_to_object_pose_node.py`
- `orion_holoocean_bridge/.../target_sensor_to_object_pose_node.py`
- `orion_holoocean_bridge/config/holoocean_bridge_params.yaml`
- `README.md`
