# My_orion

ROS 2 工作空间，用于 Orion 机械臂的 MoveIt 运动规划，支持 **PyBullet 仿真** 与 **HoloOcean** 联调。

## 包结构

| 包名 | 说明 |
|------|------|
| **orion_description** | 机器人 URDF、网格与描述资源 |
| **orion_moveit_config** | MoveIt 配置（SRDF、关节限位、OMPL、Pilz PTP/LIN、运动学、控制器等） |
| **orion_mtc** | 基于 MoveIt Task Constructor 的抓放节点；物体位姿由话题输入，工业风格单路径（PTP pregrasp → LIN approach/lift，preplace → LIN lower → retreat），yaml 配置放置点与距离参数 |
| **orion_pybullet_sim** | PyBullet 仿真控制器：提供 `FollowJointTrajectory` action 与 `joint_states` 话题 |
| **orion_holoocean_bridge** | HoloOcean 桥接：将 ArmSensor 转为 Orion `joint_states`（右臂 6DOF+夹爪），并将轨迹转发给 HoloOcean Agent 执行 |

## 依赖

- ROS 2（建议 Humble 或更高）
- MoveIt 2
- MoveIt Task Constructor（core + msgs）
- Pilz Industrial Motion Planner（PTP/LIN）
- PyBullet（`pip install pybullet`）
- **HoloOcean 联调时**：holoocean-ros（含 `holoocean_interfaces`），需通过环境变量 `HOLOOCEAN_ROS_INSTALL` 指定其 install 目录，或先 `source` 该工作区

## 构建

```bash
cd /path/to/My_orion
colcon build --symlink-install
source install/setup.bash
```

## 运行

### 抓放（PyBullet 仿真，话题驱动）

启动 MoveIt + RViz + PyBullet 控制器；**不自动执行**，需通过话题触发：

1. 启动：
   ```bash
   ros2 launch orion_mtc pick_place_pybullet.launch.py
   ```
2. 发布物体位姿（`base_link` 下，`geometry_msgs/msg/PoseStamped`）：
   ```bash
   ros2 topic pub --once /object_pose geometry_msgs/msg/PoseStamped \
     "{header: {frame_id: 'base_link'}, pose: {position: {x: 0.35, y: -0.15, z: 0.4}, orientation: {w: 1.0}}}"
   ```
3. 发布触发执行一次 pick-place：
   ```bash
   ros2 topic pub --once /pick_place_trigger std_msgs/msg/Empty "{}"
   ```

轨迹在 PyBullet 仿真中执行并在 RViz 中显示。抓取与放置均由 orion_mtc 内部根据物体位姿与 yaml 放置目标计算（pregrasp = 物体上方，preplace = 放置点上方，再 LIN 下压/下降）。

- **话题**：`/object_pose`（物体位姿，frame_id 须为 `base_link`）、`/pick_place_trigger`（触发一次 pick-place）
- **yaml 配置**：`orion_mtc/config/pick_place_params.yaml` 中配置 approach/lift/place/retreat/lower 等距离与放置目标，物体位姿仅由话题提供

### 规划（MoveIt + HoloOcean 仿真联调）

关节状态来自 HoloOcean 的 ArmSensor（`/holoocean/rov0/ArmSensor`），规划在 MoveIt 中完成，轨迹通过桥接节点发给 HoloOcean 执行。

**测试流程：**

1. 确保已安装 holoocean-ros 并设置 `HOLOOCEAN_ROS_INSTALL`（或已 source 其 install）。
2. 启动：
   ```bash
   ros2 launch orion_mtc pick_place_holoocean.launch.py
   ```
3. 物体位姿由 `target_sensor_to_object_pose` 从 TargetSensor + ROV 里程计自动发布到 `/object_pose`（默认抓取 `目标[1]`）；若需手动测试可自行发布位姿。
4. 发布空消息触发一次 pick-place：
   ```bash
   ros2 topic pub --once /pick_place_trigger std_msgs/msg/Empty "{}"
   ```

桥接节点：`arm_sensor_to_joint_state`（ArmSensor → `joint_states`）、`trajectory_to_agent_bridge`（轨迹 → HoloOcean Agent）、`target_sensor_to_object_pose`（TargetSensor + ROV 里程计 → `object_pose`）。参数见 `orion_holoocean_bridge/config/holoocean_bridge_params.yaml`。

- **TargetSensor 调试输出**：`target_sensor_to_object_pose` 会逐行打印 `ROV` 当前 world 坐标，并逐行打印 `目标[0]`、`目标[1]`、`目标[2]` 的 `world -> base_link` 变换结果（日志节流约 1Hz）。
- **抓取目标选择**：默认抓取目标为 `target_index: 1`（即 `目标[1]`），可在 `orion_holoocean_bridge/config/holoocean_bridge_params.yaml` 中修改。

### 仅查看机器人模型

```bash
ros2 launch orion_description display.launch.py
```

### 仅 MoveIt Demo（无 MTC、有 PyBullet）

```bash
ros2 launch orion_pybullet_sim pybullet_sim.launch.py
```

## 许可证

Apache-2.0
