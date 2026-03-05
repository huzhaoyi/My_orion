# My_orion

ROS 2 工作空间，用于 Orion 机械臂的 MoveIt 运动规划与 PyBullet 仿真。

## 包结构

| 包名 | 说明 |
|------|------|
| **orion_description** | 机器人 URDF、网格与描述资源 |
| **orion_moveit_config** | MoveIt 配置（SRDF、关节限位、OMPL、Pilz PTP、运动学、控制器等） |
| **orion_mtc** | 基于 MoveIt Task Constructor 的抓放节点；物体位姿由话题输入，工业风格单路径（PTP pregrasp → LIN approach/lift，preplace → LIN lower → retreat），yaml 配置放置点与距离参数 |
| **orion_pybullet_sim** | PyBullet 仿真控制器：提供 `FollowJointTrajectory` action 与 `joint_states` 话题 |

## 依赖

- ROS 2（建议 Humble 或更高）
- MoveIt 2
- MoveIt Task Constructor（core + msgs）
- Pilz Industrial Motion Planner（PTP/LIN）
- PyBullet（`pip install pybullet`）

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
