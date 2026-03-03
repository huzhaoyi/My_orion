# My_orion

ROS 2 工作空间，用于 Orion 机械臂的 MoveIt 运动规划与 PyBullet 仿真。

## 包结构

| 包名 | 说明 |
|------|------|
| **orion_description** | 机器人 URDF、网格与描述资源 |
| **orion_moveit_config** | MoveIt 配置（SRDF、关节限位、OMPL、Pilz PTP、运动学、控制器等） |
| **orion_mtc** | 基于 MoveIt Task Constructor 的抓放节点（PTP 远距离连接 + 笛卡尔接近/抬起/撤离） |
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

### 抓放（PyBullet 仿真）

唯一入口：启动 MoveIt + RViz + PyBullet 控制器，MTC 节点延迟约 12 秒后自动执行一次抓放，轨迹在仿真中执行并在 RViz 中显示：

```bash
ros2 launch orion_mtc pick_place_pybullet.launch.py
```

### 仅查看机器人模型

```bash
ros2 launch orion_description display.launch.py
```

### 仅 MoveIt Demo（无 MTC、无 PyBullet）

```bash
ros2 launch orion_moveit_config demo.launch.py
```

## 许可证

Apache-2.0
