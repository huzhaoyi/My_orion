# 机械臂相对 ROV 的 TF 转换说明

机械臂基座相对 ROV 仅存在**平移**，无旋转。

---

## 1. 坐标系约定

| 坐标系 | 说明 |
|--------|------|
| **世界系 (world / map)** | HoloOcean 全局系。TargetSensor 的 `positions`、DynamicsSensorOdom 的 pose 均在此系，单位：米。 |
| **ROV 系 (COM / rov0)** | ROV 本体/质心坐标系。 |
| **机械臂 base_link** | Orion 左/右臂基座，MoveIt/MTC 在该系下规划。 |

---

## 2. 机械臂基座在 ROV 系下的平移（仅平移，无旋转）

在 **ROV 坐标系** 下，机械臂基座原点相对 ROV 原点的位置：

| 臂 | x [m] | y [m] | z [m] |
|----|-------|-------|-------|
| **左臂** | 1.55 | 0.5653 | -0.283628 |
| **右臂** | 1.55 | -0.5653 | -0.283628 |

即：

- **左臂 base_link 相对 ROV**：平移 `(1.55, 0.5653, -0.283628)`，旋转为单位阵。
- **右臂 base_link 相对 ROV**：平移 `(1.55, -0.5653, -0.283628)`，旋转为单位阵。

---

## 3. TF 树结构

```
holoocean_global_frame (或 map)
    └── rov0 / COM                    # 由 /holoocean/rov0/DynamicsSensorOdom 或 PoseSensor 驱动
            ├── left_arm_base_link     # 固定: 平移 (1.55, 0.5653, -0.283628)
            └── right_arm_base_link    # 固定: 平移 (1.55, -0.5653, -0.283628)
```

- **world → rov0**：由 HoloOcean 话题提供（位姿随 ROV 运动变化）。
- **rov0 → left_arm_base_link / right_arm_base_link**：静态平移，无旋转，可用 `StaticTransformBroadcaster` 发布。

---

## 4. 坐标变换公式

### 4.1 世界系 → ROV 系

由 `/holoocean/rov0/DynamicsSensorOdom` 得到 ROV 在世界系下的位姿：

- 位置：`pose.pose.position` → \( t_{world \to rov} \)
- 姿态：`pose.pose.orientation`（四元数）→ \( R_{world \to rov} \)

点从世界系到 ROV 系：

\[
p_{rov} = R_{world \to rov}^{-1} \, (p_{world} - t_{world \to rov})
\]

### 4.2 ROV 系 → 左/右臂 base_link（仅平移）

- **左臂**：\( t_{rov \to left} = (1.55,\, 0.5653,\, -0.283628)^T \)  
  \( p_{left} = p_{rov} - t_{rov \to left} \)

- **右臂**：\( t_{rov \to right} = (1.55,\, -0.5653,\, -0.283628)^T \)  
  \( p_{right} = p_{rov} - t_{rov \to right} \)

### 4.3 世界系抓取点 → 左臂 base_link（供 MTC /object_pose）

1. 世界 → ROV：\( p_{rov} = R_{world \to rov}^{-1} (p_{world} - t_{world \to rov}) \)
2. ROV → 左臂：\( p_{left} = p_{rov} - (1.55,\, 0.5653,\, -0.283628)^T \)

右臂同理，平移改为 \( (1.55,\, -0.5653,\, -0.283628)^T \)。

---

## 5. 代码用常量

```python
# 左臂 base 在 ROV 系下 [m]，仅平移
LEFT_ARM_BASE_IN_ROV_X = 1.55
LEFT_ARM_BASE_IN_ROV_Y = 0.5653
LEFT_ARM_BASE_IN_ROV_Z = -0.283628

# 右臂 base 在 ROV 系下 [m]，仅平移
RIGHT_ARM_BASE_IN_ROV_X = 1.55
RIGHT_ARM_BASE_IN_ROV_Y = -0.5653
RIGHT_ARM_BASE_IN_ROV_Z = -0.283628
```

---

## 6. 相关话题

| 话题 | 用途 |
|------|------|
| `/holoocean/rov0/DynamicsSensorOdom` | 世界系下 ROV 位姿 (nav_msgs/Odometry, frame_id=holoocean_global_frame) |
| `/holoocean/rov0/TargetSensor` | 世界系下抓取目标 positions + directions |
| `/object_pose` | MTC 输入，需为 base_link 下的 PoseStamped |
