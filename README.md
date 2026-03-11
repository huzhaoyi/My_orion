# My_orion

ROS 2 工作空间，用于 Orion 机械臂与 **HoloOcean** 仿真联调：关节状态来自 HoloOcean ArmSensor，MoveIt 规划、MTC 抓放，轨迹经桥接发给 HoloOcean 执行。

## 包结构

| 包名 | 说明 |
|------|------|
| **orion_description** | 机器人 URDF、网格与描述资源 |
| **orion_moveit_config** | MoveIt 配置（SRDF、关节限位、OMPL、Pilz PTP/LIN、运动学、控制器等） |
| **orion_mtc_msgs** | 抓放接口定义：Pick/Place Action、GetRobotState 服务 |
| **orion_mtc** | 基于 MoveIt Task Constructor 的抓放节点；抓取与放置拆成独立接口，底层共享规划与执行；话题触发或 Action 调用，状态机 IDLE→PICKING→HOLDING→PLACING→IDLE |
| **orion_holoocean_bridge** | HoloOcean 桥接：ArmSensor → `joint_states`，轨迹 → HoloOcean Agent；TargetSensor + ROV 里程计 → `/object_pose` |

## 依赖

- ROS 2（建议 Humble 或更高）
- MoveIt 2
- MoveIt Task Constructor（core + msgs）
- Pilz Industrial Motion Planner（PTP/LIN）
- holoocean-ros（含 `holoocean_interfaces`），通过环境变量 `HOLOOCEAN_ROS_INSTALL` 指定其 install 目录，或先 `source` 该工作区

## 构建

```bash
cd /path/to/My_orion
colcon build --symlink-install
source install/setup.bash
```

## orion_mtc 接口说明

抓取与放置为两个独立业务能力，通过状态 **HOLDING** 衔接：

- **抓取**：选目标、生成抓取位姿、闭合夹爪、确认抓稳后保存持物上下文（含 `tcp_to_object`），状态变为 HOLDING。
- **放置**：仅在 HOLDING 时可用；输入为**物体目标位姿**（非末端位姿），内部用持物上下文计算 TCP 目标，执行 pre-place → lower → open → detach → retreat 后清空持物、回到 IDLE。

| 类型 | 名称 | 说明 |
|------|------|------|
| Action | `/manipulator/pick` | 抓取：Goal 为 `object_pose`（base_link）、可选 `object_id`；Result 含 `success`、`task_id`、`held_object_id` |
| Action | `/manipulator/place` | 放置：Goal 为 `target_pose`（物体目标位姿，base_link）；Result 含 `success`、`task_id` |
| 服务 | `/manipulator/get_robot_state` | 返回当前 `mode`、`task_id`、`held_object_id`、`has_held_object`、`last_error` |
| 话题 | `/manipulator/object_pose` | 物体位姿（PoseStamped，frame_id=base_link），抓取使用；由桥接 target_sensor 自动发布 |
| 话题 | `/manipulator/place_pose` | 放置目标位姿（PoseStamped），放置使用 |
| 话题 | `/manipulator/pick_trigger` | 空消息：仅执行**抓取**（需有 `/manipulator/object_pose`） |
| 话题 | `/manipulator/place_trigger` | 空消息：仅执行**放置**（需有 `/manipulator/place_pose`，且当前为 HOLDING） |

业务规则：未持物时禁止 place；已持物时禁止再次 pick；放置成功后清空持物并回到 IDLE。

## 运行（HoloOcean 联调）

关节状态来自 HoloOcean 的 ArmSensor，规划在 MoveIt 中完成，轨迹通过桥接节点发给 HoloOcean 执行。

**启动：**

```bash
# 确保已设置 HOLOOCEAN_ROS_INSTALL 或已 source holoocean-ros 的 install
ros2 launch orion_mtc pick_place_holoocean.launch.py
```

物体位姿由 `target_sensor_to_object_pose` 从 TargetSensor + ROV 里程计**自动发布**到 `/manipulator/object_pose`（默认抓取 `目标[1]`）。流程为**拆分接口**：先抓取再放置（或 place_release），无一体化 pick-place 话题。所有接口均在 `/manipulator` 命名空间下。

**前置条件（抓取）**：`/manipulator/pick_trigger` 和话题式抓取都依赖 **已有** `/manipulator/object_pose`。若 HoloOcean 未跑或暂无目标数据，`target_sensor_to_object_pose` 不会发布，会出现 “doPick: no object pose after wait, abort”。此时可：① 先启动 HoloOcean 并确保有目标被跟踪；或 ② 手动发布一次 object_pose 再触发 pick（见下）；或 ③ 直接用 Action `/manipulator/pick` 并在 Goal 里带 `object_pose`（无需话题）。

**触发方式：**

- 仅抓取（需已存在 `/manipulator/object_pose`，例如由 target_sensor 发布）：
  ```bash
  ros2 topic pub --once /manipulator/pick_trigger std_msgs/msg/Empty "{}"
  ```
- 无 HoloOcean/目标时，先手动发物体位姿再抓取：
  ```bash
  ros2 topic pub --once /manipulator/object_pose geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: 'base_link'}, pose: {position: {x: 0.35, y: -0.15, z: 0.4}, orientation: {w: 1.0}}}"
  ros2 topic pub --once /manipulator/pick_trigger std_msgs/msg/Empty "{}"
  ```
- 仅放置（需先抓取成功，再发 `/manipulator/place_pose` 后触发）：
  ```bash
  ros2 topic pub --once /manipulator/place_pose geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: 'base_link'}, pose: {position: {x: 0.45, y: 0.0, z: 0.4}, orientation: {w: 1.0}}}"
  ros2 topic pub --once /manipulator/place_trigger std_msgs/msg/Empty "{}"
  ```
- Action 仅抓取：
  ```bash
  ros2 action send_goal /manipulator/pick orion_mtc_msgs/action/Pick \
    "{object_pose: {header: {frame_id: 'base_link'}, pose: {position: {x: 0.35, y: -0.15, z: 0.4}, orientation: {w: 1.0}}}, object_id: 'cube_1'}"
  ```
- Action 仅放置（当前须为 HOLDING）：
  ```bash
  ros2 action send_goal /manipulator/place orion_mtc_msgs/action/Place \
    "{target_pose: {header: {frame_id: 'base_link'}, pose: {position: {x: 0.45, y: 0.0, z: 0.4}, orientation: {w: 1.0}}}}"
  ```

**配置：**

- 桥接与目标索引：`orion_holoocean_bridge/config/holoocean_bridge_params.yaml`（如 `target_index: 1` 对应 `目标[1]`）
- 抓放参数：`orion_mtc/config/pick_place_params.yaml`（approach/lift/place/retreat/lower 等）

**仅查看机器人模型：**

```bash
ros2 launch orion_description display.launch.py
```

**仅 MoveIt + RViz（无 MTC，关节由 GUI 或外部发布）：**

```bash
ros2 launch orion_moveit_config demo.launch.py
```

## 如何测试

1. **编译与接口检查**

   ```bash
   cd /path/to/My_orion
   colcon build --symlink-install
   source install/setup.bash
   ros2 interface show orion_mtc_msgs/action/Pick
   ros2 interface show orion_mtc_msgs/action/Place
   ros2 interface show orion_mtc_msgs/srv/GetRobotState
   ```

2. **启动 HoloOcean 场景与 launch**

   终端 1：`ros2 launch orion_mtc pick_place_holoocean.launch.py`

3. **终端 2：触发与状态查询**

   - 先抓取：`ros2 topic pub --once /manipulator/pick_trigger std_msgs/msg/Empty "{}"`；再查状态 `ros2 service call /manipulator/get_robot_state orion_mtc_msgs/srv/GetRobotState "{}"`（应为 HOLDING）；再发 `/manipulator/place_pose` 与 `ros2 topic pub --once /manipulator/place_trigger std_msgs/msg/Empty "{}"`
   - 或使用 Action：`/manipulator/pick`、`/manipulator/place`、`/manipulator/place_release`
   - 状态查询：`ros2 service call /manipulator/get_robot_state orion_mtc_msgs/srv/GetRobotState "{}"`

4. **业务规则**：未抓取时发 place 应被拒绝；已持物时再发 pick 应被拒绝。

## 许可证

Apache-2.0
