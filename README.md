# My_orion

ROS 2 工作空间，用于 Orion 机械臂与 **HoloOcean** 仿真联调：关节状态来自 HoloOcean ArmSensor，MoveIt 规划、MTC 抓放，轨迹经桥接发给 HoloOcean 执行。

## 包结构

| 包名 | 说明 |
|------|------|
| **orion_description** | 机器人 URDF、网格与描述资源 |
| **orion_moveit_config** | MoveIt 配置（SRDF、关节限位、OMPL、Pilz PTP/LIN、运动学、控制器等） |
| **orion_mtc_msgs** | 抓放接口定义：Pick/Place/PlaceRelease Action，GetRobotState、GetQueueState、SubmitJob、CancelJob、GetRecentJobs、ResetHeldObject、SyncHeldObject 服务 |
| **orion_mtc** | 基于 MoveIt Task Constructor 的抓放节点；任务队列 + 后台 Worker 消费，支持优先级与去重；**抓取**：多目标 TargetCache → TargetSelector → GraspGenerator 多候选尝试；**放置**：PlaceGenerator 多候选（位置/yaw 采样）依次尝试；Action 即时执行，话题/SubmitJob 异步入队；RuntimePolicy 与恢复动作可配置 |
| **orion_holoocean_bridge** | HoloOcean 桥接：ArmSensor → `joint_states`，轨迹 → HoloOcean Agent；TargetSensor + ROV 里程计 → `/object_pose` 与 `/target_set`（多目标 base_link） |

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

抓取与放置为独立业务能力，通过状态 **HOLDING** 衔接。任务可经 **Action（即时执行）** 或 **话题/SubmitJob（异步入队）** 提交；队列按优先级调度，支持去重与取消未执行 job。

- **抓取**：有 `/manipulator/target_set` 时，从多目标中选一个（TargetSelector）、生成多抓取候选（GraspGenerator）并依次尝试；无 target_set 时回退到单 `/object_pose`。抓稳后保存持物上下文，状态变为 HOLDING_TRACKED/HOLDING_UNTRACKED。
- **放置**：仅在 HOLDING 时可用；输入为物体目标位姿，PlaceGenerator 在目标位附近采样多候选（位置 ±1cm、yaw 0°/±10°/±20°），依次规划/执行直至成功或耗尽，再执行 pre-place → lower → open → detach → retreat 后清空持物、回到 IDLE。
- **放置释放**（PlaceRelease）：按 TCP 目标位姿执行放置并 detach，适用于无跟踪的持物。

| 类型 | 名称 | 说明 |
|------|------|------|
| Action | `/manipulator/pick` | 即时抓取：Goal 为 `object_pose`（base_link）、可选 `object_id` |
| Action | `/manipulator/place` | 即时放置：Goal 为 `target_pose`（物体目标位姿，base_link） |
| Action | `/manipulator/place_release` | 即时放置释放：Goal 为 `target_tcp_pose` |
| 服务 | `/manipulator/get_robot_state` | 当前 `mode`、`task_id`、`held_object_id`、`has_held_object`、`last_error` |
| 服务 | `/manipulator/get_queue_state` | 队列与 Worker：`queue_size`、`current_job_id/type`、`next_job_id/type`、`worker_status`、`worker_running`、`queue_empty`、`last_error` |
| 服务 | `/manipulator/submit_job` | 异步入队：提交 PICK/PLACE/PLACE_RELEASE/RESET_HELD_OBJECT/SYNC_HELD_OBJECT，返回 `job_id` 或拒绝原因 |
| 服务 | `/manipulator/cancel_job` | 取消队列中未执行的 job（`job_id`）；正在执行的返回 cannot cancel running job |
| 服务 | `/manipulator/get_recent_jobs` | 最近 N 条执行记录（含 result_code、created_at/started_at/finished_at），便于长跑排查 |
| 服务 | `/manipulator/reset_held_object` | 清空持物状态并清理 scene 中的 attach |
| 服务 | `/manipulator/sync_held_object` | 同步持物状态（tracked/untracked）到 scene |
| 话题 | `/manipulator/target_set` | 多目标集合（TargetSet：num_targets + positions/directions，base_link）；有则 pick_trigger 走多目标选择+多抓取候选 |
| 话题 | `/manipulator/object_pose` | 单物体位姿（PoseStamped，base_link）；无 target_set 时抓取回退使用，由桥接发布 |
| 话题 | `/manipulator/place_pose` | 放置目标位姿（PoseStamped），放置使用；内部会生成多候选依次尝试 |
| 话题 | `/manipulator/pick_trigger` | 空消息：异步入队**抓取**（有 target_set 则多目标选一+多候选；否则等待 object_pose） |
| 话题 | `/manipulator/place_trigger` | 空消息：异步入队**放置**（无 place_pose 时使用默认参数；内部多候选尝试） |
| 话题 | `/manipulator/tf`、`/manipulator/tf_static` | 机械臂 TF（HoloOcean 联调 launch 下由 robot_state_publisher / move_group 发布并订阅，与全局 `/tf` 隔离） |

业务规则：未持物时禁止 place；已持物时禁止再次 pick。队列默认按优先级调度（RESET/SYNC/PLACE_RELEASE 高于 PICK/PLACE），同类型同目标短时间窗口内去重；配置见 `orion_mtc/config/runtime_policy.yaml`、`pick_place_params.yaml`。

## 运行（HoloOcean 联调）

关节状态来自 HoloOcean 的 ArmSensor，规划在 MoveIt 中完成，轨迹通过桥接节点发给 HoloOcean 执行。

**启动：**

```bash
# 确保已设置 HOLOOCEAN_ROS_INSTALL 或已 source holoocean-ros 的 install
ros2 launch orion_mtc pick_place_holoocean.launch.py
```

桥接 `target_sensor_to_object_pose` 从 TargetSensor + ROV 里程计**自动发布**：`/manipulator/target_set`（多目标，供多目标选择+多抓取候选）、`/manipulator/object_pose`（单目标，兼容与回退）。流程为**拆分接口**：先抓取再放置（或 place_release），无一体化 pick-place 话题。所有接口均在 `/manipulator` 命名空间下。**TF**：本 launch 下机械臂相关 TF 发布/订阅在 `/manipulator/tf`、`/manipulator/tf_static`（由 `tf_under_manipulator:=true` 启用），与仿真侧或其他 TF 源隔离。

**前置条件（抓取）**：有 target_set 时，pick_trigger 直接入队并由 Worker 从多目标中选一、多抓取候选依次尝试；**无** target_set 时需有 `/manipulator/object_pose`（或等待 3s），否则不入队。若 HoloOcean 未跑或暂无目标，可：① 先启动 HoloOcean 并确保有目标；或 ② 手动发布 object_pose 再 pick_trigger；或 ③ 用 Action `/manipulator/pick` 并在 Goal 里带 `object_pose`。

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
- 抓放几何参数：`orion_mtc/config/pick_place_params.yaml`（approach/lift/place/retreat/lower 等）
- 运行策略：`orion_mtc/config/runtime_policy.yaml`（auto_start_worker、reject_new_jobs_while_busy、失败后恢复等）

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
   ros2 interface show orion_mtc_msgs/msg/TargetSet
   ros2 interface show orion_mtc_msgs/srv/GetRobotState
   ros2 interface show orion_mtc_msgs/srv/GetQueueState
   ros2 interface show orion_mtc_msgs/srv/SubmitJob
   ros2 interface show orion_mtc_msgs/srv/CancelJob
   ros2 interface show orion_mtc_msgs/srv/GetRecentJobs
   ```

2. **启动 HoloOcean 场景与 launch**

   终端 1：`ros2 launch orion_mtc pick_place_holoocean.launch.py`

3. **终端 2：触发与状态查询**

   - **多目标抓取**（有 target_set 时）：`ros2 topic pub --once /manipulator/pick_trigger std_msgs/msg/Empty "{}"`，Worker 会从 target_set 选目标并尝试多抓取候选。
   - **单目标回退**（无 target_set 时需先有 object_pose）：同上命令，若 3s 内无 object_pose 则不入队。
   - **放置**（需先抓取成功）：`ros2 topic pub --once /manipulator/place_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'base_link'}, pose: {position: {x: 0.45, y: 0.0, z: 0.4}, orientation: {w: 1.0}}}"`，再 `ros2 topic pub --once /manipulator/place_trigger std_msgs/msg/Empty "{}"`；内部会生成多放置候选依次尝试。
   - **队列与 Worker**：`ros2 service call /manipulator/get_queue_state orion_mtc_msgs/srv/GetQueueState "{}"`（含 `next_job_id`、`next_job_priority`）。
   - **机器人状态**：`ros2 service call /manipulator/get_robot_state orion_mtc_msgs/srv/GetRobotState "{}"`
   - **最近执行记录**：`ros2 service call /manipulator/get_recent_jobs orion_mtc_msgs/srv/GetRecentJobs "{max_count: 20}"`
   - **取消排队任务**：`ros2 service call /manipulator/cancel_job orion_mtc_msgs/srv/CancelJob "{job_id: 'job_xxx'}"`
   - **即时执行**：Action `/manipulator/pick`、`/manipulator/place`、`/manipulator/place_release`

4. **业务规则**：未持物时 place 被拒绝；已持物时 pick 被拒绝。队列满或策略 `reject_new_jobs_while_busy` 时 submit_job 返回拒绝原因。抓取优先使用 target_set（多目标+多候选），放置对单次 target_pose 会内部多候选尝试。

## 许可证

Apache-2.0
