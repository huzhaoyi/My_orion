# arm_bag_tools：联调录制与世界模型样例 bag

**完整流程见 [`RECORDING.md`](RECORDING.md)**：录包 → 验收 → 离线抽取 `(s_t, a_t, s_{t+1})` → 交付。

本目录用于 HoloOcean + MTC 联调时 **录制 rosbag2**，**离线抽取**插孔训练数据，并可选 **一键分析**臂链 cmd/sensor 对比（调试用）。

**世界模型 / 同事样例交付**：默认 `record_arm_chain.sh` 使用 **`ARM_BAG_PROFILE=job`**。推荐 **一条 episode = 一个 bag**：**Target 抓取（`robotic_arm_cmd` type=0）+ 插孔（type=1）** 连续录制；`jobs.jsonl` 中同一 `episode_id` 写 2 行（PICK + TARGET_INSERT）。模板见 **`jobs.jsonl.example`**，话题见 **`topic_list.txt`**。

**说明**：仓库内 `tools/` 下除本目录外，另有根级脚本如 `compute_max_reach_from_urdf.py`（由 URDF 推算臂展供 feasibility 配置）；**带子目录的工具仅 `arm_bag_tools/`**，其子文件夹含义见下表。

## 目录说明

| 路径 | 用途 |
|------|------|
| **`bags/`** | **默认 rosbag 输出目录**。`record_arm_chain.sh` 默认创建 `sim_job_时间戳/`（job 档）或 `ARM_BAG_PROFILE=chain` 时 `sim_arm_debug_时间戳/`；也可 `./record_arm_chain.sh ep_target_001` 指定名。 |
| **`analysis/`** | **分析 / 抽取输出根目录**。子目录见下。 |
| **`analysis/<bag名>/`** | `analyze_arm_bag.sh` 输出：臂链 cmd/sensor 对比 PNG + CSV。 |
| **`analysis/insert_extract/`** | `extract_insert_trajectories.sh` 输出：每 episode 的 `transitions.npz`、csv、`metadata.json`；批量时有 `manifest.json`。 |

其余文件均在本目录**根下**（不再放入子文件夹）：

| 文件 | 用途 |
|------|------|
| **`RECORDING.md`** | **录包 + 离线抽取文档**（推荐先读）：环境、录包、验收、抽取、交付 |
| `record_arm_chain.sh` | 启动 rosbag2 录制。默认 **`job` 档**（任务+感知+锁存+载体）；`ARM_BAG_PROFILE=chain` 为旧版最小臂链。输出见 `bags/`。 |
| `jobs.jsonl.example` | **Target 抓+插 episode** 元数据模板（`episode_id` + 每 job 一行）；含成功/抓失败/插失败示例。 |
| `run_episode_record.sh` | **自动** Target 抓+插 + 录 bag + 写 `jobs.jsonl`（见下文）。 |
| `run_episode_record.py` | 上述逻辑 Python 实现；`./run_episode_record.sh --help`。 |
| `topic_list.txt` | job 档话题名、消息类型、频率与单位说明（交付用）。 |
| `analyze_arm_bag.sh` | 一键调用 `plot_arm_bag.py`：默认选 `bags/` 下**最近修改**的 bag，结果写入 `analysis/`。 |
| `plot_arm_bag.py` | 读 bag 目录，画 cmd / sensor / joint_states 对比图；导出 CSV/PNG。 |
| `extract_insert_trajectories.sh` / `.py` | **从 bag 抽取插孔 (s_t, a_t, s_{t+1})** → npz/csv；详见 [`RECORDING.md`](RECORDING.md) §13 |
| `requirements.txt` | 分析脚本依赖：`numpy`、`matplotlib`（与 ROS 环境分离安装时可用 `pip install -r requirements.txt`）。 |

## 端到端流程（世界模型交付）

```
全栈 launch → run_episode_record → ros2 bag info 验收
    → extract_insert_trajectories → transitions.npz + jobs.jsonl 交付
```

```bash
cd tools/arm_bag_tools
cp jobs.jsonl.example jobs.jsonl   # 首次

# 1. 录包（全栈须已运行）
./run_episode_record.sh --episode-id ep_target_001

# 2. 离线抽取
./extract_insert_trajectories.sh bags/ep_target_001 --plot

# 3. 查看结果
ls analysis/insert_extract/ep_target_001/
python3 -c "import numpy as np; d=np.load('analysis/insert_extract/ep_target_001/transitions.npz',allow_pickle=True); print(d['s'].shape, d['state_names'])"
```

批量：

```bash
./run_episode_record.sh --count 5 --prefix ep_target
./extract_insert_trajectories.sh --jobs-jsonl jobs.jsonl \
  --merge-out analysis/insert_extract/all_insert_full10.npz --plot
```

## 典型流程（联调 / 臂链分析）

1. 终端内：`source /opt/ros/$ROS_DISTRO/setup.bash` 与 `install/setup.bash`（需含 `holoocean_interfaces`）。
2. `cd` 到本目录，执行 `ARM_BAG_PROFILE=chain ./record_arm_chain.sh`，结束后在 `bags/` 下得到新子目录。
3. 执行 `./analyze_arm_bag.sh`（或 `./analyze_arm_bag.sh <bag 子目录名>`），在 `analysis/<同名>/` 查看 PNG 与 CSV。
4. 不需要旧数据时：`./clean_arm_bags.sh` 或 `./clean_arm_bags.sh -y`。
5. 调整「首次异常」阈值：`ARM_BAG_SPIKE_DEG=45 ./analyze_arm_bag.sh`（默认 60°）。

| `clean_arm_bags.sh` | 清空 **`bags/*`** 与 **`analysis/*`**，保留两目录下的 `.gitkeep`。 |

## 自动录 episode（推荐）

全栈 launch 后，**一条命令**完成：开录 → type=0 抓 → type=1 插 → 停录 → 追加 `jobs.jsonl`。

```bash
cd tools/arm_bag_tools
cp jobs.jsonl.example jobs.jsonl   # 首次

# 检查 Action / 感知是否就绪
./run_episode_record.sh --dry-run

# 单条 episode
./run_episode_record.sh --episode-id ep_target_001

# 连续 5 条（每条前 reset_held + go_ready）
./run_episode_record.sh --count 5 --prefix ep_target

# 仅抓取（抓失败样例等）
./run_episode_record.sh --pick-only --episode-id ep_target_pick_fail_001
```

**行为说明**：

- 抓取 keypoint：`/manipulator/object_pose_targetsensor`（回退 `/manipulator/object_pose`）
- 插孔 keypoint：`/manipulator/target_insert_holes` 指定 `--insert-index`（默认 0）
- 业务走 **`/manipulator/robotic_arm_cmd` Action**（同步）；`jobs.jsonl` 用 Action 结果与时间戳（`robotic_arm_cmd` 路径不一定产生 `job_event`）
- 录包话题与 `record_arm_chain.sh` job 档一致（含 `/manipulator/insert_rel_state`）

**插孔 world model 对齐**（全栈 launch 默认启动 `insert_relative_state` 节点）：

| 量 | 话题 / 字段 |
|----|-------------|
| s_t / s_{t+1} | `/manipulator/insert_rel_state`（`active=true` 段；`relative_pose` 或 `lateral_error_m` / `axial_error_m` / `angle_error_rad`） |
| a_t | `/holoocean/command/agent/arm`（~50 Hz，deg） |
| 窗口 | `move to pre-insert` ENTER → `open hand` ENTER 前 |

离线抽取用 **`./extract_insert_trajectories.sh`**（见 [`RECORDING.md`](RECORDING.md) §13），输出 `analysis/insert_extract/<episode_id>/transitions.npz`。

**失败样例**：脚本不会自动制造失败；需现场扰动（偏目标/偏孔位）或 `--pick-only` 录抓失败 episode。

## 手动录 episode（备选）

**任务链**（不要用缆绳 `type=2`）：

| 步骤 | `robotic_arm_cmd` | MTC job | 说明 |
|------|-------------------|---------|------|
| 1 | **type=0** | `PICK` | `GraspSource::TARGET_SENSOR`，感知来自 TargetSensor |
| 2 | **type=1** | `TARGET_INSERT` | 需已持物；`keypoint` 为孔位 |

1. 全栈 launch：`ros2 launch sealien_ctrlpilot_manipulator_orion sealien_ctrlpilot_manipulator_orion.launch.py`。
2. `cp jobs.jsonl.example jobs.jsonl`。
3. **开录**（一条 episode 一个 bag）：
   ```bash
   ./record_arm_chain.sh ep_target_001
   ```
4. 发 **type=0** Target 抓取 → 等 `/manipulator/job_event` PICK 终态。
5. 若抓取成功，发 **type=1** 插孔 → 等 TARGET_INSERT 终态。（中间可能 `go_ready_before_insert`，**不要停录**。）
6. **Ctrl+C** 停录；`jobs.jsonl` 写 **2 行**（同一 `episode_id`、`bag_dir`），`job_id`/时间从 `job_event` 抄。
7. 验收：`ros2 bag info bags/ep_target_001`。

**建议 episode  mix**：

- 抓+插都成功（`ep_target_001`）
- 仅抓失败（`ep_target_002`，通常 1 个 job 后停录）
- 抓成功 + 插失败（`ep_target_003`）

**仿真真值**：

- Target 抓取：`left_arm_gripped >= 0.5`（未过 → PICK 硬失败）
- 插孔：`target_set.latches[i] > 0.5`（latch 未命中时 job 仍可能 SUCCEEDED，需保留时间序列）

## 与环境变量的关系

- **`ARM_BAG_PROFILE`**：`job`（默认，世界模型样例）或 `chain`（仅臂链调试）。
- **`ARM_BAG_OPEN=1`**：与 `./analyze_arm_bag.sh` 同用时，若存在图形界面，会尝试用系统默认程序打开生成的 PNG。

## 与 git

- `bags/`、`analysis/` 内**除 `.gitkeep 外**的内容默认忽略，避免大文件入库。
- 若需提交某次 bag 作样例，须自行改 `.gitignore` 或使用仓库外的备份路径录制。
