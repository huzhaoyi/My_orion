# 录包与离线抽取文档

本文档说明如何在 **HoloOcean + ROS 2 + MTC** 联调环境下：

1. **录制** Target 抓取 + 插孔 episode 的 rosbag2；
2. **离线抽取** 插孔段 `(s_t, a_t, s_{t+1})` 训练数据。

供世界模型 / 同事样例交付使用。

相关文件：

| 文件 | 说明 |
|------|------|
| `record_arm_chain.sh` | 手动开/停录 |
| `run_episode_record.sh` / `.py` | 自动执行业务 + 录包 + 写 `jobs.jsonl` |
| `extract_insert_trajectories.sh` / `.py` | 从 bag 抽取 `(s_t, a_t, s_{t+1})` → npz/csv |
| `jobs.jsonl.example` | 元数据字段模板 |
| `topic_list.txt` | 话题类型、频率、单位 |
| `README.md` | 目录结构与调试分析工具 |

---

## 1. 交付形态

### 1.1 Episode 定义

- **一条 episode = 一个 bag 目录**（例如 `bags/ep_target_001/`）。
- 推荐任务链：**Target 抓取**（`robotic_arm_cmd` type=0）→ **插孔**（type=1）。
- **不要用缆绳抓取**（type=2）；与当前样例定义不一致。
- `jobs.jsonl` 中同一 `episode_id` 写 **2 行**（PICK + TARGET_INSERT）；抓失败 episode 通常只有 1 行。

### 1.2 录包档位

| 档位 | 环境变量 | 话题数 | 用途 |
|------|----------|--------|------|
| **job**（默认） | `ARM_BAG_PROFILE=job` | 28 | 世界模型 / 样例交付 |
| **chain** | `ARM_BAG_PROFILE=chain` | 7 | 仅臂控制链调试 |

默认 **job 档**已包含 `/manipulator/insert_rel_state`（插孔相对位姿，用于构造 s_t / s_{t+1}）。

### 1.3 离散成功真值

| 阶段 | 信号 | 判定 |
|------|------|------|
| 抓取 | `/manipulator/left_arm_gripped` | `>= 0.5` 为抓稳 |
| 插孔 | `/manipulator/target_set.latches[i]` | `> 0.5` 为 latch 命中 |

注意：插孔 job 可能 **SUCCEEDED 但 latch 未命中**；训练/评估应保留 latch 时间序列，不能只看 job 终态。

---

## 2. 环境准备

### 2.1 编译

```bash
source /opt/ros/humble/setup.bash
cd ~/sealien_ws

colcon build --packages-select \
  sealien_ctrlpilot_manipulator_orion_mtc_msgs \
  sealien_ctrlpilot_manipulator_orion_holoocean_bridge \
  sealien_ctrlpilot_manipulator_orion

source install/setup.bash
```

若 holoocean-ros 为独立工作空间：

```bash
export HOLOOCEAN_ROS_INSTALL=/path/to/holoocean-ros/install
source $HOLOOCEAN_ROS_INSTALL/setup.bash
```

### 2.2 依赖包

**录包**（`run_episode_record` / `record_arm_chain`）终端需能导入：

- `holoocean_interfaces`
- `sealien_ctrlpilot_msgmanagement`（`RoboticArmCmd` Action）
- `sealien_ctrlpilot_manipulator_orion_mtc_msgs`

**离线抽取**（`extract_insert_trajectories`）额外需要：

- `rosbag2_py`、`rclpy`（source ROS2 后可用）
- `numpy`
- `sealien_ctrlpilot_manipulator_orion_holoocean_bridge`（`--mode auto/recompute` 时用于 `compute_relative_state`）
- 可选 `matplotlib`（`--plot`）

---

## 3. 启动全栈（终端 A）

```bash
source /opt/ros/humble/setup.bash
source ~/sealien_ws/install/setup.bash
# 如有 holoocean-ros：source 其 install/setup.bash

ros2 launch sealien_ctrlpilot_manipulator_orion sealien_ctrlpilot_manipulator_orion.launch.py
```

默认会启动：

- MoveIt + MTC
- HoloOcean 桥接（ArmSensor → joint_states、轨迹 → AgentCommand）
- **`insert_relative_state`** 节点（发布 `/manipulator/insert_rel_state`）

若需关闭相对位姿节点（不推荐用于 world model 录包）：

```bash
ros2 launch sealien_ctrlpilot_manipulator_orion sealien_ctrlpilot_manipulator_orion.launch.py \
  enable_insert_rel_state:=false
```

等待 HoloOcean 仿真跑起来、TargetSensor 有数据后再录包。

---

## 4. 录包前检查（终端 B）

```bash
source /opt/ros/humble/setup.bash
source ~/sealien_ws/install/setup.bash

cd ~/sealien_ws/src/sealien_ctrlpilot_manipulator_orion/tools/arm_bag_tools

# 节点与关键话题
ros2 node list | grep -E 'insert_relative_state|mtc_node'
ros2 topic hz /holoocean/rov0/TargetSensor --window 20
ros2 action list | grep robotic_arm_cmd

# 自动脚本 dry-run（不录包、不发任务）
./run_episode_record.sh --dry-run
```

`--dry-run` 通过表示：Action 可用、抓取/插孔感知话题有数据。

---

## 5. 自动录包（推荐）

### 5.1 首次准备

```bash
cd tools/arm_bag_tools
cp jobs.jsonl.example jobs.jsonl   # 仅首次；后续脚本会追加写入
```

### 5.2 录一条成功 episode

```bash
./run_episode_record.sh --episode-id ep_target_001
```

脚本顺序：

1. 等待抓取位姿（`/manipulator/target_set` 的 `targets[grasp-index]`，默认 index=0，与 submit_job 一致）
2. 等待插孔位姿（`/manipulator/target_insert_holes`，默认 index=0）
3. **开录** → `bags/ep_target_001/`
4. 缓冲 2 s（`--pre-buffer-sec`）
5. 发 **type=0** Target 抓取
6. 抓成功后间隔 1 s，发 **type=1** 插孔
7. 缓冲 2 s（`--post-buffer-sec`）后 **停录**
8. 追加 **2 行**到 `jobs.jsonl`

### 5.3 批量录制

```bash
# 连续 5 条：ep_target_001 … ep_target_005
# 每条前自动 reset_held + go_ready
./run_episode_record.sh --count 5 --prefix ep_target
```

### 5.4 仅抓取（抓失败样例）

```bash
./run_episode_record.sh --pick-only --episode-id ep_target_pick_fail_001
```

### 5.5 常用参数

```bash
./run_episode_record.sh --help
```

| 参数 | 默认 | 说明 |
|------|------|------|
| `--episode-id` | 自动生成 | bag 子目录名 |
| `--count` | 1 | 连续 episode 数 |
| `--prefix` | `ep_target` | 批量 ID 前缀 |
| `--insert-index` | 0 | 插哪个孔（`target_insert_holes` 下标） |
| `--grasp-index` | 0 | 抓哪个 Target（`target_set.targets` 下标，与 Web target_0 一致） |
| `--max-grasp-reach-m` | 1.9 | 抓取位姿 workspace 校验 [m] |
| `--wait-pose-sec` | 60 | 等待感知超时 [s] |
| `--pre-buffer-sec` | 2 | 开录后缓冲 [s] |
| `--post-buffer-sec` | 2 | 停录前缓冲 [s] |
| `--action-timeout-sec` | 900 | 单次 Action 最长等待 [s] |
| `--pick-only` | false | 只录抓取 |
| `--no-reset-between` | — | 批量时不复位 |
| `--dry-run` | — | 只检查，不录包 |

---

## 6. 手动录包

适用于：**插失败**、Web 面板发令、或不想用自动脚本时。

### 6.1 开录

```bash
cd tools/arm_bag_tools
cp jobs.jsonl.example jobs.jsonl   # 首次

./record_arm_chain.sh ep_target_001
# 保持运行，不要 Ctrl+C
```

也可指定绝对路径：`./record_arm_chain.sh /tmp/my_bag`

### 6.2 执行业务

在 **另一终端** 或 Web 面板依次发送：

| 顺序 | `robotic_arm_cmd` | MTC job | 说明 |
|------|-------------------|---------|------|
| 1 | **type=0** | PICK | TargetSensor 抓取 |
| 2 | **type=1** | TARGET_INSERT | 需已持物；keypoint 为孔位 |

中间可能出现 `go_ready_before_insert`，**全程不要停录**。

### 6.3 停录与元数据

1. 回到录包终端 **Ctrl+C** 停录。
2. 手动编辑 `jobs.jsonl`，参照 `jobs.jsonl.example` 写 **2 行**（同一 `episode_id`、`bag_dir`）。
3. 时间戳、`job_id` 可从 `/manipulator/job_event` 抄（自动脚本路径不一定产生 job_event）。

---

## 7. 验收

### 7.1 bag 基本信息

```bash
ros2 bag info bags/ep_target_001
```

确认 **28 个话题**（job 档）均有记录，重点检查：

| 话题 | 用途 |
|------|------|
| `/holoocean/command/agent/arm` | 实际控制 a_t（~50 Hz，deg） |
| `/manipulator/insert_rel_state` | 插孔相对位姿 s_t |
| `/manipulator/task_stage` | 阶段切分 |
| `/manipulator/left_arm_gripped` | 抓取真值 |
| `/manipulator/target_set` | latch 真值 |
| `/holoocean/rov0/TargetSensor` | 目标感知 |

### 7.2 查看元数据

```bash
tail -2 jobs.jsonl
python3 -m json.tool <<< "$(tail -1 jobs.jsonl)"
```

自动脚本写入的字段含 `started_timestamp`、`finished_timestamp`、`action_result_code`、`insert_latch_gt` 等。

### 7.3 插孔 world model 窗口

`/manipulator/insert_rel_state` 在以下窗口内 **`active=true`** 且几何字段有效：

- **开始**：TARGET_INSERT 阶段 `move to pre-insert` **ENTER**
- **结束**：`open hand` **ENTER** 前

离线构造 `(s_t, a_t, s_{t+1})` 推荐直接使用抽取脚本（见 **§13**），无需手工对齐：

```bash
./extract_insert_trajectories.sh bags/ep_target_001 --plot
```

手工理解对应关系：

| 量 | 来源 |
|----|------|
| s_t | `insert_rel_state`（`relative_pose` 或 `lateral_error_m` / `axial_error_m` / `angle_error_rad`） |
| a_t | `/holoocean/command/agent/arm`（左臂 7 维 deg，默认） |
| s_{t+1} | 下一帧有效 `insert_rel_state` |

脚本在 s_t 时刻对 AgentCommand 做最近邻匹配（默认 `max_align_dt_s=0.05`）。

---

## 8. 建议录制的 episode 组合

| episode_id 示例 | 内容 | 录制方式 |
|-----------------|------|----------|
| `ep_target_001` | 抓 + 插都成功 | `./run_episode_record.sh --episode-id ep_target_001` |
| `ep_target_pick_fail_001` | 仅抓失败 | `--pick-only` 或手动偏目标 |
| `ep_target_insert_fail_001` | 抓成功 + 插失败 | **手动**：开录后偏孔位/扰动，再发 type=1 |

自动脚本 **不会** 主动制造失败轨迹；失败样例需现场扰动或 `--pick-only`。

---

## 9. 交付清单

交给同事时建议包含：

```
tools/arm_bag_tools/
├── bags/ep_target_001/                    # rosbag2 原始数据
├── jobs.jsonl                             # episode / job 元数据
├── topic_list.txt                         # 话题说明（推荐）
└── analysis/insert_extract/               # 离线抽取结果（推荐）
    ├── manifest.json                      # 批量处理汇总
    ├── all_insert_full10.npz              # 可选：合并数据集
    └── ep_target_001/
        ├── transitions.npz                # (s, a, s_next) 训练用
        ├── transitions.csv
        ├── states.csv
        └── metadata.json
```

说明：

- 一条 bag 内 PICK 与 TARGET_INSERT **按时间连续**；切分用 `jobs.jsonl` 时间戳或 `/manipulator/job_event` / `task_stage`。
- 插孔训练段由 `extract_insert_trajectories` 从 `insert_rel_state.active=true`（或 TF 重算窗口）自动切片。
- B 类连续力/接触真值本仿真栈 **不提供**（见 `topic_list.txt` 注释）。

---

## 10. 常见问题

| 现象 | 可能原因 | 处理 |
|------|----------|------|
| `--dry-run` 报 Action 不可用 | 全栈未起或 MTC 未就绪 | 检查 launch 日志、`ros2 action list` |
| 等待抓取/插孔位姿超时 | TargetSensor 无数据、仿真未跑 | 查 `/holoocean/rov0/TargetSensor` |
| bag 无 `insert_rel_state` | 节点未起或录包脚本过旧 | 确认 launch 与 `record_arm_chain.sh` / `run_episode_record.py` 已更新 |
| `insert_rel_state` 全是 `active=false` | 未进入插孔段或任务失败 | 正常；只有 pre-insert 窗口内为 true |
| 插孔 job 成功但 latch=0 | MTC 与 HoloOcean latch 不同步 | 保留 bag；用 `target_set.latches` 作真值 |
| 想清空旧 bag | — | `./clean_arm_bags.sh -y` |
| 抽取报「转移样本不足」 | 未进插孔段 / bag 缺话题 | 见 **§13.4** |
| recompute 失败 | 旧 bag 无 TF 或孔位 | 用新 job 档重录，或 `--mode topic` 且确认有 `insert_rel_state` |

---

## 11. 调试分析（可选）

job 档主要用于交付；臂链 cmd/sensor 对比可用：

```bash
ARM_BAG_PROFILE=chain ./record_arm_chain.sh
./analyze_arm_bag.sh
```

分析结果在 `analysis/<bag名>/`（PNG + CSV）。该分析 **不覆盖** job 档全部话题，交付验收以 `ros2 bag info` 为准。

---

## 12. 端到端快速命令

```bash
# 终端 A：全栈
ros2 launch sealien_ctrlpilot_manipulator_orion sealien_ctrlpilot_manipulator_orion.launch.py

# 终端 B：录一条 episode
cd tools/arm_bag_tools && cp -n jobs.jsonl.example jobs.jsonl
./run_episode_record.sh --dry-run
./run_episode_record.sh --episode-id ep_target_001

# 验收 bag
ros2 bag info bags/ep_target_001
tail -2 jobs.jsonl

# 离线抽取 (s_t, a_t, s_{t+1})
./extract_insert_trajectories.sh bags/ep_target_001 --plot
ls analysis/insert_extract/ep_target_001/
```

批量录 + 批量抽：

```bash
./run_episode_record.sh --count 5 --prefix ep_target
./extract_insert_trajectories.sh \
  --jobs-jsonl jobs.jsonl \
  --merge-out analysis/insert_extract/all_insert_full10.npz \
  --plot
```

---

## 13. 离线抽取 `(s_t, a_t, s_{t+1})`

录包完成后，用 **`extract_insert_trajectories.sh`** 从 bag 生成训练用转移数据。  
脚本路径：`tools/arm_bag_tools/extract_insert_trajectories.py`

### 13.1 处理流程

```
rosbag2 (job 档)
    │
    ├─ 读取 insert_rel_state ──► active=true 样本序列 ──► s_t, s_{t+1}
    │   （缺失时 --mode auto）
    │       └─ task_stage 窗口 + TF(peg_tip) + target_insert_holes ──► 重算
    │
    ├─ 读取 AgentCommand ──► 在 s_t 时刻最近邻匹配 ──► a_t
    │
    └─ 输出 transitions.npz / csv + metadata.json
```

**窗口**（与录包时 `insert_relative_state` 节点一致）：

- 开始：`move to pre-insert` **ENTER**
- 结束：`open hand` **ENTER** 前

### 13.2 单条 bag

```bash
cd tools/arm_bag_tools
source /opt/ros/humble/setup.bash
source ~/sealien_ws/install/setup.bash

./extract_insert_trajectories.sh bags/ep_target_001 --plot
```

输出目录：`analysis/insert_extract/ep_target_001/`

| 文件 | 内容 |
|------|------|
| `transitions.npz` | 压缩数组：`s`, `a`, `s_next`, `t_abs_s`, `t_rel_s`, `dt_s`, `align_dt_s`, `latch`, `state_names`, `action_names` |
| `transitions.csv` | 同上，便于 Excel / pandas |
| `states.csv` | 窗口内全部状态采样（含 `active`、几何误差、相对位姿） |
| `metadata.json` | episode 元信息、窗口来源、对齐丢弃数等 |
| `plots/errors.png` | `--plot`：e_lat / e_ax / angle 曲线 |
| `plots/action_norm.png` | `--plot`：\|\|a\|\| 曲线 |

**Python 加载示例**：

```python
import numpy as np
d = np.load("analysis/insert_extract/ep_target_001/transitions.npz", allow_pickle=True)
s, a, s_next = d["s"], d["a"], d["s_next"]
print(d["state_names"], d["action_names"], s.shape)
```

### 13.3 批量（jobs.jsonl）

只处理 `jobs.jsonl` 中含 **TARGET_INSERT** 行的 episode：

```bash
./extract_insert_trajectories.sh \
  --jobs-jsonl jobs.jsonl \
  --bags-root bags \
  --merge-out analysis/insert_extract/all_insert_full10.npz \
  --plot
```

| 输出 | 说明 |
|------|------|
| `analysis/insert_extract/<episode_id>/` | 每条 episode 独立目录 |
| `analysis/insert_extract/manifest.json` | 成功/失败 episode 清单 |
| `all_insert_full10.npz` | 合并后的 `s`/`a`/`s_next` + `episode_id` 标签 |

### 13.4 状态 / 动作定义

| `--state-representation` | 维度 | 字段 |
|--------------------------|------|------|
| `errors`（默认之一） | 3 | `lateral_error_m`, `axial_error_m`, `angle_error_rad` |
| `pose7` | 7 | 孔系下相对位姿 xyz + quat |
| **`full10`（默认）** | 10 | 误差 3 + 位姿 7 |

| `--action-dims` | 维度 | 说明 |
|-----------------|------|------|
| **`left7`（默认）** | 7 | AgentCommand 左臂 6 关节 + 夹爪，deg |
| `full14` | 14 | 双臂 AgentCommand |

### 13.5 常用参数

```bash
./extract_insert_trajectories.sh --help
```

| 参数 | 默认 | 说明 |
|------|------|------|
| `--mode` | `auto` | `topic`：仅 `insert_rel_state`；`recompute`：强制 TF+孔位重算 |
| `--state-representation` | `full10` | 见 §13.4 |
| `--action-dims` | `left7` | 见 §13.4 |
| `--max-align-dt-s` | 0.05 | s_t 与 AgentCommand 最大对齐误差 [s] |
| `--min-samples` | 10 | 最少状态样本，不足则报错 |
| `--min-transitions` | 5 | 最少转移条数，不足则报错 |
| `--insert-index` | 0 | 重算模式孔位下标 |
| `--out-root` | `analysis/insert_extract` | 输出根目录 |
| `--merge-out` | — | 合并所有 episode 为一个 npz |
| `--plot` | false | 输出误差/动作图 |
| `--insert-axis-local-xyz` | `[0,-1,0]` | 重算模式，与 `peg_insert` 一致 |
| `--peg-rod-axis-tcp-xyz` | `[0,1,0]` | 重算模式，与 `peg_insert` 一致 |

### 13.6 常见报错

| 报错 | 原因 | 处理 |
|------|------|------|
| 有效状态样本不足 | bag 无插孔段 / `active` 全 false | 确认 episode 含 TARGET_INSERT 且 launch 启用了 `insert_relative_state` |
| 转移样本不足 | 动作对齐失败过多 | 增大 `--max-align-dt-s`；检查 bag 是否含 AgentCommand |
| recompute 失败 | 缺 `/manipulator/tf` 或 `target_insert_holes` | 用 job 档重录；或确保 bag 含上述话题 |
| 导入 holoocean_bridge 失败 | 未 source 工作空间 | `source install/setup.bash` 后再跑 |
| `--pick-only` episode | jobs.jsonl 无 TARGET_INSERT | 批量模式会自动跳过；单 bag 指定时会失败 |
