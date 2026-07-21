#!/usr/bin/env bash
# HoloOcean + MTC 联调 rosbag2 录制。
#
# 用法:
#   ./record_arm_chain.sh                         # job 档，默认名+时间戳 → bags/sim_job_YYYYMMDD_HHMMSS
#   ./record_arm_chain.sh ep_target_001          # episode 档 → bags/ep_target_001
#   ./record_arm_chain.sh /tmp/custom_bag       # 绝对路径
#   ARM_BAG_PROFILE=chain ./record_arm_chain.sh # 仅臂控制链（旧版最小集，调试用）
#
# 建议流程（世界模型 / 同事样例）:
#   1. 全栈 launch 已运行
#   2. 本脚本开录 → robotic_arm_cmd type=0(Target 抓) → type=1(插孔) → job_event 终态后 Ctrl+C
#   3. jobs.jsonl 同一 episode_id 写 PICK + TARGET_INSERT 两行（见 jobs.jsonl.example）
#
# 停止: Ctrl+C
# 分析: ./analyze_arm_bag.sh（主要覆盖臂链 cmd/sensor；job 档需另用 ros2 bag info 验收）

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BAGS_ROOT="${SCRIPT_DIR}/bags"
mkdir -p "${BAGS_ROOT}"

# job：任务级样例（job_event、感知、锁存、载体速度等）；chain：仅臂控制链
ARM_BAG_PROFILE="${ARM_BAG_PROFILE:-job}"

if [ -n "${1:-}" ]
then
    if [[ "${1}" = /* ]]
    then
        OUT_DIR="${1}"
    else
        OUT_DIR="${BAGS_ROOT}/${1}"
    fi
else
    if [ "${ARM_BAG_PROFILE}" = "chain" ]
    then
        OUT_DIR="${BAGS_ROOT}/sim_arm_debug_$(date +%Y%m%d_%H%M%S)"
    else
        OUT_DIR="${BAGS_ROOT}/sim_job_$(date +%Y%m%d_%H%M%S)"
    fi
fi

# 示例: ./record_arm_chain.sh ep_target_001

# 最小臂链（调试 / plot_arm_bag.py）
CHAIN_TOPICS=(
    /joint_states
    /holoocean/rov0/ArmSensor
    /holoocean/command/agent/arm
    /arm_controller/follow_joint_trajectory/_action/feedback
    /arm_controller/follow_joint_trajectory/_action/status
    /hand_controller/follow_joint_trajectory/_action/feedback
    /hand_controller/follow_joint_trajectory/_action/status
)

# 世界模型 Target 抓+插 episode（一条 bag 含 PICK + TARGET_INSERT，按 job_event 切分）
JOB_TOPICS=(
    "${CHAIN_TOPICS[@]}"
    /manipulator/job_event
    /manipulator/task_stage
    /manipulator/runtime_status
    /manipulator/held_object_state
    /manipulator/left_arm_gripped
    /holoocean/rov0/TargetSensor
    /manipulator/target_set
    /manipulator/object_pose
    /manipulator/object_axis
    /manipulator/perception_state
    /manipulator/target_insert_holes
    /manipulator/insert_rel_state
    /holoocean/rov0/PoseSensor
    /holoocean/rov0/VelocitySensor
    /holoocean/rov0/IMUSensor
    /holoocean/rov0/DepthSensor
    /holoocean/rov0/DVLSensorVelocity
    /manipulator/tf
    /manipulator/tf_static
)

if [ "${ARM_BAG_PROFILE}" = "chain" ]
then
    TOPICS=("${CHAIN_TOPICS[@]}")
else
    TOPICS=("${JOB_TOPICS[@]}")
fi

echo "[record_arm_chain] profile: ${ARM_BAG_PROFILE}"
echo "[record_arm_chain] 输出目录: ${OUT_DIR}"
echo "[record_arm_chain] 话题数: ${#TOPICS[@]}"
echo "[record_arm_chain] 停止录制: Ctrl+C"
echo "[record_arm_chain] 验收: ros2 bag info \"${OUT_DIR}\""

exec ros2 bag record --include-hidden-topics -o "${OUT_DIR}" "${TOPICS[@]}"
