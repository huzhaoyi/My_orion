#!/usr/bin/env bash
# 录制臂控制链：joint_states、ArmSensor、AgentCommand，以及 FollowJointTrajectory 的 hidden action 话题。
# 默认输出: 本目录下 bags/sim_arm_debug_YYYYMMDD_HHMMSS
# 用法:
#   ./record_arm_chain.sh                    # 默认名+时间戳
#   ./record_arm_chain.sh my_run             # bags/my_run
#   ./record_arm_chain.sh /tmp/custom_bag    # 绝对路径仍按原样
# 停止: Ctrl+C

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BAGS_ROOT="${SCRIPT_DIR}/bags"
mkdir -p "${BAGS_ROOT}"

if [ -n "${1:-}" ]
then
    if [[ "${1}" = /* ]]
    then
        OUT_DIR="${1}"
    else
        OUT_DIR="${BAGS_ROOT}/${1}"
    fi
else
    OUT_DIR="${BAGS_ROOT}/sim_arm_debug_$(date +%Y%m%d_%H%M%S)"
fi

echo "[record_arm_chain] 输出目录: ${OUT_DIR}"
echo "[record_arm_chain] 停止录制: Ctrl+C"

exec ros2 bag record --include-hidden-topics -o "${OUT_DIR}" \
  /joint_states \
  /holoocean/rov0/ArmSensor \
  /holoocean/command/agent/arm \
  /arm_controller/follow_joint_trajectory/_action/feedback \
  /arm_controller/follow_joint_trajectory/_action/status \
  /hand_controller/follow_joint_trajectory/_action/feedback \
  /hand_controller/follow_joint_trajectory/_action/status
