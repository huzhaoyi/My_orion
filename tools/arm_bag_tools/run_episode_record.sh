#!/usr/bin/env bash
# Target 抓+插 episode：自动执行业务 + 录 rosbag2 + 追加 jobs.jsonl
#
# 用法:
#   ./run_episode_record.sh --episode-id ep_target_001
#   ./run_episode_record.sh --count 5 --prefix ep_target
#   ./run_episode_record.sh --pick-only --episode-id ep_target_pick_fail_001
#   ./run_episode_record.sh --dry-run
#
# 前置: 全栈 launch 已运行；本终端已 source ROS2 + 工作空间（含 sealien_ctrlpilot_msgmanagement）

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -z "${ROS_DISTRO:-}" ]
then
    echo "[run_episode_record] 请先 source ROS2 与工作空间，例如:"
    echo "  source /opt/ros/humble/setup.bash"
    echo "  source <ws>/install/setup.bash"
    exit 1
fi

exec python3 "${SCRIPT_DIR}/run_episode_record.py" "$@"
