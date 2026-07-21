#!/usr/bin/env bash
# 将 bags/、jobs.jsonl、analysis/insert_extract/ 打成交付目录（默认再打 tar.gz）。
#
# 用法:
#   ./pack_delivery.sh
#   ./pack_delivery.sh --no-tar
#   ./pack_delivery.sh --out delivery/my_pack
#
# 不依赖 ROS；仅需 Python3（可选 PyYAML，无则轻量解析 metadata.yaml）。

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "${SCRIPT_DIR}/pack_delivery.py" "$@"
