#!/usr/bin/env bash
# 启动 Orion 上位机网页（静态 HTTP 服务 + 可选打开浏览器）

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"
PORT="${1:-8080}"
HOST="${2:-127.0.0.1}"
URL="http://${HOST}:${PORT}/"

echo "Orion 上位机: 服务目录 $SCRIPT_DIR"
echo "访问地址: $URL"
echo "连接 ROS: 需先启动 rosbridge，例如: ros2 launch rosbridge_server rosbridge_websocket_launch.xml"
echo "          默认 WS: ws://localhost:9090，可用 ?ws=ws://host:port 或 ?ns=/manipulator 覆盖"
echo "按 Ctrl+C 停止服务"
echo ""

if command -v xdg-open >/dev/null 2>&1; then
  (sleep 1.5 && xdg-open "$URL" 2>/dev/null) &
elif command -v open >/dev/null 2>&1; then
  (sleep 1.5 && open "$URL" 2>/dev/null) &
fi

exec python3 -m http.server "$PORT" --bind "$HOST"
