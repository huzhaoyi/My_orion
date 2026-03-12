#!/usr/bin/env bash
# 启动 Orion 上位机网页（静态 HTTP 服务 + 可选打开浏览器）

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"
PORT="${1:-8080}"
HOST="${2:-0.0.0.0}"
# 绑定 0.0.0.0 时本机用 localhost，外机用本机 IP 访问
if [ "$HOST" = "0.0.0.0" ]; then
  URL="http://localhost:${PORT}/"
  # 尝试获取本机 IP 供外机/局域网访问
  if command -v hostname >/dev/null 2>&1; then
    LOCAL_IP=$(hostname -I 2>/dev/null | awk '{print $1}')
  fi
  if [ -z "$LOCAL_IP" ] && command -v ip >/dev/null 2>&1; then
    LOCAL_IP=$(ip route get 1 2>/dev/null | awk '{print $7; exit}')
  fi
  if [ -z "$LOCAL_IP" ]; then
    LOCAL_IP="<本机IP>"
  fi
  echo "Orion 上位机: 服务目录 $SCRIPT_DIR"
  echo "访问地址: 本机 $URL  外机/局域网 http://${LOCAL_IP}:${PORT}/"
else
  URL="http://${HOST}:${PORT}/"
  echo "Orion 上位机: 服务目录 $SCRIPT_DIR"
  echo "访问地址: $URL"
fi
echo "连接 ROS: 需先启动 rosbridge，例如: ros2 launch rosbridge_server rosbridge_websocket_launch.xml"
echo "          WS 默认随页面主机（当前访问的 host）:9090，可用 ?ws=ws://host:port 或 ?ns=/manipulator 覆盖"
echo "按 Ctrl+C 停止服务"
echo ""

if command -v xdg-open >/dev/null 2>&1; then
  (sleep 1.5 && xdg-open "$URL" 2>/dev/null) &
elif command -v open >/dev/null 2>&1; then
  (sleep 1.5 && open "$URL" 2>/dev/null) &
fi

exec python3 -m http.server "$PORT" --bind "$HOST"
