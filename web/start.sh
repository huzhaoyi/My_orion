#!/usr/bin/env bash
# 启动 Orion 上位机网页（静态 HTTP 服务 + 可选打开浏览器）

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"
PORT="${1:-8080}"
# 默认监听 0.0.0.0，局域网内可访问；仅本机时：./start.sh 8080 127.0.0.1
HOST="${2:-0.0.0.0}"
if [ "$HOST" = "0.0.0.0" ]; then
  URL="http://127.0.0.1:${PORT}/"
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
  echo "访问地址: 本机 $URL  局域网可选 http://${LOCAL_IP}:${PORT}/"
else
  URL="http://${HOST}:${PORT}/"
  echo "Orion 上位机: 服务目录 $SCRIPT_DIR"
  echo "访问地址: $URL（仅本机绑定 ${HOST}）"
fi
echo "连接 ROS: 需先启动 rosbridge，例如: ros2 launch rosbridge_server rosbridge_websocket_launch.xml"
echo "          WS 默认 ws://127.0.0.1:9091（与 pick_holoocean rosbridge 一致）；跨机时用 ?ws=ws://实际IP:9091"
echo "按 Ctrl+C 停止服务"
echo ""

if [ ! -f "$SCRIPT_DIR/node_modules/three/build/three.module.js" ]; then
  echo "【提示】未检测到 Three.js 依赖，开始自动安装（npm install）..."
  if ! command -v npm >/dev/null 2>&1; then
    echo "【错误】未找到 npm，请先安装 Node.js/npm 后重试。"
    exit 1
  fi
  npm install --no-audit --no-fund
  if [ ! -f "$SCRIPT_DIR/node_modules/three/build/three.module.js" ]; then
    echo "【错误】Three.js 安装后仍未找到目标文件，无法启动 3D 页面。"
    exit 1
  fi
  echo "【完成】Three.js 依赖安装成功。"
  echo ""
fi

if command -v xdg-open >/dev/null 2>&1; then
  (sleep 1.5 && xdg-open "$URL" 2>/dev/null) &
elif command -v open >/dev/null 2>&1; then
  (sleep 1.5 && open "$URL" 2>/dev/null) &
fi

exec python3 -m http.server "$PORT" --bind "$HOST"
