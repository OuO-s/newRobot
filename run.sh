#!/usr/bin/env bash
# 天链远程控制 webapp 启动脚本
# 默认监听 0.0.0.0:8001,可用环境变量改:
#   HOST=127.0.0.1 PORT=9000 ./run.sh
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
cd "$HERE"

HOST="${HOST:-0.0.0.0}"
PORT="${PORT:-8001}"

echo "[tianlian_webapp] starting on http://$HOST:$PORT  (static at /)"
exec python3 -m uvicorn app:app --host "$HOST" --port "$PORT" "$@"
