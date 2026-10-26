#!/usr/bin/env bash
# 天链远程控制 webapp 守护启动脚本。
#
# 用法:
#   ./start_server.sh start    前台/后台启动并崩溃自动重启(开机自启用这个)
#   ./start_server.sh stop     停止(杀掉 supervisor 与 uvicorn)
#   ./start_server.sh status   查看运行状态
#
# 开机自启已通过 crontab @reboot 挂载,见 README「开机自启」。
set -u

HERE="$(cd "$(dirname "$0")" && pwd)"
cd "$HERE"

PY=/usr/bin/python3                 # conda base(3.14)没有 fastapi,必须系统 3.10

# rclpy/fd_msgs 只有 source 后才可见(depth_provider 进程内取流+深度租约要用);
# fd_ws overlay 提供 fd_msgs 租约消息。子进程拉起处各自清了 PYTHONPATH,
# 不会串进 yolo venv / bridge。set -u 下 ROS 脚本会踩未绑定变量,先关掉。
set +u
source /opt/ros/humble/setup.bash
[ -f /home/user/fd_ws/install/setup.bash ] && source /home/user/fd_ws/install/setup.bash
set -u

HOST="${HOST:-0.0.0.0}"
PORT="${PORT:-8001}"
PID_FILE="$HERE/.webapp.pid"
LOG_DIR="$HERE/logs"
LOG="$LOG_DIR/webapp.log"
MAX_LOG_BYTES=$((10*1024*1024))    # 超过 10MB 截断,防止写满盘

mkdir -p "$LOG_DIR"

supervisor_pid()  { [ -f "$PID_FILE" ] && cat "$PID_FILE" 2>/dev/null; }
is_alive()        { [ -n "${1:-}" ] && kill -0 "$1" 2>/dev/null; }

log_rotate() {
    [ -f "$LOG" ] || return 0
    local size
    size=$(stat -c%s "$LOG" 2>/dev/null || echo 0)
    if [ "$size" -gt "$MAX_LOG_BYTES" ]; then
        : > "$LOG"
        echo "$(date '+%F %T') log truncated (${size}B)" >> "$LOG"
    fi
}

start() {
    local old
    old=$(supervisor_pid)
    if is_alive "$old"; then
        echo "已在运行 (supervisor pid=$old),端口 $PORT"
        return 0
    fi
    rm -f "$PID_FILE"
    # supervisor 循环放后台;crontab @reboot 与手动 start 都适用
    (
        echo "$(date '+%F %T') supervisor started, port $PORT"
        while true; do
            log_rotate
            $PY -m uvicorn app:app --host "$HOST" --port "$PORT" >> "$LOG" 2>&1
            rc=$?
            echo "$(date '+%F %T') uvicorn exited rc=$rc, 3s 后重启" >> "$LOG"
            sleep 3
        done
    ) &
    echo $! > "$PID_FILE"
    sleep 1.5
    if ss -tln 2>/dev/null | grep -q ":$PORT "; then
        local ip
        ip=$(hostname -I 2>/dev/null | tr ' ' '\n' | grep -v '^172\.1[6-9]\.\|^172\.2[0-9]\.\|^172\.3[01]\.\|^127\.' | head -1)
        echo "已启动: http://${ip:-localhost}:$PORT/  (supervisor pid=$(cat "$PID_FILE"))"
    else
        echo "启动中… 1.5s 后端口未就绪,查看日志: $LOG"
    fi
}

stop() {
    local sup uv
    sup=$(supervisor_pid)
    if is_alive "$sup"; then
        # 先杀 supervisor 防止它把 uvicorn 再拉起来
        kill "$sup" 2>/dev/null
        sleep 0.5
    fi
    # 杀 supervisor 的子进程(uvicorn)
    if is_alive "$sup"; then
        pkill -P "$sup" 2>/dev/null
    fi
    # 兜底:按命令行匹配本目录的 uvicorn
    pgrep -f "uvicorn app:app.*--port $PORT" 2>/dev/null | while read -r uv; do
        kill "$uv" 2>/dev/null
    done
    # SIGTERM 后 uvicorn 可能挂在优雅退出(等 keep-alive/后台任务)不退,
    # 与下个实例并存占资源 —— 1s 后升级 SIGKILL 收掉(子进程 bridge/检测
    # 服务是独立拉起的,残留 uvicorn 被杀不影响它们,下实例会探测复用)
    sleep 1
    pgrep -f "uvicorn app:app.*--port $PORT" 2>/dev/null | while read -r uv; do
        kill -9 "$uv" 2>/dev/null
    done
    rm -f "$PID_FILE"
    echo "已停止"
}

status() {
    local sup
    sup=$(supervisor_pid)
    if is_alive "$sup"; then
        echo "运行中 (supervisor pid=$sup)"
        ss -tln 2>/dev/null | grep ":$PORT " || echo "(端口 $PORT 未监听)"
        tail -5 "$LOG" 2>/dev/null
    else
        echo "未运行"
        exit 1
    fi
}

case "${1:-start}" in
    start)   start ;;
    stop)    stop ;;
    restart) stop; sleep 1; start ;;
    status)  status ;;
    *) echo "用法: $0 {start|stop|restart|status}"; exit 1 ;;
esac
