#!/usr/bin/env bash
# 左臂相机 YOLO 训练图采集启动器(collect_yolo.py 需 ROS 环境)。
#
#   ./collect_yolo.sh                        交互预览(需桌面 DISPLAY)
#   ./collect_yolo.sh --auto 60 --interval 2  无人值守连拍(SSH 可用)
#
# 与 start_server.sh 同款环境:系统 Python 3.10 + ROS humble + fd_ws
# overlay(rclpy/fd_msgs 只有 source 后可见;conda base/upper_computer
# 是 3.11,导不了 rclpy)。
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
PY=/usr/bin/python3.10

set +u
source /opt/ros/humble/setup.bash
[ -f /home/user/fd_ws/install/setup.bash ] && source /home/user/fd_ws/install/setup.bash
set -u

exec "$PY" "$HERE/collect_yolo.py" "$@"
