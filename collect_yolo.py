#!/usr/bin/env python3.10
# -*- coding: utf-8 -*-
"""左臂相机 YOLO 训练图采集(现场背景域)
================================================================
现场背景与旧训练集不一致 → 就地重采微调集。复用 webapp 的
DepthSession 免租约取流(need_depth=False:只订彩色+内参,秒开,
不申请深度租约,与 webapp 预览/吸取会话互不干扰),存 640x480 JPEG
待标注。

用法(带自举:环境不对会自动换到系统 3.10+ROS 重跑,IDE 里直接 Run
或任意解释器 python collect_yolo.py 均可):
    python collect_yolo.py                    # 命令行交互采集(按键即拍)
    python collect_yolo.py --cls back         # 空格默认存 back
    python collect_yolo.py --auto 60 --interval 2   # 无人值守连拍
    python collect_yolo.py --role head        # 左臂标签轮换无帧时换角色

交互按键(免回车;要看画面可开 webapp 的相机预览页):
    f=正面 b=反面 n=负样本(写空标签) SPACE=默认类
    d=撤销上一张 q/ESC/Ctrl+C=退出

输出(默认 ~/tianlian/collect_field/,与旧 collect_wrist 同构,
labelimg → make_split_wrist.py 一路兼容):
    front/ back/ neg/  +  labels/{front,back,neg}/
    neg 顺手写空 txt(负样本免标注);front/back 留给 labelimg。
    文件名 {cls}_YYYYMMDD_HHMMSSmmm.jpg —— 类别前缀+毫秒时间戳,
    多次续采不撞名。

跑前确认: fd_cameras_hub 在跑。注意: 调试器(debugpy)下自举
re-exec 会断链,断点调试请用 ./collect_yolo.sh 起。
"""
import argparse
import os
import select
import shlex
import sys
import termios
import time
import tty
from datetime import datetime
from pathlib import Path

# ---- 自举:让脚本可被任意解释器直接 python 运行 ------------------------
# cv2/rclpy/fd_msgs 只在「系统 3.10 + source ROS」环境里有;base(3.14)/
# upper_computer(3.11)直接跑必炸。检测不合格就带 ROS 环境 re-exec 到
# /usr/bin/python3.10,等效 collect_yolo.sh 但不依赖终端包装。
_BOOT_FLAG = "COLLECT_YOLO_BOOTSTRAPPED"
_SYS_PY = "/usr/bin/python3.10"
_ROS_SETUP = ("/opt/ros/humble/setup.bash",
              "/home/user/fd_ws/install/setup.bash")


def _bootstrap_reexec() -> None:
    if os.environ.get(_BOOT_FLAG):
        return                      # 已 re-exec 过,防死循环
    try:
        import cv2, fd_msgs, rclpy  # noqa: F401
        return                      # 环境已齐(collect_yolo.sh 起的等),直跑
    except ImportError:
        pass
    env = os.environ.copy()
    env[_BOOT_FLAG] = "1"
    env.pop("PYTHONHOME", None)     # conda 残留会顶掉系统 3.10 的库
    env.pop("PYTHONPATH", None)     # 清掉让 setup.bash 重铺 ROS 路径
    script = Path(__file__).resolve()
    inner = f'exec "{_SYS_PY}" "{script}" {shlex.join(sys.argv[1:])}'
    cmd = "; ".join([f"source {f}" for f in _ROS_SETUP if os.path.isfile(f)]
                    + [inner])
    os.execve("/bin/bash", ["/bin/bash", "-c", cmd], env)


_bootstrap_reexec()

# 任意 cwd 启动都能找到 depth_provider(与脚本同目录)
sys.path.insert(0, str(Path(__file__).resolve().parent))

import cv2

from depth_provider import DepthError, DepthSession, ensure_ros_env

CLASSES = ("front", "back", "neg")       # neg=负样本(空背景帧)
DEFAULT_OUT = Path.home() / "tianlian" / "collect_field"


class _StdinKeys:
    """终端裸键读取(免回车)。非 TTY(管道/nohup)时恒无键。"""

    def __init__(self):
        self._fd = sys.stdin.fileno()
        self._saved = (termios.tcgetattr(self._fd)
                       if sys.stdin.isatty() else None)
        self._eof = False
        if self._saved is not None:
            tty.setcbreak(self._fd)

    def get(self, timeout_s: float):
        """返回单字符(已 lower)或 None。"""
        if self._saved is None or self._eof:
            return None
        r, _, _ = select.select([sys.stdin], [], [], timeout_s)
        if not r:
            return None
        ch = sys.stdin.read(1)
        if not ch:                    # stdin 关闭(EOF):记下,免 select 空转
            self._eof = True
            return None
        return ch.lower()

    def close(self):
        if self._saved is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._saved)


# --------------------------------------------------------------- 采集动作
def save_shot(frame, out_root: Path, cls: str, quality: int):
    """存一帧 {cls}_时间戳.jpg;neg 顺手写空标签。返回 (jpg, txt|None)。"""
    stem = datetime.now().strftime(f"{cls}_%Y%m%d_%H%M%S%f")[:-3]  # 毫秒截断
    jpg = out_root / cls / f"{stem}.jpg"
    cv2.imwrite(str(jpg), frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
    txt = None
    if cls == "neg":                     # 负样本无框:空 txt,后续切分直接收
        txt = out_root / "labels" / cls / f"{stem}.txt"
        txt.touch()
    return jpg, txt


def undo_shot(jpg: Path, txt) -> None:
    for p in (jpg, txt):
        if p is not None and p.exists():
            p.unlink()


def interactive(sess: DepthSession, out_root: Path, args) -> None:
    """命令行交互:按 f/b/n 即拍一张(取的是新帧),d 撤销,q 退出。"""
    kb = _StdinKeys()
    keys = {"f": "front", "b": "back", "n": "neg", " ": args.cls}
    print("按 f=正面  b=反面  n=负样本  空格="
          f"{args.cls}(免回车)   d=撤销  q=退出   —— 画面看 webapp 预览页")
    counts = dict.fromkeys(CLASSES, 0)
    last = None                          # (jpg, txt, cls) 供撤销
    try:
        while True:
            ch = kb.get(0.1) or ""
            if ch in ("q", "\x1b"):                # \x1b = ESC
                break
            cls = keys.get(ch)
            if cls:
                try:
                    frame = sess.grab_color(timeout_s=10.0)["color"]
                except DepthError as e:
                    print(f"\n取帧失败: {e}")
                    break
                jpg, txt = save_shot(frame, out_root, cls, args.quality)
                counts[cls] += 1
                last = (jpg, txt, cls)
                print(f"saved {jpg.name}  "
                      + " ".join(f"{c}:{counts[c]}" for c in CLASSES))
            elif ch == "d":
                if last is not None:
                    undo_shot(*last[:2])
                    counts[last[2]] -= 1
                    print(f"undo  {last[0].name}")
                    last = None
                else:
                    print("没有可撤销的(一次会话内仅最近一张)")
    finally:
        kb.close()


def auto_capture(sess: DepthSession, out_root: Path, args) -> None:
    for i in range(1, args.auto + 1):
        frame = sess.grab_color(timeout_s=10.0)["color"]   # 超时抛 DepthError
        jpg, _ = save_shot(frame, out_root, args.cls, args.quality)
        print(f"[{i}/{args.auto}] {jpg}")
        if i < args.auto:
            time.sleep(args.interval)


def main():
    ap = argparse.ArgumentParser(
        description="左臂相机 YOLO 训练图采集(可直接 python 运行,环境自举)")
    ap.add_argument("--out", default=str(DEFAULT_OUT),
                    help=f"输出根目录(默认 {DEFAULT_OUT})")
    ap.add_argument("--role", default="left", choices=("left", "head", "right"),
                    help="相机角色(默认 left;标签轮换无帧时换)")
    ap.add_argument("--cls", default="front", choices=CLASSES,
                    help="默认类别:--auto 的落点 + 交互模式空格键")
    ap.add_argument("--auto", type=int, default=0, metavar="N",
                    help="无人值守连拍 N 张(无预览)")
    ap.add_argument("--interval", type=float, default=2.0,
                    help="--auto 拍照间隔秒(默认 2)")
    ap.add_argument("--quality", type=int, default=92,
                    help="JPEG 质量(默认 92)")
    args = ap.parse_args()

    ensure_ros_env()
    try:
        import fd_msgs  # noqa: F401  rclpy 已在 ensure_ros_env 试过
    except ImportError as e:
        raise SystemExit(
            f"fd_msgs 导入失败({e})。自举未生效 —— 请检查:\n"
            "  1) /usr/bin/python3.10 存在且装了 cv2/rclpy\n"
            "  2) ROS humble 与 fd_ws 环境完整")

    out_root = Path(args.out).expanduser()
    for c in CLASSES:
        (out_root / c).mkdir(parents=True, exist_ok=True)
        (out_root / "labels" / c).mkdir(parents=True, exist_ok=True)

    sess = DepthSession(args.role, client_prefix="collect_yolo_")
    info = sess.start(need_depth=False)   # 免租约:不动深度管线
    print(f"相机就绪: role={info['role']} 彩色={info['color_ok']} "
          f"内参={'有' if info['intrinsics'] else '无'} "
          f"耗时{info['waited_s']}s")
    print(f"输出目录: {out_root}")

    try:
        if args.auto:
            auto_capture(sess, out_root, args)
        else:
            interactive(sess, out_root, args)
    except KeyboardInterrupt:
        print("\n手动中断")
    finally:
        sess.stop()

    total = {c: len(list((out_root / c).glob("*.jpg"))) for c in CLASSES}
    print(f"目录累计: " + " ".join(f"{c}={total[c]}" for c in CLASSES)
          + f"  (共 {sum(total.values())} 张)")
    print("后续: labelimg 标 front/back(neg 已空标)→ 参照 "
          "old/tianlian/weilong/make_split_wrist.py 切分 → 现场集微调")


if __name__ == "__main__":
    main()
