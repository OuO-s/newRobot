#!/usr/bin/env python3.10
# -*- coding: utf-8 -*-
"""左臂相机 YOLO 训练图采集(现场背景域)
================================================================
现场背景与旧训练集不一致 → 就地重采微调集。复用 webapp 的
DepthSession 免租约取流(need_depth=False:只订彩色+内参,秒开,
不申请深度租约,与 webapp 预览/吸取会话互不干扰),默认以相机上限
1920x1080 采集(D435/D438 彩色最大档,30fps)待标注。

高分辨率热切换(不经 hub 重启、不用 sudo):
    hub 的 width/height 落库后不会自动重启相机管线,但 want_depth_hw
    (enable_depth/enable_align_depth/租约 任一为真)翻转必重启并按新
    配置起流 —— 本脚本经 /fd_parameter 服务拨 enable_depth 一来一回,
    pipeline 即以新分辨率重启。顺序上先在老分辨率带深度重启一次、再
    改分辨率关深度:全程绝不以 >720p 请求深度流(D435 深度上限
    1280x720,彩深共用 width/height,1080p+深度会起流失败)。
    采集期间请勿跑吸取/拿深度租约(会触发 1080p 深度起流失败);
    webapp 预览此时同为 1080p,切换瞬间闪断几秒属正常。
    进程被 kill -9 会残留高分辨率 → 用 --restore 应急恢复。

用法(带自举:环境不对会自动换到系统 3.10+ROS 重跑,IDE 里直接 Run
或任意解释器 python collect_yolo.py 均可):
    python collect_yolo.py                    # 命令行交互采集(按键即拍)
    python collect_yolo.py --cls back         # 空格默认存 back
    python collect_yolo.py --auto 60 --interval 2   # 无人值守连拍
    python collect_yolo.py --res 1280x720     # 指定采集分辨率
    python collect_yolo.py --res 640x480      # 不切换(hub 默认流)
    python collect_yolo.py --role head        # 左臂标签轮换无帧时换角色
    python collect_yolo.py --restore          # 应急:hub 流恢复 640x480

交互按键(免回车;要看画面可开 webapp 的相机预览页):
    f=正面 b=反面 n=负样本(写空标签) SPACE=默认类
    d=撤销上一张 q/ESC/Ctrl+C=退出

输出(默认 ~/tianlian/collect_field/,与旧 collect_wrist 同构,
labelimg → make_split_wrist.py 一路兼容;标签坐标归一化,混采
640/1080 两档分辨率也不影响训练):
    front/ back/ neg/  +  labels/{front,back,neg}/
    neg 顺手写空 txt(负样本免标注);front/back 留给 labelimg。
    文件名 {cls}_YYYYMMDD_HHMMSSmmm.jpg —— 类别前缀+毫秒时间戳,
    多次续采不撞名。

跑前确认: fd_cameras_hub 与 fd_parameter 服务在跑。注意: 调试器
(debugpy)下自举 re-exec 会断链,断点调试请用 ./collect_yolo.sh 起。
"""
import argparse
import os
import select
import shlex
import sys
import termios
import threading
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

# hub 各角色的参数模块名(fd_cameras_hub_node.cpp kRole* 表;注意 head 角色
# 在 hub 内部叫 center,话题却是 camera_head)
PARAM_MODULE = {"left": "left_camera_param_module",
                "head": "center_camera_param_module",
                "right": "right_camera_param_module"}


class HubParams:
    """fd_parameter 服务极简客户端:hub 相机流分辨率热切换用。

    与 DepthSession 同模式:rclpy.init 兜底 + 私有 executor 线程
    (进程级全局 executor 并发调用会互踩,见 depth_provider 注释)。
    """

    _TYPE_BOOL, _TYPE_INT32 = 1, 3

    def __init__(self, role: str):
        try:
            import rclpy
            import rclpy.executors
            from rclpy.node import Node
            from fd_msgs.srv import GetParam, SetParam
        except ImportError as e:
            raise SystemExit(f"rclpy/fd_msgs 不可用,无法切流: {e}")
        if not rclpy.ok():
            rclpy.init()
        self.role = role
        self.module = PARAM_MODULE[role]
        self._node = Node(f"collect_yolo_param_{os.getpid() % 100000}")
        self._get = self._node.create_client(GetParam, "/fd_parameter/get_param")
        self._set = self._node.create_client(SetParam, "/fd_parameter/set_param")
        ex = rclpy.executors.SingleThreadedExecutor()
        ex.add_node(self._node)
        threading.Thread(target=ex.spin, daemon=True).start()
        if not (self._get.wait_for_service(5.0) and self._set.wait_for_service(5.0)):
            raise SystemExit("fd_parameter 服务不可用(fd_parameter_server 没跑?)")

    def _call(self, cli, req, timeout_s: float = 8.0):
        fut = cli.call_async(req)
        end = time.time() + timeout_s
        while time.time() < end:
            if fut.done():
                return fut.result()
            time.sleep(0.05)
        raise SystemExit(f"fd_parameter 服务调用超时({timeout_s}s)")

    def get_int(self, key: str, default: int) -> int:
        from fd_msgs.srv import GetParam
        req = GetParam.Request(module=self.module, key=key,
                               type=self._TYPE_INT32, mode=1)   # MODE_TRY_GET
        r = self._call(self._get, req)
        return r.int32_value if r.found else default

    def get_bool(self, key: str, default: bool) -> bool:
        from fd_msgs.srv import GetParam
        req = GetParam.Request(module=self.module, key=key,
                               type=self._TYPE_BOOL, mode=1)
        r = self._call(self._get, req)
        return r.bool_value if r.found else default

    def set_int(self, key: str, value: int) -> None:
        from fd_msgs.srv import SetParam
        r = self._call(self._set, SetParam.Request(
            module=self.module, key=key, type=self._TYPE_INT32,
            int32_value=value))
        if not r.success:
            raise SystemExit(f"set_param {key}={value} 失败: {r.message}")

    def set_bool(self, key: str, value: bool) -> None:
        from fd_msgs.srv import SetParam
        r = self._call(self._set, SetParam.Request(
            module=self.module, key=key, type=self._TYPE_BOOL,
            bool_value=value))
        if not r.success:
            raise SystemExit(f"set_param {key}={value} 失败: {r.message}")

    def snapshot(self) -> dict:
        return {"width": self.get_int("width", 640),
                "height": self.get_int("height", 480),
                "enable_depth": self.get_bool("enable_depth", False),
                "enable_align_depth": self.get_bool("enable_align_depth", False)}

    def wait_stream_width(self, width: int, timeout_s: float = 60.0) -> bool:
        """等 hub pipeline 按新分辨率起流(camera_info.width 到位即完成)。

        hub 重启管线后订阅偶发掉匹配不再收帧,每 4s 重建订阅兜底;
        1080p 起流实测可超 25s,超时给足 60s(也覆盖 5s 重试自愈)。"""
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CameraInfo
        cur = {"w": None}
        end = time.time() + timeout_s
        while time.time() < end:
            sub = self._node.create_subscription(
                CameraInfo, f"/camera/camera_{self.role}/color/camera_info",
                lambda m: cur.__setitem__("w", m.width), qos_profile_sensor_data)
            t_end = min(time.time() + 4.0, end)
            while time.time() < t_end and cur["w"] != width:
                time.sleep(0.1)
            self._node.destroy_subscription(sub)
            if cur["w"] == width:
                return True
        return False

    def wait_first_frame(self, width: int, timeout_s: float = 60.0) -> bool:
        """等第一帧目标宽度的图像(切档后首帧可能要几十秒,别让
        DepthSession.start 的 10s 窗扑空)。订阅同样 4s 重建兜底。"""
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image
        cur = {"w": None}
        end = time.time() + timeout_s
        while time.time() < end:
            sub = self._node.create_subscription(
                Image, f"/camera/camera_{self.role}/color/image_raw",
                lambda m: cur.__setitem__("w", m.width), qos_profile_sensor_data)
            t_end = min(time.time() + 4.0, end)
            while time.time() < t_end and cur["w"] != width:
                time.sleep(0.1)
            self._node.destroy_subscription(sub)
            if cur["w"] == width:
                return True
        return False


def hot_swap_res(hp: HubParams, target: tuple) -> bool:
    """把 hub 彩色流热切到 target=(w,h),返回是否切换成功。

    拨 enable_depth 触发 pipeline 重启(见模块 docstring)。两分支的
    深度请求都只发生在 ≤720p 的老分辨率下,flags 结束时回到原值。"""
    cur = hp.snapshot()
    tw, th = target
    if (cur["width"], cur["height"]) == (tw, th):
        return True                      # 已在目标档,无需切换
    if cur["enable_depth"] or cur["enable_align_depth"]:
        if tw > 1280 or th > 720:        # 深度常开:彩深共档,>720p 炸深度
            print("⚠ 该角色配了常开深度流,已钳到 1280x720(深度安全上限)")
            tw, th = 1280, 720
        hp.set_bool("enable_depth", False)       # 双关 → 老分辨率纯彩重启
        hp.set_bool("enable_align_depth", False)
        time.sleep(3.0)
        hp.set_int("width", tw)
        hp.set_int("height", th)
        time.sleep(0.5)
        hp.set_bool("enable_depth", cur["enable_depth"])    # 原样恢复 → 新档重启
        hp.set_bool("enable_align_depth", cur["enable_align_depth"])
    else:
        hp.set_bool("enable_depth", True)        # 老分辨率带深度重启(640x480 合法)
        time.sleep(3.0)
        hp.set_int("width", tw)
        hp.set_int("height", th)
        time.sleep(0.5)
        hp.set_bool("enable_depth", False)       # 纯彩按新档重启,不请求 1080p 深度
    return hp.wait_stream_width(tw)


def manual_restore_hint(role: str) -> str:
    m = PARAM_MODULE[role]
    cmd = ("ros2 service call /fd_parameter/set_param fd_msgs/srv/SetParam "
           '"{module: \'%s\', key: \'%s\', type: 3, int32_value: %d}"')
    return "\n".join([cmd % (m, "width", 640), cmd % (m, "height", 480),
                      "  (或直接跑 python collect_yolo.py --restore)"])


# --------------------------------------------------------------- 采集动作


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
                print(f"saved {jpg.name} [{frame.shape[1]}x{frame.shape[0]}]  "
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
        print(f"[{i}/{args.auto}] {jpg} [{frame.shape[1]}x{frame.shape[0]}]")
        if i < args.auto:
            time.sleep(args.interval)


def parse_res(s: str) -> tuple:
    try:
        w, h = s.lower().split("x")
        return (int(w), int(h))
    except ValueError:
        raise SystemExit(f"--res 应为 WxH 格式(如 1920x1080),收到: {s}")


def main():
    ap = argparse.ArgumentParser(
        description="左臂相机 YOLO 训练图采集(可直接 python 运行,环境自举,"
                    "默认以相机上限 1920x1080 热切换 hub 流采集)")
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
    ap.add_argument("--res", default="1920x1080", metavar="WxH",
                    help="采集分辨率(默认 1920x1080=D435/D438 彩色上限@30fps;"
                         "640x480=hub 默认流不切换;>720p 需深度流空闲)")
    ap.add_argument("--restore", action="store_true",
                    help="应急:把 hub 流恢复 640x480 后退出(kill -9 残留用)")
    args = ap.parse_args()
    target = parse_res(args.res)

    ensure_ros_env()
    try:
        import fd_msgs  # noqa: F401  rclpy 已在 ensure_ros_env 试过
    except ImportError as e:
        raise SystemExit(
            f"fd_msgs 导入失败({e})。自举未生效 —— 请检查:\n"
            "  1) /usr/bin/python3.10 存在且装了 cv2/rclpy\n"
            "  2) ROS humble 与 fd_ws 环境完整")

    hp = HubParams(args.role)
    orig = hp.snapshot()

    if args.restore:                     # 应急恢复:不管现状,拨回 640x480
        ok = hot_swap_res(hp, (640, 480))
        print("已恢复 640x480" if ok else
              "恢复未确认(流可能仍卡高档),可手工执行:\n"
              + manual_restore_hint(args.role))
        return

    out_root = Path(args.out).expanduser()
    for c in CLASSES:
        (out_root / c).mkdir(parents=True, exist_ok=True)
        (out_root / "labels" / c).mkdir(parents=True, exist_ok=True)

    switched = False
    sess = None
    try:
        if target != (orig["width"], orig["height"]):
            print(f"hub 流热切换: {orig['width']}x{orig['height']} → "
                  f"{target[0]}x{target[1]}(切换/恢复各约 6s,预览闪断属正常)")
            if not hot_swap_res(hp, target):
                hot_swap_res(hp, (orig["width"], orig["height"]))   # 尽力回滚
                raise SystemExit(f"切到 {target[0]}x{target[1]} 失败"
                                 "(相机不支持该档或深度流被占?),已回滚")
            switched = True
            print("等首帧(高档位起流慢,最长 60s)…")
            if not hp.wait_first_frame(target[0]):
                raise SystemExit(f"已切到 {target[0]}x{target[1]} 但 60s 无图像帧,"
                                 "相机起流异常,退出(finally 会恢复原档)")

        sess = DepthSession(args.role, client_prefix="collect_yolo_")
        info = sess.start(need_depth=False)   # 免租约:不动深度管线
        print(f"相机就绪: role={info['role']} 彩色={info['color_ok']} "
              f"{target[0]}x{target[1]} "
              f"内参={'有' if info['intrinsics'] else '无'} "
              f"耗时{info['waited_s']}s")
        print(f"输出目录: {out_root}")

        if args.auto:
            auto_capture(sess, out_root, args)
        else:
            interactive(sess, out_root, args)
    except KeyboardInterrupt:
        print("\n手动中断")
    finally:
        if sess is not None:
            sess.stop()
        if switched:
            if hot_swap_res(hp, (orig["width"], orig["height"])):
                print(f"hub 流已恢复 {orig['width']}x{orig['height']}")
            else:
                print("⚠ hub 流恢复未确认!手工执行:\n"
                      + manual_restore_hint(args.role))

    total = {c: len(list((out_root / c).glob("*.jpg"))) for c in CLASSES}
    print(f"目录累计: " + " ".join(f"{c}={total[c]}" for c in CLASSES)
          + f"  (共 {sum(total.values())} 张)")
    print("后续: labelimg 标 front/back(neg 已空标)→ 参照 "
          "old/tianlian/weilong/make_split_wrist.py 切分 → 现场集微调")


if __name__ == "__main__":
    main()
