#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""相机 ROS2 话题 → 本地 MJPEG/快照桥接服务。

上位机 GUI(conda 3.11)无法导入 rclpy,本进程改用系统 Python 3.10 +
ROS2 Humble 订阅机器人三路彩色话题,缓存最新帧并转发为本地 HTTP:

  GET /health                 {ok, cams:{left:{fps,w,h,frames},...}}
  GET /snapshot?cam=left      最新一帧 JPEG(暂无帧时 503)
  GET /stream?cam=left&fps=15 multipart/x-mixed-replace MJPEG 流(丢帧跟随)
  GET /joints                 {ok, age_ms, hz, left_deg[7], right_deg[7]}

/joints 是 /joint_states(100Hz)的订阅缓存(2026-11-05):DDS 风暴期
:8000 的 get_joint_angle 服务往返劣化到 0.6~5s,而话题推送不受影响
(厂商 :8091 调试页即靠 topic 缓存保持顺滑),webapp 点动等高频读
改走这里,毫秒级拿到 ≤10ms 新鲜的关节角。

GUI 以 bash -c "source /opt/ros/humble/setup.bash && exec python3 camera_bridge.py"
方式拉起(app.py MainWindow.ensure_camera_bridge),退出时回收。
手动调试:source /opt/ros/humble/setup.bash && python3 camera_bridge.py
"""
import argparse
import json
import os
import re
import signal
import subprocess
import sys
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, JointState
except ImportError as e:  # pragma: no cover - 须在 ROS2 环境下运行
    raise SystemExit(f"需要 ROS2 环境(先 source /opt/ros/humble/setup.bash):{e}")

# encoding → (通道序, 通道数);发布端 RealSense 彩色流为 rgb8
ENCODINGS = {
    "rgb8": (False, 3), "bgr8": (True, 3),
    "rgba8": (False, 4), "bgra8": (True, 4),
}


class FrameStore:
    """每路相机缓存最新一帧(订阅线程写,HTTP 线程读,锁保护)。"""

    def __init__(self):
        self._lock = threading.Lock()
        self._data: dict = {}      # cam -> (seq, h, w, encoding, raw bytes)
        self._stamps: dict = {}    # cam -> deque[time.monotonic] 用于帧率统计
        self._jpeg: dict = {}      # cam -> (seq, jpeg bytes) 编码缓存

    def update(self, cam, h, w, encoding, raw: bytes) -> None:
        with self._lock:
            seq = (self._data.get(cam, (0,))[0] if cam in self._data else 0) + 1
            self._data[cam] = (seq, h, w, encoding, raw)
            self._stamps.setdefault(cam, deque(maxlen=120)).append(time.monotonic())

    def stats(self, cam) -> dict:
        with self._lock:
            item = self._data.get(cam)
            stamps = list(self._stamps.get(cam, ()))
        now = time.monotonic()
        recent = [t for t in stamps if now - t <= 3.0]
        out = {"fps": round(len(recent) / 3.0, 1) if stamps else 0.0,
               "frames": len(stamps)}
        if item:
            out.update(w=item[2], h=item[1], encoding=item[3],
                       age_ms=int((now - stamps[-1]) * 1000) if stamps else -1)
        return out

    def jpeg(self, cam, max_width: int, quality: int):
        """最新帧编码为 JPEG;返回 (seq, jpeg) 或 None(暂无帧)。"""
        with self._lock:
            item = self._data.get(cam)
            cached = self._jpeg.get(cam)
            if not item:
                return None
            seq = item[0]
            if cached and cached[0] == seq:
                return cached
        # 编码放锁外(几十 ms 级不占锁;期间可能又来新帧,取到哪帧算哪帧)
        _, h, w, encoding, raw = item
        swap_bgr, ch = ENCODINGS.get(encoding, (True, 3))
        img = np.frombuffer(raw, dtype=np.uint8).reshape(h, w, ch)
        import cv2
        if ch == 4:
            img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR if not swap_bgr
                               else cv2.COLOR_BGRA2BGR)
        elif not swap_bgr:
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        if max_width and w > max_width:
            nh = int(h * max_width / w)
            img = cv2.resize(img, (max_width, nh), interpolation=cv2.INTER_AREA)
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, quality])
        if not ok:
            return None
        jpeg = buf.tobytes()
        with self._lock:
            self._jpeg[cam] = (seq, jpeg)
        return seq, jpeg


STORE = FrameStore()
OPTS = {"max_width": 0, "quality": 82}


# 臂关节名:left_tl_robot_joint1..7 / right_tl_robot_joint1..7
# (与 fd_arm_http http_node._joint_names 一致;消息里名字顺序是乱的,
#  必须按名字映射,不能按下标)
_ARM_JOINT_RE = re.compile(r"^(left|right)_tl_robot_joint([1-7])$")


class JointStore:
    """/joint_states 订阅缓存(订阅线程写,HTTP 线程读,锁保护)。

    只保留双臂 7+7 关节角(度),带单调时钟戳与更新率统计。"""

    def __init__(self):
        self._lock = threading.Lock()
        self._stamps: deque = deque(maxlen=400)   # 100Hz×4s,足够算率
        self._left: list = []                     # [deg×7],按 joint1..7 排序
        self._right: list = []

    def update(self, msg: JointState) -> None:
        left, right = [None] * 7, [None] * 7
        for name, pos in zip(msg.name, msg.position):
            m = _ARM_JOINT_RE.match(str(name))
            if not m:
                continue           # torso/head/waist 等非臂关节
            deg = float(pos) * 57.29577951308232
            arr = left if m.group(1) == "left" else right
            arr[int(m.group(2)) - 1] = deg
        if all(v is not None for v in left) and all(v is not None for v in right):
            with self._lock:
                self._left, self._right = left, right
                self._stamps.append(time.monotonic())
        # 缺关节(消息异常/单臂配置)直接丢弃本帧,不污染缓存

    def snapshot(self) -> dict:
        """{age_ms, hz, left_deg, right_deg};从未收到过则 age_ms=-1。"""
        with self._lock:
            left, right = list(self._left), list(self._right)
            stamps = list(self._stamps)
        now = time.monotonic()
        recent = [t for t in stamps if now - t <= 3.0]
        return {
            "age_ms": int((now - stamps[-1]) * 1000) if stamps else -1,
            "hz": round(len(recent) / 3.0, 1) if stamps else 0.0,
            "left_deg": [round(v, 3) for v in left] if left else None,
            "right_deg": [round(v, 3) for v in right] if right else None,
        }


JOINTS = JointStore()


class ManipStore:
    """/fd_robot/manipulator/current_joint_states 订阅缓存(夹爪实际开口)。

    tl_driver 持续上报 *_jaw 实际位置(0~1000)——这是真实反馈不是指令
    回声::8000 指令黑洞期它纹丝不动(2026-11-12 实证,判夹爪动没动
    不能信 :8000 的 get_manipulator_states,那个恒 1000)。"""

    def __init__(self):
        self._lock = threading.Lock()
        self._jaw: dict = {}                    # name → (position, monotonic)

    def update(self, msg: JointState) -> None:
        now = time.monotonic()
        with self._lock:
            for name, pos in zip(msg.name, msg.position):
                self._jaw[str(name)] = (float(pos), now)

    def jaw(self, name: str):
        with self._lock:
            return self._jaw.get(name)          # (pos, mono) | None


MANIP_STORE = ManipStore()
MANIP = {"pub": None}         # BridgeNode 的 joint_state_cmd publisher

# —— 机械臂运动代理(2026-11-05) ——
# :8000 fd_arm_http 是 Python rclpy:一次 /move_robotarm 要过 5 次 DDS 往返
# (set_speed + get_state(取插值起点) + goal + result…),DDS 风暴期(对端
# x86 栈 30MB/s 互灌,单次服务往返一半概率 ~3s)一个动作膨胀到 30s+
# (2026-11-05 实测:A2_place 58° 行程 37.5s 仍未到位,同速 65° 的 A1 仅
# 7.7s);:8091 调试页快就快在 C++ rclcpp 直发 action。本进程已是系统
# 3.10 + rclpy,故加运动代理:插值算法直接用厂商 fd_arm_api 库(同一份
# 代码,行为一致),插值起点取 /joint_states 缓存(省一次 get_state),
# set_speed 结果按臂缓存(同速不重发),最后单发 follow_trajectory
# action 直达 fd_arm_server(C++,4ms 流式在它内部)。webapp 侧代理
# 不可达自动回落 :8000。dry_run=1 只插值不下发(排障/对拍用)。
ARM_PROXY = {"client": None, "err": "", "sem": threading.Semaphore(2),
             "speed_last": {}, "cfg": None}


def _arm_client():
    """惰性初始化 fd_arm_api.ArmClient(自带节点+自旋线程;失败只试一次)。"""
    if ARM_PROXY["client"] is not None or ARM_PROXY["err"]:
        return ARM_PROXY["client"]
    try:
        from fd_arm_api import ArmClient
        try:
            from fd_arm_api.app_config import get_config
            ARM_PROXY["cfg"] = get_config()
        except Exception:                                # noqa: BLE001
            ARM_PROXY["cfg"] = None
        ARM_PROXY["client"] = ArmClient(node=None)
        print("ARM_PROXY_READY fd_arm_api client online", flush=True)
    except Exception as e:                               # noqa: BLE001
        ARM_PROXY["err"] = f"{type(e).__name__}: {e}"
        print(f"ARM_PROXY_INIT_FAILED {ARM_PROXY['err']}", flush=True)
    return ARM_PROXY["client"]


def _arm_unavailable() -> tuple:
    return 503, {"ok": False, "available": False, "error": ARM_PROXY["err"] or
                 "fd_arm_api 不可用(需 source ROS 环境启动本桥)"}


def _arm_motion_cfg():
    m = getattr(ARM_PROXY["cfg"], "motion", None)
    dt = float(getattr(m, "default_trajectory_dt_s", 0.0) or 0.0) if m else 0.0
    through = bool(getattr(m, "smooth_through_waypoints", True)) if m else True
    dwell = float(getattr(m, "streaming_waypoint_dwell_s", 0.0) or 0.0) if m else 0.0
    return (dt or 0.01), through, dwell


def _arm_move_traj(body: dict) -> tuple:
    """轨迹直发:插值(本地)→ follow action(单次)。返回 (http码, dict)。

    stage 语义(webapp 据此决定是否回落 :8000):
      pre/args/available —— goal 从未发出,回落安全;
      result —— goal 已发出(含被拒/超时/异常),绝不可回落重发。
    """
    import math
    client = _arm_client()
    if client is None:
        return _arm_unavailable()
    arm = body.get("arm") or ""
    traj_deg = body.get("trajectory_deg") or []
    if arm not in ("armleft", "armright") or not traj_deg:
        return 400, {"ok": False, "stage": "args", "error": "缺少 arm/trajectory_deg"}
    speed = int(body.get("speed") or 15)
    dof = min(len(pt) for pt in traj_deg)
    wps = [[math.radians(float(v)) for v in pt[:dof]] for pt in traj_deg]
    # 插值起点:显式传入 → /joint_states 缓存(≤500ms,0 服务往返)→ get_state 兜底
    q_deg = body.get("q_init_deg")
    if not q_deg:
        snap = JOINTS.snapshot()
        cached = snap.get("left_deg" if arm == "armleft" else "right_deg")
        if cached and 0 <= snap.get("age_ms", -1) <= 500 and len(cached) >= dof:
            q_deg = cached
    if q_deg and len(q_deg) >= dof:
        q_init = [math.radians(float(v)) for v in q_deg[:dof]]
    else:
        st = client.get_state(arm)
        if not st.success or len(st.joint_positions_rad) < dof:
            return 502, {"ok": False, "stage": "pre",
                         "error": f"读插值起点失败: {st.message}"}
        q_init = [float(v) for v in st.joint_positions_rad[:dof]]
    from fd_arm_api.trajectory_interpolator import (
        block_timeout_from_trajectory, interpolate_trajectory_smooth,
        motion_limits_from_speed_percent)
    dt, through_cfg, dwell_cfg = _arm_motion_cfg()
    through = body.get("smooth_through")
    through = bool(through) if through is not None else through_cfg
    dwell = dwell_cfg if (body.get("block", 1) and through) else 0.0
    try:
        max_vel, max_acc = motion_limits_from_speed_percent(speed)
        dense = interpolate_trajectory_smooth(
            wps, q_init, max_vel, max_acc, dt=dt,
            through_waypoints=through, waypoint_dwell_s=dwell)
    except Exception as e:                               # noqa: BLE001
        return 500, {"ok": False, "stage": "pre", "error": f"插值失败: {e}"}
    if not dense:
        return 500, {"ok": False, "stage": "pre", "error": "插值结果为空"}
    if body.get("dry_run"):
        dur = (len(dense) - 1) * dt
        return 200, {"ok": True, "dry_run": True, "points": len(dense),
                     "dt": dt, "duration_s": round(dur, 2),
                     "timeout_s": round(block_timeout_from_trajectory(dense, dt), 1),
                     "first": [round(math.degrees(v), 3) for v in dense[0]],
                     "last": [round(math.degrees(v), 3) for v in dense[-1]]}
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    side = "left" if arm == "armleft" else "right"
    msg = JointTrajectory()
    msg.joint_names = [f"{side}_tl_robot_joint{i + 1}" for i in range(dof)]
    t = 0.0
    for wp in dense:
        pt = JointTrajectoryPoint()
        pt.positions = [float(x) for x in wp[:dof]]
        pt.time_from_start.sec = int(t)
        pt.time_from_start.nanosec = int((t - int(t)) * 1e9)
        msg.points.append(pt)
        t += dt
    timeout = max(float(body.get("timeout") or 60.0),
                  block_timeout_from_trajectory(dense, dt))
    with ARM_PROXY["sem"]:
        if ARM_PROXY["speed_last"].get(arm) != speed:
            try:
                if client.set_speed(arm, speed).success:
                    ARM_PROXY["speed_last"][arm] = speed
            except Exception:                            # noqa: BLE001
                pass        # 与厂商一致:set_speed 失败不挡运动
        try:
            r = client.follow_trajectory(
                arm, msg, timeout_sec=timeout, pre_interpolated=True)
        except Exception as e:                           # noqa: BLE001
            return 502, {"ok": False, "stage": "result",
                         "error": f"follow 异常: {type(e).__name__}: {e}"}
    # —— result 丢失自愈(2026-11-06):follow 超时 ≠ 没走到。DDS 发现抖动
    # 会把 action result 回传路由到失效端点(15:56 实测:92 轮全优后一次
    # timeout,臂实际已到示教末点 ±0.01°,同刻 fd_log_manager 报本客户端
    # 节点 "missing from kRosNodeBoardRegistry (3 misses)")。超时且
    # /joint_states 缓存新鲜(≤500ms)时对轨迹末点校验,到位(≤2°/关节,
    # 与厂商到位判据同口径)按成功放行;webapp 侧 _traj 还有段后验证+
    # 补走二次兜底,误放行会被拦下。
    ok, note = bool(r.success), ""
    if not ok and "timeout" in (r.message or "").lower():
        snap = JOINTS.snapshot()
        cur = snap.get("left_deg" if arm == "armleft" else "right_deg")
        if cur and len(cur) >= dof and 0 <= snap.get("age_ms", -1) <= 500:
            tgt_deg = [math.degrees(v) for v in dense[-1][:dof]]
            err = max(abs(a - b) for a, b in zip(tgt_deg, cur[:dof]))
            if err <= 2.0:
                ok, note = True, f"result丢失·位置已校验(err {err:.2f}°)"
                print(f"ARM_PROXY_RESULT_RECOVERED {arm} "
                      f"err={err:.2f}deg raw_msg={r.message}", flush=True)
    print(f"ARM_PROXY_MOVE {arm} wps={len(wps)} dense={len(dense)} "
          f"dur={(len(dense) - 1) * dt:.1f}s speed={speed} "
          f"ok={ok} msg={r.message}{';' + note if note else ''}", flush=True)
    return 200, {"ok": ok, "stage": "result",
                 "message": (r.message or "") + (f";{note}" if note else ""),
                 "points": len(dense), "timeout_s": round(timeout, 1)}


def _arm_set_speed(body: dict) -> tuple:
    client = _arm_client()
    if client is None:
        return _arm_unavailable()
    arm = body.get("arm") or ""
    try:
        speed = int(body.get("speed") or 0)
    except (TypeError, ValueError):
        speed = 0
    if arm not in ("armleft", "armright") or not 1 <= speed <= 100:
        return 400, {"ok": False, "error": "arm 无效或 speed 不在 1..100"}
    try:
        r = client.set_speed(arm, speed)
    except Exception as e:                               # noqa: BLE001
        return 502, {"ok": False, "error": f"{type(e).__name__}: {e}"}
    if r.success:
        ARM_PROXY["speed_last"][arm] = speed
    return 200, {"ok": bool(r.success), "message": r.message}


def _arm_move_location(body: dict) -> tuple:
    """预设位姿直发 ArmMoveToLocation action(与 :8091 同一条路),
    目标解析与执行全在 fd_arm_server(C++)内,不经 fd_arm_http。"""
    client = _arm_client()
    if client is None:
        return _arm_unavailable()
    arm = body.get("arm") or ""
    location = str(body.get("location") or "")
    if arm not in ("armleft", "armright") or not location:
        return 400, {"ok": False, "stage": "args", "error": "缺少 arm/location"}
    timeout = float(body.get("timeout") or 60.0)
    speed = body.get("speed")
    with ARM_PROXY["sem"]:
        if isinstance(speed, int) and 1 <= speed <= 100 \
                and ARM_PROXY["speed_last"].get(arm) != speed:
            try:
                if client.set_speed(arm, speed).success:
                    ARM_PROXY["speed_last"][arm] = speed
            except Exception:                            # noqa: BLE001
                pass
        try:
            r = client.move_to_location(
                arm, location,
                use_joint_motion=bool(body.get("use_joint_motion")),
                timeout_sec=timeout)
        except Exception as e:                           # noqa: BLE001
            return 502, {"ok": False, "stage": "result",
                         "error": f"{type(e).__name__}: {e}"}
    return 200, {"ok": bool(r.success), "stage": "result", "message": r.message}


def _arm_power(body: dict) -> tuple:
    """上/下电(含急停下电)。arm 支持 both(与 :8000 语义一致)。"""
    client = _arm_client()
    if client is None:
        return _arm_unavailable()
    arm = body.get("arm") or "both"
    arms = ["armleft", "armright"] if arm == "both" else [arm]
    if not set(arms) <= {"armleft", "armright"}:
        return 400, {"ok": False, "error": "arm 必须是 armleft/armright/both"}
    power_on = bool(body.get("on"))
    msgs = []
    for a in arms:
        try:
            r = client.power(a, power_on)
        except Exception as e:                           # noqa: BLE001
            return 502, {"ok": False, "error": f"{a}: {type(e).__name__}: {e}"}
        msgs.append(f"{a}: {'ok' if r.success else r.message}")
        if not r.success:
            return 200, {"ok": False, "message": "; ".join(msgs)}
    return 200, {"ok": True, "message": "; ".join(msgs)}

# —— 自愈看门狗:fd_cameras_hub 重建发布端(角色轮换/USB 自愈)后,启动期创建的
# 订阅在 lo 无多播的 DDS 发现抖动下可能永远失配——进程活着但收不到帧。
# 检测按路进行:某路干涸且该话题上发布端仍存在(典型失配特征)→ 重建订阅;
# 全路干涸(hub 未起/整体失配)也重建。物理没开的相机无发布端,不空转。 ——
WD_CHECK_S = 30.0   # 检查周期
WD_DRY_S = 45.0     # 单路无帧超过此时长算干涸(hub USB 自愈约 45s,别低于它)
WD_GRACE_S = 90.0   # 启动宽限(等 DDS 发现/相机上电)
WD_MAX_RESUB = 6    # 连续干涸重订次数上限,超过=participant 卡死 → 换新进程重启


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt, *args):  # 静默访问日志
        pass

    def _json(self, code: int, obj: dict) -> None:
        body = json.dumps(obj, ensure_ascii=False).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        path = self.path.split("?", 1)[0]
        handlers = {
            "/arm/move_traj": _arm_move_traj,
            "/arm/set_speed": _arm_set_speed,
            "/arm/move_location": _arm_move_location,
            "/arm/power": _arm_power,
        }
        fn = handlers.get(path)
        if fn is None:
            self._json(404, {"ok": False, "error": f"未知路径 {path}"})
            return
        try:
            n = int(self.headers.get("Content-Length") or 0)
            body = json.loads(self.rfile.read(n).decode()) if n else {}
        except Exception as e:                           # noqa: BLE001
            self._json(400, {"ok": False, "error": f"请求体 JSON 无效: {e}"})
            return
        # 代理层异常绝不能带崩桥(相机/关节缓存是主业)
        try:
            code, out = fn(body)
        except Exception as e:                           # noqa: BLE001
            code, out = 500, {"ok": False, "stage": "pre",
                              "error": f"代理内部异常: {type(e).__name__}: {e}"}
        self._json(code, out)

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        params = {}
        if "?" in self.path:
            import urllib.parse
            params = {k: v[0] for k, v in
                      urllib.parse.parse_qs(self.path.split("?", 1)[1]).items()}
        if path == "/health":
            cams = {}
            for cam in ("left", "head", "right"):
                cams[cam] = STORE.stats(cam)
            self._json(200, {"ok": True, "service": "camera_bridge",
                             "cams": cams, **OPTS})
            return
        if path == "/joints":
            snap = JOINTS.snapshot()
            self._json(200, {"ok": True, "service": "camera_bridge",
                             **snap})
            return
        if path == "/snapshot":
            cam = params.get("cam", "left")
            got = STORE.jpeg(cam, OPTS["max_width"], OPTS["quality"])
            if not got:
                self._json(503, {"ok": False, "error": f"相机 {cam} 暂无帧"})
                return
            _, jpeg = got
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(jpeg)))
            self.end_headers()
            self.wfile.write(jpeg)
            return
        if path == "/stream":
            cam = params.get("cam", "left")
            fps = max(1, min(30, int(params.get("fps", 15))))
            self.send_response(200)
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            last_seq = -1
            period = 1.0 / fps
            try:
                while True:
                    t0 = time.monotonic()
                    got = STORE.jpeg(cam, OPTS["max_width"], OPTS["quality"])
                    if got and got[0] != last_seq:
                        last_seq = got[0]
                        jpeg = got[1]
                        part = ("--frame\r\nContent-Type: image/jpeg\r\n"
                                f"Content-Length: {len(jpeg)}\r\n\r\n"
                                ).encode() + jpeg + b"\r\n"
                        self.wfile.write(part)
                        self.wfile.flush()
                    # 源帧率高于 fps 时自然丢帧跟随;无新帧时轻睡等待
                    remain = period - (time.monotonic() - t0)
                    time.sleep(remain if remain > 0 else 0.002)
            except (BrokenPipeError, ConnectionResetError):
                return  # 客户端断开
            return
        self._json(404, {"ok": False, "error": f"未知路径 {path}"})


class BridgeNode(Node):
    def __init__(self, cams):
        super().__init__("uc_camera_bridge")
        self._cam_names = list(cams)
        self._subs: dict = {}
        self._t0 = time.monotonic()
        self._was_dry = False
        self._dry_n = 0
        self._joint_sub = None
        self._create_subs()
        self._create_joint_sub()
        # 看门狗在 executor 线程内跑,销毁/重建订阅(图变更)是安全的
        self.create_timer(WD_CHECK_S, self._watchdog)

    def _create_subs(self) -> None:
        for cam in self._cam_names:
            self._subs[cam] = self.create_subscription(
                Image, f"/camera/camera_{cam}/color/image_raw",
                (lambda m, c=cam: self._on_image(m, c)),
                qos_profile_sensor_data)

    def _create_joint_sub(self) -> None:
        # best-effort 订阅与任何发布端 QoS 都兼容(reliable pub ↔ best-effort
        # sub 可匹配;反向不兼容),丢包无所谓,只要最新一帧
        self._joint_sub = self.create_subscription(
            JointState, "/joint_states", JOINTS.update,
            qos_profile_sensor_data)

    def _relaunch_detached(self) -> None:
        """用全新进程重启自己(升级动作)。

        曾用 os.execv:不换 PID,FastDDS 按进程号派的发现端口也不变,
        2026-10-26 实证 exec 57 次全部无效(相机热插拔后发布端重建类
        失配,同 PID 的新 participant 依然发现不到发布端);而外部
        kill+重拉(新 PID)秒恢复。故改为:派一个 detached 子进程,
        等本进程死后以全新 PID 拉起本脚本,然后 SIGKILL 自己。
        子进程继承本进程的 stdout(即 camera_bridge.log)。"""
        # 2026-11-05:进程内新增运动代理 —— 动作执行中途自杀会让 webapp
        # 误判代理不可用而回落 :8000 重发同一轨迹(双发)。重启前先等
        # 运动静默(拿满代理双槽);极端情况等不到,最多 120s 后照常
        # 重启(相机业务优先级更高)。
        try:
            for _ in range(2):
                ARM_PROXY["sem"].acquire(timeout=60.0)
        except Exception:                                # noqa: BLE001
            pass
        cmd = (f"sleep 1.5; exec {sys.executable} "
               f"{os.path.abspath(__file__)} {' '.join(sys.argv[1:])}")
        subprocess.Popen(["/bin/bash", "-c", cmd], start_new_session=True)
        os.kill(os.getpid(), signal.SIGKILL)

    def _watchdog(self) -> None:
        if time.monotonic() - self._t0 < WD_GRACE_S:
            return
        self._watchdog_joints()
        stats = {c: STORE.stats(c) for c in self._cam_names}
        ages = {c: st.get("age_ms", -1) for c, st in stats.items()}
        # age_ms=-1 表示从未收到帧
        dry = {c for c, a in ages.items() if a < 0 or a > WD_DRY_S * 1000}
        # 发布端还在却收不到帧 = 订阅失配(该路发布端被 hub 重建过);
        # 发布端不在 = 相机/hub 真没开,重建也没用,等它出现
        mismatch = {c for c in dry
                    if self.count_publishers(
                        f"/camera/camera_{c}/color/image_raw") > 0}
        all_dry = len(dry) == len(self._cam_names)
        if mismatch or all_dry:
            self._dry_n += 1
            print(f"CAMERA_BRIDGE_WATCHDOG resubscribe dry={sorted(dry)} "
                  f"mismatch={sorted(mismatch)} ages_ms={ages} "
                  f"({self._dry_n}/{WD_MAX_RESUB})", flush=True)
            # 同进程重订救不了 participant 级卡死(2026-10-22 实证:重订近
            # 一小时仍 0 帧而话题 hz 正常,重拉进程秒恢复)。exec 换新
            # participant;HTTP socket 是 CLOEXEC 不会泄漏给新进程。
            if self._dry_n >= WD_MAX_RESUB:
                print("CAMERA_BRIDGE_WATCHDOG restart: resubscribe "
                      "exhausted, relaunch as fresh process", flush=True)
                self._relaunch_detached()
            for sub in self._subs.values():
                try:
                    self.destroy_subscription(sub)
                except Exception:                            # noqa: BLE001
                    pass
            self._create_subs()
            self._was_dry = True
        elif self._was_dry and not dry:
            self._was_dry = False
            self._dry_n = 0
            print(f"CAMERA_BRIDGE_WATCHDOG recovered ages_ms={ages}",
                  flush=True)

    def _watchdog_joints(self) -> None:
        """/joint_states 干涸(订阅失配同相机)→ 重建关节订阅。

        不参与进程重启升级:webapp 对关节缓存失效的兜底就是回退 :8000,
        慢但不坏;相机才需要激进的换进程自愈。"""
        age = JOINTS.snapshot()["age_ms"]
        if 0 <= age <= WD_DRY_S * 1000:
            return
        if self.count_publishers("/joint_states") == 0:
            return     # 发布端不在(如整机未上电),等它出现
        print(f"JOINTS_BRIDGE_WATCHDOG resubscribe age_ms={age}",
              flush=True)
        try:
            self.destroy_subscription(self._joint_sub)
        except Exception:                                    # noqa: BLE001
            pass
        self._create_joint_sub()

    def _on_image(self, msg: Image, cam: str) -> None:
        STORE.update(cam, msg.height, msg.width, msg.encoding, bytes(msg.data))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=7101)
    ap.add_argument("--max-width", type=int, default=0,
                    help="JPEG 输出最大宽(0=原样 640)")
    ap.add_argument("--jpeg-quality", type=int, default=82)
    ap.add_argument("--cams", default="left,head,right")
    args = ap.parse_args()
    OPTS["max_width"] = args.max_width
    OPTS["quality"] = args.jpeg_quality
    cams = [c.strip() for c in args.cams.split(",") if c.strip()]

    # 域号兜底:fd_cameras_hub 固定跑在 25。webapp 自动拉起/看门狗重启链
    # 曾都漏设 ROS_DOMAIN_ID → 落在默认 0 上永远看不见 hub,自愈循环空转
    # (2026-10-28 实证:242 次重启全因 domain 0)。setdefault 不覆盖显式值。
    os.environ.setdefault("ROS_DOMAIN_ID", "25")
    os.environ.setdefault("ROS_LOCALHOST_ONLY", "0")
    rclpy.init()
    node = BridgeNode(cams)
    httpd = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    print(f"CAMERA_BRIDGE_READY port={args.port} cams={','.join(cams)} "
          f"max_width={args.max_width} quality={args.jpeg_quality}", flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # SIGTERM/KeyboardInterrupt 下 spin 可能已触发部分清理,重复关停
        # 会抛 RCLError 刷一屏 traceback —— 逐项兜住,退出干净
        for cleanup in (node.destroy_node, rclpy.shutdown, httpd.shutdown):
            try:
                cleanup()
            except Exception:                            # noqa: BLE001
                pass


if __name__ == "__main__":
    main()
