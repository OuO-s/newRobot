# -*- coding: utf-8 -*-
"""Web 后端服务层:纯 Python 业务封装,无 Qt 依赖。

线程模型:每个长生命周期后台轮询用 daemon `threading.Thread`(不用 QThread)。
"""
from __future__ import annotations

import json
import threading
import time
from typing import Callable, Optional

from robot_client import RobotError, TianlianRobot
from slamtec_client import SlamtecChassis, SlamtecError


# ===========================================================================
# 无信号占位帧(替代历史上的固件 imageCapture 兜底)
# ===========================================================================
_PLACEHOLDER_LOCK = threading.Lock()
_PLACEHOLDER_CACHE: dict = {}      # (cam, reason) -> (ts, jpeg)


def _no_signal_jpeg(cam: str, reason: str) -> bytes:
    """深灰底占位帧:桥接无帧时明确显示"无信号",绝不显示拉伸畸变帧。"""
    key = (cam, reason)
    with _PLACEHOLDER_LOCK:
        hit = _PLACEHOLDER_CACHE.get(key)
        if hit and time.time() - hit[0] < 5.0:
            return hit[1]
    from io import BytesIO
    from PIL import Image, ImageDraw, ImageFont
    img = Image.new("RGB", (640, 480), (28, 30, 34))
    d = ImageDraw.Draw(img)
    try:
        font = ImageFont.load_default(size=30)
        small = ImageFont.load_default(size=18)
    except TypeError:                                   # 旧 Pillow 无 size 参数
        font = small = ImageFont.load_default()
    lines = [("无信号", font, (200, 204, 210)), (cam, font, (158, 162, 168)),
             (reason, small, (158, 162, 168)),
             (time.strftime("%H:%M:%S"), small, (110, 114, 120))]
    y = 480 / 2 - 80
    for text, f, fill in lines:
        w = d.textlength(text, font=f)
        d.text(((640 - w) / 2, y), text, fill=fill, font=f)
        y += 46 if f is font else 32
    buf = BytesIO()
    img.save(buf, "JPEG", quality=80)
    jpeg = buf.getvalue()
    with _PLACEHOLDER_LOCK:
        _PLACEHOLDER_CACHE[key] = (time.time(), jpeg)
    return jpeg


# ===========================================================================
# 整机状态轮询(1Hz)
# ===========================================================================
class StatusPoller:
    """周期查询整机状态,聚合成一个 dict 缓存,供 SSE/HTTP 读取。"""

    INTERVAL_S = 1.0
    MAX_FAIL = 5

    def __init__(self, robot: TianlianRobot):
        self.robot = robot
        self._snapshot: dict = {}
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._fails = 0
        self.on_soft_error: Optional[Callable[[str], None]] = None
        self.on_lost: Optional[Callable[[str], None]] = None

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._fails = 0
        self._thread = threading.Thread(target=self._run, name="StatusPoller",
                                        daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=3)

    def snapshot(self) -> dict:
        with self._lock:
            return dict(self._snapshot)

    def _emit(self, fn: Optional[Callable], msg: str) -> None:
        if fn:
            try:
                fn(msg)
            except Exception:                                       # noqa: BLE001
                pass

    def _run(self) -> None:
        while not self._stop.is_set():
            data: dict = {}
            fails_here = 0

            try:
                data["joints"] = self.robot.get_joint_angles()
            except (RobotError, KeyError, TypeError, ValueError) as e:
                data["joints"] = None
                self._emit(self.on_soft_error, f"关节: {e}")
            try:
                data["end_poses"] = self.robot.get_end_poses()
            except (RobotError, KeyError, TypeError, ValueError) as e:
                data["end_poses"] = None
                self._emit(self.on_soft_error, f"末端: {e}")
            try:
                data["manipulator"] = self.robot.get_manipulator_states()
            except RobotError:
                data["manipulator"] = None

            for key, fn in (
                ("lift",          self.robot.lift_info),
                ("chassis_pose",  self.robot.chassis_pose),
                ("waist",         self.robot.get_waist_joint),
                ("head",          self.robot.get_head_joint),
                ("emergency",     self.robot.emergency_status),
            ):
                try:
                    data[key] = fn()
                except (RobotError, KeyError, TypeError, ValueError) as e:
                    data[key] = None
                    fails_here += 1
                    self._emit(self.on_soft_error, f"{key}: {e}")

            with self._lock:
                self._snapshot = data

            if fails_here >= 2:
                self._fails += 1
                if self._fails >= self.MAX_FAIL:
                    self._emit(self.on_lost,
                               f"连续 {self._fails} 次轮询失败,连接已断开")
                    return
            else:
                self._fails = 0

            if self._stop.wait(self.INTERVAL_S):
                return


# ===========================================================================
# 多相机抓拍(每路独立线程,缓存最新一帧)
# ===========================================================================
class CameraStreamService:
    """每路相机一个取帧线程,缓存最新 JPEG 供 MJPEG / snapshot 端点使用。

    帧只来自 camera_bridge 子进程(ROS2 彩色话题直订阅,原生 640×480,
    无固件 1920×1080 拉伸畸变)。桥接进程失联时自动重拉;暂无帧时显示
    无信号占位——**固件 imageCapture 已彻底不用**(它会拉伸画面,历史上
    两次"页面畸变"均源于对它的静默降级)。
    """

    BRIDGE_URL = "http://127.0.0.1:7101"

    def __init__(self, robot: TianlianRobot, bridge_ensure=None):
        self.robot = robot
        self._bridge_ensure = bridge_ensure    # 桥接失联时用来复活(app.py 传入)
        self._last_ensure = 0.0                # 复活节流
        self._frames: dict[str, bytes] = {}
        self._errors: dict[str, str] = {}
        self._meta: dict[str, dict] = {}      # cam -> {"kb":..,"ts":..,"src":..,"w":..,"h":..}
        self._lock = threading.Lock()
        self._threads: dict[str, threading.Thread] = {}
        self._stops: dict[str, threading.Event] = {}
        self._intervals: dict[str, int] = {}

    def start(self, cam: str, interval_ms: int = 1000) -> None:
        # 下限 100ms:桥接取帧便宜,可支撑 ~10fps 预览;固件兜底自身慢,
        # 循环按耗时自适应,不会被这个下限打爆
        self._intervals[cam] = max(100, int(interval_ms))
        thread = self._threads.get(cam)
        if thread and thread.is_alive():
            return
        stop = threading.Event()
        self._stops[cam] = stop
        thread = threading.Thread(target=self._run, args=(cam, stop),
                                  name=f"Cam-{cam}", daemon=True)
        self._threads[cam] = thread
        thread.start()

    def stop(self, cam: Optional[str] = None) -> None:
        cams = [cam] if cam else list(self._stops)
        for c in cams:
            evt = self._stops.get(c)
            if evt:
                evt.set()

    def stop_all(self) -> None:
        self.stop()

    def latest(self, cam: str) -> Optional[bytes]:
        with self._lock:
            return self._frames.get(cam)

    def meta(self, cam: str) -> dict:
        with self._lock:
            return dict(self._meta.get(cam, {}))

    def meta_all(self) -> dict:
        with self._lock:
            return {c: dict(m) for c, m in self._meta.items()}

    def last_error(self, cam: str) -> Optional[str]:
        with self._lock:
            return self._errors.get(cam)

    def active_cams(self) -> list[str]:
        return [c for c, t in self._threads.items() if t.is_alive()]

    def _fetch_bridge(self, cam: str):
        """桥接 /snapshot 取最新帧 JPEG。

        返回 (帧或 None, 桥接进程是否失联):HTTP 503 只是暂无帧(进程活着,
        bridge 自身有看门狗会自愈);连接层失败才是进程没了,需要复活。
        """
        import urllib.error
        import urllib.request
        try:
            with urllib.request.urlopen(
                    f"{self.BRIDGE_URL}/snapshot?cam={cam}", timeout=2.0) as r:
                if r.status != 200:
                    return None, False
                return r.read(), False
        except urllib.error.HTTPError:
            return None, False
        except Exception:                               # noqa: BLE001
            return None, True

    def _bridge_stats(self, cam: str) -> Optional[dict]:
        """桥接 /health 里该相机的 w/h/fps(用于元信息展示)。"""
        import urllib.request
        try:
            with urllib.request.urlopen(f"{self.BRIDGE_URL}/health",
                                        timeout=1.0) as r:
                info = json.loads(r.read())
            return (info.get("cams") or {}).get(cam)
        except Exception:                               # noqa: BLE001
            return None

    def _run(self, cam: str, stop: threading.Event) -> None:
        interval = self._intervals.get(cam, 1000) / 1000.0
        last_health = 0.0
        while not stop.is_set():
            t0 = time.time()
            data, bridge_down = self._fetch_bridge(cam)
            src = "ros"
            if bridge_down and time.time() - self._last_ensure > 30.0:
                # 桥接进程失联:先试着拉回来
                self._last_ensure = time.time()
                if self._bridge_ensure:
                    try:
                        self._bridge_ensure()
                    except Exception:                        # noqa: BLE001
                        pass
            if data is None:
                # 2026-10-22 根治:不再走固件 imageCapture 兜底——它把 4:3
                # 硬拉成 16:9,画面必歪,两次"页面畸变"都是这条静默降级路径。
                # 没帧就显示无信号占位;bridge 侧有看门狗+webapp 自动复活,
                # 恢复后 src 自动翻回 ros。畸变帧从链路上不可能再出现。
                src = "none"
                data = _no_signal_jpeg(
                    cam, "相机桥接失联,正在自动拉起" if bridge_down
                    else "相机桥接暂无帧,等待看门狗重建订阅")
                with self._lock:
                    self._errors[cam] = (
                        "bridge 失联,已触发自动拉起" if bridge_down
                        else "bridge 无帧,等待看门狗自愈")
            with self._lock:
                self._frames[cam] = data
                self._meta[cam] = {"kb": round(len(data) / 1024, 1),
                                   "ts": time.time(), "src": src}
                self._errors.pop(cam, None)
            # 每 ~5s 从桥接补一次原生分辨率/帧率信息
            if src == "ros" and time.time() - last_health > 5.0:
                last_health = time.time()
                st = self._bridge_stats(cam)
                if st:
                    with self._lock:
                        self._meta[cam].update(
                            w=st.get("w"), h=st.get("h"),
                            fps=st.get("fps"), age_ms=st.get("age_ms"))
            elapsed = time.time() - t0
            if stop.wait(max(0.05, interval - elapsed)):
                return


# ===========================================================================
# 思岚底盘位姿/激光轮询
# ===========================================================================
class ChassisService:
    """Slamware 底盘轮询(位姿 + 激光 2Hz),地图按需加载缓存。"""

    INTERVAL_S = 0.5

    def __init__(self, chassis: SlamtecChassis):
        self.chassis = chassis
        self._pose: Optional[dict] = None
        self._laser: list[dict] = []
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._map: Optional[dict] = None
        self._map_lock = threading.Lock()

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="ChassisPoller",
                                        daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=3)

    def pose(self) -> Optional[dict]:
        with self._lock:
            return dict(self._pose) if self._pose else None

    def laser(self) -> list[dict]:
        with self._lock:
            return list(self._laser)

    def grid_map(self, refresh: bool = False) -> Optional[dict]:
        if refresh:
            with self._map_lock:
                self._map = None
        with self._map_lock:
            if self._map is None:
                try:
                    self._map = self.chassis.get_grid_map()
                except SlamtecError:
                    return None
            return self._map

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                pose = self.chassis.get_pose()
                laser = []
                try:
                    laser = self.chassis.get_laserscan().get("laser_points", [])
                except SlamtecError:
                    pass
                with self._lock:
                    self._pose = pose
                    self._laser = laser
            except SlamtecError:
                pass
            if self._stop.wait(self.INTERVAL_S):
                return


# ===========================================================================
# 全局应用状态(单用户,单进程)
# ===========================================================================
class AppState:
    """应用运行时单例:连接到的机器人 / 底盘 / 各种服务。"""

    def __init__(self) -> None:
        self.robot: Optional[TianlianRobot] = None
        self.status_poller: Optional[StatusPoller] = None
        self.camera: Optional[CameraStreamService] = None
        self.chassis_obj: Optional[SlamtecChassis] = None
        self.chassis_service: Optional[ChassisService] = None

    def disconnect_robot(self) -> None:
        if self.status_poller:
            self.status_poller.stop()
            self.status_poller = None
        if self.camera:
            self.camera.stop_all()
            self.camera = None
        self.robot = None

    def disconnect_chassis(self) -> None:
        if self.chassis_service:
            self.chassis_service.stop()
            self.chassis_service = None
        self.chassis_obj = None

    def shutdown(self) -> None:
        self.disconnect_robot()
        self.disconnect_chassis()
