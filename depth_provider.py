# -*- coding: utf-8 -*-
"""深度取流:fd_cameras_hub 租约 + ROS2 话题订阅(webapp 进程内)。
================================================================
移植自 old/tianlian/PBVS代码包/PBVS/core/fd_hub_camera.py(2026-10 在本机
实测过的实现),按 webapp 需要改造:

  - 只按角色取流(left/head/right),不再按序列号推断;
  - 租约位掩码: left=1 / head(center)=2 / right=4(fd_msgs 约定);
  - grab_pair(): 取一对时间戳对齐的 color+aligned_depth(对齐深度沿用彩色
    帧的 header.stamp,静止取景下两者差应 ≤10ms);
  - health(): 帧率/租约状态,供 UI 徽章轮询;
  - 会话幂等: 换角色即重建节点;结束/关停时释放租约(心跳停 5s 后 hub 也会
    自行收回,但主动释放更礼貌,别的取流者能立刻拿到)。

话题(QoS=BEST_EFFORT,与 hub 一致):
  /camera/camera_<role>/color/image_raw                 640x480 rgb8
  /camera/camera_<role>/aligned_depth_to_color/image_raw 16UC1 mm(同像素栅格)
  /camera/camera_<role>/color/camera_info               内参(与彩色流同源)

⚠ 无订阅者时 hub 不发布对齐深度 —— 先订阅、后拿租约。
⚠ 必须以 user 身份运行(webapp 满足);SHM 传输 profile 在 ensure_ros_env 兜底。
"""
from __future__ import annotations

import os
import threading
import time
from typing import Optional

import numpy as np

# 租约位掩码(fd_msgs/srv/SetCameraDepthLease.roles)。注意 hub 内部把头部
# 相机叫 "center",话题却叫 camera_head —— UI 的 head 标签对应位 2。
LEASE_BIT = {"left": 1, "head": 2, "center": 2, "right": 4}
ROLES = ("left", "head", "right")


class DepthError(Exception):
    """取流失败(无帧/租约失败/ROS 环境缺失),消息面向用户可读。"""


def ensure_ros_env() -> None:
    """rclpy 导入与 DDS 环境兜底。webapp 正常由带 ROS 环境的 shell 起时是空操作。"""
    os.environ.setdefault("ROS_DOMAIN_ID", "25")
    os.environ.setdefault("ROS_LOCALHOST_ONLY", "0")
    shm = "/home/user/fd_ws/src/fd_bringup/dds/transport_shm_consumer.xml"
    if os.path.exists(shm):
        os.environ.setdefault("FASTRTPS_DEFAULT_PROFILES_FILE", shm)
    try:
        import rclpy  # noqa: F401
    except ImportError:
        cand = "/opt/ros/humble/lib/python3.10/site-packages"
        if os.path.isdir(cand):
            import sys
            if cand not in sys.path:
                sys.path.append(cand)


class DepthSession:
    """单角色会话: 一个节点 + 三路订阅 + 租约心跳线程。

    need_depth=False 时只订阅彩色+内参、不申请租约 —— 对准测量这类
    纯 PnP 用途秒开(省掉租约与深度管线重启的 1.3~3s),且与别处持有的
    深度会话互不干扰。
    """

    def __init__(self, role: str, client_prefix: str = "webapp_pick_"):
        if role not in ROLES:
            raise DepthError(f"未知相机角色: {role}(可选 {ROLES})")
        self.role = role
        self.client_id = f"{client_prefix}{os.getpid()}"
        self.is_running = False
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._color: Optional[np.ndarray] = None      # BGR
        self._depth: Optional[np.ndarray] = None      # uint16 mm
        self._intr: Optional[dict] = None
        self._color_ts = 0.0                          # 收帧墙钟
        self._depth_ts = 0.0
        self._color_stamp = 0.0                       # header.stamp(秒)
        self._depth_stamp = 0.0
        self._color_n = 0                             # 帧计数(health 算帧率)
        self._depth_n = 0
        self._lease_ok = False
        self._spin_err: str | None = None
        self._node = None
        self._rclpy = None
        self._spin_thread: Optional[threading.Thread] = None
        self._hb_thread: Optional[threading.Thread] = None

    @property
    def color_age_s(self) -> float:
        """最近一帧彩色帧距今秒数(从未收帧=inf)——会话健康检查用。

        hub 重启相机管线后,本进程的 DDS 订阅会偶发掉匹配不再收帧
        (bridge 等其它进程正常):is_running 仍 True,靠帧龄才能发现。"""
        with self._lock:
            return time.time() - self._color_ts if self._color_ts else float("inf")

    @property
    def alive(self) -> bool:
        """会话真正可用:已启动、自旋线程在跑且从未带错退出。

        spin 线程死于 wait set 竞态时 is_running 仍 True、心跳仍发(租约
        不掉),帧却永远停更 —— 只看 is_running 会把死会话当活的复用
        (2026-10-21 扫描 503:扫描 already_running 复用了死会话)。"""
        return (self.is_running and self._spin_err is None
                and self._spin_thread is not None
                and self._spin_thread.is_alive())

    # ------------------------------------------------------------- 生命周期
    def start(self, need_depth: bool = True) -> dict:
        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.qos import qos_profile_sensor_data
            from sensor_msgs.msg import CameraInfo, Image
        except ImportError as e:
            raise DepthError(f"ROS2 环境不可用(rclpy/sensor_msgs 导入失败): {e}")
        try:
            from fd_msgs.msg import CameraDepthLease
            from fd_msgs.srv import SetCameraDepthLease
        except ImportError as e:
            raise DepthError(f"fd_msgs 不可用(需 source /opt/ros/humble 与 "
                             f"fd_ws 环境): {e}")

        if not rclpy.ok():
            rclpy.init()
        self._rclpy = rclpy
        node = Node(f"webapp_depth_{self.role}")
        self._node = node
        pre = f"/camera/camera_{self.role}"

        def stamp_sec(m) -> float:
            return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

        def on_color(m):
            img = np.frombuffer(m.data, dtype=np.uint8).reshape(
                m.height, m.width, 3)
            if m.encoding == "rgb8":
                img = img[:, :, ::-1].copy()          # → BGR
            with self._lock:
                self._color = img
                self._color_ts = time.time()
                self._color_stamp = stamp_sec(m)
                self._color_n += 1

        def on_depth(m):
            d = np.frombuffer(m.data, dtype=np.uint16).reshape(
                m.height, m.width)
            with self._lock:
                self._depth = d
                self._depth_ts = time.time()
                self._depth_stamp = stamp_sec(m)
                self._depth_n += 1

        def on_info(m):
            with self._lock:
                self._intr = {"fx": float(m.k[0]), "fy": float(m.k[4]),
                              "ppx": float(m.k[2]), "ppy": float(m.k[5]),
                              "width": m.width, "height": m.height}

        node.create_subscription(Image, f"{pre}/color/image_raw", on_color,
                                 qos_profile_sensor_data)
        node.create_subscription(Image,
                                 f"{pre}/aligned_depth_to_color/image_raw",
                                 on_depth, qos_profile_sensor_data)
        node.create_subscription(CameraInfo, f"{pre}/color/camera_info",
                                 on_info, qos_profile_sensor_data)

        # ---- 实体建全再起自旋;租约服务调用由自旋线程驱动 ----
        # (rclpy.spin_once/spin_until_future_complete 无 executor 参数时走
        #  进程级全局 executor,多线程并发调用会在 add/remove_node 上互踩,
        #  wait set 撞坏抛 IndexError: wait set index too big —— 扫描 503
        #  2026-10-21 事故根因,故一律用会话私有 executor + call_async 轮询)
        cli = None
        if need_depth:
            self._hb_pub = node.create_publisher(
                CameraDepthLease, "/fd_camera/depth_lease", 5)
            cli = node.create_client(SetCameraDepthLease,
                                     "/fd_camera/set_depth_lease")
        self._stop.clear()
        self._spin_thread = threading.Thread(target=self._spin_loop,
                                             daemon=True)
        self._spin_thread.start()
        if need_depth:
            # ---- 深度租约(先订阅后拿;hub 深度管线重启有间歇失败,重试 3 次) ----
            if not cli.wait_for_service(timeout_sec=5.0):
                self.stop()
                raise DepthError("fd_camera/set_depth_lease 服务不可用"
                                 "(fd_cameras_hub 未运行?)")
            last_msg = ""
            for _ in range(3):
                req = SetCameraDepthLease.Request()
                req.client_id = self.client_id
                req.enable = True
                req.roles = LEASE_BIT[self.role]
                fut = cli.call_async(req)
                t0 = time.time()
                while not fut.done() and time.time() - t0 < 5.0:
                    time.sleep(0.05)
                resp = fut.result() if fut.done() else None
                if resp is not None and resp.success:
                    self._lease_ok = True
                    break
                last_msg = resp.message if resp is not None else "超时"
                time.sleep(1.5)
            if not self._lease_ok:
                self.stop()
                raise DepthError(f"深度租约获取失败: {last_msg}")
            self._hb_thread = threading.Thread(target=self._heartbeat_loop,
                                               daemon=True)
            self._hb_thread.start()

        # 等首帧(hub 6~21fps;对齐深度在授租后要起管线,可能数秒)
        t0 = time.time()
        while time.time() - t0 < 10.0:
            with self._lock:
                if self._color is not None and (
                        not need_depth or self._depth is not None):
                    break
            time.sleep(0.2)
        self.is_running = True
        with self._lock:
            color_ok = self._color is not None
            depth_ok = self._depth is not None
            intr = dict(self._intr) if self._intr else None
        if not color_ok:
            self.stop()
            raise DepthError(f"相机角色 {self.role} 10s 内无彩色帧 —— "
                             f"标签可能又轮换了,请换角色重试")
        return {"role": self.role, "backend": "fd_hub",
                "color_ok": color_ok, "depth_ok": depth_ok,
                "depth_leased": need_depth,
                "intrinsics": intr, "waited_s": round(time.time() - t0, 1)}

    def _spin_loop(self):
        # 专用 executor,绝不碰模块级全局 executor(见 start 注释)
        from rclpy.executors import SingleThreadedExecutor
        exe = SingleThreadedExecutor()
        exe.add_node(self._node)
        try:
            while not self._stop.is_set():
                exe.spin_once(timeout_sec=0.1)
        except Exception as e:                        # noqa: BLE001
            self._spin_err = f"{type(e).__name__}: {e}"
        finally:
            try:
                exe.remove_node(self._node)
            except Exception:                         # noqa: BLE001
                pass

    def _heartbeat_loop(self):
        from fd_msgs.msg import CameraDepthLease
        while not self._stop.is_set():
            try:
                m = CameraDepthLease()
                m.client_id = self.client_id
                m.stamp = self._node.get_clock().now().to_msg()
                self._hb_pub.publish(m)
            except Exception:                         # noqa: BLE001
                pass
            self._stop.wait(0.7)                      # ~1.4Hz,>5s 超时余量足

    def stop(self):
        self._stop.set()
        # 先收线程再拆节点:自旋中 destroy_node 会把 wait set 从线程脚下
        # 抽走,偶发撞出 IndexError(老版本不 join 的竞态)
        for th in (self._spin_thread, self._hb_thread):
            if th is not None:
                th.join(timeout=2.0)
        # 释放租约(失败静默;心跳停 5s 后 hub 也会自行收回)
        try:
            from rclpy.executors import SingleThreadedExecutor
            from fd_msgs.srv import SetCameraDepthLease
            node = self._node
            if node is not None:
                cli = node.create_client(SetCameraDepthLease,
                                         "/fd_camera/set_depth_lease")
                if cli.wait_for_service(timeout_sec=2.0):
                    req = SetCameraDepthLease.Request()
                    req.client_id = self.client_id
                    req.enable = False
                    req.roles = LEASE_BIT[self.role]
                    fut = cli.call_async(req)
                    exe = SingleThreadedExecutor()
                    exe.add_node(node)
                    exe.spin_until_future_complete(fut, timeout_sec=3.0)
                    exe.remove_node(node)
        except Exception:                             # noqa: BLE001
            pass
        try:
            if self._node is not None:
                self._node.destroy_node()
        except Exception:                             # noqa: BLE001
            pass
        # rclpy.shutdown() 不做:进程可能还要再起会话
        self._node = None
        self._lease_ok = False
        self.is_running = False

    # ------------------------------------------------------------- 取数
    def grab_pair(self, timeout_s: float = 10.0) -> dict:
        """取一对彩色+对齐深度。返回 {color, depth, intr, stamp, synced}。

        对齐深度帧的 header.stamp 沿用对应彩色帧,静止取景下两者应严格相等;
        实测差 >10ms 视为不同步(退化为各取最新,标 synced=false)。
        """
        # 清空缓冲,确保拿到的是调用之后的 新 帧(而非上一次扫描的旧帧)
        with self._lock:
            self._color = None
            self._depth = None
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            with self._lock:
                c, d = self._color, self._depth
                cs, ds = self._color_stamp, self._depth_stamp
                intr = dict(self._intr) if self._intr else None
            if c is not None and d is not None:
                synced = abs(cs - ds) <= 0.010
                if synced:
                    return {"color": c, "depth": d, "intr": intr,
                            "stamp": cs, "synced": True}
                # 不同步但都有帧:再等一小会儿看能否凑齐同步对
                time.sleep(0.3)
                with self._lock:
                    c2, d2 = self._color, self._depth
                    cs2 = self._color_stamp
                if c2 is not None and d2 is not None:
                    return {"color": c2, "depth": d2, "intr": intr,
                            "stamp": cs2, "synced": False}
        # 超时:单独报告哪路缺帧
        with self._lock:
            c, d = self._color, self._depth
        missing = []
        if c is None:
            missing.append("彩色")
        if d is None:
            missing.append("对齐深度(检查租约/换角色)")
        raise DepthError(f"相机 {self.role} 取帧超时({timeout_s}s): "
                         f"缺 {'、'.join(missing) or '同步对'}")

    def grab_color(self, timeout_s: float = 5.0) -> dict:
        """取一帧**新**彩色帧(清缓冲,防测量用到旧帧)。返回
        {color, intr, stamp};超时抛 DepthError。免租约会话的主要取数口。"""
        with self._lock:
            self._color = None
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            with self._lock:
                c, cs, intr = self._color, self._color_stamp, self._intr
            if c is not None:
                return {"color": c, "intr": dict(intr) if intr else None,
                        "stamp": cs}
            time.sleep(0.05)
        raise DepthError(f"相机 {self.role} {timeout_s}s 内无新彩色帧"
                         "(标签轮换?换角色或查 hub)")

    def pixel_to_3d(self, u: float, v: float, radius: int = 14):
        """质心区域中值深度反投影,相机系坐标(米)。无效返回 None。

        区域中值抗包装反光空洞;中心点反投影(u,v 取整)。半径 14 是旧⑭
        实测值(bag_pick.py pixel_to_3d_region radius=14)。
        """
        with self._lock:
            d, intr = self._depth, self._intr
            fresh = d is not None and time.time() - self._depth_ts <= 3.0
        if not fresh or intr is None:
            return None
        u, v = int(round(u)), int(round(v))
        h, w = d.shape
        y0, y1 = max(0, v - radius), min(h, v + radius + 1)
        x0, x1 = max(0, u - radius), min(w, u + radius + 1)
        region = d[y0:y1, x0:x1]
        region = region[region > 0]
        if region.size < 5:
            return None
        z = float(np.median(region)) / 1000.0
        if not (0.1 < z < 3.0):
            return None
        return np.array([(u - intr["ppx"]) * z / intr["fx"],
                         (v - intr["ppy"]) * z / intr["fy"], z],
                        dtype=np.float64)

    def box_depth_3d(self, box_xyxy, min_px: int = 30):
        """检测框区域中值深度反投影(质心像素)。pixel_to_3d 失败时的兜底。"""
        with self._lock:
            d, intr = self._depth, self._intr
            fresh = d is not None and time.time() - self._depth_ts <= 3.0
        if not fresh or intr is None:
            return None
        h, w = d.shape
        x1, y1, x2, y2 = [int(round(max(0, v))) for v in box_xyxy]
        x2, y2 = min(w, x2), min(h, y2)
        if x2 - x1 < 2 or y2 - y1 < 2:
            return None
        vals = d[y1:y2, x1:x2].astype(np.float64)
        vals = vals[vals > 0]
        if vals.size < min_px:
            return None
        z = float(np.median(vals)) / 1000.0
        if not (0.1 < z < 3.0):
            return None
        uc, vc = (x1 + x2) / 2.0, (y1 + y2) / 2.0
        return np.array([(uc - intr["ppx"]) * z / intr["fx"],
                         (vc - intr["ppy"]) * z / intr["fy"], z],
                        dtype=np.float64)

    def box_face_points(self, box_xyxy, shrink=0.12, tol_mm=12.0,
                        min_frac=0.35, min_px=40):
        """检测框内**最近深度面**的 3D 点云质心(相机系,米)。

        为什么不用框质心像素+圆中值(pixel_to_3d):横向框质心 ≠ 真实
        可抓中心 —— 袋子歪斜/正反面外观不同/框吃进邻摞都会把质心横移。
        实测(2026-10-20↔10-22 两次校正的残差对比):Y(深度/吸嘴)分量
        92.14/92.23/91.55 稳定 ±0.4mm、Z −5.94/−6.53 稳定,**X 分量
        +1.94→−6.30 跳 8.2mm** —— 场景相关误差几乎全在横向质心,不在
        深度。取袋正面整个点云的 3D 质心,横向错位天然被消掉,深度也
        从 r=14 小圆升级成整面。
        做法:框内缩 shrink → 深度直方图(10mm 桶)取**最近强峰**(袋
        正面,≥max(15% 像素,30px) 的最低深度桶)→ 峰 ±tol_mm 内像素
        全部反投影 → 3D 质心。覆盖不足(遮挡/出框)返回 None,上层退
        回旧链路。→ {"p_cam", "z_mm", "n_pts", "frac"} 或 None。
        """
        with self._lock:
            d, intr = self._depth, self._intr
            fresh = d is not None and time.time() - self._depth_ts <= 3.0
        if not fresh or intr is None:
            return None
        h, w = d.shape
        x1, y1, x2, y2 = [int(round(max(0, v))) for v in box_xyxy]
        x2, y2 = min(w, x2), min(h, y2)
        bx, by = (x2 - x1) * shrink, (y2 - y1) * shrink
        x1, y1, x2, y2 = (int(round(x1 + bx)), int(round(y1 + by)),
                          int(round(x2 - bx)), int(round(y2 - by)))
        if x2 - x1 < 4 or y2 - y1 < 4:
            return None
        vals = d[y1:y2, x1:x2].astype(np.float64)
        mask = (vals > 100.0) & (vals < 3000.0)      # 有效深度(mm)
        if int(mask.sum()) < min_px:
            return None
        vv = vals[mask]
        # 最近强峰: 10mm 桶直方图,最低深度且样本足够的桶
        hist, edges = np.histogram(vv, bins=np.arange(0.0, 3000.0, 10.0))
        strong = np.where(hist >= max(0.15 * vv.size, 30.0))[0]
        if strong.size == 0:
            return None
        z_pk = (edges[strong[0]] + edges[strong[0] + 1]) / 2.0    # mm
        frac = float((np.abs(vv - z_pk) <= tol_mm).sum()) / float(vv.size)
        if frac < min_frac or float((np.abs(vv - z_pk) <= tol_mm).sum()) < min_px:
            return None
        # 框内逐像素反投影(只算选中簇),质心即面中心
        ys, xs = np.nonzero(mask)
        zs = vals[ys, xs]
        keep = np.abs(zs - z_pk) <= tol_mm
        xs, ys, zs = xs[keep], ys[keep], zs[keep] / 1000.0
        p_cam = np.array([
            float(np.mean((xs + x1 + 0.5 - intr["ppx"]) * zs / intr["fx"])),
            float(np.mean((ys + y1 + 0.5 - intr["ppy"]) * zs / intr["fy"])),
            float(np.mean(zs))], dtype=np.float64)
        return {"p_cam": p_cam, "z_mm": round(float(np.mean(zs)) * 1000, 1),
                "n_pts": int(keep.sum()), "frac": round(frac, 2)}

    def health(self) -> dict:
        now = time.time()
        with self._lock:
            color_age = (now - self._color_ts) if self._color_ts else None
            depth_age = (now - self._depth_ts) if self._depth_ts else None
            return {
                "role": self.role,
                "running": self.is_running,
                "lease": self._lease_ok,
                "color_frames": self._color_n,
                "depth_frames": self._depth_n,
                "color_age_s": round(color_age, 2) if color_age else None,
                "depth_age_s": round(depth_age, 2) if depth_age else None,
                "intrinsics": bool(self._intr),
                "spin_err": self._spin_err,
            }


class DepthProvider:
    """进程级单例:当前角色的会话。app.py 持有,接口全部线程安全。"""

    def __init__(self):
        self._session: Optional[DepthSession] = None
        self._guard = threading.Lock()

    def start_session(self, role: str, need_depth: bool = True) -> dict:
        with self._guard:
            cur = self._session
            if cur is not None and cur.is_running:
                if cur.role == role and cur.alive:
                    return {"already_running": True, **cur.health()}
                cur.stop()                            # 换角色/自旋已死:释放重建
            ensure_ros_env()
            s = DepthSession(role)
            info = s.start(need_depth=need_depth)
            self._session = s
            return info

    @property
    def session(self) -> Optional[DepthSession]:
        s = self._session
        return s if s is not None and s.alive else None

    def health(self) -> dict:
        s = self.session
        if s is None:
            return {"role": None, "running": False}
        return s.health()

    def shutdown(self):
        with self._guard:
            if self._session is not None:
                self._session.stop()
                self._session = None
