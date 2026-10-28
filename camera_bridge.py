#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""相机 ROS2 话题 → 本地 MJPEG/快照桥接服务。

上位机 GUI(conda 3.11)无法导入 rclpy,本进程改用系统 Python 3.10 +
ROS2 Humble 订阅机器人三路彩色话题,缓存最新帧并转发为本地 HTTP:

  GET /health                 {ok, cams:{left:{fps,w,h,frames},...}}
  GET /snapshot?cam=left      最新一帧 JPEG(暂无帧时 503)
  GET /stream?cam=left&fps=15 multipart/x-mixed-replace MJPEG 流(丢帧跟随)

GUI 以 bash -c "source /opt/ros/humble/setup.bash && exec python3 camera_bridge.py"
方式拉起(app.py MainWindow.ensure_camera_bridge),退出时回收。
手动调试:source /opt/ros/humble/setup.bash && python3 camera_bridge.py
"""
import argparse
import json
import os
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
    from sensor_msgs.msg import Image
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
        self._create_subs()
        # 看门狗在 executor 线程内跑,销毁/重建订阅(图变更)是安全的
        self.create_timer(WD_CHECK_S, self._watchdog)

    def _create_subs(self) -> None:
        for cam in self._cam_names:
            self._subs[cam] = self.create_subscription(
                Image, f"/camera/camera_{cam}/color/image_raw",
                (lambda m, c=cam: self._on_image(m, c)),
                qos_profile_sensor_data)

    def _relaunch_detached(self) -> None:
        """用全新进程重启自己(升级动作)。

        曾用 os.execv:不换 PID,FastDDS 按进程号派的发现端口也不变,
        2026-10-26 实证 exec 57 次全部无效(相机热插拔后发布端重建类
        失配,同 PID 的新 participant 依然发现不到发布端);而外部
        kill+重拉(新 PID)秒恢复。故改为:派一个 detached 子进程,
        等本进程死后以全新 PID 拉起本脚本,然后 SIGKILL 自己。
        子进程继承本进程的 stdout(即 camera_bridge.log)。"""
        cmd = (f"sleep 1.5; exec {sys.executable} "
               f"{os.path.abspath(__file__)} {' '.join(sys.argv[1:])}")
        subprocess.Popen(["/bin/bash", "-c", cmd], start_new_session=True)
        os.kill(os.getpid(), signal.SIGKILL)

    def _watchdog(self) -> None:
        if time.monotonic() - self._t0 < WD_GRACE_S:
            return
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
