#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""物料位姿检测服务(卫龙魔芋爽 pose 模型) — 供上位机软件调用
================================================================
功能: 检测画面中物料的 正面/反面(cls) + 4 脚点关键姿 + 旋转角度,
      并给出整姿建议(翻面/调转/就绪)。

模型: YOLO11n-pose imgsz=896, 两类 front(正面)/back(反面), kpt_shape [4,2]
      脚点顺序: 1、2 = 袋子物理头部两角, 3、4 = 尾部两角。
来源: 旧工程 old/tianlian/weilong/infer_weilong_pose.py(2026-10-05 腕相机域微调)。

为什么是独立进程: 模型依赖 Jetson 版 torch + ultralytics, 只存在于
yolo11_env(Python 3.10 venv); 上位机 GUI 是 conda upper_computer(3.11,
PySide6), 两边解释器不可互换。本服务由 GUI 作为子进程拉起, HTTP 通信。

接口(默认 127.0.0.1:7100):
  GET  /health           → {"ok":true,"model":...,"device":...,"imgsz":896}
  POST /detect?conf=0.25 → body=图片字节(JPEG/PNG)
                           → {"ok":true,"latency_ms":...,"detections":[...]}
    detection 字段: conf, cls(front/back), cls_prob, box[xyxy], kpts[[x,y]×4],
      centroid[u,v], angle_deg(minAreaRect 主轴), action(flip/rotate/ok/none),
      head_up, head_dev_deg(头部与图像上方夹角,顺时针为正), rotate_deg(调转角)

启动(手动调试):
    LD_LIBRARY_PATH=<yolo11_env>/lib/python3.10/site-packages/nvidia/cusparselt/lib:... \
    <yolo11_env>/bin/python pose_service.py
"""

import argparse
import base64
import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import torch
from ultralytics import YOLO

CLS_NAMES = ["front", "back"]
UP_TOL_DEG = 30.0          # 头部方向与图像上方夹角阈值, 小于视为"朝上"


class Detector:
    """模型封装: 单例加载, 线程锁串行推理(CUDA 模型非线程安全)"""

    def __init__(self, weights, device="cuda:0", conf=0.25, imgsz=896):
        self.model = YOLO(weights)
        self.device = device if torch.cuda.is_available() else "cpu"
        self.conf, self.imgsz = conf, imgsz
        self.lock = threading.Lock()
        self.weights = weights
        self.warmup_ms = 0.0
        t0 = time.time()
        with self.lock:                       # 预热: 首次推理含 CUDA 上下文创建
            self.model.predict(np.zeros((480, 640, 3), np.uint8),
                               imgsz=imgsz, device=self.device,
                               conf=conf, verbose=False)
        self.warmup_ms = (time.time() - t0) * 1000

    def detect(self, bgr, conf=None):
        conf = conf or self.conf
        with self.lock:
            res = self.model.predict(bgr, imgsz=self.imgsz, device=self.device,
                                     conf=conf, verbose=False)[0]
        out = []
        for i in range(len(res.boxes)):
            box = res.boxes.xyxy[i].cpu().numpy()
            ci = int(res.boxes.cls[i])
            kpts = (res.keypoints.xy[i].cpu().numpy()
                    if res.keypoints is not None else None)
            item = {"conf": float(res.boxes.conf[i]),
                    "cls": CLS_NAMES[ci] if ci < len(CLS_NAMES) else str(ci),
                    "cls_prob": None,
                    "box": [float(v) for v in box],
                    "kpts": [[float(x), float(y)] for x, y in kpts]
                            if kpts is not None else []}
            if kpts is not None and len(kpts) >= 3:
                (cx, cy), _wh, a = cv2.minAreaRect(
                    np.asarray(kpts, np.float32))
                item["centroid"] = [float(cx), float(cy)]
                item["angle_deg"] = float(a)
            else:
                item["centroid"] = None
                item["angle_deg"] = None
            item.update(orient_action(item))
            out.append(item)
        return out


def head_dev_deg(item):
    """头部(1-2中点)→尾部(3-4中点)方向与图像上方的夹角(顺时针为正)"""
    k = item["kpts"]
    if len(k) != 4:
        return None, None
    m12 = ((k[0][0] + k[1][0]) / 2, (k[0][1] + k[1][1]) / 2)
    m34 = ((k[2][0] + k[3][0]) / 2, (k[2][1] + k[3][1]) / 2)
    dx, dy = m12[0] - m34[0], m12[1] - m34[1]        # 图像 y 向下
    if abs(dx) < 1e-6 and abs(dy) < 1e-6:
        return None, None
    dev = math.degrees(math.atan2(dx, -dy))
    return dev, abs(dev) <= UP_TOL_DEG


def orient_action(item, up_tol=UP_TOL_DEG):
    """整姿决策: 反面→翻面; 正面但头部不朝上→调转; 正面且朝上→就绪"""
    dev, head_up = head_dev_deg(item)
    if item["cls"] == "back":
        return {"action": "flip", "head_up": head_up,
                "head_dev_deg": dev, "rotate_deg": None}
    if dev is None:
        return {"action": "none", "head_up": None,
                "head_dev_deg": None, "rotate_deg": None}
    head_up = abs(dev) <= up_tol
    return {"action": "ok" if head_up else "rotate", "head_up": head_up,
            "head_dev_deg": dev, "rotate_deg": 0.0 if head_up else -dev}


# 标注配色与 GUI PoseOverlayView 一致: 绿=front / 红=back / 黄=头尾轴线与建议
_CLR_FRONT = (111, 207, 62)      # #3ecf6f (BGR)
_CLR_BACK = (74, 90, 224)        # #e05a4a
_CLR_HINT = (90, 211, 255)       # #ffd35a

_ACTION_TEXT = {"flip": "FLIP", "rotate": "ROTATE", "ok": "READY",
                "none": "NO-KPTS"}


def _dashed_line(img, p1, p2, color, thickness=2, on=8, off=6):
    """cv2 无虚线,按段画。"""
    import numpy as _np
    p1, p2 = _np.array(p1, float), _np.array(p2, float)
    total = _np.linalg.norm(p2 - p1)
    if total < 1e-3:
        return
    d = (p2 - p1) / total
    t = 0.0
    while t < total:
        a = p1 + d * t
        b = p1 + d * min(t + on, total)
        cv2.line(img, (int(a[0]), int(a[1])), (int(b[0]), int(b[1])),
                 color, thickness, cv2.LINE_AA)
        t += on + off


def draw_detections(img, detections, numbers=False):
    """在原图副本上画: 检测框、4 脚点(1、2=头部)、头尾虚线轴线、类别+建议标签。
    cv2 Hershey 字体只有 ASCII,故标签用英文(GUI 端 Qt 画的是中文)。
    numbers=True 时再画检测序号 #n(与响应 detections 列表顺序一致,供吸取选袋)。"""
    vis = img.copy()
    for n, d in enumerate(detections, 1):
        color = _CLR_FRONT if d.get("cls") == "front" else _CLR_BACK
        x1, y1, x2, y2 = [int(v) for v in d["box"]]
        cv2.rectangle(vis, (x1, y1), (x2, y2), color, 2, cv2.LINE_AA)
        if numbers:
            cv2.putText(vis, f"#{n}", (x1 + 4, y1 + 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        _CLR_HINT, 2, cv2.LINE_AA)
        # 4 脚点 + 编号(1、2 = 袋子头部两角)
        for i, (kx, ky) in enumerate(d.get("kpts") or []):
            c = (int(kx), int(ky))
            cv2.circle(vis, c, 4, color, -1, cv2.LINE_AA)
            cv2.putText(vis, str(i + 1), (c[0] + 6, c[1] - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)
        # 头尾轴线: 1-2 中点 → 3-4 中点
        kpts = d.get("kpts") or []
        if len(kpts) == 4:
            m12 = ((kpts[0][0] + kpts[1][0]) / 2, (kpts[0][1] + kpts[1][1]) / 2)
            m34 = ((kpts[2][0] + kpts[3][0]) / 2, (kpts[2][1] + kpts[3][1]) / 2)
            _dashed_line(vis, m12, m34, _CLR_HINT, 2)
        # 标签两行: 类别+置信度 / 建议|头部偏角
        act = _ACTION_TEXT.get(d.get("action"), "?")
        rd = d.get("rotate_deg")
        line2 = act + (f" {rd:+.0f}deg" if rd is not None else "")
        hd = d.get("head_dev_deg")
        if hd is not None:
            line2 += f" | dev {hd:+.1f}deg"
        line1 = f"{str(d.get('cls')).upper()} {d.get('conf', 0):.2f}"
        ty = y1 - 40 if y1 >= 46 else y2 + 6     # 框顶放不下就放框内下沿
        for i, txt in enumerate((line1, line2)):
            cv2.putText(vis, txt, (x1, ty + i * 17),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        color if i == 0 else _CLR_HINT, 1, cv2.LINE_AA)
    return vis


class Handler(BaseHTTPRequestHandler):
    detector: Detector = None          # 由 main() 注入

    def log_message(self, fmt, *args):  # 静默默认访问日志
        pass

    def _send(self, code, obj):
        body = json.dumps(obj, ensure_ascii=False).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path.startswith("/health"):
            d = self.detector
            self._send(200, {"ok": True, "model": d.weights,
                             "device": d.device, "imgsz": d.imgsz,
                             "classes": CLS_NAMES,
                             "warmup_ms": round(d.warmup_ms, 1)})
        else:
            self._send(200, {"ok": True, "service": "pose",
                             "endpoints": ["/health", "/detect"]})

    def do_POST(self):
        if not self.path.startswith("/detect"):
            self._send(404, {"ok": False, "error": "not found"})
            return
        conf = 0.25
        if "?" in self.path:
            for kv in self.path.split("?", 1)[1].split("&"):
                if kv.startswith("conf="):
                    try:
                        conf = float(kv.split("=", 1)[1])
                    except ValueError:
                        pass
        n = int(self.headers.get("Content-Length") or 0)
        data = self.rfile.read(n) if n else b""
        img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            self._send(400, {"ok": False,
                             "error": f"body 不是有效图片({len(data)}字节)"})
            return
        t0 = time.time()
        try:
            dets = self.detector.detect(img, conf=conf)
        except Exception as e:                       # noqa: BLE001
            self._send(500, {"ok": False, "error": f"推理失败: {e}"})
            return
        # 标注帧(base64 JPEG): webapp 前端直接 <img> 显示;GUI 端自己画,忽略即可
        image_b64 = None
        try:
            ok, buf = cv2.imencode(".jpg", draw_detections(img, dets, numbers=True),
                                   [cv2.IMWRITE_JPEG_QUALITY, 88])
            if ok:
                image_b64 = base64.b64encode(buf.tobytes()).decode()
        except Exception:                             # noqa: BLE001
            pass                                      # 标注失败不影响检测结果
        self._send(200, {"ok": True,
                         "latency_ms": round((time.time() - t0) * 1000, 1),
                         "width": int(img.shape[1]), "height": int(img.shape[0]),
                         "detections": dets, "image_b64": image_b64})


def _default_weights():
    """权重三级查找: 环境变量 → webapp 本地 models/ → upper_computer 源路径。
    webapp 要求整目录可拷贝复用,正常应走前两级;第三级只是兜底并在日志提示。"""
    import os
    here = os.path.dirname(os.path.abspath(__file__))
    cands = [
        os.environ.get("POSE_WEIGHTS", ""),
        os.path.join(here, "models", "weilong_pose_best.pt"),
        "/home/user/tianlian/upper_computer/models/weilong_pose_best.pt",
    ]
    for p in cands:
        if p and os.path.isfile(p):
            return p
    return cands[1]      # 都不存在时返回本地路径,让上层报出可读的错


def _train_imgsz(weights):
    """从 ckpt train_args 读训练分辨率; 读不到按旧模型默认 896。
    新模型(2026-10-22 现场 640 微调)若仍用 896 推理会静默偏移训练域。"""
    try:
        ck = torch.load(weights, map_location="cpu", weights_only=False)
        return int((ck.get("train_args") or {}).get("imgsz") or 896)
    except Exception:                                       # noqa: BLE001
        return 896


def main():
    import os
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", default=_default_weights())
    ap.add_argument("--port", type=int, default=7100)
    ap.add_argument("--device", default="cuda:0")
    ap.add_argument("--imgsz", type=int, default=0)     # 0=跟 ckpt 训练分辨率
    args = ap.parse_args()

    imgsz = args.imgsz or _train_imgsz(args.weights)
    print(f"[pose_service] 加载模型 {args.weights} (device={args.device} "
          f"imgsz={imgsz}) ...", flush=True)
    Handler.detector = Detector(args.weights, device=args.device, imgsz=imgsz)
    srv = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    print(f"POSE_SERVICE_READY port={args.port} "
          f"device={Handler.detector.device} "
          f"warmup={Handler.detector.warmup_ms:.0f}ms", flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        srv.server_close()


if __name__ == "__main__":
    main()
