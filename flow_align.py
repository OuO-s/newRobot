# -*- coding: utf-8 -*-
"""对准闸门的视觉测量: 码检测 → PnP → 基座系位置。
================================================================
从 pbvs/tianlian_pbvs/core/marker_detector.py 移植(2026-10-20 实测版):
  - cv2 4.5.4 无 ArUcoDetector 类(4.7+ API) → legacy detectMarkers;
  - QUIRC 未链接,QR 解码需要 pyzbar(webapp 进程没装 → 导入期自动尾部
    追加 upper_computer env 的 site-packages 复用);
  - IPPE_SQUARE 物点顺序 [(-h,+h),(+h,+h),(+h,-h),(-h,-h)] 与图像点
    TL,TR,BR,BL 配对(厂商原版 x/y 转置 bug 已修,见源仓注释)。

坐标链(与 pick_flow 同一约定):
  p_base = T_base_ee(get_end_poses 上报系) @ T_ee_camera(手眼) @ p_cam
对准用的只有 Δ = p_base_now − p_base_ref —— 同码同距离下 PnP 的
系统比例误差(marker_size 偏差)在 Δ 中抵消。
"""
from __future__ import annotations

import cv2
import numpy as np

try:
    from pyzbar.pyzbar import decode as _zbar_decode
    HAS_PYZBAR = True
except ImportError:
    # webapp 进程(系统 3.10)没装 pyzbar;upper_computer env 里有(纯 Python
    # 包, libzbar.so.0 系统已带)→ sys.path 尾部追加复用(与 camera_bridge
    # 等检测脚本同一手法), 追加在尾部不会遮蔽系统 numpy/cv2
    import sys
    sys.path.append(
        "/home/user/miniconda3/envs/upper_computer/lib/python3.11/site-packages")
    try:
        from pyzbar.pyzbar import decode as _zbar_decode
        HAS_PYZBAR = True
    except ImportError:
        _zbar_decode = None
        HAS_PYZBAR = False


# ===========================================================================
# 检测(彩色码预处理一并保留: 现场红/金码贴在铁皮上时 B/S 通道兜底)
# ===========================================================================
def _pre_blue_otsu(bgr):
    """红/金码 → 黑, 白/银/灰底 → 白(B 通道: 有色码 B 低,金属/白 B 高)"""
    _, t = cv2.threshold(bgr[:, :, 0], 0, 255,
                         cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    return t


def _pre_saturation_otsu(bgr):
    s = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)[:, :, 1]
    _, t = cv2.threshold(s, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    return 255 - t


_PREPROCESSORS = (
    ("raw", lambda im: im),
    ("B", lambda im: _pre_blue_otsu(im)),
    ("S", lambda im: _pre_saturation_otsu(im)),
)


def _to_bgr(img):
    if img.ndim == 2:
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    return img


def _order_corners_quad(pts):
    """4 角点 → 稳定次序 [左上, 右上, 右下, 左下](与 PnP 物点配对用)。"""
    p = np.asarray(pts, dtype=np.float32).reshape(4, 2)
    c = p.mean(axis=0)
    ang = np.arctan2(p[:, 1] - c[1], p[:, 0] - c[0])
    p = p[np.argsort(ang)]
    start = int(np.argmin(p.sum(axis=1)))          # x+y 最小 = 左上
    return np.roll(p, -start, axis=0)


class MarkerFinder:
    """ArUco(4x4_50, 与现役夹具码同字典) + 可选 QR;按身份挑目标。"""

    def __init__(self, aruco_dict=cv2.aruco.DICT_4X4_50):
        self._dict = cv2.aruco.getPredefinedDictionary(aruco_dict)
        self._subpix = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                        30, 0.05)

    def find(self, bgr: np.ndarray, marker: dict) -> dict | None:
        """按配置挑一个码。marker={"type":"aruco","id":N} 或
        {"type":"qr","data":"TL-xxx"}。返回
        {key, corners[(u,v)×4 TL,TR,BR,BL], center} 或 None。"""
        if bgr is None:
            return None
        if marker.get("type") == "qr":
            if not HAS_PYZBAR:
                raise ValueError("QR 解码库 pyzbar 不可用(upper_computer env "
                                 "也找不到)——改用 ArUco,或先装 pyzbar 再"
                                 "重启 webapp")
            return self._find_qr(bgr, marker.get("data", ""))
        return self._find_aruco(bgr, int(marker.get("id", -1)))

    # ------------------------------------------------------------- ArUco
    def _find_aruco(self, bgr, want_id):
        for name, pre in _PREPROCESSORS:
            img = _to_bgr(pre(bgr))
            corners, ids, _ = cv2.aruco.detectMarkers(img, self._dict)
            if ids is None:
                continue
            for cs, mid in zip(corners, ids):
                if int(mid[0]) != want_id:
                    continue
                pts = np.asarray(cs[0], dtype=np.float32)
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                pts = cv2.cornerSubPix(gray, pts, (5, 5), (-1, -1),
                                       self._subpix)
                pts = _order_corners_quad(pts)
                return {"key": f"aruco:{want_id}",
                        "corners": [(float(x), float(y)) for x, y in pts],
                        "center": (float(pts[:, 0].mean()),
                                   float(pts[:, 1].mean())),
                        "channel": name}
        return None

    # ------------------------------------------------------------- QR
    def _find_qr(self, bgr, want_data):
        if not HAS_PYZBAR:
            return None
        for name, pre in _PREPROCESSORS:
            res = _zbar_decode(_to_bgr(pre(bgr)))
            for r in res:
                if r.type != "QRCODE" or not r.polygon or len(r.polygon) != 4:
                    continue
                data = r.data.decode("utf-8", "replace")
                if want_data and data != want_data:
                    continue
                pts = _order_corners_quad([(p.x, p.y) for p in r.polygon])
                return {"key": f"qr:{data[:32]}",
                        "corners": [(float(x), float(y)) for x, y in pts],
                        "center": (float(pts[:, 0].mean()),
                                   float(pts[:, 1].mean())),
                        "channel": name}
        return None

    # ------------------------------------------------------------- PnP
    @staticmethod
    def solve_center(corners, intr: dict, marker_size_m: float):
        """IPPE_SQUARE PnP → 码中心相机系 3D(米)。intr 需 fx/fy/ppx/ppy。"""
        if not intr or not marker_size_m or marker_size_m <= 0:
            return None
        K = np.array([[intr["fx"], 0, intr["ppx"]],
                      [0, intr["fy"], intr["ppy"]],
                      [0, 0, 1]], dtype=np.float64)
        half = float(marker_size_m) / 2.0
        # IPPE_SQUARE 物点硬约定(Y 轴向上、左上角起): 与 TL,TR,BR,BL 配对
        obj = np.array([[-half, half, 0], [half, half, 0],
                        [half, -half, 0], [-half, -half, 0]],
                       dtype=np.float64)
        ok, _, tvec = cv2.solvePnP(obj, np.asarray(corners, np.float64),
                                   K, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            return None
        pos = tvec.ravel().astype(np.float64)
        return pos if 0.05 < pos[2] < 4.0 else None


def measure_marker_base(session, finder: MarkerFinder, marker: dict,
                        marker_size_m: float, T_base_ee, T_ee_cam,
                        frames: int = 3, timeout_s: float = 6.0) -> dict:
    """取 frames 帧新彩色帧,逐帧检测+PnP,中值得码中心基座系位置。

    返回 {p_base, p_cam_median, z_cam_mm, frames_used, key, corner_px,
    spread_mm}; 码找不到/深度不合理抛 ValueError(消息面向用户)。
    保险一: 单批解码不足 3 帧时补采合并到 ≥3 帧 —— 小码@0.4m 远距常
    只解出 1~2 帧,此时 spread 无意义(单帧恒 0),单帧 PnP z 抖 ±10mm
    会直灌闸门把容差 4mm 的判据打飞(2026-10-22 16:56 右臂 z_cam 3 秒
    跳 ±12mm 即此)。测码时臂是静止的,跨批合并安全。
    保险二: 帧间离散(spread,相机系 mm)超阈时再采一批、留更稳的一批 ——
    小码在 0.4m 外 PnP 的 z 抖动可达 ±4mm,不能让单批抖动直接进
    闸门视差判据(z 容差才 3~4mm)。
    """
    def _batch():
        pts, last_err = [], "无帧"
        for _ in range(max(1, frames)):
            pair = session.grab_color(timeout_s=timeout_s)
            found = finder.find(pair["color"], marker)
            if not found:
                last_err = "画面中未找到指定码"
                continue
            p_cam = MarkerFinder.solve_center(found["corners"],
                                              pair.get("intr"),
                                              marker_size_m)
            if p_cam is None:
                last_err = "PnP 无解或 Z 异常(码尺寸/内参?)"
                continue
            T = T_base_ee @ T_ee_cam
            p_base = (T @ np.array([p_cam[0], p_cam[1], p_cam[2], 1.0]))[:3]
            pts.append((p_cam, p_base, found))
        return pts, last_err

    def _spread(pts):
        arr = np.array([p[0] for p in pts])
        return (np.max(arr, axis=0) - np.min(arr, axis=0)) * 1000.0

    # 保险一: 凑帧数 —— 解码帧 <3 时补采(最多两批)合并,凑不够也照常
    # 返回,frames_used 如实反映(闸门日志可见)
    min_pts = min(3, max(1, frames))
    pts, last_err = _batch()
    for _ in range(2):
        if len(pts) >= min_pts:
            break
        pts2, e2 = _batch()
        pts += pts2
        if pts2 or e2 != "无帧":
            last_err = e2
    if not pts:
        raise ValueError(f"对准测量失败: {last_err}(marker={marker})")
    spread = _spread(pts)
    if len(pts) >= 3 and (spread[2] > 6.0
                          or max(spread[0], spread[1]) > 4.0):
        pts2, _ = _batch()                 # 抖动大:再采一批,留更稳的
        if pts2 and len(pts2) >= min_pts:
            # 只换不并: 点少的批 spread 天然小,拿 1~2 帧换掉 ≥3 帧
            # 会把凑帧数的努力抵消掉
            s2 = _spread(pts2)
            if float(np.sum(s2)) < float(np.sum(spread)):
                pts, spread = pts2, s2
        elif pts2:
            pts += pts2                    # 不够 min_pts: 并入,中值更稳
            spread = _spread(pts)
    pcs = np.median(np.array([p[0] for p in pts]), axis=0)
    pbs = np.median(np.array([p[1] for p in pts]), axis=0)
    found = pts[-1][2]
    return {"p_base": [round(float(v), 5) for v in pbs],
            "p_cam": [round(float(v), 5) for v in pcs],
            "z_cam_mm": round(float(pcs[2]) * 1000.0, 1),
            "frames_used": len(pts), "key": found["key"],
            "spread_mm": [round(float(v), 1) for v in spread],
            "corner_px": found["corners"]}
