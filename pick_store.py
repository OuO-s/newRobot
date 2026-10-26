# -*- coding: utf-8 -*-
"""一步到位(吸取)的持久化: 配置 / 手眼矩阵 / 最近一次扫描。
================================================================
目录 pick_data/(与 webapp 同级,整目录可拷):
  pick_config.json            运行参数 + 示教(扫描位/对准校正)
  handeye_result_tianlian_left.yaml   手眼标定拷贝(2026-10-20 出)
  last_scan.json              最近一次扫描结果(干跑免重新检测)

手眼约定(与 hand_in_eye/scripts/3_verify_handeye.py 一致):
  T_base_board = T_base_ee @ X @ T_cam_board
  → p_base = T_base_ee(get_end_poses, 上报系) @ T_ee_camera @ p_cam
"""
from __future__ import annotations

import json
import os
import tempfile
import time
from pathlib import Path

import numpy as np
import yaml

PICK_DIR = Path(__file__).resolve().parent / "pick_data"
CONFIG_PATH = PICK_DIR / "pick_config.json"
LAST_SCAN_PATH = PICK_DIR / "last_scan.json"
HANDEYE_PATH = PICK_DIR / "handeye_result_tianlian_left.yaml"
CALIBRATED_ROLE = "left"        # 手眼标定用的相机角色,换角色要告警

DEFAULTS: dict = {
    "arm": "armleft",
    "cam_role": "left",
    "conf": 0.5,
    "hover_mm": 6.0,             # 终点 = 袋面 + hover
    "motion_mode": "direct",     # direct=扫描位→袋心一段直线 / corridor=分段走廊
    "cruise_clearance_mm": 40.0, # 越顶走廊 = 最高袋面 + clearance
    "press_mm": 0.0,             # 到位后下压(0..10,首跑默认 0)
    "max_press_mm": 10.0,
    "hold_s": 2.0,               # 到位保持(外部接真空窗口)
    "ghost_min_z_mm": 200.0,     # 近距鬼影(检测到自家夹爪)拒收线
    "layer_mm": 25.0,            # 深度分层阈值
    "max_horiz_m": 0.25,         # 水平行程上限
    "max_dz_m": 0.25,            # |袋z − 对准基准z| 上限
    "xy_res_tol_mm": 8.0,        # 段后 FK 验证容差
    "z_res_tol_mm": 5.0,
    "speed": 15,                 # 吸取全程速度%(首跑建议 8)
    "settle_s": 1.2,             # 回放扫描位后等待机身稳定
    "move_timeout_s": 60.0,
    "vacuum": {"mode": "manual",         # manual=外部触发 / gripper=联动夹爪
               "gripper_position": 0,
               "gripper_velocity": 500}, # 0 会静默不动,必须非零
    "residual_mm": [0.0, 0.0, 0.0],      # 对准校正全局残差
    "per_bag_residual_mm": {},           # {"1": [dx,dy,dz]} 分袋残差(旧 suct_adj)
    "scan_pose": None,          # {"joints_deg":[..], "end_pose":{..}, "created":}
    "align": None,              # {"rpy_deg":[..], "z_ref_m":f, "created":}
}

# 数值型参数的取值范围(POST config 时钳位)
_CLAMP = {
    "conf": (0.05, 0.9),
    "hover_mm": (0.0, 50.0),
    "cruise_clearance_mm": (10.0, 150.0),
    "press_mm": (0.0, 10.0),
    "hold_s": (0.0, 30.0),
    "ghost_min_z_mm": (50.0, 1000.0),
    "layer_mm": (5.0, 100.0),
    "max_horiz_m": (0.05, 0.5),
    "max_dz_m": (0.05, 0.5),
    "xy_res_tol_mm": (2.0, 50.0),
    "z_res_tol_mm": (2.0, 50.0),
    "speed": (1, 100),
    "settle_s": (0.0, 10.0),
    "move_timeout_s": (10.0, 300.0),
}


def load_config() -> dict:
    cfg = dict(DEFAULTS)
    if CONFIG_PATH.is_file():
        try:
            data = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
            if isinstance(data, dict):
                cfg.update(data)
        except Exception:                             # noqa: BLE001
            pass                                     # 损坏则回默认
    return cfg


def save_config(cfg: dict) -> None:
    PICK_DIR.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=str(PICK_DIR), suffix=".tmp")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            json.dump(cfg, f, ensure_ascii=False, indent=2)
        os.replace(tmp, CONFIG_PATH)
    except Exception:                                 # noqa: BLE001
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise


def update_config(patch: dict) -> dict:
    """合并补丁并钳位数值;未知键抛 ValueError。返回更新后的完整配置。"""
    cfg = load_config()
    bad = [k for k in patch if k not in DEFAULTS]
    if bad:
        raise ValueError(f"未知配置项: {bad}")
    for k, v in patch.items():
        if k in _CLAMP and isinstance(v, (int, float)):
            lo, hi = _CLAMP[k]
            v = float(min(max(float(v), lo), hi))
            if k in ("speed",):
                v = int(v)
        if k == "cam_role" and v not in ("left", "head", "right"):
            raise ValueError("cam_role 只能是 left/head/right")
        if k == "motion_mode" and v not in ("direct", "corridor"):
            raise ValueError("motion_mode 只能是 direct/corridor")
        if k == "vacuum":
            v = {"mode": str(v.get("mode", "manual")),
                 "gripper_position": float(v.get("gripper_position", 0)),
                 "gripper_velocity": max(
                     500, float(v.get("gripper_velocity", 500)))}
        cfg[k] = v
    save_config(cfg)
    return cfg


def load_handeye_T() -> np.ndarray:
    """读 T_ee_camera(4x4)。缺文件/坏文件抛 FileNotFoundError/ValueError。"""
    if not HANDEYE_PATH.is_file():
        raise FileNotFoundError(f"手眼标定文件不存在: {HANDEYE_PATH}")
    data = yaml.safe_load(HANDEYE_PATH.read_text(encoding="utf-8"))
    T = np.asarray(data.get("T_ee_camera"), dtype=float)
    if T.shape != (4, 4) or not np.isfinite(T).all():
        raise ValueError("T_ee_camera 不是合法 4x4 矩阵")
    return T


def handeye_info() -> dict:
    if not HANDEYE_PATH.is_file():
        return {"ok": False, "path": str(HANDEYE_PATH)}
    try:
        data = yaml.safe_load(HANDEYE_PATH.read_text(encoding="utf-8"))
        rs = data.get("residual_stats") or {}
        return {"ok": True, "path": str(HANDEYE_PATH),
                "time": data.get("time"),
                "method": (rs.get("method_used")
                           or ",".join((data.get("methods") or {}).keys())),
                "pos_std_m": rs.get("pos_std_m"),
                "calibrated_role": CALIBRATED_ROLE}
    except Exception as e:                            # noqa: BLE001
        return {"ok": False, "error": str(e)}


def save_last_scan(payload: dict) -> None:
    PICK_DIR.mkdir(parents=True, exist_ok=True)
    payload = dict(payload)
    payload["saved_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
    LAST_SCAN_PATH.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")


def load_last_scan() -> dict | None:
    try:
        return json.loads(LAST_SCAN_PATH.read_text(encoding="utf-8"))
    except Exception:                                 # noqa: BLE001
        return None
