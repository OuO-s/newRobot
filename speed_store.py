# -*- coding: utf-8 -*-
"""全局速度(双臂分别)持久化: 所有臂运动的统一速度来源。
================================================================
目录 speed_data/(与 webapp 同级,整目录可拷):
  global_speed.json     {"armleft": 15, "armright": 15}

页面顶栏是唯一入口; 点动/笛卡尔/示教回放/点位直达/一步到位吸取/
双臂流水线都在**下发时**取本值 —— 改了立即生效(下一拍/下一段即用
新值),正在跑的任务无需重启。请求体显式给 speed 仍优先于全局值。
"""
from __future__ import annotations

import json
import os
import tempfile
import threading
from pathlib import Path

SPEED_DIR = Path(__file__).resolve().parent / "speed_data"
CONFIG_PATH = SPEED_DIR / "global_speed.json"

DEFAULTS = {"armleft": 15, "armright": 15}
_LO, _HI = 1, 100
_LOCK = threading.Lock()


def get_speeds() -> dict:
    out = dict(DEFAULTS)
    if CONFIG_PATH.is_file():
        try:
            data = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
            for k in DEFAULTS:
                v = data.get(k)
                if isinstance(v, (int, float)):
                    out[k] = int(min(max(int(v), _LO), _HI))
        except Exception:                              # noqa: BLE001
            pass                                       # 损坏则回默认
    return out


def get_speed(arm: str) -> int:
    """取某臂的全局速度(未知臂名回 15,与历史默认一致)。"""
    return get_speeds().get(arm, 15)


def set_speeds(patch: dict) -> dict:
    """合并补丁并钳位持久化,返回更新后的完整速度表。"""
    with _LOCK:
        cfg = get_speeds()
        for k in DEFAULTS:
            v = (patch or {}).get(k)
            if isinstance(v, (int, float)):
                cfg[k] = int(min(max(int(v), _LO), _HI))
        SPEED_DIR.mkdir(parents=True, exist_ok=True)
        fd, tmp = tempfile.mkstemp(dir=str(SPEED_DIR), suffix=".tmp")
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as f:
                json.dump(cfg, f, ensure_ascii=False, indent=2)
            os.replace(tmp, CONFIG_PATH)
        except Exception:                              # noqa: BLE001
            try:
                os.unlink(tmp)
            except OSError:
                pass
            raise
        return cfg
