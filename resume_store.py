# -*- coding: utf-8 -*-
"""会话自恢复持久化: 记住上次整机连接与开着的相机预览。
================================================================
目录 resume_data/(与 webapp 同级,整目录可拷):
  session.json    {"connect": {host, web_port, arm_port},
                   "cams": {"left": 1000, "head": 1000, ...}}

app.py 启动时 _auto_resume() 线程读它: 自动重连上次整机(等网关就绪,
5s×60 次重试),连上后按记录逐路恢复相机预览——重启/开机后浏览器里的
预览自回,无需重新连接/勾选。每次成功连接、开关相机都会更新本文件。

显式 /api/disconnect 不清记录(下次重启仍自动恢复);换 host 连接时
直接覆盖。前端 host 默认值是错的(192.168.1.10),自动恢复正好绕开。
"""
from __future__ import annotations

import json
import os
import tempfile
import threading
from pathlib import Path
from typing import Optional

RESUME_DIR = Path(__file__).resolve().parent / "resume_data"
SESSION_PATH = RESUME_DIR / "session.json"
ROLES = ("left", "head", "right")
_LOCK = threading.Lock()


def _load() -> dict:
    if SESSION_PATH.is_file():
        try:
            data = json.loads(SESSION_PATH.read_text(encoding="utf-8"))
            if isinstance(data, dict):
                return data
        except Exception:                              # noqa: BLE001
            pass                                       # 损坏则视为无记录
    return {}


def _save(data: dict) -> None:
    RESUME_DIR.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=str(RESUME_DIR), suffix=".tmp")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        os.replace(tmp, SESSION_PATH)
    except Exception:                                  # noqa: BLE001
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise


def get_connect() -> Optional[dict]:
    """上次成功连接参数 {host, web_port, arm_port};无记录返回 None。"""
    c = _load().get("connect")
    if isinstance(c, dict) and isinstance(c.get("host"), str):
        return {"host": c["host"],
                "web_port": int(c.get("web_port", 5000)),
                "arm_port": int(c.get("arm_port", 8000))}
    return None


def save_connect(host: str, web_port: int = 5000, arm_port: int = 8000) -> None:
    """记录一次成功连接(保留已有的相机记录)。"""
    with _LOCK:
        _save({"connect": {"host": host, "web_port": int(web_port),
                           "arm_port": int(arm_port)},
               "cams": _load().get("cams", {})})


def get_cams() -> dict:
    """上次开着的相机 {role: interval_ms},按 ROLES 过滤+钳位。"""
    cams = _load().get("cams")
    out: dict = {}
    if isinstance(cams, dict):
        for k in ROLES:
            v = cams.get(k)
            if isinstance(v, (int, float)):
                out[k] = max(100, int(v))
    return out


def set_cam(cam: str, interval_ms: Optional[int]) -> None:
    """记录一路相机预览(None=关,移除记录)。"""
    with _LOCK:
        data = _load()
        cams = data.get("cams") if isinstance(data.get("cams"), dict) else {}
        if interval_ms is None:
            cams.pop(cam, None)
        else:
            cams[cam] = max(100, int(interval_ms))
        data["cams"] = cams
        _save(data)
