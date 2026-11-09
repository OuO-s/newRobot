# -*- coding: utf-8 -*-
"""
cutter_client.py — 裁剪驱动器桥接客户端
镜像 slamtec_client.py 风格: 纯 requests, 无状态, 失败抛异常
对应桥接服务: cutter_bridge.py (10.245.169.20:8100)
"""
import time
import requests


class CutterError(Exception):
    pass


class CutterClient:
    def __init__(self, base_url: str, timeout: float = 3.0):
        self.base_url = base_url.rstrip('/')
        self.timeout = timeout

    def _get(self, path):
        try:
            r = requests.get(self.base_url + path, timeout=self.timeout)
            r.raise_for_status()
            return r.json()
        except Exception as e:
            raise CutterError(f'裁剪桥接不可达 ({e})') from e

    def _post(self, path, body=None):
        try:
            r = requests.post(self.base_url + path, json=body or {}, timeout=self.timeout)
            r.raise_for_status()
            d = r.json()
            if not d.get('ok', True):
                raise CutterError(f'裁剪指令失败: {d}')
            return d
        except CutterError:
            raise
        except Exception as e:
            raise CutterError(f'裁剪桥接请求失败 ({e})') from e

    # ---- 基础 ----
    def health(self):
        return self._get('/api/cutter/health')

    def status(self):
        return self._get('/api/cutter/status')

    def stop(self):
        return self._post('/api/cutter/stop')

    def set_speed(self, rpm: int):
        return self._post('/api/cutter/speed', {'rpm': int(rpm)})

    def save_currents(self):
        return self._post('/api/cutter/save_currents')

    # ---- 高层 ----
    def move(self, turns: float):
        """触发裁剪运动, turns 圈(负=反转), 不等待完成"""
        return self._post('/api/cutter/move', {'turns': turns})

    def move_and_wait(self, turns: float, rpm: int = None,
                      poll_s: float = 0.3, timeout_s: float = 60.0):
        """触发运动并阻塞等待到位. 超时/失联抛 CutterError."""
        if rpm:
            self.set_speed(rpm)
        self.move(turns)
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            st = self.status()
            if not st.get('moving', False):
                return st  # 已到位
            time.sleep(poll_s)
        # 超时兜底停一次再报错
        try:
            self.stop()
        except Exception:
            pass
        raise CutterError(f'裁剪运动超时 ({timeout_s}s), 已发送停止')

    def trigger(self, cfg: dict, act_cfg: dict):
        """流水线触发入口.
        cfg     = flow_config 顶层 "cutter" 节
        act_cfg = 动作里的 {"turns": -4} (缺省用 cfg["default_turns"])
        """
        turns = act_cfg.get('turns', cfg.get('default_turns', -4))
        return self.move_and_wait(
            turns,
            rpm=cfg.get('rpm'),
            poll_s=cfg.get('poll_s', 0.3),
            timeout_s=cfg.get('move_timeout_s', 60),
        )
