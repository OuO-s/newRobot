# -*- coding: utf-8 -*-
"""思岚 Slamware 底盘 RESTful API 封装(接口定义见 https://docs.slamtec.com)。

Slamware 底盘内置 HTTP 服务(默认端口 14444),提供位姿、运动行为(Action)、
栅格地图、激光帧等接口。本模块只封装上位机软件用到的子集。
"""
from __future__ import annotations

import struct

import requests


class SlamtecError(Exception):
    """Slamware 接口失败。"""


ACTION_MOVE_TO = "slamtec.agent.actions.MoveToAction"
ACTION_MOVE_BY = "slamtec.agent.actions.MoveByAction"
ACTION_ROTATE_TO = "slamtec.agent.actions.RotateToAction"
ACTION_GO_HOME = "slamtec.agent.actions.GoHomeAction"


class SlamtecChassis:
    """Slamware REST 客户端。"""

    def __init__(self, host: str, port: int = 14444, timeout: float = 3.0):
        self.base = f"http://{host}:{port}"
        self.timeout = timeout
        self.session = requests.Session()

    # ------------------------------------------------------------------ 基础
    def _request(self, method: str, path: str, json_body: dict | None = None,
                 timeout: float | None = None) -> requests.Response:
        try:
            r = self.session.request(method, self.base + path, json=json_body,
                                     timeout=timeout or self.timeout)
        except requests.RequestException as e:
            raise SlamtecError(f"连接 {self.base} 失败: {e.__class__.__name__}") from e
        if r.status_code not in (200, 204):
            raise SlamtecError(f"{method} {path} HTTP {r.status_code}")
        return r

    def ping(self) -> bool:
        info = self._request("GET", "/api/core/system/v1/robot/info").json()
        return bool(info)

    # ------------------------------------------------------------------ 位姿
    def get_pose(self) -> dict:
        """位姿 {x, y, z, yaw, pitch, roll}(yaw 弧度)。"""
        return self._request("GET", "/api/core/slam/v1/localization/pose").json()

    def set_pose(self, x: float, y: float, yaw: float) -> None:
        self._request("PUT", "/api/core/slam/v1/localization/pose",
                      {"x": x, "y": y, "z": 0.0, "yaw": yaw, "pitch": 0.0, "roll": 0.0})

    # ------------------------------------------------------------------ 运动
    def move_to(self, x: float, y: float, yaw: float | None = None,
                precise: bool = True, mode: int = 0) -> int:
        """自主导航到目标点,返回 action_id。yaw 单位弧度。"""
        move_options: dict = {"mode": mode}
        if precise:
            move_options["flags"] = ["precise"]
        if yaw is not None:
            flags = move_options.setdefault("flags", [])
            if "with_yaw" not in flags:
                flags.append("with_yaw")
            move_options["yaw"] = yaw
        payload = self._request("POST", "/api/core/motion/v1/actions", {
            "action_name": ACTION_MOVE_TO,
            "options": {"target": {"x": x, "y": y, "z": 0.0},
                        "move_options": move_options},
        }).json()
        return int(payload.get("action_id", -1))

    def move_by(self, direction: int | None = None, theta: float | None = None,
                duration_ms: int = 500) -> int:
        """遥控运动。direction 0 前 / 1 后 / 2 右转 / 3 左转;theta 为角速度。

        遥控不避障,持续时间不宜过长;连续运动需周期性重复调用。
        """
        options: dict = {"duration": int(duration_ms)}
        if direction is not None:
            options["direction"] = int(direction)
        if theta is not None:
            options["theta"] = float(theta)
        payload = self._request("POST", "/api/core/motion/v1/actions", {
            "action_name": ACTION_MOVE_BY, "options": options}).json()
        return int(payload.get("action_id", -1))

    def rotate_to(self, angle_rad: float) -> int:
        payload = self._request("POST", "/api/core/motion/v1/actions", {
            "action_name": ACTION_ROTATE_TO, "options": {"angle": angle_rad}}).json()
        return int(payload.get("action_id", -1))

    def go_home(self, dock: bool = True) -> int:
        payload = self._request("POST", "/api/core/motion/v1/actions", {
            "action_name": ACTION_GO_HOME,
            "options": {"gohome_options": {"flags": "dock" if dock else "no_dock",
                                            "back_to_landing": True}},
        }).json()
        return int(payload.get("action_id", -1))

    def get_action(self, action_id: int) -> dict:
        """查询 Action 状态:{action_id, stage, state:{status, result, reason}}。"""
        return self._request("GET", f"/api/core/motion/v1/actions/{action_id}").json()

    def cancel_action(self) -> None:
        """终止当前运动行为。"""
        self._request("DELETE", "/api/core/motion/v1/actions/:current")

    # ------------------------------------------------------------ 地图/激光
    def get_grid_map(self) -> dict:
        """获取栅格地图。响应为二进制:36 字节头 + 地图数据(每格一字节)。"""
        r = self._request("GET", "/api/core/slam/v1/maps/explore", timeout=10.0)
        data = r.content
        if len(data) < 36:
            raise SlamtecError("地图数据过短")
        x0, y0, xsize, ysize, res = struct.unpack_from("<ffIIf", data, 0)
        (data_len,) = struct.unpack_from("<I", data, 32)
        cells = data[36:]
        if data_len and data_len != len(cells):
            raise SlamtecError(f"地图长度不匹配: 头部 {data_len}, 实际 {len(cells)}")
        if xsize * ysize != len(cells):
            raise SlamtecError(f"地图尺寸不匹配: {xsize}x{ysize} != {len(cells)}")
        return {"x0": x0, "y0": y0, "xsize": xsize, "ysize": ysize,
                "resolution": res, "data": cells}

    def get_laserscan(self) -> dict:
        """当前激光帧 {pose:{x,y,yaw...}, laser_points:[{angle,distance,valid}]}。"""
        return self._request("GET", "/api/core/system/v1/laserscan").json()

    def get_robot_info(self) -> dict:
        return self._request("GET", "/api/core/system/v1/robot/info").json()
