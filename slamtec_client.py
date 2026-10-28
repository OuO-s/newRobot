# -*- coding: utf-8 -*-
"""思岚 Slamware 底盘 RESTful API 封装(接口定义见 https://docs.slamtec.com)。

Slamware 底盘内置 HTTP 服务(默认端口 14444),提供位姿、运动行为(Action)、
栅格地图、激光帧等接口。本模块只封装上位机软件用到的子集。
"""
from __future__ import annotations

import base64
import json
import socket
import struct
import time
import uuid

import requests


class SlamtecError(Exception):
    """Slamware 接口失败。"""


ACTION_MOVE_TO = "slamtec.agent.actions.MoveToAction"
ACTION_MOVE_BY = "slamtec.agent.actions.MoveByAction"
ACTION_ROTATE_TO = "slamtec.agent.actions.RotateToAction"
ACTION_GO_HOME = "slamtec.agent.actions.GoHomeAction"
ACTION_MANUAL_RELOC = "slamtec.agent.actions.ManualRelocalizationAction"


class SlamtecChassis:
    """Slamware REST 客户端。"""

    def __init__(self, host: str, port: int = 14444, timeout: float = 3.0):
        self._host = host
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
                precise: bool = True, mode: int = 0,
                ignore_dynamic: bool = False,
                speed_ratio: float | None = None) -> int:
        """自主导航到目标点,返回 action_id。yaw 单位弧度。

        mode 0 自由规划 / 1 严格沿轨(遇障停等) / 2 轨道优先(遇障绕行)。
        ignore_dynamic=True 时追加官方 flag avoid_path_ignoring_dynamic_obstacles
        (思岚飞书 FAQ「直线行驶·方式2」原文;对应 C++ SDK 5.1.1 枚举
        MoveOptionFlagFindPathIgnoringDynamicObstacles 0x400=搜路失败时忽略
        激光动态障碍二次搜索。若固件对该字符串报 400,换
        find_path_ignoring_dynamic_obstacles 再试)。
        """
        move_options: dict = {"mode": mode}
        if precise:
            move_options["flags"] = ["precise"]
        if yaw is not None:
            flags = move_options.setdefault("flags", [])
            if "with_yaw" not in flags:
                flags.append("with_yaw")
            move_options["yaw"] = yaw
        if ignore_dynamic:
            flags = move_options.setdefault("flags", [])
            if "avoid_path_ignoring_dynamic_obstacles" not in flags:
                flags.append("avoid_path_ignoring_dynamic_obstacles")
        # 速度比例(官方 move_options.speed_ratio,插件>=4.5.4):本次导航最大速=全局上限×比例
        if speed_ratio is not None:
            move_options["speed_ratio"] = max(0.1, min(1.0, float(speed_ratio)))
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

    # ------------------------------------------------------- 虚拟轨道(1445 通道)
    def get_track_segments(self) -> list[dict]:
        """读底盘虚拟轨道(RoboStudio 画的,存于底盘)。

        REST 1448 没有对应接口,走底盘原生 1445 JSON 通道:
        先 getrobotconfig 握手,再 getlines{usage:"virtual_track"}。
        lines 字段是 base64 的自定义二进制:9 字节头 + N×20 字节记录
        {f32 x1, y1, x2, y2; u32 id}(实测 Hermes 5.2.1)。
        返回 [{id, x1, y1, x2, y2}];无轨道返回空表,失败抛 SlamtecError。
        """
        try:
            sock = socket.create_connection((self._host, 1445), timeout=4)
        except OSError as e:
            raise SlamtecError(f"连 1445 失败: {e.__class__.__name__}") from e
        sock.settimeout(5)
        try:
            return self._track_via_1445(sock)
        finally:
            sock.close()

    def _track_via_1445(self, sock: socket.socket) -> list[dict]:
        rid = 0
        buf = b""

        def read_json() -> dict | None:
            nonlocal buf
            while True:
                for i in range(len(buf)):
                    if buf[i:i + 1] != b"{":
                        continue
                    depth, instr, esc = 0, False, False
                    for j in range(i, len(buf)):
                        c = buf[j:j + 1]
                        if esc:
                            esc = False
                        elif c == b"\\":
                            esc = True
                        elif c == b'"':
                            instr = not instr
                        elif not instr:
                            if c == b"{":
                                depth += 1
                            elif c == b"}":
                                depth -= 1
                                if depth == 0:
                                    obj, buf = buf[i:j + 1], buf[j + 1:]
                                    return json.loads(obj.decode("utf-8", "replace"))
                    break
                data = sock.recv(65536)
                if not data:
                    return None
                buf += data

        def send(cmd: str, args=None) -> dict | None:
            nonlocal rid
            rid += 1
            sock.sendall((json.dumps({"args": args, "command": cmd,
                                      "request_id": rid}) + "\r\n\r\n").encode())
            t0 = time.time()
            while time.time() - t0 < 6:
                r = read_json()
                if r is not None and r.get("request_id") == rid:
                    return r
            return None

        send("getrobotconfig")                       # 握手(内容不用)
        r = send("getlines", {"usage": "virtual_track"})
        if r is None or "result" not in r:
            raise SlamtecError("getlines 无响应")
        blob = (r["result"] or {}).get("lines") or ""
        if not blob:
            return []
        try:
            raw = base64.b64decode(blob)
        except (ValueError, TypeError) as e:
            raise SlamtecError(f"lines 编码异常: {e}") from e
        n = (len(raw) - 9) // 20
        if n < 0 or (len(raw) - 9) % 20:
            raise SlamtecError(f"lines 长度异常: {len(raw)}")
        segs = []
        for i in range(n):
            x1, y1, x2, y2, lid = struct.unpack_from("<ffffI", raw, 9 + i * 20)
            segs.append({"id": lid, "x1": x1, "y1": y1, "x2": x2, "y2": y2})
        return segs

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

    def relocalize(self) -> int:
        """自动重定位:底盘主动旋转/扫描匹配重建定位(机器人会原地转动)。"""
        payload = self._request("POST", "/api/core/motion/v1/actions", {
            "action_name": ACTION_MANUAL_RELOC}).json()
        return int(payload.get("action_id", -1))

    def save_map(self) -> None:
        """把当前运行时地图持久化为设备内置 stcm(覆盖上次保存)。"""
        self._request("POST", "/api/multi-floor/map/v1/stcm/:save", {})

    def reload_map(self) -> None:
        """用设备内置 stcm 重载运行时地图(当前地图被覆盖)。"""
        self._request("POST", "/api/multi-floor/map/v1/stcm/:reload", {})

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

    def get_pois(self) -> list:
        """RoboStudio 存的点位(兴趣点):[{id, metadata:{display_name}, pose}]。"""
        return self._request("GET", "/api/core/artifact/v1/pois").json()

    def get_poi(self, poi_id: str) -> dict:
        """单个点位详情。"""
        return self._request("GET", f"/api/core/artifact/v1/pois/{poi_id}").json()

    def add_poi(self, name: str, x: float, y: float, yaw: float | None = None,
                poi_id: str | None = None) -> str:
        """新建点位。厂商示例不带 pose,这里带上;固件若忽略会落在 (0,0),
        调用方须读回校验(get_poi)。"""
        if not poi_id:
            poi_id = "web-" + uuid.uuid4().hex[:12]
        self._request("POST", "/api/core/artifact/v1/pois", {
            "id": poi_id,
            "metadata": {"cargo_id": "", "display_name": name,
                         "station_id": "", "type": "TAG"},
            "pose": {"x": float(x), "y": float(y), "z": 0.0,
                     "yaw": float(yaw or 0.0), "pitch": 0.0, "roll": 0.0},
        })
        return poi_id

    def delete_poi(self, poi_id: str) -> None:
        self._request("DELETE", f"/api/core/artifact/v1/pois/{poi_id}")

    def get_homedocks(self) -> list:
        """充电桩位列表,结构同 POI。"""
        return self._request("GET", "/api/core/slam/v1/homedocks").json()

    def get_power_status(self) -> dict:
        """电源状态: {batteryPercentage, isCharging, dockingStatus, powerStage, ...}。"""
        return self._request("GET", "/api/core/system/v1/power/status").json()

    def get_health(self) -> dict:
        """健康状态: {hasError, hasFatal, hasWarning, hasLidarDisconnected, ...}。"""
        return self._request("GET", "/api/core/system/v1/robot/health").json()

    def get_speed(self) -> dict:
        """当前速度: {vx, vy, omega}。"""
        return self._request("GET", "/api/core/motion/v1/speed").json()

    def get_system_parameter(self, param: str) -> str:
        """读设备系统参数(如 base.max_moving_speed,单位 m/s)。"""
        from urllib.parse import quote
        return self._request("GET", "/api/core/system/v1/parameter?param="
                             + quote(param)).json()

    def set_system_parameter(self, param: str, value) -> None:
        """写设备系统参数(即时生效;厂商示例:base.max_moving_speed=速度m/s)。"""
        self._request("PUT", "/api/core/system/v1/parameter",
                      {"param": param, "value": value})

    def get_current_action(self) -> dict | None:
        """当前运动行为;空闲(404)返回 None。"""
        try:
            return self._request("GET", "/api/core/motion/v1/actions/:current").json()
        except SlamtecError as e:
            if "HTTP 404" in str(e):
                return None
            raise

    def set_mapping_enabled(self, enable: bool) -> None:
        """开/关建图模式(开启后移动会持续更新地图)。"""
        self._request("PUT", "/api/core/slam/v1/mapping/:enable", {"enable": bool(enable)})

    def get_robot_info(self) -> dict:
        return self._request("GET", "/api/core/system/v1/robot/info").json()
