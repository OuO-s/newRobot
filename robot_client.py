# -*- coding: utf-8 -*-
"""天链 Y2-P 机器人 HTTP 接口封装。

两套服务(见《天链机器人 HTTP 接口参考手册》):
  - web_interface  端口 5000:整机网关(升降/底盘/腰头关节/抓拍/状态/臂控透传)
  - fd_arm_http    端口 8000:机械臂完整控制(上电/轨迹/笛卡尔/预设位姿/末端)

返回约定::5000 业务成功 status==1,失败 status==-1;:8000 使用 success 布尔。
本模块统一在失败时抛出 RobotError。
"""
from __future__ import annotations

import time

import requests

DEFAULT_TIMEOUT = 3.0        # 查询类请求超时(秒)
MOTION_TIMEOUT = 8.0         # 非阻塞运动指令超时(秒)


class RobotError(Exception):
    """机器人接口业务失败或通信失败。"""


_IMAGE_MAGICS = (b"\x89PNG", b"\xff\xd8\xff", b"GIF8", b"BM",
                 b"II*\x00", b"MM\x00*", b"RIFF")


def _looks_like_image(data: bytes) -> bool:
    """按魔数判断响应体是否为图片,防止把错误页/半截文件当图片。"""
    return any(data.startswith(m) for m in _IMAGE_MAGICS)


def _text_of(payload: dict) -> str:
    return str(payload.get("message") or payload.get("error") or "").strip()


class TianlianRobot:
    """天链机器人 HTTP 客户端(5000 + 8000)。"""

    def __init__(self, host: str, web_port: int = 5000, arm_port: int = 8000):
        self.host = host
        self.web_port = web_port
        self.arm_port = arm_port
        self.session = requests.Session()

    # ------------------------------------------------------------------ 基础
    def _url(self, port: int, path: str) -> str:
        return f"http://{self.host}:{port}{path}"

    def _get(self, port: int, path: str, timeout: float = DEFAULT_TIMEOUT,
             params: dict | None = None) -> dict:
        try:
            r = self.session.get(self._url(port, path), params=params, timeout=timeout)
        except requests.RequestException as e:
            raise RobotError(f"连接 {self.host}:{port} 失败: {e.__class__.__name__}") from e
        if r.status_code != 200:
            raise RobotError(f"GET {path} HTTP {r.status_code}")
        return r.json()

    def _post(self, port: int, path: str, body: dict | None = None,
              timeout: float = DEFAULT_TIMEOUT, params: dict | None = None) -> dict:
        try:
            r = self.session.post(self._url(port, path), json=body or {},
                                  params=params, timeout=timeout)
        except requests.RequestException as e:
            raise RobotError(f"连接 {self.host}:{port} 失败: {e.__class__.__name__}") from e
        if r.status_code != 200:
            raise RobotError(f"POST {path} HTTP {r.status_code}")
        return r.json()

    def _post_web(self, path: str, body: dict | None = None,
                  timeout: float = DEFAULT_TIMEOUT) -> dict:
        """:5000 接口,status==-1 视为失败。"""
        payload = self._post(self.web_port, path, body, timeout)
        if payload.get("status") == -1:
            raise RobotError(_text_of(payload) or f"{path} 执行失败")
        return payload

    def _post_arm(self, path: str, body: dict | None = None,
                  timeout: float = MOTION_TIMEOUT) -> dict:
        """:8000 接口,success==False 视为失败。"""
        payload = self._post(self.arm_port, path, body, timeout)
        if payload.get("success") is False:
            raise RobotError(_text_of(payload) or f"{path} 执行失败")
        return payload

    # ------------------------------------------------------------ 连接检测
    def ping_web(self) -> bool:
        return self._get(self.web_port, "/").get("status") == "ok"

    def ping_arm(self) -> bool:
        return self._get(self.arm_port, "/").get("status") == "ok"

    # -------------------------------------------------------------- 升降(5000)
    def lift_move_position(self, distance_m: float, speed: float = 0.02,
                           direction: int = 0) -> dict:
        """位置模式:运动到目标高度(米)。"""
        return self._post_web("/setLiftAction", {
            "direction": direction, "speed": str(speed), "distance": str(distance_m)})

    def lift_move_velocity(self, direction: int, speed: float = 0.02) -> dict:
        """速度模式:distance 传字符串 "None"。direction 0 正向 / 1 反向。"""
        return self._post_web("/setLiftAction", {
            "direction": direction, "speed": str(speed), "distance": "None"})

    def lift_stop(self) -> dict:
        return self._post_web("/liftStopAction")

    def lift_homing(self) -> dict:
        return self._post_web("/liftHoming")

    def lift_clear_error(self) -> dict:
        return self._post_web("/liftClearError")

    def lift_info(self) -> float:
        """返回当前升降高度(米);查询失败抛异常。"""
        payload = self._get(self.web_port, "/liftInfo")
        if payload.get("status") != 1:
            raise RobotError(_text_of(payload) or "升降状态不可用")
        try:
            return float(payload["data"]["position"])
        except (KeyError, TypeError, ValueError) as e:
            raise RobotError(f"升降状态格式错误: {payload}") from e

    # ------------------------------------------------------------ 底盘(5000)
    def chassis_move(self, direction: int, duration_ms: int = 500,
                     linear_speed: float = 0.2, angular_speed: float = 0.3) -> dict:
        """定时方向运动。direction 0 前 / 1 后 / 2 右转 / 3 左转。"""
        return self._post_web("/setChassisMove", {
            "direction": direction, "duration_ms": int(duration_ms),
            "linear_speed": linear_speed, "angular_speed": angular_speed})

    def chassis_stop(self) -> dict:
        return self._post_web("/chassisStopAction")

    def chassis_goto_pose(self, x: float, y: float, yaw: float) -> dict:
        """地图位姿导航(x/y 米,yaw 度;手册字段说明以角度计)。"""
        return self._post_web("/chassisGotoPose", {"x": x, "y": y, "yaw": yaw})

    def chassis_pose(self) -> tuple[float, float, float] | None:
        """当前地图位姿 (x, y, yaw);无定位数据时返回 None。"""
        payload = self._get(self.web_port, "/getChassisPose")
        if payload.get("status") != 1 or "x" not in payload:
            return None
        return float(payload["x"]), float(payload["y"]), float(payload.get("yaw", 0.0))

    # -------------------------------------------------------- 腰/头颈(5000)
    def move_joint_waist_head(self, joint_name: str, angle: float,
                              speed: float = 5.0) -> dict:
        """joint_name: waist / head1(俯仰) / head2(偏航),角度单位度。"""
        return self._post_web("/moveJoint", {
            "joint_name": joint_name, "angle": angle, "speed": speed})

    def get_waist_joint(self) -> float | None:
        payload = self._get(self.web_port, "/getWaistJoint")
        if payload.get("status") != 1:
            return None
        try:
            return float(payload["data"]["waist"])
        except (KeyError, TypeError, ValueError) as e:
            raise RobotError(f"腰关节状态格式错误: {payload}") from e

    def get_head_joint(self) -> tuple[float, float] | None:
        payload = self._get(self.web_port, "/getHeadJoint")
        if payload.get("status") != 1:
            return None
        try:
            d = payload["data"]
            return float(d["head1"]), float(d["head2"])
        except (KeyError, TypeError, ValueError) as e:
            raise RobotError(f"头关节状态格式错误: {payload}") from e

    # --------------------------------------------------------- 状态查询(5000)
    def emergency_status(self) -> bool:
        """急停状态:true 表示已急停。"""
        payload = self._get(self.web_port, "/getEmergencyStop")
        return str(payload.get("emergency_status", "false")).lower() == "true"

    def hardware_info(self) -> dict:
        payload = self._get(self.web_port, "/robotHardwareInfo")
        return payload.get("data", {})

    # --------------------------------------------------------- 相机抓拍(5000)
    CAPTURE_TIMEOUT = 10.0        # 抓拍/下载超时(首次曝光较慢,放宽)
    CAPTURE_RETRY = 4             # 下载重试次数
    CAPTURE_RETRY_DELAY = 0.5     # 重试间隔(抓拍接口返回时图片可能尚未写盘)

    def capture_image(self, camera: str = "left") -> bytes:
        """触发一次抓拍并返回图片字节。camera: left / head / right。

        实机容错:
        - 图片地址字段名/格式存在固件差异(Image_url 可能是绝对 URL,
          也可能是不含开头斜杠的相对路径),统一归一化后再下载;
        - 抓拍接口先返回地址、图片随后写盘,首次下载可能 404/空文件,
          短暂重试;
        - 响应体做图片魔数校验,错误页/半截文件不会被当作画面。
        """
        payload = self._get(self.web_port, "/imageCapture", params={"camera": camera},
                            timeout=self.CAPTURE_TIMEOUT)
        if payload.get("status") == -1:
            raise RobotError(_text_of(payload) or "抓拍失败")
        url = ""
        for key in ("Image_url", "image_url", "imageUrl", "url"):
            url = str(payload.get(key) or "").strip()
            if url:
                break
        if not url:
            raise RobotError(_text_of(payload)
                             or f"抓拍失败: 响应中没有图片地址: {payload}")

        full = (url if url.startswith(("http://", "https://"))
                else self._url(self.web_port, "/" + url.lstrip("/")))
        last_err = "下载抓拍图片失败"
        for attempt in range(self.CAPTURE_RETRY):
            try:
                r = self.session.get(full, timeout=self.CAPTURE_TIMEOUT)
            except requests.RequestException as e:
                last_err = f"下载抓拍图片失败({e.__class__.__name__})"
            else:
                if r.status_code == 200 and r.content:
                    if _looks_like_image(r.content):
                        return r.content
                    last_err = (f"图片内容异常(收到 {len(r.content)} 字节,"
                                "不是有效图片,可能写入未完成)")
                elif r.status_code == 200:
                    last_err = "抓拍图片为空(文件可能尚未写完)"
                else:
                    last_err = f"下载抓拍图片 HTTP {r.status_code}"
            if attempt < self.CAPTURE_RETRY - 1:
                time.sleep(self.CAPTURE_RETRY_DELAY)
        raise RobotError(f"{last_err}: {full}")

    # ------------------------------------------------------- 机械臂查询(8000)
    def get_joint_angles(self) -> dict:
        """返回 {armleft: [deg...], armright: [deg...]};未上电时抛异常。"""
        payload = self._get(self.arm_port, "/get_joint_angle")
        if not payload.get("success"):
            raise RobotError(_text_of(payload) or "机械臂未初始化或未接收到数据")
        return {
            "armleft": payload.get("armleft_joint_angle_deg") or [],
            "armright": payload.get("armright_joint_angle_deg") or [],
        }

    def get_end_poses(self) -> dict:
        """返回 {armleft: {"position":[x,y,z],"orientation":[x,y,z,w]}, ...}。"""
        payload = self._get(self.arm_port, "/get_end_pose")
        if not payload.get("success"):
            raise RobotError(_text_of(payload) or "机械臂未初始化或未接收到数据")
        return {
            "armleft": payload.get("armleft_end_pose") or {},
            "armright": payload.get("armright_end_pose") or {},
        }

    def get_manipulator_states(self) -> dict:
        """末端执行器状态:{left:{type,open|joints}, right:{...}}。"""
        payload = self._get(self.arm_port, "/get_manipulator_states")
        if not payload.get("success"):
            raise RobotError(_text_of(payload) or "未接收到末端执行器状态")
        return {"left": payload.get("left") or {}, "right": payload.get("right") or {}}

    # ----------------------------------------------------- 机械臂控制(8000)
    def arm_power_on(self, robot_name: str = "both", speed: int = 15) -> dict:
        return self._post_arm("/initialize_robotarm",
                              {"robot_name": robot_name, "speed": speed}, timeout=15.0)

    def arm_power_off(self, robot_name: str = "both") -> dict:
        return self._post_arm("/stop_robotarm", {"robot_name": robot_name})

    def arm_stop_all(self) -> None:
        """停止机械臂运动(下电)。失败静默(用于急停路径)。"""
        try:
            self.arm_power_off("both")
        except RobotError:
            pass

    def move_robotarm_traj(self, robot_name: str, trajectory_deg: list[list[float]],
                           block: int = 1, timeout: float = 30.0,
                           speed: int = 15) -> dict:
        """关节空间轨迹运动。trajectory 每点为 N 个关节角(度)。"""
        return self._post_arm("/move_robotarm", {
            "robot_name": robot_name, "trajectory": trajectory_deg,
            "unit": "deg", "block": block, "timeout": timeout, "speed": speed,
        }, timeout=timeout + 8.0)

    def move_arm_direction(self, robot_name: str, axis: str, distance: float,
                           frame: str = "base", block: int = 1,
                           timeout: float = 20.0, speed: int = 15) -> dict:
        """末端沿轴直线平移(笛卡尔)。axis: x/y/z;frame: base/tool/end。"""
        return self._post_arm("/move_arm_direction", {
            "robot_name": robot_name, "axis": axis, "distance": distance,
            "frame": frame, "block": block, "timeout": timeout, "speed": speed,
        }, timeout=timeout + 8.0)

    # ------------------------------------------------------- 预设位姿(8000)
    def arm_locations(self, robot_name: str) -> list[str]:
        payload = self._post_arm("/get_arm_locations", {"robot_name": robot_name})
        return payload.get("all_location_name") or []

    def save_arm_location(self, robot_name: str, location_name: str,
                          joint_angles_deg: list[float] | None = None) -> dict:
        body = {"robot_name": robot_name, "location_name": location_name}
        if joint_angles_deg is not None:
            body["joint_angles"] = joint_angles_deg
        return self._post_arm("/save_arm_location", body)

    def delete_arm_location(self, robot_name: str, location_name: str) -> dict:
        return self._post_arm("/delete_arm_location", {
            "robot_name": robot_name, "location_name": location_name})

    def move_to_location(self, robot_name: str, location_name: str,
                         timeout: float = 60.0, speed: int = 15,
                         use_joint_motion: bool = False) -> dict:
        return self._post_arm("/move_to_location", {
            "robot_name": robot_name, "location_name": location_name,
            "block": 1, "timeout": timeout, "speed": speed,
            "use_joint_motion": use_joint_motion,
        }, timeout=timeout + 8.0)

    # ------------------------------------------------------- 末端执行器(8000)
    def set_gripper(self, side: str, position: float,
                    velocity: float | None = None, effort: float | None = None) -> dict:
        """二指夹爪控制。side: left/right;position 开口 0~1000。"""
        target: dict = {"position": float(position)}
        if velocity is not None:
            target["velocity"] = float(velocity)
        if effort is not None:
            target["effort"] = float(effort)
        return self._post_arm("/set_manipulator_states", {side: {"open": target}})

    def set_hand_joints(self, side: str, names: list[str], position: float) -> dict:
        """灵巧手:所有指定关节运动到同一 position(0~1000)。"""
        joints = {n: {"position": float(position)} for n in names}
        return self._post_arm("/set_manipulator_states", {side: {"joints": joints}})

    def set_end_effector(self, side: str, position: float) -> dict:
        """按当前末端类型自动选择夹爪/灵巧手接口。"""
        states = self.get_manipulator_states()
        info = states.get(side) or {}
        if info.get("type") == "hand":
            names = list((info.get("joints") or {}).keys())
            if not names:
                raise RobotError("灵巧手关节名未知")
            return self.set_hand_joints(side, names, position)
        return self.set_gripper(side, position)
