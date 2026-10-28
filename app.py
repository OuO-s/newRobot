# -*- coding: utf-8 -*-
"""天链机器人远程控制 FastAPI 后端(独立模块,无需 PySide6)。

直接运行:
    uvicorn app:app --host 0.0.0.0 --port 8001
或:
    ./run.sh

浏览器访问 http://<host>:8001/ 。

模块内含:
  - robot_client.py / slamtec_client.py   天链 :5000/:8000 + 思岚 REST 封装
  - ik_helper.py / ik_config.py            末端位姿逆解子进程
  - cartesian.py                           笛卡尔增量 + IK 折线规划
  - teach_store.py                         示教文件 JSON 持久化
  - services.py                            纯 Python 业务线程
  - app.py                                 FastAPI 入口
  - static/                                浏览器端 7 Tab
"""
from __future__ import annotations

import asyncio
import csv
import json
import math
import os
import signal
import subprocess
import threading
import time
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Optional

# ik_helper 逆解用 URDF 钉死真机型号(见 ik_helper._resolve_urdf 注释):
# startup.yaml 配的 y2_p_ds_705vg 与 :8000 上报系(实测=y2_p_dt_710vg)不符。
os.environ.setdefault("IK_URDF_MODEL", "y2_p_dt_710vg")

import yaml
from fastapi import FastAPI, HTTPException, Query
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

from cartesian import cartesian_delta_move, plan_cartesian_path
from ik_config import IK_HELPER, SYS_PYTHON
from robot_client import RobotError, TianlianRobot
from services import (AppState, CameraStreamService, ChassisService,
                      StatusPoller)
from slamtec_client import SlamtecChassis, SlamtecError
from teach_store import TEACH_DIR, TeachFile

import pick_store
from depth_provider import DepthError, DepthProvider
from pick_flow import (PickJob, compute_align_residual, quat_to_rpy_deg,
                       scan_once)

import flow_runlog
import flow_store
import resume_store
import speed_store
from flow import (ArmLoop, FlowError, FlowRunner, fix_align_gate,
                  goto_align_ref, record_align_ref, test_align_gate)

STATE = AppState()

# 一步到位(吸取):深度取流单例 + 当前任务
_PICK_PROVIDER = DepthProvider()
_PICK_JOB: Optional[PickJob] = None
_PICK_LOCK = threading.Lock()

# 双臂流水线(与一步到位模块互斥: 都会动左臂/占相机租约)
_FLOW = FlowRunner()
_TEST_PLAY: dict = {}          # 单动作试跑 {"runner","loop","thread"}
_TEST_PLAY_LOCK = threading.Lock()

# pose_service 子进程(物料检测)
_POSE_PROC: Optional[subprocess.Popen] = None
_POSE_PORT = 7100
# 推理环境是 old 工程里的 venv(Jetson 版 torch + ultralytics, conda 装不了),
# 与 GUI 侧 app.py 的常量保持一致
POSE_YOLO_ENV_PY = "/home/user/tianlian/old/tianlian/yolo11_env/bin/python"
POSE_YOLO_ENV_SITE = ("/home/user/tianlian/old/tianlian/yolo11_env/"
                      "lib/python3.10/site-packages")

# camera_bridge 子进程(ROS2 彩色话题直订阅 → 本地 HTTP 帧服务)。
# 走它拿原生 640×480 帧;固件 /imageCapture 会把 4:3 硬拉成 1920×1080(畸变),
# 只作兜底。端口与 GUI 侧一致,外部已在跑(如 GUI 起的)则直接复用。
_CAM_BRIDGE_PORT = 7101
_CAM_BRIDGE_PROC: Optional[subprocess.Popen] = None


def _cam_bridge_alive() -> bool:
    import urllib.request
    try:
        with urllib.request.urlopen(
                f"http://127.0.0.1:{_CAM_BRIDGE_PORT}/health",
                timeout=1.0) as r:
            return r.status == 200
    except Exception:                                  # noqa: BLE001
        return False


def _ensure_cam_bridge() -> dict:
    """确保相机桥接在跑(外部实例优先复用),返回状态描述。"""
    global _CAM_BRIDGE_PROC
    if _cam_bridge_alive():
        return {"ok": True, "external": True, "port": _CAM_BRIDGE_PORT}
    if _CAM_BRIDGE_PROC is not None and _CAM_BRIDGE_PROC.poll() is None:
        return {"ok": True, "starting": True, "port": _CAM_BRIDGE_PORT}
    script = Path(__file__).resolve().with_name("camera_bridge.py")
    if not script.is_file():
        return {"ok": False, "error": f"camera_bridge.py 不存在: {script}"}
    env = os.environ.copy()
    env.pop("PYTHONPATH", None)     # 清掉 webapp 环境残留,避免串版本
    env.pop("PYTHONHOME", None)
    # rclpy 只在 ROS2 环境的系统 Python 3.10 里可用(GUI 同款拉起方式)
    cmd = (f"source /opt/ros/humble/setup.bash && "
           f"exec /usr/bin/python3 {script} --port {_CAM_BRIDGE_PORT}")
    log_f = open(script.parent / "logs" / "camera_bridge.log", "ab")
    _CAM_BRIDGE_PROC = subprocess.Popen(["/bin/bash", "-c", cmd],
                                        env=env, stdout=log_f,
                                        stderr=subprocess.STDOUT)
    return {"ok": True, "pid": _CAM_BRIDGE_PROC.pid, "port": _CAM_BRIDGE_PORT}


def _stop_cam_bridge() -> None:
    global _CAM_BRIDGE_PROC
    p = _CAM_BRIDGE_PROC
    if p and p.poll() is None:
        try:
            p.send_signal(signal.SIGTERM)
            p.wait(timeout=3)
        except Exception:                              # noqa: BLE001
            try:
                p.kill()
            except Exception:                          # noqa: BLE001
                pass
    _CAM_BRIDGE_PROC = None


# ===========================================================================
# Pydantic 请求体
# ===========================================================================
class ConnectBody(BaseModel):
    host: str
    web_port: int = 5000
    arm_port: int = 8000


class ChassisConnectBody(BaseModel):
    host: str
    port: int = 1448


class LiftMoveBody(BaseModel):
    distance: float
    speed: float = 0.02
    direction: int = 0


class LiftVelocityBody(BaseModel):
    direction: int
    speed: float = 0.02


class WaistBody(BaseModel):
    angle: float


class ArmPowerBody(BaseModel):
    robot_name: str = "both"
    speed: int = 15


class PoseSaveBody(BaseModel):
    arm: str
    name: str


class PoseGotoBody(BaseModel):
    arm: str
    name: str
    speed: Optional[int] = None       # None=该臂全局速度(顶栏)
    timeout: float = 90.0


class PoseDeleteBody(BaseModel):
    arm: str
    name: str


class ChassisMoveToBody(BaseModel):
    x: float
    y: float
    yaw: Optional[float] = None
    precise: bool = True
    mode: int = 0
    speed_ratio: Optional[float] = None   # 0.1~1,本次导航速度比例(None=不限制)


class ChassisJoystickBody(BaseModel):
    direction: Optional[int] = None
    theta: Optional[float] = None
    duration_ms: int = 500


class ChassisRotateBody(BaseModel):
    angle_rad: float


class ChassisGoHomeBody(BaseModel):
    dock: bool = True


class ChassisSetPoseBody(BaseModel):
    x: float
    y: float
    yaw: Optional[float] = None    # 缺省保持当前朝向


class ChassisMappingBody(BaseModel):
    enable: bool


class ChassisMaxSpeedBody(BaseModel):
    max_moving_speed: float    # m/s,厂商上限 3.0


class ChassisTracksBody(BaseModel):
    tracks: list[dict] = []    # [{name, points: [{x, y, yaw?}]}]


class ChassisPoiBody(BaseModel):
    name: str
    x: float
    y: float
    yaw: Optional[float] = None


class ChassisPoiEditBody(ChassisPoiBody):
    id: str


class ChassisPoiIdBody(BaseModel):
    id: str


class ChassisPoiImportBody(BaseModel):
    pois: list[dict]          # [{name, x, y, yaw?}]


class CameraStartBody(BaseModel):
    cam: str
    interval_ms: int = 1000


class CartJogBody(BaseModel):
    arm: str                                  # armleft / armright
    axis: str                                  # x / y / z
    distance: float                            # 米
    frame: str = "base"                        # base / tool
    speed: Optional[int] = None               # None=该臂全局速度(顶栏)
    step_m: float = 0.02


class CartRotateBody(BaseModel):
    arm: str
    axis: str                                  # rx / ry / rz
    angle_deg: float
    frame: str = "base"
    speed: Optional[int] = None               # None=该臂全局速度(顶栏)


class CartMoveToBody(BaseModel):
    arm: str
    pos: list[float]                           # [x, y, z] 米
    rpy: list[float]                           # [roll, pitch, yaw] 度
    speed: Optional[int] = None               # None=该臂全局速度(顶栏)
    timeout: float = 30.0


class GripperBody(BaseModel):
    side: str                                  # armleft / armright
    position: float                            # 0=闭合, 1000=张开
    speed: int = 500


class TeachSaveBody(BaseModel):
    kind: str                                  # joint_teach / cartesian_teach
    name: str
    arm: str
    speed: Optional[int] = None               # None=该臂全局速度(顶栏)
    points: list[dict]


class TeachPlayBody(BaseModel):
    name: str
    arm: str
    speed: Optional[int] = None               # None=该臂全局速度(顶栏)


class PoseDetectBody(BaseModel):
    cam: str = "left"
    conf: float = 0.25


class SpeedBody(BaseModel):
    armleft: Optional[int] = None
    armright: Optional[int] = None


# ===========================================================================
# 工具
# ===========================================================================
async def _call(fn, *args, **kwargs):
    return await asyncio.to_thread(fn, *args, **kwargs)


def _err(msg: str, code: int = 400) -> HTTPException:
    return HTTPException(status_code=code, detail=msg)


def _is_preempted(msg: str) -> bool:
    """轨迹被更新的目标抢占(厂商日志详情=preempted)。

    点动/连续操作里,新指令取消旧指令是正常语义:被取消的那次请求会以
    失败返回,但手臂实际在执行新指令——不算错误。"""
    m = msg or ""
    low = m.lower()
    return "preempt" in low or "抢占" in m or "取消" in m


def _normalize_arm(s: str) -> str:
    s = (s or "").strip().lower()
    if s in ("armleft", "左臂", "left"):
        return "armleft"
    if s in ("armright", "右臂", "right"):
        return "armright"
    raise _err("arm 必须是 armleft/armright")


def _speed_of(arm: str, explicit: Optional[int] = None) -> int:
    """速度解析: 请求显式给值用显式值,否则取该臂的全局速度(顶栏)。"""
    if explicit is not None:
        return max(1, min(100, int(explicit)))
    return speed_store.get_speed(arm)


# ===========================================================================
# 生命周期
# ===========================================================================
@asynccontextmanager
async def lifespan(app: FastAPI):
    threading.Thread(target=_auto_resume, name="auto_resume",
                     daemon=True).start()
    yield
    _stop_pose_proc()
    _PICK_PROVIDER.shutdown()      # 释放深度租约
    STATE.shutdown()
    _stop_pose_proc()
    _stop_cam_bridge()


app = FastAPI(title="天链机器人远程控制", version="0.1.0", lifespan=lifespan)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)


# ===========================================================================
# 健康
# ===========================================================================
@app.get("/api/health")
async def health():
    pose_alive = _pose_proc.poll() is None if _POSE_PROC else False
    return {
        "ok": True,
        "robot_connected": STATE.robot is not None,
        "chassis_connected": STATE.chassis_obj is not None,
        "camera_active": STATE.camera.active_cams() if STATE.camera else [],
        "pose_service_alive": pose_alive,
        "ik_helper_present": IK_HELPER.is_file() and Path(SYS_PYTHON).is_file(),
    }


# ===========================================================================
# 机器人连接
# ===========================================================================
@app.post("/api/connect")
async def connect_robot(body: ConnectBody):
    STATE.disconnect_robot()
    robot = TianlianRobot(body.host, body.web_port, body.arm_port)
    try:
        ok = await _call(robot.ping_web)
    except RobotError as e:
        raise _err(f"整机网关不可达: {e}", 502)
    if not ok:
        raise _err("整机网关 / 返回非 ok", 502)
    arm_ok = False
    try:
        arm_ok = await _call(robot.ping_arm)
    except RobotError:
        arm_ok = False
    STATE.robot = robot
    STATE.status_poller = StatusPoller(robot)
    STATE.status_poller.start()
    STATE.camera = CameraStreamService(robot, bridge_ensure=_ensure_cam_bridge)
    # 相机桥接独立于固件连接(直订 ROS 话题),连接整机时顺手确保在跑
    bridge = _ensure_cam_bridge()
    resume_store.save_connect(body.host, body.web_port, body.arm_port)
    return {"ok": True, "host": body.host, "web_port": body.web_port,
            "arm_port": body.arm_port, "arm_ok": arm_ok, "cam_bridge": bridge}


@app.post("/api/disconnect")
async def disconnect_robot():
    STATE.disconnect_robot()
    return {"ok": True}


# ===========================================================================
# 开机自恢复: 自动重连上次整机 + 恢复相机预览(resume_data/session.json)
# ===========================================================================
_RESUME_RETRY_S = 5.0
_RESUME_MAX_TRIES = 60          # ~5 分钟,等整机网关随开机就绪


def _auto_resume() -> None:
    """启动线程:恢复上次会话。只跑一次,用户先手动连上则以手动为准;
    全部失败则静默退出,等用户手动连接(痕迹在 webapp.log)。"""
    conn = resume_store.get_connect()
    if conn is None:
        return
    for i in range(_RESUME_MAX_TRIES):
        if STATE.robot:
            break
        try:
            robot = TianlianRobot(**conn)
            if robot.ping_web():
                STATE.robot = robot
                STATE.status_poller = StatusPoller(robot)
                STATE.status_poller.start()
                STATE.camera = CameraStreamService(
                    robot, bridge_ensure=_ensure_cam_bridge)
                print(f"[auto-resume] 整机已自动连接 {conn['host']}"
                      f"(第 {i + 1} 次尝试)", flush=True)
                break
        except Exception:                            # noqa: BLE001
            pass                                     # 网关未就绪,继续等
        time.sleep(_RESUME_RETRY_S)
    if not STATE.camera:
        return
    _ensure_cam_bridge()
    for cam, interval_ms in resume_store.get_cams().items():
        try:
            STATE.camera.start(cam, interval_ms)
            print(f"[auto-resume] 相机 {cam} 预览已恢复"
                  f"(interval {interval_ms}ms)", flush=True)
        except Exception as e:                        # noqa: BLE001
            print(f"[auto-resume] 相机 {cam} 恢复失败: {e}", flush=True)


# ===========================================================================
# 状态
# ===========================================================================
@app.get("/api/status")
async def get_status():
    if not STATE.status_poller:
        raise _err("机器人未连接", 409)
    return STATE.status_poller.snapshot()


@app.get("/api/status/stream")
async def status_stream():
    async def gen():
        while True:
            if not STATE.status_poller:
                yield "event: disconnect\ndata: {}\n\n"
                await asyncio.sleep(1.0)
                continue
            snap = STATE.status_poller.snapshot()
            yield f"data: {json.dumps(snap, ensure_ascii=False)}\n\n"
            await asyncio.sleep(1.0)
    return StreamingResponse(gen(), media_type="text/event-stream")


# ===========================================================================
# 全局速度(双臂分别)—— 顶栏唯一入口,所有臂运动默认取这里
# ===========================================================================
@app.get("/api/speed")
async def speed_get():
    return speed_store.get_speeds()


@app.post("/api/speed")
async def speed_set(body: SpeedBody):
    return {"ok": True, "speeds": speed_store.set_speeds(
        {"armleft": body.armleft, "armright": body.armright})}


# ===========================================================================
# 机械臂
# ===========================================================================
@app.post("/api/arm/power_on")
async def arm_power_on(body: ArmPowerBody):
    if not STATE.robot:
        raise _err("机器人未连接", 409)
    try:
        return await _call(STATE.robot.arm_power_on, body.robot_name, body.speed)
    except RobotError as e:
        raise _err(str(e), 502)


@app.post("/api/arm/power_off")
async def arm_power_off(body: ArmPowerBody):
    if not STATE.robot:
        raise _err("机器人未连接", 409)
    try:
        return await _call(STATE.robot.arm_power_off, body.robot_name)
    except RobotError as e:
        raise _err(str(e), 502)


@app.post("/api/arm/emergency_stop")
async def arm_emergency_stop():
    _FLOW.abort("急停")                     # 流水线在段边界停下
    if _PICK_JOB is not None and not _PICK_JOB._done:
        _PICK_JOB.abort_event.set()
    if _TEST_PLAY.get("runner") is not None:
        _TEST_PLAY["runner"].abort_event.set()
    if STATE.robot:
        await _call(STATE.robot.arm_stop_all)
        try:
            await _call(STATE.robot.lift_stop)
        except RobotError:
            pass
    if STATE.chassis_obj:
        try:
            await _call(STATE.chassis_obj.cancel_action)
        except SlamtecError:
            pass
    return {"ok": True}


# ===========================================================================
# 升降
# ===========================================================================
@app.get("/api/lift/info")
async def lift_info():
    if not STATE.robot:
        raise _err("机器人未连接", 409)
    try:
        return {"position_m": await _call(STATE.robot.lift_info)}
    except RobotError as e:
        raise _err(str(e), 502)


@app.post("/api/lift/move")
async def lift_move(body: LiftMoveBody):
    if not STATE.robot:
        raise _err("机器人未连接", 409)
    try:
        return await _call(STATE.robot.lift_move_position,
                           body.distance, body.speed, body.direction)
    except RobotError as e:
        raise _err(str(e), 502)


@app.post("/api/lift/velocity")
async def lift_velocity(body: LiftVelocityBody):
    if not STATE.robot:
        raise _err("机器人未连接", 409)
    try:
        return await _call(STATE.robot.lift_move_velocity,
                           body.direction, body.speed)
    except RobotError as e:
        raise _err(str(e), 502)


@app.post("/api/lift/stop")
async def lift_stop():
    if not STATE.robot:
        raise _err("机器人未连接", 409)
    try:
        return await _call(STATE.robot.lift_stop)
    except RobotError as e:
        raise _err(str(e), 502)


@app.post("/api/lift/homing")
async def lift_homing():
    if not STATE.robot:
        raise _err("机器人未连接", 409)
    try:
        return await _call(STATE.robot.lift_homing)
    except RobotError as e:
        raise _err(str(e), 502)


@app.post("/api/lift/clear_error")
async def lift_clear_error():
    if not STATE.robot:
        raise _err("机器人未连接", 409)
    try:
        return await _call(STATE.robot.lift_clear_error)
    except RobotError as e:
        raise _err(str(e), 502)


# ===========================================================================
# 腰 / 头关节
# ===========================================================================
@app.post("/api/waist/move")
async def waist_move(body: WaistBody):
    if not STATE.robot:
        raise _err("机器人未连接", 409)
    try:
        return await _call(STATE.robot.move_joint_waist_head,
                           "waist", body.angle, 15.0, 0)
    except RobotError as e:
        raise _err(str(e), 502)


@app.post("/api/head/move")
async def head_move(body: WaistBody):
    if not STATE.robot:
        raise _err("机器人未连接", 409)
    try:
        return await _call(STATE.robot.move_joint_waist_head,
                           "head", body.angle, 15.0, 0)
    except RobotError as e:
        raise _err(str(e), 502)



class JointJogBody(BaseModel):
    arm: str                                   # armleft / armright
    joint: int                                 # 0..6
    delta: float                               # 本次增量(度),可正可负
    speed: Optional[int] = None               # None=该臂全局速度(顶栏)
    absolute: Optional[float] = None           # 直接给目标角(优先于 delta)


# 软限位:机械臂 7 关节不绝对禁止旋转,这里只防缠绕/撞限位
JOINT_SOFT_LIMIT_DEG = 175.0

# jog 游标: arm -> {"q": [7 floats], "locked": bool}
#
# 为什么需要游标:伺服有执行时间,每次请求读回的反馈角滞后于上一拍的目标。
# 若按「反馈角 + delta」累加,反馈永远追不上命令 → 原地抖动/不动。
# 游标保存"已命令到的位置",在它之上继续累加,才能真正连续走。
#
# 重同步策略(关键):按住期间 commanded 必然领先 feedback,这是正常的,
# 不能拿 lag 当重同步判据 —— 否则每拍都"重同步"回滞后的反馈,造成抖动。
# 正确判据是「是否有人在游标之外动过」:
#   - 前端按住开始 / 绝对定位 / jog_reset 时,显式置 locked 标记该臂已接管;
#     locked 期间一律以游标为准,不因 lag 重同步。
#   - 若命令下发后失败,解锁让下一拍按反馈重新起步。
#   - unlocked(空闲)且首次调用时,用反馈初始化游标。
_JOG: dict[str, dict] = {}
_JOG_LOCK = threading.Lock()


def _jog_cursor(arm: str) -> dict:
    with _JOG_LOCK:
        return _JOG.setdefault(arm, {"q": None, "locked": False})


def _jog_reset(arm: str) -> None:
    """丢弃游标。下一拍从实时反馈重新起步。"""
    with _JOG_LOCK:
        _JOG.pop(arm, None)


def _jog_actual(arm: str) -> list[float]:
    """读实时反馈角,失败抛 RobotError。"""
    joints = STATE.robot.get_joint_angles()
    if not isinstance(joints, dict) or arm not in joints:
        raise RobotError(f"关节数据中找不到 {arm}(机械臂可能未上电)")
    arr = list(joints[arm])
    if len(arr) < 7:
        raise RobotError(f"关节数据长度异常({len(arr)})")
    return [float(v) for v in arr[:7]]


@app.post("/api/arm/joint_jog")
async def arm_joint_jog(body: JointJogBody):
    """单关节点动:前端按住时以固定周期连续调用,松手停发。

    delta 模式基于 jog 游标累加(不受反馈滞后影响),absolute 模式直接
    给目标角并重同步游标。
    """
    if not STATE.robot:
        raise _err("机器人未连接", 409)
    arm = _normalize_arm(body.arm)
    if not 0 <= body.joint <= 6:
        raise _err("joint 必须是 0..6")

    try:
        actual = await _call(_jog_actual, arm)
    except RobotError as e:
        raise _err(f"读关节角失败: {e}", 502)

    cur = _jog_cursor(arm)
    with _JOG_LOCK:
        first_touch = cur["q"] is None
        # 未被接管(空闲) → 以反馈为准起步
        q = list(cur["q"]) if (cur["q"] and cur["locked"]) else list(actual)

        if body.absolute is not None:
            target = float(body.absolute)
        else:
            target = q[body.joint] + float(body.delta)

        lo, hi = -JOINT_SOFT_LIMIT_DEG, JOINT_SOFT_LIMIT_DEG
        clamped = max(lo, min(hi, target))
        at_limit = (clamped != target) or abs(q[body.joint]) >= hi - 0.5

        q[body.joint] = clamped
        cur["q"] = q
        cur["locked"] = True      # 本次调用接管,后续按游标累加

    try:
        await _call(STATE.robot.move_robotarm_traj, arm, [q], 1, 20.0,
                    _speed_of(arm, body.speed))
    except RobotError as e:
        if _is_preempted(str(e)):
            # 被下一拍/新目标抢占:不算失败,游标保持(新指令在它之上继续)
            return {"ok": True, "arm": arm, "joint": body.joint,
                    "commanded_deg": round(clamped, 3),
                    "actual_deg": round(actual[body.joint], 3),
                    "lag_deg": round(clamped - actual[body.joint], 3),
                    "at_limit": at_limit, "from_feedback": first_touch,
                    "preempted": True}
        # 下发失败:解锁,让下一拍按反馈重新起步,避免游标越飘越远
        with _JOG_LOCK:
            _JOG.pop(arm, None)
        raise _err(str(e), 502)

    return {
        "ok": True,
        "arm": arm,
        "joint": body.joint,
        "commanded_deg": round(clamped, 3),   # 已命令到的位置(游标)
        "actual_deg": round(actual[body.joint], 3),   # 实时反馈
        "lag_deg": round(clamped - actual[body.joint], 3),
        "at_limit": at_limit,
        "from_feedback": first_touch,
    }


@app.post("/api/arm/jog_reset")
async def arm_jog_reset(arm: str = Query("armleft")):
    """丢弃 jog 游标,下次点动从实时反馈重新起步。

    前端在「开始按住」时调用一次,确保不会接着上一次按住的目标继续走;
    松手时也调用,使机械臂停在上次命令的位置,而不是继续累加。
    """
    arm_n = _normalize_arm(arm)
    _jog_reset(arm_n)
    return {"ok": True, "arm": arm_n}



# ===========================================================================
# 相机
# ===========================================================================
@app.post("/api/camera/start")
async def camera_start(body: CameraStartBody):
    if not STATE.camera:
        raise _err("机器人未连接", 409)
    if body.cam not in ("left", "head", "right"):
        raise _err("cam 必须是 left/head/right")
    _ensure_cam_bridge()          # 桥接可能被停过,开相机前确保在跑
    STATE.camera.start(body.cam, body.interval_ms)
    resume_store.set_cam(body.cam, body.interval_ms)
    return {"ok": True, "cam": body.cam, "interval_ms": body.interval_ms}


@app.get("/api/camera/meta")
async def camera_meta():
    """相机元信息:桥接状态 + 每路帧来源/分辨率(src=ros 为原生无畸变)。"""
    return {"bridge": _cam_bridge_alive(), "bridge_port": _CAM_BRIDGE_PORT,
            "cams": STATE.camera.meta_all() if STATE.camera else {}}


@app.post("/api/camera/stop")
async def camera_stop(cam: str = Query(...)):
    resume_store.set_cam(cam, None)     # 先落盘:断连时也要记住"已关"
    if not STATE.camera:
        return {"ok": True}
    STATE.camera.stop(cam)
    return {"ok": True, "cam": cam}


@app.get("/api/camera/snapshot")
async def camera_snapshot(cam: str = Query("left")):
    if not STATE.camera:
        raise _err("相机未启动", 409)
    data = STATE.camera.latest(cam)
    if not data:
        err = STATE.camera.last_error(cam)
        raise _err(f"相机 {cam} 暂无帧: {err or '未连接'}", 503)
    return StreamingResponse(iter([data]),
                             media_type="image/jpeg" if data[:3] == b"\xff\xd8\xff"
                                          else "image/png")


@app.get("/api/camera/stream")
async def camera_stream(cam: str = Query("left"),
                        fps: int = Query(10, ge=1, le=30)):
    if not STATE.camera:
        raise _err("相机未启动", 409)
    STATE.camera.start(cam, max(100, int(1000 / fps)))

    async def gen():
        boundary = b"--frame\r\n"
        period = 1.0 / fps
        try:
            while True:
                t0 = time.time()
                data = STATE.camera.latest(cam) if STATE.camera else None
                if data:
                    ctype = "image/jpeg" if data[:3] == b"\xff\xd8\xff" \
                            else "image/png"
                    yield (boundary
                           + f"Content-Type: {ctype}\r\n"
                             f"Content-Length: {len(data)}\r\n\r\n".encode()
                           + data + b"\r\n")
                else:
                    await asyncio.sleep(0.1)
                    continue
                remain = period - (time.time() - t0)
                if remain > 0:
                    await asyncio.sleep(remain)
        except asyncio.CancelledError:
            return

    return StreamingResponse(gen(),
                             media_type="multipart/x-mixed-replace; boundary=frame")


# ===========================================================================
# 底盘(Slamware)
# ===========================================================================
@app.post("/api/chassis/connect")
async def chassis_connect(body: ChassisConnectBody):
    STATE.disconnect_chassis()
    ch = SlamtecChassis(body.host, body.port)
    try:
        await _call(ch.ping)
    except SlamtecError as e:
        raise _err(str(e), 502)
    STATE.chassis_obj = ch
    STATE.chassis_service = ChassisService(ch)
    STATE.chassis_service.start()
    return {"ok": True, "host": body.host, "port": body.port}


@app.post("/api/chassis/disconnect")
async def chassis_disconnect():
    STATE.disconnect_chassis()
    return {"ok": True}


@app.get("/api/chassis/pose")
async def chassis_pose():
    if not STATE.chassis_service:
        raise _err("底盘未连接", 409)
    return STATE.chassis_service.pose() or {}


@app.get("/api/chassis/laser")
async def chassis_laser():
    if not STATE.chassis_service:
        raise _err("底盘未连接", 409)
    return {"laser_points": STATE.chassis_service.laser()}


@app.get("/api/chassis/map")
async def chassis_map(refresh: bool = False):
    if not STATE.chassis_service:
        raise _err("底盘未连接", 409)
    m = await _call(STATE.chassis_service.grid_map, refresh)
    if not m:
        raise _err("地图加载失败", 502)
    try:
        from PIL import Image
    except ImportError:
        raise _err("缺少 Pillow,请 pip install Pillow", 500)
    # Slamware(Hermes) explore 图: 0=未知, 127=空闲, 1~126=障碍(墙);
    # 数据第 0 行 = 最低 y(激光贴墙实测),渲染时上下翻转使显示图上方 = +y
    w, h = m["xsize"], m["ysize"]
    cells = m["data"]
    img = Image.new("RGB", (w, h), (0x1b, 0x1f, 0x26))
    px = img.load()
    for y in range(h):
        base = y * w
        for x in range(w):
            v = cells[base + x]
            if v == 127:
                px[x, y] = (0x8a, 0x92, 0x9c)
            elif v != 0:
                px[x, y] = (0xf0, 0xf2, 0xf4)
    from io import BytesIO
    buf = BytesIO()
    img.transpose(Image.FLIP_TOP_BOTTOM).save(buf, "PNG")
    return StreamingResponse(iter([buf.getvalue()]), media_type="image/png")


@app.get("/api/chassis/map_info")
async def chassis_map_info():
    """地图元数据,前端做世界坐标↔像素换算用。"""
    if not STATE.chassis_service:
        raise _err("底盘未连接", 409)
    m = await _call(STATE.chassis_service.grid_map)
    if not m:
        raise _err("地图加载失败", 502)
    return {"x0": m["x0"], "y0": m["y0"], "xsize": m["xsize"],
            "ysize": m["ysize"], "resolution": m["resolution"]}


@app.get("/api/chassis/pois")
async def chassis_pois():
    """RoboStudio 存的点位列表。"""
    if not STATE.chassis_obj:
        raise _err("底盘未连接", 409)
    try:
        return {"pois": await _call(STATE.chassis_obj.get_pois)}
    except SlamtecError as e:
        raise _err(str(e), 502)


def _poi_write_verified(ch, poi_id: str, x: float, y: float,
                        yaw: Optional[float]) -> tuple[bool, dict]:
    """写入后读回校验:固件若忽略 POST 里的 pose,点位会落在 (0,0)。"""
    try:
        pose = ch.get_poi(poi_id).get("pose") or {}
        if abs(pose.get("x", 0.0) - x) > 1e-3 or abs(pose.get("y", 0.0) - y) > 1e-3:
            return False, pose
        if yaw is not None and abs(pose.get("yaw", 0.0) - yaw) > 1e-3:
            return False, pose
        return True, pose
    except SlamtecError:
        return False, {}


async def _poi_add_verified(name: str, x: float, y: float,
                            yaw: Optional[float]) -> dict:
    ch = STATE.chassis_obj
    pid = await _call(ch.add_poi, name, x, y, yaw)
    verified, pose = await _call(_poi_write_verified, ch, pid, x, y, yaw)
    return {"name": name, "ok": True, "poi_id": pid,
            "verified": verified, "pose": pose}


@app.post("/api/chassis/pois/add")
async def chassis_poi_add(body: ChassisPoiBody):
    """新建点位,写后读回校验坐标是否生效。"""
    if not STATE.chassis_obj:
        raise _err("底盘未连接", 409)
    if not body.name.strip():
        raise _err("名称不能为空")
    try:
        return await _poi_add_verified(body.name.strip(), body.x, body.y, body.yaw)
    except SlamtecError as e:
        raise _err(str(e), 502)


@app.post("/api/chassis/pois/edit")
async def chassis_poi_edit(body: ChassisPoiEditBody):
    """编辑 = 删旧建新(id 变化,名称/坐标以表单为准)。"""
    if not STATE.chassis_obj:
        raise _err("底盘未连接", 409)
    if not body.name.strip():
        raise _err("名称不能为空")
    try:
        await _call(STATE.chassis_obj.delete_poi, body.id)
        return await _poi_add_verified(body.name.strip(), body.x, body.y, body.yaw)
    except SlamtecError as e:
        raise _err(str(e), 502)


@app.post("/api/chassis/pois/delete")
async def chassis_poi_delete(body: ChassisPoiIdBody):
    if not STATE.chassis_obj:
        raise _err("底盘未连接", 409)
    try:
        await _call(STATE.chassis_obj.delete_poi, body.id)
        return {"ok": True, "id": body.id}
    except SlamtecError as e:
        raise _err(str(e), 502)


@app.post("/api/chassis/pois/import")
async def chassis_poi_import(body: ChassisPoiImportBody):
    """批量导入(网页解析好的 CSV 行)。"""
    if not STATE.chassis_obj:
        raise _err("底盘未连接", 409)
    if not body.pois:
        raise _err("没有可导入的点位")
    results = []
    for row in body.pois:
        name = str(row.get("name", "")).strip()
        try:
            x, y = float(row["x"]), float(row["y"])
        except (KeyError, TypeError, ValueError):
            results.append({"name": name, "ok": False, "error": "x/y 非数字"})
            continue
        yaw = row.get("yaw")
        try:
            yaw = float(yaw) if yaw is not None else None
        except (TypeError, ValueError):
            yaw = None
        if not name:
            results.append({"name": name, "ok": False, "error": "名称为空"})
            continue
        try:
            results.append(await _poi_add_verified(name, x, y, yaw))
        except SlamtecError as e:
            results.append({"name": name, "ok": False, "error": str(e)})
    ok_n = sum(1 for r in results if r.get("ok"))
    bad_n = sum(1 for r in results if r.get("ok") and not r.get("verified"))
    return {"ok": True, "total": len(results), "ok_count": ok_n,
            "unverified_count": bad_n, "results": results}


@app.post("/api/chassis/move_to")
async def chassis_move_to(body: ChassisMoveToBody):
    if not STATE.chassis_obj:
        raise _err("底盘未连接", 409)
    mode = int(body.mode or 0)
    try:
        # mode 3 = 沿轨(严格) + 忽略激光动态障碍(官方 flag,沿轨直行不刹停)
        ign_dyn = mode == 3
        nav_mode = 1 if ign_dyn else mode
        action_id = await _call(STATE.chassis_obj.move_to, body.x, body.y,
                                body.yaw, body.precise, nav_mode, ign_dyn,
                                body.speed_ratio)
        return {"ok": True, "action_id": action_id}
    except SlamtecError as e:
        raise _err(str(e), 502)


@app.post("/api/chassis/joystick")
async def chassis_joystick(body: ChassisJoystickBody):
    if not STATE.chassis_obj:
        raise _err("底盘未连接", 409)
    try:
        action_id = await _call(STATE.chassis_obj.move_by,
                                body.direction, body.theta, body.duration_ms)
        return {"ok": True, "action_id": action_id}
    except SlamtecError as e:
        raise _err(str(e), 502)


@app.post("/api/chassis/cancel")
async def chassis_cancel():
    if not STATE.chassis_obj:
        return {"ok": True}
    try:
        await _call(STATE.chassis_obj.cancel_action)
    except SlamtecError:
        pass
    return {"ok": True}


@app.post("/api/chassis/rotate_to")
async def chassis_rotate_to(body: ChassisRotateBody):
    if not STATE.chassis_obj:
        raise _err("底盘未连接", 409)
    try:
        action_id = await _call(STATE.chassis_obj.rotate_to, body.angle_rad)
        return {"ok": True, "action_id": action_id}
    except SlamtecError as e:
        raise _err(str(e), 502)


@app.post("/api/chassis/go_home")
async def chassis_go_home(body: ChassisGoHomeBody):
    if not STATE.chassis_obj:
        raise _err("底盘未连接", 409)
    try:
        action_id = await _call(STATE.chassis_obj.go_home, body.dock)
        return {"ok": True, "action_id": action_id}
    except SlamtecError as e:
        raise _err(str(e), 502)


@app.post("/api/chassis/set_pose")
async def chassis_set_pose(body: ChassisSetPoseBody):
    """手动重定位:告诉底盘"我现在在地图的 (x, y, yaw)"。"""
    if not STATE.chassis_obj:
        raise _err("底盘未连接", 409)
    try:
        yaw = body.yaw
        if yaw is None:
            yaw = (await _call(STATE.chassis_service.pose) or {}).get("yaw", 0.0)
        await _call(STATE.chassis_obj.set_pose, body.x, body.y, yaw)
        return {"ok": True, "x": body.x, "y": body.y, "yaw": yaw}
    except SlamtecError as e:
        raise _err(str(e), 502)


@app.post("/api/chassis/relocalize")
async def chassis_relocalize():
    """自动重定位:底盘原地旋转重建定位(会动,慎用)。"""
    if not STATE.chassis_obj:
        raise _err("底盘未连接", 409)
    try:
        action_id = await _call(STATE.chassis_obj.relocalize)
        return {"ok": True, "action_id": action_id}
    except SlamtecError as e:
        raise _err(str(e), 502)


@app.post("/api/chassis/map_save")
async def chassis_map_save():
    """当前运行时地图持久化为设备内置 stcm。"""
    if not STATE.chassis_obj:
        raise _err("底盘未连接", 409)
    try:
        await _call(STATE.chassis_obj.save_map)
        return {"ok": True}
    except SlamtecError as e:
        raise _err(str(e), 502)


@app.post("/api/chassis/map_reload")
async def chassis_map_reload():
    """用设备内置 stcm 覆盖当前运行时地图。"""
    if not STATE.chassis_obj:
        raise _err("底盘未连接", 409)
    try:
        await _call(STATE.chassis_obj.reload_map)
        if STATE.chassis_service:
            STATE.chassis_service.grid_map(refresh=True)
        return {"ok": True}
    except SlamtecError as e:
        raise _err(str(e), 502)


@app.get("/api/chassis/power")
async def chassis_power():
    """电量/充电状态。"""
    if not STATE.chassis_obj:
        raise _err("底盘未连接", 409)
    try:
        return await _call(STATE.chassis_obj.get_power_status)
    except SlamtecError as e:
        raise _err(str(e), 502)


@app.get("/api/chassis/health")
async def chassis_health():
    """底盘健康状态。"""
    if not STATE.chassis_obj:
        raise _err("底盘未连接", 409)
    try:
        return await _call(STATE.chassis_obj.get_health)
    except SlamtecError as e:
        raise _err(str(e), 502)


@app.get("/api/chassis/speed")
async def chassis_speed():
    """当前速度 {x, y, omega}。"""
    if not STATE.chassis_obj:
        raise _err("底盘未连接", 409)
    try:
        return await _call(STATE.chassis_obj.get_speed)
    except SlamtecError as e:
        raise _err(str(e), 502)


@app.get("/api/chassis/max_speed")
async def chassis_max_speed_get():
    """导航最大速度(base.max_moving_speed,m/s)。"""
    if not STATE.chassis_obj:
        raise _err("底盘未连接", 409)
    try:
        v = await _call(STATE.chassis_obj.get_system_parameter,
                        "base.max_moving_speed")
        return {"max_moving_speed": float(v)}
    except (SlamtecError, TypeError, ValueError) as e:
        raise _err(f"读取速度参数失败: {e}", 502)


@app.post("/api/chassis/max_speed")
async def chassis_max_speed_set(body: ChassisMaxSpeedBody):
    """设置导航最大速度(即时生效,影响点导航/沿轨/巡逻;读回校验)。"""
    if not STATE.chassis_obj:
        raise _err("底盘未连接", 409)
    if not 0.05 <= body.max_moving_speed <= 3.0:
        raise _err("速度范围 0.05~3.0 m/s")
    try:
        await _call(STATE.chassis_obj.set_system_parameter,
                    "base.max_moving_speed", body.max_moving_speed)
        v = await _call(STATE.chassis_obj.get_system_parameter,
                        "base.max_moving_speed")
        return {"ok": True, "max_moving_speed": float(v)}
    except (SlamtecError, TypeError, ValueError) as e:
        raise _err(f"设置速度失败: {e}", 502)


@app.get("/api/chassis/homedocks")
async def chassis_homedocks():
    """充电桩位置列表。"""
    if not STATE.chassis_obj:
        raise _err("底盘未连接", 409)
    try:
        return {"docks": await _call(STATE.chassis_obj.get_homedocks)}
    except SlamtecError as e:
        raise _err(str(e), 502)


@app.get("/api/chassis/action_current")
async def chassis_action_current():
    """当前运动行为;空闲返回 {"action": null}。"""
    if not STATE.chassis_obj:
        raise _err("底盘未连接", 409)
    try:
        return {"action": await _call(STATE.chassis_obj.get_current_action)}
    except SlamtecError as e:
        raise _err(str(e), 502)


@app.get("/api/chassis/action/{action_id}")
async def chassis_action(action_id: int):
    """按 id 查询运动行为状态(404 → {"action": null})。"""
    if not STATE.chassis_obj:
        raise _err("底盘未连接", 409)
    try:
        return {"action": await _call(STATE.chassis_obj.get_action, action_id)}
    except SlamtecError as e:
        if "HTTP 404" in str(e):
            return {"action": None}
        raise _err(str(e), 502)


@app.post("/api/chassis/mapping")
async def chassis_mapping(body: ChassisMappingBody):
    """开/关建图模式(开启后移动会持续更新运行时地图,建议配合保存 stcm)。"""
    if not STATE.chassis_obj:
        raise _err("底盘未连接", 409)
    try:
        await _call(STATE.chassis_obj.set_mapping_enabled, body.enable)
        if STATE.chassis_service:
            STATE.chassis_service.grid_map(refresh=True)
        return {"ok": True, "enable": body.enable}
    except SlamtecError as e:
        raise _err(str(e), 502)


# ---------------------------------------------------------------------------
# 网页自建轨道(RoboStudio 的轨道不开放 REST,这里存本地 JSON)
TRACKS_PATH = Path(__file__).resolve().parent / "chassis_tracks.json"


@app.get("/api/chassis/tracks")
async def chassis_tracks_get():
    if TRACKS_PATH.exists():
        try:
            return json.loads(TRACKS_PATH.read_text("utf-8"))
        except (ValueError, OSError):
            pass
    return {"tracks": []}


@app.post("/api/chassis/tracks")
async def chassis_tracks_save(body: ChassisTracksBody):
    """整表保存(前端编辑后回传全部轨道)。"""
    for t in body.tracks:
        if not t.get("name") or not isinstance(t.get("points"), list):
            raise _err("轨道格式: [{name, points: [{x, y, yaw?}]}]", 400)
    try:
        TRACKS_PATH.write_text(
            json.dumps({"tracks": body.tracks}, ensure_ascii=False, indent=2),
            "utf-8")
    except OSError as e:
        raise _err(f"轨道保存失败: {e}", 500)
    return {"ok": True, "count": len(body.tracks)}


# ===========================================================================
# 巡逻队列(循环跑点)—— 后端驱动,浏览器关掉也继续;每段耗时落 CSV
# ===========================================================================
PATROL_LOG_DIR = Path(__file__).resolve().parent / "logs" / "chassis_patrol"
PATROL: dict = {"st": None, "task": None}


class ChassisPatrolBody(BaseModel):
    points: list[dict]           # [{name, x, y, yaw?}] 按顺序循环
    rounds: int = 0              # 0 = 无限循环
    round_pause_s: float = 0.0   # 每轮结束原处等待
    leg_timeout_s: float = 300.0  # 单段(去一个点)超时
    nav_mode: int = 0            # 0 自动规划 / 1 沿轨停等 / 2 沿轨绕行 / 3 沿轨+忽略动态障碍
    speed_ratio: Optional[float] = None  # 0.1~1,每段导航速度比例(None=不限制)


_PATROL_COLS = ["时间", "轮次", "序号", "点位", "X", "Y", "Yaw",
                "出发", "到达", "耗时s", "结果", "备注"]


def _patrol_log_write(log_path: Path, row: dict) -> None:
    # BOM 只在新建文件时写一次(utf-8-sig 追加模式会把 BOM 插到文件中间)
    newfile = not log_path.exists()
    with log_path.open("a", encoding="utf-8", newline="") as f:
        if newfile:
            f.write("﻿")
        w = csv.DictWriter(f, fieldnames=_PATROL_COLS)
        if newfile:
            w.writeheader()
        w.writerow({c: row.get(c, "") for c in _PATROL_COLS})


async def _patrol_wait_action(action_id: int, st: dict) -> tuple[bool, str]:
    """轮询 action 到结束。stop 置位/超时 → 取消当前动作并返回 False。"""
    ch = STATE.chassis_obj
    t0 = time.time()
    while not st["stop"]:
        if time.time() - t0 > st["leg_timeout_s"]:
            try:
                await _call(ch.cancel_action)
            except SlamtecError:
                pass
            return False, "超时"
        await asyncio.sleep(0.5)
        try:
            a = await _call(ch.get_action, action_id)
        except SlamtecError:
            return True, ""                      # 查不到 = 已结束被清理
        s = (a.get("state") or {}).get("status", a.get("status"))
        if isinstance(s, (int, float)) and s <= 3:
            continue                              # 0新建/1执行/3暂停
        if isinstance(s, str) and any(k in s.lower()
                                      for k in ("run", "wait", "new",
                                                "start", "pend")):
            continue
        return True, ""
    try:
        await _call(ch.cancel_action)
    except SlamtecError:
        pass
    return False, "停止"


async def _patrol_leg(ch, st: dict, pt: dict) -> tuple[bool, str, str]:
    """走一"腿":带 path(轨道密点)则逐点链走(中间点不精停,终点精停+朝向),
    否则单次 move_to。nav_mode>0 时按底盘虚拟轨道走(单次 move_to 带 mode)。
    返回 (ok, reason, note)。"""
    nav_mode = int(st.get("nav_mode") or 0)
    pt_over = pt.get("nav_mode") is not None
    if pt_over:                        # 点级覆盖:这一腿单独用该模式(如 0=自动导航)
        nav_mode = int(pt["nav_mode"])
    if nav_mode:
        # 3 = 沿轨(严格) + 忽略激光动态障碍(官方 flag,搜路失败时忽略动障重搜)
        ign_dyn = nav_mode == 3
        try:
            aid = await _call(ch.move_to, pt["x"], pt["y"],
                              pt.get("yaw"), True, 1 if ign_dyn else nav_mode,
                              ign_dyn, st.get("speed_ratio"))
        except SlamtecError as e:
            return False, f"失败({e})", "沿轨(底盘)"
        ok, reason = await _patrol_wait_action(aid, st)
        return ok, reason, "沿轨(底盘)"
    path = pt.get("path") or []
    note = "自动导航(此点单独)" if pt_over else ""
    if path:
        note = f"沿轨 {len(path)} 步"
        for j, sub in enumerate(path):
            if st["stop"]:
                return False, "停止", note
            last = (j == len(path) - 1)
            yaw = sub.get("yaw") if last else None   # 仅终点带朝向
            try:
                aid = await _call(ch.move_to, float(sub["x"]), float(sub["y"]),
                                  yaw, last)
            except SlamtecError as e:
                return False, f"失败({e})", note
            ok, reason = await _patrol_wait_action(aid, st)
            if not ok:
                return ok, reason, note
        return True, "", note
    try:
        aid = await _call(ch.move_to, pt["x"], pt["y"], pt.get("yaw"),
                          True, 0, False, st.get("speed_ratio"))
    except SlamtecError as e:
        return False, f"失败({e})", note
    ok, reason = await _patrol_wait_action(aid, st)
    return ok, reason, note


async def _patrol_task(st: dict) -> None:
    ch = STATE.chassis_obj
    log_path = st["log_path"]
    rnd = 0
    try:
        while not st["stop"] and (st["rounds"] == 0 or rnd < st["rounds"]):
            rnd += 1
            st["cur_round"] = rnd
            round_t0 = time.time()
            for i, pt in enumerate(st["points"]):
                if st["stop"]:
                    break
                st["cur_idx"] = i + 1
                st["cur_pt"] = pt["name"]
                depart = time.time()
                st["cur_leg_start"] = depart
                ok, reason, note = await _patrol_leg(ch, st, pt)
                arrive = time.time()
                result = "ok" if ok else (reason or "停止")
                row = _patrol_row(rnd, i + 1, pt, depart, arrive, result, note)
                _patrol_log_write(log_path, row)
                st["recent"].append(row)
                if not ok:
                    st["last_error"] = f"第 {rnd} 轮第 {i + 1} 点 {result}"
                    return
            row = _patrol_row(rnd, 0, {"name": "—轮结束—", "x": "", "y": "",
                                       "yaw": None},
                              round_t0, time.time(), "ok", "")
            row["耗时s"] = round(time.time() - round_t0, 1)
            _patrol_log_write(log_path, row)
            st["recent"].append(row)
            st["rounds_done"] = rnd
            if st["rounds"] == 0 or rnd < st["rounds"]:
                t0 = time.time()
                while (not st["stop"] and st["round_pause_s"] > 0
                       and time.time() - t0 < st["round_pause_s"]):
                    await asyncio.sleep(0.2)
    finally:
        st["running"] = False
        st["cur_leg_start"] = None
        try:
            if st["stop"]:
                await _call(ch.cancel_action)
        except SlamtecError:
            pass


def _patrol_row(rnd: int, idx: int, pt: dict, depart: float, arrive: float,
                result: str, note: str) -> dict:
    return {"时间": time.strftime("%H:%M:%S"),
            "轮次": rnd, "序号": idx, "点位": pt.get("name", ""),
            "X": pt.get("x", ""), "Y": pt.get("y", ""),
            "Yaw": pt.get("yaw") if pt.get("yaw") is not None else "",
            "出发": time.strftime("%H:%M:%S", time.localtime(depart)),
            "到达": time.strftime("%H:%M:%S", time.localtime(arrive)),
            "耗时s": round(arrive - depart, 1), "结果": result, "备注": note}


def _patrol_public(st: dict | None) -> dict:
    if not st:
        return {"running": False}
    return {"running": st["running"], "stop": st["stop"],
            "cur_round": st["cur_round"], "cur_idx": st["cur_idx"],
            "cur_pt": st["cur_pt"],
            "cur_leg_elapsed_s": (round(time.time() - st["cur_leg_start"], 1)
                                  if st.get("cur_leg_start") else None),
            "rounds": st["rounds"], "rounds_done": st.get("rounds_done", 0),
            "points_n": len(st["points"]), "log_file": st["log_path"].name,
            "nav_mode": st.get("nav_mode", 0),
            "speed_ratio": st.get("speed_ratio"),
            "last_error": st.get("last_error", ""),
            "recent": st["recent"][-50:]}


@app.post("/api/chassis/patrol/start")
async def chassis_patrol_start(body: ChassisPatrolBody):
    if not STATE.chassis_obj:
        raise _err("底盘未连接", 409)
    if PATROL["task"] and not PATROL["task"].done():
        raise _err("巡逻已在运行,先停止", 409)
    pts = []
    for i, p in enumerate(body.points):
        try:
            pt = {"name": str(p.get("name") or f"点{i+1}").strip() or f"点{i+1}",
                  "x": float(p["x"]), "y": float(p["y"]),
                  "yaw": float(p["yaw"]) if p.get("yaw") is not None else None}
        except (KeyError, TypeError, ValueError):
            raise _err(f"第 {i+1} 个点位数据不合法(需要 name/x/y)")
        # 可选:参照轨道的密点路径(前端由轨道折线展开)
        raw_path = p.get("path")
        if raw_path:
            if not isinstance(raw_path, list) or len(raw_path) > 500:
                raise _err(f"第 {i+1} 个点的 path 不合法(须为列表且 ≤500 点)")
            path = []
            try:
                for sub in raw_path:
                    path.append({"x": float(sub["x"]), "y": float(sub["y"]),
                                 "yaw": (float(sub["yaw"])
                                         if sub.get("yaw") is not None else None)})
            except (KeyError, TypeError, ValueError):
                raise _err(f"第 {i+1} 个点的 path 点数据不合法")
            pt["path"] = path
        pts.append(pt)
    if not pts:
        raise _err("队列为空")
    if body.rounds < 0:
        raise _err("轮数不能为负(0=无限)")
    PATROL_LOG_DIR.mkdir(parents=True, exist_ok=True)
    st = {"running": True, "stop": False, "points": pts,
          "nav_mode": min(3, max(0, int(body.nav_mode))),
                "speed_ratio": (None if body.speed_ratio is None
                                else max(0.1, min(1.0, float(body.speed_ratio)))),
          "rounds": int(body.rounds),
          "round_pause_s": max(0.0, float(body.round_pause_s)),
          "leg_timeout_s": min(3600.0, max(10.0, float(body.leg_timeout_s))),
          "cur_round": 0, "cur_idx": 0, "cur_pt": "", "cur_leg_start": None,
          "log_path": PATROL_LOG_DIR / time.strftime("patrol_%Y%m%d_%H%M%S.csv"),
          "recent": [], "rounds_done": 0, "last_error": ""}
    PATROL["st"] = st
    PATROL["task"] = asyncio.create_task(_patrol_task(st))
    return {"ok": True, "points": len(pts), "rounds": st["rounds"],
            "log_file": st["log_path"].name}


@app.post("/api/chassis/patrol/stop")
async def chassis_patrol_stop():
    st = PATROL.get("st")
    if st and st["running"]:
        st["stop"] = True
        if STATE.chassis_obj:
            try:
                await _call(STATE.chassis_obj.cancel_action)
            except SlamtecError:
                pass
    return {"ok": True}


@app.get("/api/chassis/patrol/status")
async def chassis_patrol_status():
    return _patrol_public(PATROL.get("st"))


@app.get("/api/chassis/patrol/logs")
async def chassis_patrol_logs():
    files = []
    if PATROL_LOG_DIR.exists():
        for f in sorted(PATROL_LOG_DIR.glob("patrol_*.csv"), reverse=True):
            files.append({"name": f.name,
                          "size": f.stat().st_size,
                          "mtime": time.strftime("%m-%d %H:%M",
                                                 time.localtime(f.stat().st_mtime))})
    return {"files": files}


@app.get("/api/chassis/patrol/logs/download")
async def chassis_patrol_log_download(file: str):
    name = Path(file).name
    if (name != file or not name.startswith("patrol_")
            or not name.endswith(".csv")):
        raise _err("文件名不合法", 400)
    p = PATROL_LOG_DIR / name
    if not p.exists():
        raise _err("日志不存在", 404)
    return FileResponse(p, media_type="text/csv; charset=utf-8",
                        filename=name)


@app.get("/api/chassis/patrol/logs/export_all")
async def chassis_patrol_logs_export_all():
    """全部巡逻日志合并导出 CSV(首列=来源文件,Excel 直开)。"""
    from io import StringIO
    merged = []
    for f in sorted(PATROL_LOG_DIR.glob("patrol_*.csv")):
        try:
            with f.open("r", encoding="utf-8-sig", newline="") as fh:
                rd = csv.reader(fh)
                header = None
                for row in rd:
                    if header is None:
                        header = row          # 跳过各文件自己的表头
                        continue
                    merged.append([f.name] + row)
        except OSError:
            continue
    buf = StringIO()
    w = csv.writer(buf)
    w.writerow(["文件"] + _PATROL_COLS)
    w.writerows(merged)
    return Response(content="﻿" + buf.getvalue(),
                    media_type="text/csv; charset=utf-8",
                    headers={"Content-Disposition":
                             'attachment; filename="patrol_all.csv"'})


# ===========================================================================
# 点位复现
# ===========================================================================
# ---------------------------------------------------------------------------
# 点位复现:机器人 :8000 的 location 服务(fd_arm_server)间歇性无响应时,
# 直读本机点位 yaml 兜底(fd_arm_server 与本服务同机)。保存/删除仍必须走
# 机器人服务(它内存里有一份快照,直写文件会被下次保存整写覆盖)。
POSE_YAML_PATH = os.environ.get(
    "POSE_YAML_PATH",
    "/home/user/fd_ws/src/standard/fd_arm/fd_arm_api/data/arm_location.yaml",
)


def _pose_yaml_all() -> dict:
    """读点位 yaml → {arm: {name: [关节角deg]}};不可读时返回 {}。"""
    try:
        with open(POSE_YAML_PATH, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        return {a: dict(pts or {}) for a, pts in data.items()}
    except Exception:
        return {}


@app.get("/api/pose/list")
async def pose_list(arm: str = Query(...)):
    if not STATE.robot:
        raise _err("机器人未连接", 409)
    arm_n = _normalize_arm(arm)
    try:
        names = await _call(STATE.robot.arm_locations, arm_n)
        return {"arm": arm_n, "names": names, "source": "robot"}
    except RobotError as e:
        pts = _pose_yaml_all().get(arm_n) or {}
        if not pts:
            raise _err(f"获取点位失败(机器人服务无响应,本机点位文件也没有该臂数据): {e}", 502)
        return {"arm": arm_n, "names": list(pts), "source": "file",
                "warning": "机器人点位服务无响应,列表来自本机点位文件;保存/删除需服务恢复后再用"}


@app.post("/api/pose/save")
async def pose_save(body: PoseSaveBody):
    if not STATE.robot:
        raise _err("机器人未连接", 409)
    if not body.name.strip():
        raise _err("name 不能为空")
    arm_n = _normalize_arm(body.arm)
    try:
        return await _call(STATE.robot.save_arm_location, arm_n, body.name.strip())
    except RobotError as e:
        raise _err(str(e), 502)


@app.post("/api/pose/goto")
async def pose_goto(body: PoseGotoBody):
    if not STATE.robot:
        raise _err("机器人未连接", 409)
    arm_n = _normalize_arm(body.arm)
    name = body.name.strip()
    speed = _speed_of(arm_n, body.speed)
    try:
        return await _call(STATE.robot.move_to_location, arm_n, name,
                           body.timeout, speed)
    except RobotError as e:
        # 厂商 move_to_location 无响应 → 从本机点位文件取角度,改走轨迹接口
        q = (_pose_yaml_all().get(arm_n) or {}).get(name)
        if not q or len(q) < 7:
            raise _err(f"直达失败,且本机点位文件无该点位: {e}", 502)
        r = await _call(STATE.robot.move_robotarm_traj, arm_n, [q],
                        1, body.timeout, speed)
        r.setdefault("message", "")
        r["fallback"] = "move_to_location 服务无响应,已改用本机点位角度 + 轨迹运动"
        return r


@app.post("/api/pose/delete")
async def pose_delete(body: PoseDeleteBody):
    if not STATE.robot:
        raise _err("机器人未连接", 409)
    arm_n = _normalize_arm(body.arm)
    try:
        return await _call(STATE.robot.delete_arm_location, arm_n, body.name)
    except RobotError as e:
        raise _err(str(e), 502)


# ===========================================================================
# 笛卡尔移动
# ===========================================================================
def _q_deg(arm: str) -> list:
    """读取指定臂当前关节角(7 维,度)。"""
    if not STATE.robot:
        raise _err("机器人未连接", 409)
    try:
        data = STATE.robot.get_joint_angles()
    except RobotError as e:
        raise _err(str(e), 502)
    if not isinstance(data, dict) or arm not in data:
        raise _err(f"关节数据中找不到 {arm}", 502)
    return list(data[arm])


def _end_pose(arm: str) -> dict:
    """读取指定臂当前末端位姿(position / orientation 四元数 / rpy)。"""
    if not STATE.robot:
        raise _err("机器人未连接", 409)
    try:
        data = STATE.robot.get_end_poses()
    except RobotError as e:
        raise _err(str(e), 502)
    if not isinstance(data, dict) or arm not in data:
        raise _err(f"末端数据中找不到 {arm}", 502)
    return data[arm]


@app.get("/api/cartesian/current")
async def cartesian_current(arm: str = Query("armleft")):
    arm_n = _normalize_arm(arm)
    return _end_pose(arm_n)


@app.post("/api/cartesian/jog")
async def cartesian_jog(body: CartJogBody):
    if not STATE.robot:
        raise _err("机器人未连接", 409)
    arm_n = _normalize_arm(body.arm)
    if body.axis not in ("x", "y", "z"):
        raise _err("axis 必须是 x/y/z")
    if body.frame not in ("base", "tool"):
        raise _err("frame 必须是 base/tool")
    deltas = {"x": 0.0, "y": 0.0, "z": 0.0}
    deltas[body.axis] = body.distance
    try:
        await _call(cartesian_delta_move, STATE.robot, arm_n,
                    deltas["x"], deltas["y"], deltas["z"],
                    speed=_speed_of(arm_n, body.speed), step_m=body.step_m,
                    frame=body.frame)
    except RobotError as e:
        if _is_preempted(str(e)):
            return {"ok": True, "preempted": True}
        if "near-zero trajectory" in str(e):
            raise _err("步距过小:厂商接口要求单次平移至少引起 0.5° 关节变化"
                       "(换算成末端位移随机位姿约 0.5~1mm);建议步距 ≥1mm,"
                       "更精细的对准请改用目标点移动", 400)
        raise _err(str(e), 502)
    return {"ok": True}


@app.post("/api/cartesian/rotate")
async def cartesian_rotate(body: CartRotateBody):
    arm_n = _normalize_arm(body.arm)
    if body.axis not in ("rx", "ry", "rz"):
        raise _err("axis 必须是 rx/ry/rz")
    if body.frame not in ("base", "tool"):
        raise _err("frame 必须是 base/tool")
    q = _q_deg(arm_n)
    try:
        out = await _call(plan_cartesian_path, arm_n, q,
                          op="rotate", axis=body.axis,
                          angle_deg=body.angle_deg, frame=body.frame)
    except RobotError as e:
        if getattr(e, "planner_unavailable", False):
            raise _err(str(e), 503)
        raise _err(str(e), 502)
    traj = out.get("waypoints_deg")
    if not traj:
        raise _err("逆解未返回轨迹", 502)
    try:
        await _call(STATE.robot.move_robotarm_traj, arm_n, traj, 1, 60.0,
                    _speed_of(arm_n, body.speed))
    except RobotError as e:
        if _is_preempted(str(e)):
            return {"ok": True, "preempted": True, "n_points": len(traj)}
        raise _err(str(e), 502)
    return {"ok": True, "n_points": len(traj)}


@app.post("/api/cartesian/move_to")
async def cartesian_move_to(body: CartMoveToBody):
    arm_n = _normalize_arm(body.arm)
    if len(body.pos) != 3 or len(body.rpy) != 3:
        raise _err("pos 和 rpy 必须是 3 元数组")
    cur = _end_pose(arm_n)
    cur_pos = cur.get("position") or cur.get("pos")
    cur_quat = cur.get("orientation") or cur.get("quat")
    if not (isinstance(cur_pos, (list, tuple)) and isinstance(cur_quat, (list, tuple))):
        raise _err("当前末端位姿数据不完整", 502)
    q = _q_deg(arm_n)
    try:
        out = await _call(plan_cartesian_path, arm_n, q,
                          op="target", target_pos_m=list(body.pos),
                          target_rpy_deg=list(body.rpy),
                          cur_pos_m=list(cur_pos), cur_quat=list(cur_quat))
    except RobotError as e:
        if getattr(e, "planner_unavailable", False):
            raise _err(str(e), 503)
        raise _err(str(e), 502)
    traj = out.get("waypoints_deg")
    if not traj:
        raise _err("逆解未返回轨迹", 502)
    try:
        await _call(STATE.robot.move_robotarm_traj, arm_n, traj, 1,
                    body.timeout, _speed_of(arm_n, body.speed))
    except RobotError as e:
        if _is_preempted(str(e)):
            return {"ok": True, "preempted": True, "n_points": len(traj)}
        raise _err(str(e), 502)
    return {"ok": True, "n_points": len(traj)}


@app.post("/api/gripper")
async def gripper(body: GripperBody):
    if not STATE.robot:
        raise _err("机器人未连接", 409)
    side = _normalize_arm(body.side)
    try:
        return await _call(STATE.robot.set_gripper, side,
                           body.position, body.speed)
    except RobotError as e:
        raise _err(str(e), 502)


@app.get("/api/effector")
async def effector_status():
    """末端执行器探测:各臂 type(gripper/hand/null=未检测到)。

    末端由底层驱动上报(:8000 只转发),null 说明该臂没挂/没上电/线缆断,
    夹爪按钮会报"未检测到末端执行器"——这个接口让 UI 提前说清楚。"""
    if not STATE.robot:
        raise _err("机器人未连接", 409)
    try:
        states = await _call(STATE.robot.get_manipulator_states)
    except RobotError as e:
        raise _err(str(e), 502)
    out = {}
    for arm_n, side in (("armleft", "left"), ("armright", "right")):
        info = states.get(side) or {}
        item = {"type": info.get("type")}
        if info.get("type") == "gripper":
            item["open"] = (info.get("open") or {}).get("position")
        out[arm_n] = item
    return out


# ===========================================================================
# 示教(joint / cartesian)保存 / 加载 / 列出 / 删除 / 回放
# ===========================================================================
def _teach_path(kind: str, name: str) -> Path:
    safe = "".join(c for c in name if c not in r'\/:*?"<>|')
    return TEACH_DIR / f"{kind}__{safe}.json"


@app.get("/api/teach/{kind}/list")
async def teach_list(kind: str):
    if kind not in ("joint_teach", "cartesian_teach"):
        raise _err("kind 必须是 joint_teach/cartesian_teach")
    TEACH_DIR.mkdir(parents=True, exist_ok=True)
    items = []
    for p in sorted(TEACH_DIR.glob(f"{kind}__*.json")):
        try:
            t = TeachFile.load(p)
            items.append({
                "name": t.name,
                "file": p.name,
                "arm": t.arm,
                "speed": t.speed,
                "created": t.created,
                "n_points": len(t.points),
            })
        except Exception:                                       # noqa: BLE001
            continue
    return {"kind": kind, "items": items}


@app.get("/api/teach/{kind}/load")
async def teach_load(kind: str, name: str = Query(...)):
    if kind not in ("joint_teach", "cartesian_teach"):
        raise _err("kind 必须是 joint_teach/cartesian_teach")
    p = _teach_path(kind, name)
    if not p.is_file():
        raise _err("工程不存在", 404)
    t = TeachFile.load(p)
    return {
        "kind": t.kind, "name": t.name, "arm": t.arm,
        "speed": t.speed, "created": t.created, "points": t.points,
    }


@app.delete("/api/teach/{kind}/delete")
async def teach_delete(kind: str, name: str = Query(...)):
    if kind not in ("joint_teach", "cartesian_teach"):
        raise _err("kind 必须是 joint_teach/cartesian_teach")
    p = _teach_path(kind, name)
    if p.is_file():
        p.unlink()
    return {"ok": True}


@app.post("/api/teach/save")
async def teach_save(body: TeachSaveBody):
    if body.kind not in ("joint_teach", "cartesian_teach"):
        raise _err("kind 必须是 joint_teach/cartesian_teach")
    if not body.name.strip():
        raise _err("name 不能为空")
    arm_n = _normalize_arm(body.arm)
    t = TeachFile(body.kind, arm_n)
    t.name = body.name.strip()
    t.speed = _speed_of(arm_n, body.speed)      # 存当前全局速度作记录
    for pt in body.points:
        if body.kind == "joint_teach":
            jd = pt.get("joints_deg")
            if not (isinstance(jd, list) and len(jd) == 7):
                continue
            t.add_joint_point(jd, pt.get("gripper"), pt.get("note", ""))
        else:
            pos = pt.get("position")
            ori = pt.get("orientation")
            if not (isinstance(pos, list) and len(pos) == 3 and
                    isinstance(ori, list) and len(ori) == 4):
                continue
            t.add_cartesian_point(pos, ori, pt.get("gripper"), pt.get("note", ""))
    p = _teach_path(body.kind, t.name)
    t.save(p)
    return {"ok": True, "path": str(p), "n_points": len(t.points)}


@app.post("/api/teach/joint/play")
async def teach_joint_play(body: TeachPlayBody):
    if not STATE.robot:
        raise _err("机器人未连接", 409)
    arm_n = _normalize_arm(body.arm)
    p = _teach_path("joint_teach", body.name)
    if not p.is_file():
        raise _err("工程不存在", 404)
    t = TeachFile.load(p)
    traj = t.joints_list()
    if not traj:
        raise _err("工程无路点", 400)
    speed = _speed_of(arm_n, body.speed)      # 全局速度优先于工程存的值
    try:
        await _call(STATE.robot.move_robotarm_traj, arm_n, traj, 1, 120.0, speed)
    except RobotError as e:
        if _is_preempted(str(e)):
            return {"ok": True, "preempted": True, "n_points": len(traj)}
        raise _err(str(e), 502)
    return {"ok": True, "n_points": len(traj)}


@app.post("/api/teach/cart/play")
async def teach_cart_play(body: TeachPlayBody):
    """笛卡尔示教回放:相邻路点位姿差值用 IK 规划为关节轨迹,逐段下发。

    资源允许的话可全段合并为一次 move_robotarm_traj,简化实现。
    """
    if not STATE.robot:
        raise _err("机器人未连接", 409)
    arm_n = _normalize_arm(body.arm)
    p = _teach_path("cartesian_teach", body.name)
    if not p.is_file():
        raise _err("工程不存在", 404)
    t = TeachFile.load(p)
    if not t.points:
        raise _err("工程无路点", 400)
    speed = _speed_of(arm_n, body.speed)      # 全局速度优先于工程存的值

    # 起点为当前末端
    cur = _end_pose(arm_n)
    cur_pos = cur.get("position") or cur.get("pos")
    cur_quat = cur.get("orientation") or cur.get("quat")
    if not (isinstance(cur_pos, (list, tuple)) and isinstance(cur_quat, (list, tuple))):
        raise _err("当前末端位姿数据不完整", 502)

    # 逐段 IK
    segs = 0
    for pt in t.points:
        pos = pt.get("position")
        ori = pt.get("orientation")
        if not (isinstance(pos, list) and len(pos) == 3 and
                isinstance(ori, list) and len(ori) == 4):
            continue
        try:
            rpy = _quat_to_rpy(ori)
        except Exception:                                       # noqa: BLE001
            rpy = [0.0, 0.0, 0.0]
        q = _q_deg(arm_n)
        try:
            out = await _call(plan_cartesian_path, arm_n, q,
                              op="target", target_pos_m=pos,
                              target_rpy_deg=rpy,
                              cur_pos_m=list(cur_pos), cur_quat=list(cur_quat))
        except RobotError as e:
            if getattr(e, "planner_unavailable", False):
                raise _err(str(e), 503)
            raise _err(str(e), 502)
        traj = out.get("waypoints_deg")
        if not traj:
            raise _err("逆解未返回轨迹", 502)
        try:
            await _call(STATE.robot.move_robotarm_traj, arm_n, traj, 1,
                        60.0, speed)
        except RobotError as e:
            raise _err(str(e), 502)
        cur_pos, cur_quat = pos, ori
        segs += 1
    return {"ok": True, "segments": segs}


def _quat_to_rpy(quat: list) -> list:
    """四元数 (x, y, z, w) → RPY 度。粗略实现,够用于回放目标姿态。"""
    import math
    x, y, z, w = quat
    # roll
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 \
            else math.asin(sinp)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return [math.degrees(roll), math.degrees(pitch), math.degrees(yaw)]


# ===========================================================================
# 物料检测(pose_service 独立子进程)
# ===========================================================================
def _stop_pose_proc() -> None:
    global _POSE_PROC
    p = _POSE_PROC
    if p and p.poll() is None:
        try:
            p.send_signal(signal.SIGTERM)
            p.wait(timeout=3)
        except Exception:                                       # noqa: BLE001
            try: p.kill()
            except Exception:                                   # noqa: BLE001
                pass
    _POSE_PROC = None


@app.get("/api/pose_service/health")
async def pose_service_health():
    import urllib.request
    try:
        with urllib.request.urlopen(f"http://127.0.0.1:{_POSE_PORT}/health",
                                    timeout=1.0) as r:
            # 端口探活优先:webapp 重启后上一代拉起的 pose_service 成了
            # 外部孤儿,只认 _POSE_PROC 会误报"未启动"(与 camera_bridge
            # 的外部复用同款处理,2026-10-21)
            external = not (_POSE_PROC is not None
                            and _POSE_PROC.poll() is None)
            return {"alive": True, "port": _POSE_PORT, "external": external,
                    "http_status": r.status}
    except Exception as e:                                      # noqa: BLE001
        return {"alive": False, "port": _POSE_PORT, "reason": str(e)}


@app.post("/api/pose_service/start")
async def pose_service_start():
    global _POSE_PROC
    if _POSE_PROC and _POSE_PROC.poll() is None:
        return {"ok": True, "already_running": True, "pid": _POSE_PROC.pid}
    import urllib.request
    try:                                    # 外部实例(孤儿)占用则直接复用
        with urllib.request.urlopen(f"http://127.0.0.1:{_POSE_PORT}/health",
                                    timeout=1.0) as r:
            if r.status == 200:
                return {"ok": True, "already_running": True,
                        "external": True, "port": _POSE_PORT}
    except Exception:                                          # noqa: BLE001
        pass
    if not Path(POSE_YOLO_ENV_PY).is_file():
        raise _err(f"找不到 yolo11_env 解释器({POSE_YOLO_ENV_PY})", 500)
    script = Path(__file__).resolve().with_name("pose_service.py")
    if not script.is_file():
        raise _err(f"pose_service.py 不存在: {script}", 500)
    weights = script.parent / "models" / "weilong_pose_best.pt"
    if not weights.is_file():
        # 兜底: 本地没拷权重时用 upper_computer 的源文件(日志会提示)
        alt = Path(os.environ.get(
            "POSE_WEIGHTS", "/home/user/tianlian/upper_computer/models/weilong_pose_best.pt"))
        weights = alt if alt.is_file() else weights
    if not weights.is_file():
        raise _err(f"找不到模型权重(找过 {script.parent / 'models'} 与 "
                   f"upper_computer/models): {weights}", 500)
    env = os.environ.copy()
    env.pop("PYTHONHOME", None)
    env.pop("PYTHONPATH", None)      # 清 ROS2 路径,避免干扰 venv
    # 只补 venv 里 torch 缺的 cusparselt;cuDNN/cuBLAS 等用系统 Jetson CUDA 栈
    # (venv 自带 cudnn 目录会覆盖系统版本 → CUDNN_STATUS_NOT_INITIALIZED, 勿加)
    extra = [f"{POSE_YOLO_ENV_SITE}/nvidia/cusparselt/lib"]
    env["LD_LIBRARY_PATH"] = ":".join(extra + [env.get("LD_LIBRARY_PATH", "")])
    log_f = open(script.parent / "logs" / "pose_service.log", "ab")
    _POSE_PROC = subprocess.Popen(
        [POSE_YOLO_ENV_PY, str(script), "--weights", str(weights)], env=env,
        stdout=log_f, stderr=subprocess.STDOUT,
    )
    return {"ok": True, "pid": _POSE_PROC.pid, "port": _POSE_PORT}


@app.post("/api/pose_service/stop")
async def pose_service_stop():
    _stop_pose_proc()
    return {"ok": True}


@app.post("/api/pose_service/detect")
async def pose_service_detect(body: PoseDetectBody):
    """取指定相机最新一帧,送 pose_service 推理,返回 JSON + annotated JPEG。"""
    if not STATE.camera:
        raise _err("机器人未连接 / 相机未启动", 409)
    data = STATE.camera.latest(body.cam)
    if not data:
        # 该相机还没开流(比如直接进本页就点检测):自动拉起,等第一帧
        STATE.camera.start(body.cam, 1000)
        for _ in range(8):
            await asyncio.sleep(0.5)
            data = STATE.camera.latest(body.cam)
            if data:
                break
    if not data:
        err = STATE.camera.last_error(body.cam)
        raise _err(f"相机 {body.cam} 暂无帧" + (f"({err})" if err else ""), 503)
    import urllib.request
    url = f"http://127.0.0.1:{_POSE_PORT}/detect?conf={body.conf}"
    req = urllib.request.Request(url, data=data,
                                 headers={"Content-Type": "application/octet-stream"},
                                 method="POST")
    try:
        with urllib.request.urlopen(req, timeout=30) as r:
            payload = json.loads(r.read())
    except Exception as e:                                      # noqa: BLE001
        raise _err(f"推理失败: {e}", 502)
    return {
        "cam": body.cam,
        "image_b64": payload.get("image_b64"),
        "detections": payload.get("detections", []),
        "image_size": payload.get("image_size"),
    }


# ===========================================================================
# 一步到位: 检测 → 左臂吸取
# ===========================================================================
class PickScanBody(BaseModel):
    replay: bool = True            # 先回放扫描位再扫(会动臂)


class PickStartBody(BaseModel):
    bag_index: Optional[int] = None    # 吸取第几袋(扫描表 #,缺省 1)
    dry_run: bool = True               # 干跑(只规划不动臂)
    use_last_scan: bool = False        # 用上次扫描结果(不再检测)
    press_mm: Optional[float] = None   # 覆盖下压(0..10)
    hold_s: Optional[float] = None     # 覆盖保持时长


class PickAlignBody(BaseModel):
    bag_order: int                     # 校正的袋号(上次扫描表 #)
    per_bag_only: bool = False         # 只写分袋残差,不动全局


def _pick_robot():
    if not STATE.robot:
        raise _err("机器人未连接", 409)
    return STATE.robot


@app.get("/api/pick/config")
async def pick_config_get():
    cfg = pick_store.load_config()
    local_w = Path(__file__).resolve().parent / "models" / "weilong_pose_best.pt"
    return {
        "config": cfg,
        "handeye": pick_store.handeye_info(),
        "weights_ok": local_w.is_file() or Path(os.environ.get(
            "POSE_WEIGHTS",
            "/home/user/tianlian/upper_computer/models/weilong_pose_best.pt"
        )).is_file(),
        "last_scan": pick_store.load_last_scan() is not None,
        "role_warning": ("" if cfg.get("cam_role") == pick_store.CALIBRATED_ROLE
                         else f"手眼标定绑定相机 '{pick_store.CALIBRATED_ROLE}',"
                              f"当前选 '{cfg.get('cam_role')}' 定位会错!"),
    }


@app.post("/api/pick/config")
async def pick_config_set(body: dict):
    try:
        cfg = pick_store.update_config(body or {})
    except ValueError as e:
        raise _err(str(e), 400)
    return {"ok": True, "config": cfg}


@app.get("/api/pick/depth/health")
async def pick_depth_health():
    return _PICK_PROVIDER.health()


@app.post("/api/pick/teach/scan_pose")
async def pick_teach_scan_pose():
    """把当前关节角+末端位姿存为扫描位(不动臂)。"""
    robot = _pick_robot()
    cfg = pick_store.load_config()
    arm = cfg.get("arm", "armleft")
    try:
        q = robot.get_joint_angles().get(arm) or []
        ep = robot.get_end_poses().get(arm) or {}
    except RobotError as e:
        raise _err(str(e), 502)
    if not q or not ep.get("position"):
        raise _err("读取当前关节/位姿失败(臂未上电?)", 502)
    cfg["scan_pose"] = {
        "joints_deg": [round(float(v), 3) for v in q],
        "end_pose": ep,
        "created": time.strftime("%Y-%m-%d %H:%M:%S"),
    }
    pick_store.save_config(cfg)
    return {"ok": True, "scan_pose": cfg["scan_pose"],
            "hint": "扫描位已更新;若之前做过对准校正,建议复验一次"}


@app.post("/api/pick/teach/align")
async def pick_teach_align(body: PickAlignBody):
    """人工微调吸嘴到正确吸附点后存残差 + 对准姿态基准。"""
    robot = _pick_robot()
    cfg = pick_store.load_config()
    arm = cfg.get("arm", "armleft")
    try:
        ep = robot.get_end_poses().get(arm) or {}
    except RobotError as e:
        raise _err(str(e), 502)
    pos, quat = ep.get("position"), ep.get("orientation")
    if not pos or not quat:
        raise _err("末端位姿读取失败", 502)
    try:
        info = compute_align_residual(cfg, body.bag_order, pos)
    except ValueError as e:
        raise _err(str(e), 400)
    if body.per_bag_only:
        cfg.setdefault("per_bag_residual_mm", {})[str(body.bag_order)] = \
            info["residual_mm"]
    else:
        cfg["residual_mm"] = info["residual_mm"]
    cfg["align"] = {
        "rpy_deg": [round(v, 2) for v in quat_to_rpy_deg(quat)],
        "z_ref_m": info["z_ref_m"],
        "bag_order": body.bag_order,
        "per_bag_only": body.per_bag_only,
        "created": time.strftime("%Y-%m-%d %H:%M:%S"),
    }
    pick_store.save_config(cfg)
    return {"ok": True, **info, "align": cfg["align"],
            "stored_as": "per_bag" if body.per_bag_only else "global"}


@app.post("/api/pick/scan")
async def pick_scan(body: PickScanBody):
    """扫描(可先回放扫描位): 检测+深度+手眼 → 袋表。"""
    robot = _pick_robot()
    if _FLOW.running:
        raise _err("双臂流水线运行中,请先停流水线", 409)
    cfg = pick_store.load_config()
    try:
        out = await _call(scan_once, robot, _PICK_PROVIDER, cfg,
                          replay=body.replay)
    except DepthError as e:
        raise _err(str(e), 503)
    except RobotError as e:
        raise _err(str(e), 502)
    except ValueError as e:
        raise _err(str(e), 400)
    return out


@app.post("/api/pick/start")
async def pick_start(body: PickStartBody):
    global _PICK_JOB
    robot = _pick_robot()
    if _FLOW.running:
        raise _err("双臂流水线运行中,请先停流水线", 409)
    with _PICK_LOCK:
        if _PICK_JOB is not None and not _PICK_JOB._done:
            raise _err("已有吸取任务在跑(先停止或等完成)", 409)
        cfg = pick_store.load_config()
        try:
            job = PickJob(robot, _PICK_PROVIDER, cfg,
                          bag_index=body.bag_index, dry_run=body.dry_run,
                          use_last_scan=body.use_last_scan,
                          press_mm=body.press_mm, hold_s=body.hold_s)
        except FileNotFoundError as e:              # 手眼文件缺失,构造即验
            raise _err(str(e), 500)
        _PICK_JOB = job
        threading.Thread(target=job.run, daemon=True,
                         name="pick-job").start()
    return {"ok": True, "job_started": True,
            "dry_run": body.dry_run, "bag_index": body.bag_index}


@app.get("/api/pick/status")
async def pick_status():
    if _PICK_JOB is None:
        return {"running": False, "phase": "idle"}
    return _PICK_JOB.status()


@app.post("/api/pick/stop")
async def pick_stop():
    if _PICK_JOB is None or _PICK_JOB._done:
        return {"ok": True, "note": "无运行中的任务"}
    _PICK_JOB.abort_event.set()
    return {"ok": True, "note": "已请求软停(在段边界生效);中途硬停用顶部 ⛔"}


# ===========================================================================
# 双臂上下料流水线
# ===========================================================================
class FlowArmBody(BaseModel):
    arm: str


class FlowArmActionBody(FlowArmBody):
    action: str


class FlowWpDelBody(FlowArmActionBody):
    index: int


class FlowWpMoveBody(FlowWpDelBody):
    offset: int                 # -1 前移 / +1 后移


class FlowCenterMarkBody(FlowWpDelBody):
    center: bool                # True=中点 / False=取消


class FlowWpWaitBody(FlowWpDelBody):
    seconds: float = 0.0        # >0=到点后等待 N 秒 / <=0=取消


class FlowVacBody(FlowArmActionBody):
    event: str = "none"          # grab / release / none
    at: str = "end"              # start / end / 路点序号


class FlowTitleBody(FlowArmActionBody):
    title: str


class FlowAlignBody(BaseModel):
    arm: str
    which: str = "align"         # align=中间件 / align2=料仓(仅右臂)


class FlowStartBody(BaseModel):
    step_mode: bool = False
    arms: str | None = None      # None=双臂连续 / "left"|"right"=单臂测试1循环
    rounds: int | None = None    # None/0=连续不限轮 / N=每臂跑满 N 循环自动停


def _flow_arm(s: str) -> str:
    try:
        return _normalize_arm(s)
    except HTTPException as e:
        raise _err(f"{e.detail}(armleft/armright)", 400) from None


def _flow_guard_moving():
    if _FLOW.running:
        raise _err("流水线运行中,不能示教/试跑(先停流水线)", 409)


@app.get("/api/flow/config")
async def flow_config_get():
    cfg = flow_store.load_config()
    return {
        "config": cfg,
        "readiness": flow_store.readiness(cfg),
        "handeye": {a: flow_store.handeye_info(a) for a in flow_store.ARM_KEYS},
        "actions_meta": flow_store.ACTIONS_META,
        "post_grip_slots": flow_store.POST_GRIP_SLOTS,
        "pick_scan_pose": bool(pick_store.load_config().get("scan_pose")),
    }


@app.post("/api/flow/config")
async def flow_config_set(body: dict):
    try:
        cfg = flow_store.update_config(body or {})
    except ValueError as e:
        raise _err(str(e), 400)
    return {"ok": True, "config": cfg,
            "readiness": flow_store.readiness(cfg)}


@app.post("/api/flow/teach/waypoint")
async def flow_teach_waypoint(body: FlowArmActionBody):
    """把当前关节角+末端位姿追加为该动作的下一个路点(不动臂)。"""
    _flow_guard_moving()
    robot = _pick_robot()
    arm = _flow_arm(body.arm)
    try:
        q = robot.get_joint_angles().get(arm) or []
        ep = robot.get_end_poses().get(arm) or {}
    except RobotError as e:
        raise _err(str(e), 502)
    pos, quat = ep.get("position"), ep.get("orientation")
    if not q or not pos or not quat:
        raise _err("读取当前关节/位姿失败(臂未上电?)", 502)
    try:
        wp = flow_store.record_waypoint(arm, body.action, q,
                                        {"position": [float(v) for v in pos],
                                         "orientation": [float(v)
                                                         for v in quat]})
    except ValueError as e:
        raise _err(str(e), 400)
    return {"ok": True, "waypoint": wp,
            "count": len(flow_store.load_config()["arms"][arm]
                         ["actions"][body.action]["waypoints"])}


@app.post("/api/flow/teach/waypoint/delete")
async def flow_teach_waypoint_del(body: FlowWpDelBody):
    try:
        flow_store.delete_waypoint(_flow_arm(body.arm), body.action,
                                   body.index)
    except ValueError as e:
        raise _err(str(e), 400)
    return {"ok": True}


@app.post("/api/flow/teach/waypoint/overwrite")
async def flow_teach_waypoint_overwrite(body: FlowWpDelBody):
    """把当前关节角+末端位姿覆盖到第 index 个路点(不动臂,其余点不动)。"""
    _flow_guard_moving()
    robot = _pick_robot()
    arm = _flow_arm(body.arm)
    try:
        q = robot.get_joint_angles().get(arm) or []
        ep = robot.get_end_poses().get(arm) or {}
    except RobotError as e:
        raise _err(str(e), 502)
    pos, quat = ep.get("position"), ep.get("orientation")
    if not q or not pos or not quat:
        raise _err("读取当前关节/位姿失败(臂未上电?)", 502)
    try:
        wp = flow_store.overwrite_waypoint(
            arm, body.action, body.index, q,
            {"position": [float(v) for v in pos],
             "orientation": [float(v) for v in quat]})
    except ValueError as e:
        raise _err(str(e), 400)
    return {"ok": True, "waypoint": wp}


@app.post("/api/flow/teach/waypoint/move")
async def flow_teach_waypoint_move(body: FlowWpMoveBody):
    """第 index 个路点与相邻路点交换位置(offset=-1/+1)。"""
    _flow_guard_moving()
    try:
        flow_store.move_waypoint(_flow_arm(body.arm), body.action,
                                 body.index, body.offset)
    except ValueError as e:
        raise _err(str(e), 400)
    return {"ok": True}


@app.post("/api/flow/teach/center_mark")
async def flow_teach_center_mark(body: FlowCenterMarkBody):
    """设置/取消路点的"中点"标记(双臂中心互斥按路点细化,不动臂)。"""
    _flow_guard_moving()
    try:
        wp = flow_store.mark_center(_flow_arm(body.arm), body.action,
                                    body.index, body.center)
    except ValueError as e:
        raise _err(str(e), 400)
    return {"ok": True, "waypoint": wp,
            "center": bool(wp.get("center"))}


@app.post("/api/flow/teach/waypoint/wait")
async def flow_teach_waypoint_wait(body: FlowWpWaitBody):
    """设置/取消路点的"到点等待"(回放到该点后停留 N 秒,不动臂)。"""
    _flow_guard_moving()
    try:
        wp = flow_store.set_waypoint_wait(_flow_arm(body.arm), body.action,
                                          body.index, body.seconds)
    except ValueError as e:
        raise _err(str(e), 400)
    return {"ok": True, "waypoint": wp, "wait_s": wp.get("wait_s")}


@app.post("/api/flow/teach/clear")
async def flow_teach_clear(body: FlowArmActionBody):
    try:
        flow_store.clear_action(_flow_arm(body.arm), body.action)
    except ValueError as e:
        raise _err(str(e), 400)
    return {"ok": True}


@app.post("/api/flow/action/vacuum")
async def flow_action_vacuum(body: FlowVacBody):
    try:
        a = flow_store.set_action_vacuum(_flow_arm(body.arm), body.action,
                                         body.event, body.at)
    except ValueError as e:
        raise _err(str(e), 400)
    return {"ok": True, "vacuum": a.get("vacuum")}


@app.post("/api/flow/action/title")
async def flow_action_title(body: FlowTitleBody):
    """改动作槽位的步骤说明(A几/B几 后面那行提示)。"""
    try:
        t = flow_store.set_action_title(_flow_arm(body.arm), body.action,
                                        body.title)
    except ValueError as e:
        raise _err(str(e), 400)
    return {"ok": True, "title": t}


@app.post("/api/flow/action/play")
async def flow_action_play(body: FlowArmActionBody):
    """单动作试跑(回放该动作全部路点,含真空触发)。"""
    global _TEST_PLAY
    robot = _pick_robot()
    _flow_guard_moving()
    with _TEST_PLAY_LOCK:
        th = _TEST_PLAY.get("thread")
        if th is not None and th.is_alive():
            raise _err("已有试跑在执行(先停止)", 409)
        arm = _flow_arm(body.arm)
        runner = FlowRunner()               # 一次性上下文:不碰相机/交接
        runner.robot = robot
        runner.cfg = flow_store.load_config()
        runner.runlog = flow_runlog.RunLogger(tag=f"play_{arm}")  # 试跑日志册
        loop = ArmLoop(runner, arm)
        try:
            flow_store._action(runner.cfg, arm, body.action)   # 槽位校验
        except ValueError as e:
            raise _err(str(e), 400)
        if not runner.cfg["arms"][arm]["actions"][body.action]["waypoints"]:
            raise _err(f"{body.action} 没有路点", 400)
        th = threading.Thread(target=loop.run_once,
                              args=(body.action,), daemon=True,
                              name=f"flow-test-{arm}")
        _TEST_PLAY = {"runner": runner, "loop": loop, "thread": th}
        th.start()
    return {"ok": True, "arm": arm, "action": body.action}


@app.post("/api/flow/action/play/stop")
async def flow_action_play_stop():
    r = _TEST_PLAY.get("runner")
    if r is None:
        return {"ok": True, "note": "无试跑任务"}
    r.abort_event.set()
    return {"ok": True, "note": "已请求软停(段边界生效)"}


@app.get("/api/flow/action/play/status")
async def flow_action_play_status():
    loop = _TEST_PLAY.get("loop")
    th = _TEST_PLAY.get("thread")
    if loop is None:
        return {"running": False}
    out = loop.status()
    out["running"] = bool(th and th.is_alive())
    return out


@app.post("/api/flow/teach/align_ref")
async def flow_teach_align_ref(body: FlowAlignBody):
    """臂停在对准观察位时,记录码在基座系的位置为该站参考(不动臂)。"""
    _flow_guard_moving()
    robot = _pick_robot()
    arm = _flow_arm(body.arm)
    which = body.which if body.which in ("align", "align2") else "align"
    if which == "align2" and arm != "armright":
        raise _err("align2(料仓)只有右臂有", 400)
    try:
        out = await _call(record_align_ref, robot, _FLOW.cameras,
                          _FLOW.finder, arm, which)
    except FileNotFoundError as e:
        raise _err(f"{e} —— 右臂手眼标定流程见流水线页顶部说明", 500)
    except (RobotError, DepthError) as e:
        raise _err(str(e), 502)
    except ValueError as e:
        raise _err(str(e), 400)
    return out


@app.post("/api/flow/align/test")
async def flow_align_test(body: FlowAlignBody):
    """在当前位姿测一次闸门(不修正、不动臂),验证 Δ/手眼/容差。"""
    _flow_guard_moving()
    robot = _pick_robot()
    arm = _flow_arm(body.arm)
    which = body.which if body.which in ("align", "align2") else "align"
    try:
        return await _call(test_align_gate, robot, _FLOW.cameras,
                           _FLOW.finder, arm, which)
    except FileNotFoundError as e:
        raise _err(str(e), 500)
    except (RobotError, DepthError) as e:
        raise _err(str(e), 502)
    except ValueError as e:
        raise _err(str(e), 400)


@app.post("/api/flow/align/goto")
async def flow_align_goto(body: FlowAlignBody):
    """回到记录参考时的观察位(**动臂**,关节回放),稳定后测 Δ 不修正。"""
    _flow_guard_moving()
    th = _TEST_PLAY.get("thread")
    if th is not None and th.is_alive():
        raise _err("试跑执行中,先停止", 409)
    robot = _pick_robot()
    arm = _flow_arm(body.arm)
    which = body.which if body.which in ("align", "align2") else "align"
    if which == "align2" and arm != "armright":
        raise _err("align2(料仓)只有右臂有", 400)
    try:
        return await _call(goto_align_ref, robot, _FLOW.cameras,
                           _FLOW.finder, arm, which)
    except FileNotFoundError as e:
        raise _err(str(e), 500)
    except (RobotError, DepthError) as e:
        raise _err(str(e), 502)
    except ValueError as e:
        raise _err(str(e), 400)


@app.post("/api/flow/align/fix")
async def flow_align_fix(body: FlowAlignBody):
    """回观察位并**真跑闸门**(测 Δ→超容差实际修正→复测收敛,动臂)。

    单步验证 PBVS 用:与流水线同一条代码路径,修完停在修正后的位置。"""
    _flow_guard_moving()
    th = _TEST_PLAY.get("thread")
    if th is not None and th.is_alive():
        raise _err("试跑执行中,先停止", 409)
    robot = _pick_robot()
    arm = _flow_arm(body.arm)
    which = body.which if body.which in ("align", "align2") else "align"
    if which == "align2" and arm != "armright":
        raise _err("align2(料仓)只有右臂有", 400)
    try:
        return await _call(fix_align_gate, robot, _FLOW, arm, which)
    except FileNotFoundError as e:
        raise _err(str(e), 500)
    except (RobotError, DepthError) as e:
        raise _err(str(e), 502)
    except ValueError as e:
        raise _err(str(e), 400)


@app.post("/api/flow/start")
async def flow_start(body: FlowStartBody):
    global _PICK_JOB
    robot = _pick_robot()
    if _PICK_JOB is not None and not _PICK_JOB._done:
        raise _err("一步到位任务运行中,先停止再启动流水线", 409)
    if body.arms != "right":
        # 同进程两个深度客户端会抢同一台相机的租约(hub 一相机一租约):
        # 物料检测页若还持着左相机会话,流水线启动预热必失败(503)。
        # 这里先释放交棒;流水线停止(_finish)会还,物料检测下次扫描自动重建。
        _PICK_PROVIDER.shutdown()
    try:
        out = await _call(_FLOW.start, robot, step_mode=body.step_mode,
                          only=body.arms, rounds=body.rounds)
    except FlowError as e:
        raise _err(str(e), 400)
    except DepthError as e:
        raise _err(f"相机会话建立失败: {e}", 503)
    return out


@app.post("/api/flow/stop")
async def flow_stop():
    return await _call(_FLOW.stop)


@app.post("/api/flow/step/confirm")
async def flow_step_confirm():
    return _FLOW.confirm_step()


@app.get("/api/flow/status")
async def flow_status():
    return _FLOW.status()


@app.get("/api/flow/logs")
async def flow_logs_list():
    """流水线运行日志册清单(新→旧): 起止时间/循环数/参与臂。"""
    return {"runs": flow_runlog.list_runs()}


@app.get("/api/flow/logs/preview")
async def flow_logs_preview(start: str = "", end: str = ""):
    """客户视图运行报告(按时间段聚合,默认当天): 轮次列表 + A/B 大步
    起止耗时 + 汇总(总轮数/总时间/平均每轮)。流水线在跑时末轮可显示
    为进行中。start/end 支持 YYYY-MM-DD 或 YYYY-MM-DD HH:MM[:SS]。"""
    try:
        return flow_runlog.aggregate_report(
            start, end, active=_flow_running())
    except ValueError as e:
        raise _err(str(e), 400)


@app.get("/api/flow/logs/export")
async def flow_logs_export(start: str = "", end: str = ""):
    """运行报告导出 CSV(与预览同口径: 时间段聚合 A/B 大步 + 每轮小计 +
    总计;UTF-8 BOM,Excel 直开)。start/end 参数同 preview。"""
    try:
        csv_text, filename = flow_runlog.export_report_csv(
            start, end, active=_flow_running())
    except ValueError as e:
        raise _err(str(e), 400)
    return Response(content=csv_text, media_type="text/csv; charset=utf-8",
                    headers={"Content-Disposition":
                             f'attachment; filename="{filename}"'})


def _flow_running() -> bool:
    """流水线是否在跑(决定最新一册要不要算「进行中」轮次)。"""
    try:
        return bool(_FLOW.status().get("running"))
    except Exception:                                       # noqa: BLE001
        return False


# ===========================================================================
# IK 规划可用性探测
# ===========================================================================
@app.get("/api/ik/availability")
async def ik_availability():
    """ik_helper 进程是否可用:只看能不能起得来 + 能不能返回合法 JSON,
    不依赖具体规划结果(规划可能因关节限位等失败但系统是好的)。"""
    if not Path(SYS_PYTHON).is_file():
        return {"available": False, "reason": f"系统 Python 不存在: {SYS_PYTHON}"}
    if not IK_HELPER.is_file():
        return {"available": False, "reason": f"ik_helper 不存在: {IK_HELPER}"}
    try:
        cp = subprocess.run(
            [SYS_PYTHON, str(IK_HELPER)],
            input='{"arm":"armleft","q_deg":[0,0,0,0,0,0,0],'
                  '"op":"rotate","axis":"rx","angle_deg":0.001,'
                  '"frame":"base"}',
            capture_output=True, text=True, timeout=20,
        )
    except subprocess.TimeoutExpired:
        return {"available": False, "reason": "ik_helper 响应超时"}
    try:
        out = json.loads(cp.stdout)
    except json.JSONDecodeError:
        return {"available": False, "reason": "ik_helper 返回非 JSON",
                "stderr": cp.stderr[-200:]}
    # 子进程能起、能解析 JSON → 系统可用
    # (业务 ok=False 可能是"无变化"或"超出限位",与系统可用性无关)
    if out.get("unavailable"):
        return {"available": False, "reason": out.get("error", "unavailable")}
    return {"available": True, "probe": out}


# ===========================================================================
# 静态前端
# ===========================================================================
_STATIC = Path(__file__).resolve().parent / "static"


@app.middleware("http")
async def _static_no_cache(request, call_next):
    """静态前端禁长缓存: 每次都向服务器校验(没变仍可 304),杜绝
    浏览器拿旧 app.js 配新 index.html 混跑导致 flRefresh 报
    『读取流水线配置失败』。"""
    resp = await call_next(request)
    p = request.url.path
    if p == "/" or p.startswith("/static"):
        resp.headers["Cache-Control"] = "no-cache"
    return resp


if _STATIC.is_dir():
    app.mount("/static", StaticFiles(directory=str(_STATIC)), name="static")

    @app.get("/")
    async def index():
        return FileResponse(str(_STATIC / "index.html"),
                            headers={"Cache-Control": "no-cache"})
