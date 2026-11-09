# -*- coding: utf-8 -*-
"""笛卡尔运动规划:IK 子进程调用 + 折线增量平移。

从 upper_computer/workers.py 移植,移除了 QThread 依赖。
webapp 后端在 FastAPI 线程中同步调用(经 to_thread),无 Qt 介入。
"""
from __future__ import annotations

import json
import math
import selectors
import subprocess
import threading
from pathlib import Path
from typing import Callable, Optional

import numpy as np

from ik_config import IK_HELPER, SYS_PYTHON
from robot_client import (BridgeDown, RobotError, TianlianRobot,
                          joints_from_bridge)


# ===========================================================================
# IK 子进程入口
# ===========================================================================
class _PlannerUnavailable(RobotError):
    """逆解环境不可用(系统 Python 缺失 / tl_kinematics 装载失败)标记。"""

    def __init__(self, msg: str):
        super().__init__(f"逆解环境不可用: {msg}")
        self.planner_unavailable = True


def _check_helper() -> None:
    if not Path(SYS_PYTHON).is_file():
        raise _PlannerUnavailable(f"系统 Python 不存在: {SYS_PYTHON}")
    if not IK_HELPER.is_file():
        raise _PlannerUnavailable(f"ik_helper 不存在: {IK_HELPER}")


def _parse_ik_out(out: dict) -> dict:
    """ik_helper 响应 → dict;失败抛 RobotError(环境缺失抛不可用标记)。"""
    if not out.get("ok"):
        err = str(out.get("error") or "未知错误")
        if out.get("unavailable"):
            raise _PlannerUnavailable(err)
        raise RobotError(f"逆解规划失败: {err}")
    return out


def plan_cartesian_path(arm: str, q_deg: list, *, op: str,
                        timeout: float = 20.0, **params) -> dict:
    """做直线+姿态插值逆解,返回轨迹规划结果。

    2026-11-05 起走常驻 IkServer(稳态求解 ~3ms,一次性子进程每请求
    ~350ms);进程死亡自动重启重试。旋转/目标点/pick 对准/flow 闸门等
    所有调用方统一受益。

    op:
      rotate  axis=rx|ry|rz angle_deg=.. frame=base|tool
      target  target_pos_m=[..] target_rpy_deg=[..] cur_pos_m=[..] cur_quat=[..]
    """
    req = {"arm": arm, "q_deg": list(q_deg), "op": op, **params}
    return IkServer.call(req, timeout=min(timeout, 20.0))


# ===========================================================================
# FK 末端位姿快读(报告系)
# ===========================================================================
def _quat_to_R(x: float, y: float, z: float, w: float) -> np.ndarray:
    q = np.array([x, y, z, w], dtype=float)
    q = q / np.linalg.norm(q)
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _R_to_quat(R: np.ndarray) -> list:
    tr = float(R[0, 0] + R[1, 1] + R[2, 2])
    if tr > 0.0:
        w = math.sqrt(tr + 1.0) / 2.0
        return [(R[2, 1] - R[1, 2]) / (4.0 * w), (R[0, 2] - R[2, 0]) / (4.0 * w),
                (R[1, 0] - R[0, 1]) / (4.0 * w), w]
    i = max(range(3), key=lambda k: R[k, k])
    j, k = (i + 1) % 3, (i + 2) % 3
    t = math.sqrt(max(0.0, 1.0 + R[i, i] - R[j, j] - R[k, k]))
    q = [0.0, 0.0, 0.0]
    q[i] = t / 2.0
    q[j] = (R[j, i] + R[i, j]) / (2.0 * t)
    q[k] = (R[k, i] + R[i, k]) / (2.0 * t)
    return [q[0], q[1], q[2], (R[k, j] - R[j, k]) / (2.0 * t)]


class EndPoseFK:
    """末端位姿快读::8000 上报系 = fkine(URDF)系经一固定安装偏移
    (实测 ~80mm)。静止时用一次 :8000 读数标定 T_offset,之后
    fkine(桥缓存关节) @ 偏移 毫秒级出位姿 —— DDS 风暴期不再吃
    get_end_pose 的 0.6~3s 服务往返。双 URDF 对拍已证 tl_kinematics
    FK 与 DT 模型 0.0mm 一致,标定后逐位姿残差亚毫米级。"""

    def __init__(self, robot: TianlianRobot):
        self.robot = robot
        self._lock = threading.Lock()
        self._off: dict = {}          # arm -> 4x4 (报告系←fkine系)

    def _fk_T(self, arm: str, q_deg: list) -> np.ndarray:
        out = IkServer.call({"op": "fk", "arm": arm, "q_deg": q_deg})
        T = np.eye(4)
        T[:3, :3] = _quat_to_R(*out["quat"])
        T[:3, 3] = out["pos_m"]
        return T

    def _calibrate(self, arm: str) -> np.ndarray | None:
        """静止时标定:T_offset = T_报告 @ inv(T_fkine)。两次读角夹一次
        :8000 读数,期间关节在动(>0.05°)则放弃本次标定。"""
        q1 = joints_from_bridge(arm, 100)
        rep = self.robot.get_end_poses_raw(arm)
        q2 = joints_from_bridge(arm, 100)
        if not (q1 and q2 and rep):
            return None
        if max(abs(a - b) for a, b in zip(q1, q2)) > 0.05:
            return None
        T_rep = np.eye(4)
        T_rep[:3, :3] = _quat_to_R(*rep["orientation"])
        T_rep[:3, 3] = rep["position"]
        return T_rep @ np.linalg.inv(self._fk_T(arm, q1))

    def pose(self, arm: str) -> Optional[dict]:
        """{position(m), orientation(x,y,z,w)} 或 None(缓存干涸/未标定)。"""
        q = joints_from_bridge(arm, 150)
        if not q:
            return None
        with self._lock:
            off = self._off.get(arm)
        if off is None:
            off = self._calibrate(arm)      # 静止时一次性标定
            if off is None:
                return None
            with self._lock:
                self._off[arm] = off
        T = off @ self._fk_T(arm, q)
        return {"position": [float(v) for v in T[:3, 3]],
                "orientation": _R_to_quat(T[:3, :3])}

    def reset(self, arm: str = None) -> None:
        """丢弃标定(换机/厂商配置变更后强制重标)。arm=None 双侧。"""
        with self._lock:
            if arm is None:
                self._off.clear()
            else:
                self._off.pop(arm, None)


# ===========================================================================
# 常驻 ik_helper server(笛卡尔点动专用)
# ===========================================================================
class IkServer:
    """常驻 ik_helper --server 客户端(进程内单例,按需拉起)。

    单次模式每请求要 ~350ms(URDF 解析占大头),笛卡尔点动 150ms 一拍
    喂不起;常驻后运动学链只建一次,纯求解毫秒级(实测稳态 ~80ms,
    含奇异位形迭代)。锁串行;进程死亡/超时自动重启重试一次。
    环境变量原样继承(IK_URDF_MODEL 钉死 DT 模型 + tl_kinematics 需要
    fd_ws overlay 的 PYTHONPATH,绝不能清)。父进程退出时管道关闭,
    server 的 readline() 返回空自动退出,不留孤儿。
    """

    _lock = threading.Lock()
    _proc: subprocess.Popen | None = None

    @classmethod
    def _spawn(cls) -> subprocess.Popen:
        _check_helper()
        # stdin/stdout 管道;text 模式按行协议;stderr 继承(致命错误进 webapp 日志)
        return subprocess.Popen(
            [SYS_PYTHON, str(IK_HELPER), "--server"],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=None, text=True, bufsize=1,
            cwd=str(IK_HELPER.parent))

    @classmethod
    def _kill(cls) -> None:
        proc, cls._proc = cls._proc, None
        if proc is None:
            return
        for s in (proc.stdin, proc.stdout):
            try:
                s and s.close()
            except Exception:
                pass
        try:
            proc.kill()
            proc.wait(timeout=1.0)
        except Exception:
            pass

    @classmethod
    def _roundtrip(cls, proc: subprocess.Popen, req: dict,
                   timeout: float) -> dict:
        """一问一答;管道断/超时/坏 JSON 抛 RobotError。"""
        try:
            proc.stdin.write(json.dumps(req, ensure_ascii=False) + "\n")
            proc.stdin.flush()
        except (BrokenPipeError, ValueError, OSError):
            raise RobotError("IK server 管道已断")
        sel = selectors.DefaultSelector()
        try:
            sel.register(proc.stdout, selectors.EVENT_READ)
            ready = sel.select(timeout)
        finally:
            sel.close()
        if not ready:
            raise RobotError(f"IK server 求解超时({timeout:.0f}s)")
        line = proc.stdout.readline()
        if not line:
            raise RobotError("IK server 已退出")
        try:
            out = json.loads(line)
        except json.JSONDecodeError:
            raise RobotError(f"IK server 响应非 JSON: {line[:120]!r}")
        return out

    @classmethod
    def call(cls, req: dict, timeout: float = 5.0) -> dict:
        """线程安全调用;死亡自动重启并重试一次。返回已校验的 ok 结果。"""
        with cls._lock:
            if cls._proc is None or cls._proc.poll() is not None:
                cls._kill()
                cls._proc = cls._spawn()
            try:
                out = cls._roundtrip(cls._proc, req, timeout)
            except RobotError:
                cls._kill()                      # 状态可疑,重启后重试一次
                cls._proc = cls._spawn()
                out = cls._roundtrip(cls._proc, req, timeout)
        return _parse_ik_out(out)

    @classmethod
    def shutdown(cls) -> None:
        """主动关闭(webapp 断连机器人时顺手清掉)。"""
        with cls._lock:
            cls._kill()


# ===========================================================================
# 笛卡尔增量运动(HTTP 轴平移组合)
# ===========================================================================
def cartesian_delta_move(robot: TianlianRobot, arm: str,
                         dx: float, dy: float, dz: float,
                         speed: int = 15, step_m: float = 0.05,
                         should_stop: Optional[Callable[[], bool]] = None,
                         frame: str = "base") -> None:
    """用 move_arm_direction(x/y/z) 组合实现笛卡尔相对位移。

    HTTP 接口只支持单轴直线平移,长行程按 step_m 细分,每步按分量
    大小降序执行,得到折线逼近的斜线运动。should_stop() 返回 True 时中止。
    降序原因:厂商接口对单次平移有 ≥0.5° 关节变化下限,小分量会被拒
    (near-zero)并中止整段 —— 先走大分量,被拒时剩余未走分量必然也
    在下限以下,残差亚毫米级;若按固定 X→Y→Z,"小分量在前被拒、
    大分量在后未走"会留下几十毫米假残差(2026-10-23 闸门修正分型)。
    """
    def _stopped() -> bool:
        return callable(should_stop) and should_stop()

    axes = sorted((("x", dx), ("y", dy), ("z", dz)),
                  key=lambda ad: abs(ad[1]), reverse=True)
    max_delta = max((abs(d) for _, d in axes), default=0.0)
    if max_delta < 5e-5:      # <0.05mm 视为指令噪声,直接忽略
        return
    n = max(1, math.ceil(max_delta / max(step_m, 0.005)))
    for _ in range(n):
        if _stopped():
            return
        for axis, delta in axes:
            d = delta / n
            if abs(d) < 5e-5:  # <0.05mm 的分段不下发(厂商接口对该量级无意义)
                continue
            if _stopped():
                return
            robot.move_arm_direction(arm, axis, d, frame=frame, block=1,
                                     timeout=30.0, speed=speed)


# ===========================================================================
# IK 直发版笛卡尔增量(点动快路径)
# ===========================================================================
def cartesian_delta_move_ik(robot: TianlianRobot, arm: str,
                            dx: float, dy: float, dz: float,
                            speed: int = 15,
                            should_stop: Optional[Callable[[], bool]] = None,
                            frame: str = "base",
                            joints_reader: Optional[
                                Callable[[str], tuple[list, int] | None]] = None
                            ) -> dict:
    """笛卡尔相对位移(快路径):topic 缓存读角 → ik_helper translate →
    /move_robotarm 整段轨迹单 action 下发。

    与 cartesian_delta_move(折线组合 move_arm_direction)的区别:后者每拍
    过 :8000 的 get_state 服务往返+进程内规划+set_speed,DDS 风暴下 Python
    层最弱,150ms 一拍喂不起;本路径读角走 /joint_states 订阅缓存(毫秒级),
    求解走常驻 ik_helper(稳态 ~80ms),下发只剩一个 lean action。

    joints_reader(arm) → (q_deg, age_ms) 或 None;缺省/失效回落
    :8000 读角。返回 {"ok", "moved", "src", "max_dq_deg"}。
    """
    dist = math.sqrt(dx * dx + dy * dy + dz * dz)
    if dist < 5e-5:          # <0.05mm 视为指令噪声
        return {"ok": True, "moved": False, "reason": "below_deadband"}
    if callable(should_stop) and should_stop():
        return {"ok": True, "moved": False, "reason": "stopped"}

    q_deg, src = None, None
    if callable(joints_reader):
        got = joints_reader(arm)
        if got and got[0]:
            q_deg, src = list(got[0]), "bridge"
    if q_deg is None:        # 缓存不可用 → 阻塞读 :8000(慢,但保功能)
        angles = robot.get_joint_angles()
        q_deg = list((angles or {}).get(arm) or [])
        src = "arm_http"
    if not q_deg:
        raise RobotError("拿不到当前关节角,无法做笛卡尔规划")

    out = IkServer.call({"op": "translate", "arm": arm, "q_deg": q_deg,
                         "deltas_m": [dx, dy, dz], "frame": frame})
    traj = out.get("waypoints_deg") or []
    if not traj:
        raise RobotError("逆解未返回轨迹")
    if callable(should_stop) and should_stop():    # 求解期间松手,不下发
        return {"ok": True, "moved": False, "reason": "stopped"}
    # timeout 按关节变化量放大:max_dq 5° 内给 12s,每多 5° 加 6s,封顶 60s
    max_dq = float(out.get("max_dq_deg") or 1.0)
    timeout = min(60.0, 12.0 + 6.0 * max_dq / 5.0)
    robot.move_robotarm_traj(arm, traj, block=1, timeout=timeout, speed=speed)
    return {"ok": True, "moved": True, "src": src,
            "max_dq_deg": out.get("max_dq_deg")}
