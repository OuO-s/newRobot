# -*- coding: utf-8 -*-
"""笛卡尔运动规划:IK 子进程调用 + 折线增量平移。

从 upper_computer/workers.py 移植,移除了 QThread 依赖。
webapp 后端在 FastAPI 线程中同步调用(经 to_thread),无 Qt 介入。
"""
from __future__ import annotations

import json
import math
import subprocess
from pathlib import Path
from typing import Callable, Optional

from ik_config import IK_HELPER, SYS_PYTHON
from robot_client import RobotError, TianlianRobot


# ===========================================================================
# IK 子进程入口
# ===========================================================================
class _PlannerUnavailable(RobotError):
    """逆解环境不可用(系统 Python 缺失 / tl_kinematics 装载失败)标记。"""

    def __init__(self, msg: str):
        super().__init__(f"逆解环境不可用: {msg}")
        self.planner_unavailable = True


def plan_cartesian_path(arm: str, q_deg: list, *, op: str,
                        timeout: float = 20.0, **params) -> dict:
    """调 ik_helper.py(系统 Python)做直线+姿态插值逆解,返回轨迹规划结果。

    op:
      rotate  axis=rx|ry|rz angle_deg=.. frame=base|tool
      target  target_pos_m=[..] target_rpy_deg=[..] cur_pos_m=[..] cur_quat=[..]
    """
    req = {"arm": arm, "q_deg": list(q_deg), "op": op, **params}
    if not Path(SYS_PYTHON).is_file():
        raise _PlannerUnavailable(f"系统 Python 不存在: {SYS_PYTHON}")
    if not IK_HELPER.is_file():
        raise _PlannerUnavailable(f"ik_helper 不存在: {IK_HELPER}")
    try:
        cp = subprocess.run([SYS_PYTHON, str(IK_HELPER)],
                            input=json.dumps(req), capture_output=True,
                            text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        raise RobotError("逆解规划超时(URDF 加载或求解过慢)")
    try:
        out = json.loads(cp.stdout)
    except json.JSONDecodeError:
        tail = (cp.stderr or "").strip().splitlines()
        raise RobotError(f"逆解辅助进程异常退出({cp.returncode}): "
                         f"{tail[-1] if tail else cp.stdout[:200] or '无输出'}")
    if not out.get("ok"):
        err = str(out.get("error") or "未知错误")
        if out.get("unavailable"):
            raise _PlannerUnavailable(err)
        raise RobotError(f"逆解规划失败: {err}")
    return out


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
