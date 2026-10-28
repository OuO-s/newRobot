# -*- coding: utf-8 -*-
"""双臂上下料流水线(编排层)。
================================================================
流程(doc/双臂协同上下料工作流方案_2026-10-20.md 的 webapp 实现):

  左臂循环: ①物料检测(YOLO+深度,复用 pick 模块) ②越顶到物料上方
            ③下降+下压 ④吸取(真空沿用 pick 设置) ⑤抬升 → A1 转移到
            中间件上方 → 对准闸门 → 等 fixture_empty → A2 放入中间件
            (断真空) → 置 FILLED;①-⑤ 全视觉自动,phase 逐子步上报
  右臂循环: (B0 预备) → 等 FILLED → B1 到中间件上方 → 对准闸门 →
            B2 取料(开真空,退出中心后置 EMPTY) → [可选:B2→B3 夹爪整形] →
            B3 转移料仓上方 →
            对准闸门2 → B4 放入料仓(断真空) → 收回

协同机制:
  - 中间件令牌 EMPTY/FILLED: 左放前置条件 EMPTY,右取前置条件 FILLED;
  - 单臂测试(start only="left"/"right"): 只跑 1 个循环自动停,不等令牌;
    前置检查只查该臂 —— 左=吸料→放中间件,右=取中间件→入料仓;
  - 中心共享区互斥: A2 / B1 / B2 执行期间持有 center,另一臂的动作
    可用 wait="center_free" 串行化(默认只给 A1 配了);
    safety.center_serialize=true(首次双臂联调建议): 左臂吸取后先在
    取料区等中间件 EMPTY 再进中心,并持锁贯穿 A1→闸门→A2 —— 消除
    "左臂停在中间件上方观察位时右臂 B1/B2 进中心"的双臂同区窗口;
    路点"中点"标记(网页路点行 ●中点): 标了任一点即切到路点级 ——
    回放走向中点前抢锁,带到其后第一个非标记点(退出段走完)才交,
    闸门/跨动作持锁自然衔接;都不标=上面那套整段互斥。
  - 对准闸门(三档):
      T0  gate.mode="off"   纯示教回放(极限节拍,盲放);
      T1  gate.mode="verify"(默认) 到观察位测一次码位置:
          Δ = p_base_now − p_base_ref ≤ 容差 → 直接过(~0.3-0.5s);
      T2  T1 超差时升级     按基座系 Δ 修正臂位后复测,最多 iterations 步;
          修正量 > max_corr 视为工装被撞 → 停线报警,不盲放。
    参考是"码在基座系的位置"(经手眼 X),与臂站哪里无关 → 用户改
    A1/B1/B3 的路点不影响参考有效性;Δ 差分还让 marker_size 的比例
    误差抵消(示教与运行同距离测量)。
    注意码是静止物:p_base 不随臂移动而变,T2 修正后基座系 Δ 不会
    缩小 —— 收敛判据用"相机系视差"(当前观感 vs 参考录制观感,
    对齐到位应≈0),由 ref 里的末端位姿按同一条链路反解得到。

  - 重锚回放(reanchor,闸门 → A2/B2/B4 的衔接): 闸门收敛后记
    Δ锚 = 当前末端 − 前一动作示教末点(= T2 修正量 + 回放残余),
    放置/取料动作的示教轨迹整体平移 Δ锚 后按笛卡尔逐点走。
    没有这一步,按示教关节角回放会把闸门修正"弹回"示教点,
    视觉校准等于白做。|Δ锚| < move.reanchor_min_mm 时退回关节回放。

用户只需要在网页上"添加动作"(记录路点)和"记录对准参考",
本文件不接触任何具体点位。

运行日志(flow_runlog,logs/flow/run_*.log|.jsonl): 每次启动/试跑一册。
动作含开始·结束时间·耗时·速度%·路点数·起止位姿;运动段含耗时与到位
残差;闸门含每步测量与修正;循环含各 phase 耗时分解 —— 供节拍测算
与问题排查(jsonl 可直接脚本统计)。
"""
from __future__ import annotations

import threading
import time
from collections import deque

import numpy as np

import flow_runlog
import flow_store
import pick_store
import speed_store
from cartesian import cartesian_delta_move, plan_cartesian_path
from depth_provider import DepthError, DepthSession, ensure_ros_env
from flow_align import MarkerFinder, measure_marker_base
from pick_flow import (quat_to_rpy_deg, rpy_deg_to_R, scan_once, T_from)
from robot_client import RobotError, TianlianRobot


class _Abort(Exception):
    """软停(段边界)。"""


class FlowError(RuntimeError):
    """启动前置检查失败(消息面向用户)。"""


# ===========================================================================
# 相机会话池: 左臂扫描要深度(持有租约跨循环),对准测量只要彩色(免租约)
# ===========================================================================
class FlowCameras:
    def __init__(self):
        self._sessions: dict[str, DepthSession] = {}
        self._lock = threading.Lock()

    def get(self, role: str, need_depth: bool = False) -> DepthSession:
        with self._lock:
            s = self._sessions.get(role)
            if s is not None and s.is_running:
                if need_depth and not s._lease_ok:   # 彩色会话升级为深度会话
                    s.stop()
                    s = None
                elif not s.alive or s.color_age_s > 5.0:
                    # hub 重启相机后本进程订阅偶发掉匹配、或 spin 线程死于
                    # wait set 竞态:帧停更/自旋死即重建
                    # (2026-10-20 现场: left 停更 10min,is_running 仍 True;
                    #  2026-10-21: spin 死后 is_running 同样仍 True)
                    try:
                        s.stop()
                    except Exception:                 # noqa: BLE001
                        pass
                    s = None
                else:
                    return s
            ensure_ros_env()
            s = DepthSession(role, client_prefix="webapp_flow_")
            s.start(need_depth=need_depth)       # 失败抛 DepthError
            self._sessions[role] = s
            return s

    def shutdown(self):
        with self._lock:
            for s in self._sessions.values():
                try:
                    s.stop()
                except Exception:                     # noqa: BLE001
                    pass
            self._sessions.clear()

    # scan_once 需要的 provider 形状(start_session/session)
    def as_pick_provider(self, role: str):
        outer = self

        class _Adapter:
            def start_session(self, r, **kw):
                return outer.get(r, need_depth=True)

            @property
            def session(self):
                return outer._sessions.get(role)
        return _Adapter()


# ===========================================================================
# 中间件令牌 + 中心共享区
# ===========================================================================
class Handoff:
    def __init__(self, runner):
        self._cond = threading.Condition()
        self.fixture = "EMPTY"            # EMPTY / FILLED
        self.center_owner: str | None = None
        self.center_wp_level = False      # 锁=路点级(中点标记)还是整段级
        self._runner = runner

    def set_filled(self, by: str):
        with self._cond:
            self.fixture = "FILLED"
            self._cond.notify_all()
        self._runner.note(f"中间件 ← FILLED(由 {by} 放料)")
        self._runner.runlog.event("handoff", by, fixture="FILLED")

    def set_empty(self, by: str):
        with self._cond:
            self.fixture = "EMPTY"
            self._cond.notify_all()
        self._runner.note(f"中间件 → EMPTY(由 {by} 取走)")
        self._runner.runlog.event("handoff", by, fixture="EMPTY")

    def wait_fixture(self, want: str, loop: "ArmLoop"):
        with self._cond:
            while self.fixture != want:
                if self._runner.abort_event.is_set():
                    raise _Abort()
                loop.set_phase(f"wait_{want.lower()}")
                self._cond.wait(timeout=0.5)

    def enter_center(self, loop: "ArmLoop", wp_level: bool = False):
        if self.center_owner == loop.arm:      # 已自持(center_serialize 的
            return                             # A1),防自等死锁
        with self._cond:
            while self.center_owner is not None:
                if self._runner.abort_event.is_set():
                    raise _Abort()
                loop.set_phase("wait_center")
                self._cond.wait(timeout=0.5)
            self.center_owner = loop.arm
            self.center_wp_level = wp_level
        self._runner.runlog.event("handoff", loop.arm, center="acquire")

    def wait_center_free(self, loop: "ArmLoop"):
        with self._cond:
            while self.center_owner is not None:
                if self._runner.abort_event.is_set():
                    raise _Abort()
                loop.set_phase("wait_center")
                self._cond.wait(timeout=0.5)

    def exit_center(self, arm: str):
        with self._cond:
            if self.center_owner == arm:
                self.center_owner = None
                self.center_wp_level = False
                self._cond.notify_all()
                self._runner.runlog.event("handoff", arm, center="release")


# ===========================================================================
# 单臂循环
# ===========================================================================
class ArmLoop:
    def __init__(self, runner: "FlowRunner", arm: str):
        self.runner = runner
        self.robot: TianlianRobot = runner.robot
        self.arm = arm
        self.side = "left" if arm == "armleft" else "right"
        self.log: deque = deque(maxlen=300)
        self.phase = "idle"
        self.error: str | None = None
        self.cycles = 0
        self.cycle_ts: list[float] = []             # 最近 20 次循环耗时
        self.phase_times: dict[str, float] = {}     # 各步最近一次耗时
        self._phase_t0 = time.time()
        self._cycle_t0 = None
        # 重锚回放状态(见模块头): _taught_end_pos = 刚回放动作的示教
        # 末点(基座系),_gate_offset = 闸门收敛后的平移量(reanchor 动作消费)
        self._taught_end_pos: list | None = None
        self._gate_offset: np.ndarray | None = None
        # 本轮循环的日志累积(循环起点重置,_mark_cycle 汇总落盘)
        self._cycle_phases: list[list] = []        # [[phase, 耗时s], ...]
        self._cycle_actions: list[dict] = []
        self._cycle_gates: list[dict] = []
        # 单臂测试(start(only=...)): 只跑 1 个循环自动停,不等另一臂令牌
        self.single_shot = False
        # 轮次(start(rounds=N)): 该臂跑满 N 个循环后自动停;None=连续
        self.max_cycles = None

    # ------------------------------------------------------------ 基础
    def say(self, msg: str):
        self.log.append(f"[{time.strftime('%H:%M:%S')}] {msg}")
        self.runner.runlog.line(f"[{self.arm}] {msg}")

    def set_phase(self, phase: str):
        now = time.time()
        if self.phase and self.phase != phase:
            dur = round(now - self._phase_t0, 2)
            self.phase_times[self.phase] = dur
            if self._cycle_t0 is not None:
                self._cycle_phases.append([self.phase, dur])
        self._phase_t0 = now
        self.phase = phase
        self.runner.runlog.event("phase", self.arm, phase=phase)

    def check_abort(self):
        if self.runner.abort_event.is_set():
            raise _Abort()
        self.runner._step_gate(self)

    def run(self):
        try:
            self._loop()
        except _Abort:
            self.set_phase("aborted")
            self.say("⛔ 软停(段边界),臂停在当前位")
            self.runner.note(f"{self.arm} 软停")
            self.runner.runlog.event("arm_end", self.arm, status="aborted",
                                     pose=self._pose_snap())
        except (RobotError, DepthError, ValueError, FileNotFoundError) as e:
            self.set_phase("failed")
            self.error = str(e)
            self.say(f"✗ 失败: {e}")
            self.runner.runlog.event("arm_end", self.arm, status="failed",
                                     error=str(e), pose=self._pose_snap())
            self.runner.abort(str(e))               # 单臂失败 → 全线停
        except Exception as e:                      # noqa: BLE001
            self.set_phase("failed")
            self.error = f"{type(e).__name__}: {e}"
            self.say(f"✗ 异常: {self.error}")
            self.runner.runlog.event("arm_end", self.arm, status="failed",
                                     error=self.error, pose=self._pose_snap())
            self.runner.abort(self.error)
        finally:
            self.runner.handoff.exit_center(self.arm)

    def run_once(self, action: str):
        """单动作试跑(网页『▶试跑』):异常处理与循环同款。"""
        try:
            self.play_action(action)
        except _Abort:
            self.set_phase("aborted")
            self.say("⛔ 试跑软停,臂停在当前位")
        except (RobotError, DepthError, ValueError, FileNotFoundError) as e:
            self.set_phase("failed")
            self.error = str(e)
            self.say(f"✗ 试跑失败: {e}(臂停在当前位)")
        except Exception as e:                      # noqa: BLE001
            self.set_phase("failed")
            self.error = f"{type(e).__name__}: {e}"
            self.say(f"✗ 异常: {self.error}")
        finally:
            self.say("—— 试跑结束 ——")
            if isinstance(self.runner.runlog, flow_runlog.RunLogger):
                self.runner.runlog.close(f"试跑 {action} 结束")

    def _pose_snap(self) -> dict | None:
        """动作/异常边界的位姿快照(本臂基座系)—— 排查与日后双臂
        基座系标定用;读失败返回 None,绝不上抛。"""
        try:
            p = self.end_pose()
            return {"pos_mm": [round(v * 1000.0, 1) for v in p["position"]],
                    "quat": [round(v, 4) for v in p["orientation"]],
                    "joints_deg": [round(v, 2) for v in self.q_deg()]}
        except Exception:                               # noqa: BLE001
            return None

    # ------------------------------------------------------------ 读臂
    def end_pose(self) -> dict:
        p = self.robot.get_end_poses().get(self.arm) or {}
        pos, quat = p.get("position"), p.get("orientation")
        if not pos or not quat:
            raise RobotError("末端位姿读取失败(臂未上电?)")
        return {"position": [float(v) for v in pos],
                "orientation": [float(v) for v in quat]}

    def q_deg(self) -> list:
        q = self.robot.get_joint_angles().get(self.arm) or []
        if not q:
            raise RobotError("关节角读取失败")
        return [float(v) for v in q]

    # ------------------------------------------------------------ 运动
    def _mv_cfg(self) -> dict:
        return self.runner.cfg["move"]

    def play_action(self, name: str, reanchor: bool = False):
        """示教动作回放: 关节路点序列 + 段后验证补走 + 中途真空触发。

        动作配置 vacuum = {"event": "grab"|"release", "at": "start"|k|"end"},
        k=路点序号: 到第 k 个路点后触发(下降到位再断/开真空,然后走完剩余)。
        路点 wait_s=到点等待: 到该点(段后验证补走之后)先等 N 秒再走剩余段;
        真空触发点同点配了等待时,顺序=到点→等待→触发真空。

        reanchor=True(闸门后的 A2/B2/B4 用): 若紧前的对准闸门留有平移
        Δ锚,各路点示教位姿 +Δ锚 后按笛卡尔逐点走,让视觉修正传导进
        下降/退出轨迹;Δ 缺失或 < reanchor_min_mm 时退回纯关节回放。
        """
        meta = flow_store.ACTIONS_META[self.arm][name]
        act = self.runner.cfg["arms"][self.arm]["actions"][name]
        wps = act["waypoints"]
        if not wps:
            raise ValueError(f"{name} 没有路点 —— 请先在流水线页示教")
        if (meta.get("wait") == "center_free"
                and self.runner.handoff.center_owner != self.arm):
            # 自己已持锁(center_serialize 的 A1)时不再等 —— 防自等死锁
            self.runner.handoff.wait_center_free(self)
        if not reanchor:
            self._gate_offset = None            # 绝对回放: 挂起的修正作废
        # 动作级覆盖极少用;缺省=顶栏全局速度(每动作取一次,改了即时生效)
        speed = act.get("speed") or speed_store.get_speed(self.arm)
        vac = act.get("vacuum") or None
        off = None
        if reanchor and self._gate_offset is not None:
            mm = float(np.linalg.norm(self._gate_offset) * 1000.0)
            if mm < float(self._mv_cfg()["reanchor_min_mm"]):
                self.say(f"{name}: 闸门Δ {mm:.2f}mm < "
                         f"{self._mv_cfg()['reanchor_min_mm']}mm,"
                         "按示教关节回放")
            else:
                off = np.asarray(self._gate_offset, float)
        self.set_phase(name)
        self.say((f"{name}: 重锚回放 {len(wps)} 点"
                  f"(闸门Δ={np.round(off * 1000.0, 1).tolist()}mm)")
                 if off is not None else
                 (f"{name}: 回放 {len(wps)} 个路点"
                  + (f"(真空{ '吸' if vac['event'] == 'grab' else '断'}"
                     f" @ {vac.get('at', 'end')})" if vac else "")))

        # —— 动作计时: 起止时间/耗时/速度%随动作记录进运行日志与循环汇总
        m0 = time.monotonic()
        rec = {"action": name, "cycle": self.cycles + 1,
               "t_start": flow_runlog.iso(), "speed_pct": speed,
               "waypoints": len(wps),
               "mode": "reanchor" if off is not None else "joint",
               "vacuum": vac, "pose_start": self._pose_snap()}
        try:
            cut = len(wps) - 1                   # 真空触发前走到的路点
            if vac:
                at = vac.get("at", "end")
                if at == "start":
                    cut = -1
                elif at == "end":
                    cut = len(wps) - 1
                else:
                    cut = min(max(int(at), 0), len(wps) - 1)

            def _seg(wlist):
                """按路点"中点"标记分段走并管中心锁: 走向中点前抢锁,
                带到其后第一个非标记点(退出段走完)才交锁;末段仍是
                中点则持锁离开(由循环收尾交还)。无标记=一整段,锁不动
                (整段级锁由循环的 enter/exit 包裹管理)。"""
                hand = self.runner.handoff
                marks = [bool(w.get("center")) for w in wlist]

                def run(a, b):
                    part = wlist[a:b]
                    # ⏱到点等待(waypoint.wait_s): 到该点后停留 N 秒再走
                    # 剩余段。_traj 是整段一次下发,中途要停只能在等待点
                    # 断开续走 —— 段后验证补走已含在 _traj 内,等在到位后
                    k = 0
                    while k < len(part):
                        m = k
                        while m < len(part) - 1 and not part[m].get("wait_s"):
                            m += 1
                        seg = part[k:m + 1]
                        if off is None:
                            self._traj(seg, speed, name)
                        else:
                            self._traj_reanchor(seg, speed, name, off)
                        ws = part[m].get("wait_s")
                        if ws:
                            self.say(f"{name}: 点{a + m + 1} 到点,等 {float(ws):g}s")
                            self.runner._sleep_abort(float(ws))
                        k = m + 1

                i, n = 0, len(wlist)
                while i < n:
                    if hand.center_owner == self.arm and \
                            hand.center_wp_level:
                        j = i
                        while j < n and marks[j]:
                            j += 1
                        exit_here = j < n     # 标记段止于 j(非标记=退出点)
                        j = min(j + 1, n)     # 退出段在锁内走完
                        run(i, j)
                        if exit_here:
                            hand.exit_center(self.arm)
                            self.say(f"{name}: 已出中心(点{j}/{n}),交锁")
                    else:
                        j = i
                        while j < n and not marks[j]:
                            j += 1
                        if i < j:             # 中点前的自由段
                            run(i, j)
                        if j < n:             # j=首个中点: 该段进前抢锁
                            hand.enter_center(self, wp_level=True)
                            self.say(f"{name}: 进中心(点{j + 1}/{n})")
                    i = j

            if cut >= 0:
                _seg(wps[:cut + 1])
            if vac:
                self.runner.vacuum(self, vac["event"])
            if cut < len(wps) - 1:
                _seg(wps[cut + 1:] if cut >= 0 else wps)
            last = (wps[-1].get("end_pose") or {}).get("position")
            self._taught_end_pos = [float(v) for v in last] if last else None
            self._gate_offset = None             # 消费掉,不串到下一周期
            rec.update(status="ok", duration_s=round(time.monotonic() - m0, 3),
                       t_end=flow_runlog.iso(), pose_end=self._pose_snap())
            self.say(f"{name}: 完成 {rec['duration_s']}s"
                     f"(speed {speed}%)")
        except BaseException as e:               # 记下失败动作再原样上抛
            rec.update(status=("abort" if isinstance(e, _Abort) else "error"),
                       error=f"{type(e).__name__}: {e}",
                       duration_s=round(time.monotonic() - m0, 3),
                       t_end=flow_runlog.iso(), pose_end=self._pose_snap())
            raise
        finally:
            self.runner.runlog.event("action", self.arm, **rec)
            self._cycle_actions.append(rec)

    def _traj(self, wps: list, speed: int, name: str):
        """执行一段关节轨迹 + 段后到位验证,超差补走一次。

        fd_arm_server 大行程会"停在半途仍报 success"(到位判据仅 2°/关节),
        故必读关节复核(容差 move.joint_tol_deg),超差从当前位重发剩余段。"""
        tol = self._mv_cfg()["joint_tol_deg"]
        timeout = self._mv_cfg()["move_timeout_s"]
        target = [float(v) for v in wps[-1]["joints_deg"]]
        traj = [[float(v) for v in w["joints_deg"]] for w in wps]
        m0 = time.monotonic()
        err = None
        done = False
        for attempt in range(2):
            self.check_abort()
            send = traj
            if attempt:
                self.say(f"{name}: 未到位,从当前位补走(第 {attempt} 次)")
                cur = self.q_deg()
                if len(cur) == len(target):
                    err = max(abs(a - b) for a, b in zip(cur, target))
                    if err <= tol:
                        done = True
                        break
                    # 从当前位重发剩余段:前置当前关节、跳过已到达的前导点
                    # —— 既兑现上面 docstring,也避免厂商服务器把"与上一
                    # 目标完全相同的轨迹"当重复吞掉(2026-10-28 实测:低速
                    # 大行程补走原样重发,60s 零进展直到到位超时)
                    rest = [w[:] for w in traj]
                    while rest and max(abs(a - b)
                                       for a, b in zip(cur, rest[0])) <= tol:
                        rest.pop(0)
                    if rest:
                        send = [cur] + rest
            self.robot.move_robotarm_traj(self.arm, send, 1, timeout, speed)
            time.sleep(0.3)                          # 收敛中,稍等复读
            cur = self.q_deg()
            if len(cur) == len(target):
                err = max(abs(a - b) for a, b in zip(cur, target))
                if err <= tol:
                    done = True
                    break
        self.runner.runlog.event(
            "move", self.arm, name=name, move_kind="traj", speed_pct=speed,
            seg_points=len(wps),
            duration_s=round(time.monotonic() - m0, 3), attempts=attempt + 1,
            max_err_deg=round(err, 2) if err is not None else None, ok=done)
        if not done:
            raise RobotError(f"{name} 到位验证失败(关节差 > {tol}°,补走后仍超差)"
                             " —— 臂停在当前位,请人工检查")

    def _traj_reanchor(self, wps: list, speed: int, name: str,
                       off: np.ndarray):
        """重锚回放: 各路点示教位姿 + Δ锚 平移(姿态不变),逐点 IK 直线走。

        Δ锚 是闸门收敛后的纯平移(move_delta 不改姿态),平移后的首点
        ≈ 当前末端 → 臂从视觉校准过的位置沿示教轨迹的"形状"下降/退出,
        每点段后按放置容差做 FK 残差验证(内含补走一次)。"""
        mv = self._mv_cfg()
        for i, w in enumerate(wps):
            ep = w.get("end_pose") or {}
            if not ep.get("position") or not ep.get("orientation"):
                raise ValueError(f"{name} 路点{i} 缺 end_pose —— 重新"
                                 "示教一遍该动作即可补全")
            tgt = [float(ep["position"][k]) + float(off[k]) for k in range(3)]
            rpy = quat_to_rpy_deg([float(v) for v in ep["orientation"]])
            self.move_target(tgt, rpy, f"{name}·重锚p{i}",
                             tol_xy_mm=mv["place_xy_tol_mm"],
                             tol_z_mm=mv["place_z_tol_mm"], speed=speed)

    def move_target(self, pos_m, rpy_deg, name="move", *,
                    tol_xy_mm: float | None = None,
                    tol_z_mm: float | None = None,
                    speed: int | None = None,
                    axis_fallback: bool = False):
        """基座系直线到目标位姿(IK 规划 + 段后 FK 验证 + 补走一次)。

        tol/speed 缺省取 move 配置与全局速度;重锚放置段由
        _traj_reanchor 传入更紧的 place_* 容差与动作速度。
        axis_fallback=True(闸门修正等纯平移段): IK 规划失败(跳解
        护栏/求解失败/不可达)时降级 HTTP 轴平移,见 _axis_fallback_move。"""
        mv = self._mv_cfg()
        xy_t = (float(tol_xy_mm) if tol_xy_mm
                else float(mv["xy_res_tol_mm"]))
        z_t = float(tol_z_mm) if tol_z_mm else float(mv["z_res_tol_mm"])
        spd = (int(speed) if speed is not None
               else speed_store.get_speed(self.arm))
        m0 = time.monotonic()
        for attempt in range(2):
            self.check_abort()
            if attempt:
                self.say(f"{name}: 未到位,重规划补走")
            cur = self.end_pose()
            try:
                out = plan_cartesian_path(
                    self.arm, self.q_deg(), op="target",
                    target_pos_m=[float(v) for v in pos_m],
                    target_rpy_deg=[float(v) for v in rpy_deg],
                    cur_pos_m=cur["position"], cur_quat=cur["orientation"])
            except RobotError as exc:
                # IK 失败且调用方允许兜底(纯平移段): 改走 HTTP 轴平移
                # —— 不经 IK,无跳解风险(2026-10-23 料仓观察位腕部
                # 奇异,T2 修正 2.5mm 跳解翻转 108°+ 走不完超时)。
                # 逆解环境缺失(_PlannerUnavailable)是系统性故障,
                # 轴平移也不该继续,原样上抛。
                if axis_fallback and not getattr(
                        exc, "planner_unavailable", False):
                    self.say(f"{name}: IK 规划失败({exc})—— 降级轴平移")
                    self._axis_fallback_move(pos_m, name, xy_t, z_t, spd, m0)
                    return
                raise
            traj = out.get("waypoints_deg") or []
            if not traj:
                raise RobotError(f"{name}: 逆解未返回轨迹")
            self.robot.move_robotarm_traj(self.arm, traj, 1,
                                          mv["move_timeout_s"], spd)
            now = self.end_pose()["position"]
            dxy = float(np.hypot(now[0] - pos_m[0], now[1] - pos_m[1])) * 1000
            dz = abs(now[2] - pos_m[2]) * 1000
            if dxy <= xy_t and dz <= z_t:
                self.say(f"{name}: 到位 Δxy={dxy:.1f}mm Δz={dz:.1f}mm")
                self.runner.runlog.event(
                    "move", self.arm, name=name, move_kind="cartesian",
                    speed_pct=spd,
                    duration_s=round(time.monotonic() - m0, 3),
                    attempts=attempt + 1, dxy_mm=round(dxy, 1),
                    dz_mm=round(dz, 1), ok=True)
                return
        self.runner.runlog.event(
            "move", self.arm, name=name, move_kind="cartesian", speed_pct=spd,
            duration_s=round(time.monotonic() - m0, 3), attempts=2,
            dxy_mm=round(dxy, 1), dz_mm=round(dz, 1),
            tol_xy_mm=xy_t, tol_z_mm=z_t, ok=False)
        raise RobotError(f"{name} 段后验证超差(Δxy={dxy:.1f}mm "
                         f"Δz={dz:.1f}mm > {xy_t:.1f}/{z_t:.1f})—— 原地停")

    def _axis_fallback_move(self, pos_m, name, xy_t, z_t, spd, m0):
        """IK 规划失败的兜底: HTTP 轴平移折线走到目标(纯平移、不经
        IK,结构上不可能跳解),走完按同容差段后验证,超差原地停。"""
        cur = self.end_pose()
        rest = [float(pos_m[i]) - cur["position"][i] for i in range(3)]
        cartesian_delta_move(self.robot, self.arm, *rest, speed=spd,
                             should_stop=lambda:
                                 self.runner.abort_event.is_set(),
                             frame="base")
        self.check_abort()          # 软停中断在这里以 _Abort 结束
        now = self.end_pose()["position"]
        dxy = float(np.hypot(now[0] - pos_m[0], now[1] - pos_m[1])) * 1000
        dz = abs(now[2] - pos_m[2]) * 1000
        if dxy > xy_t or dz > z_t:
            raise RobotError(f"{name}: 轴平移降级后仍超差(Δxy={dxy:.1f}mm "
                             f"Δz={dz:.1f}mm > {xy_t:.1f}/{z_t:.1f})—— 原地停")
        self.say(f"{name}: 轴平移到位(轴平移降级)Δxy={dxy:.1f}mm "
                 f"Δz={dz:.1f}mm")
        self.runner.runlog.event(
            "move", self.arm, name=name, move_kind="axis", speed_pct=spd,
            duration_s=round(time.monotonic() - m0, 3), attempts=1,
            dxy_mm=round(dxy, 1), dz_mm=round(dz, 1), ok=True)

    def move_delta(self, delta_m, name="correct"):
        """基座系当前位置 + Δ 直线移动(对准修正用,Δ 通常 ≤40mm)。

        IK 规划失败时降级轴平移 —— 修正是纯平移,轴平移等价且无跳解
        风险(腕部奇异位形下 IK 对 2mm 级目标也会跳解)。"""
        cur = self.end_pose()
        tgt = [cur["position"][i] + float(delta_m[i]) for i in range(3)]
        self.move_target(tgt, quat_to_rpy_deg(cur["orientation"]), name,
                         axis_fallback=True)

    # ------------------------------------------------------------ 对准闸门
    def align_gate(self, which: str = "align") -> dict:
        """三档对准闸门。which: align(中间件) / align2(料仓,仅右臂)。"""
        gate = self.runner.cfg["gate"]
        self._gate_offset = None                 # 每次闸门重置,成功才留 Δ锚
        slot = self.runner.cfg["arms"][self.arm].get(which) or {}
        where = "中间件" if which == "align" else "料仓"
        if not slot.get("enabled", True):
            self.say(f"闸门[{where}]: 配置停用,直接放行")
            return {"tier": "off"}
        if gate["mode"] == "off":
            self.say(f"闸门[{where}]: T0 纯回放模式(不测量)")
            return {"tier": "T0"}
        ref = slot.get("ref")
        if not ref:
            raise ValueError(f"{where} 对准参考未记录 —— 请先在流水线页"
                             "『记录对准参考』")
        self.set_phase(f"gate_{which}")
        m0 = time.monotonic()
        self.runner.runlog.event(
            "gate_start", self.arm, which=which, mode=gate["mode"],
            tol_xy_mm=gate["tol_xy_mm"], tol_z_mm=gate["tol_z_mm"],
            ref_key=ref.get("key"), ref_created=ref.get("created"))
        # 带录制时间:参考和当前工装/示教不配套时,一眼看出参考多旧
        # (2026-10-28 右臂中间件参考录于 10-22,中间闸门一直关着没验证,
        # 重新启用后测出 Δz12.9mm 把 B2 吸取点带偏 12.8mm)
        self.say(f"闸门[{where}]: 参考 Δ0 = "
                 f"{np.round(ref['p_base'], 3).tolist()}m"
                 + (f"(录于 {ref['created']})" if ref.get("created") else ""))
        T_x = flow_store.load_handeye_T(self.arm)    # 缺文件抛 FileNotFoundError
        sess = self.runner.cameras.get(
            self.runner.cfg["arms"][self.arm]["cam_role"])
        finder = self.runner.finder
        marker, size = slot["marker"], slot["marker_size_mm"] / 1000.0

        # 视差基准 = 参考录制时相机系的码位置。优先用录制时存的原始观感
        # p_cam —— 纯相机输出,不经 FK/手眼反解:链路(手眼/FK)后来变了
        # 也不污染旧参考,视差才真正只反映"码相对臂动了多少"(PBVS)。
        # 老参考没存 p_cam 才退回由 end_pose 反解(带链路漂移风险)。
        # 码是静止物:p_base 不随臂移动而变,基座系 Δ 修完不会缩小,
        # 拿它判收敛会把同一修正反复叠加 —— 修正后的判据改用相机系视差
        # (当前观感 vs 参考录制观感,对齐到位后应≈0)。
        ep_ref = ref.get("end_pose") or {}
        ref_pc = np.array(ref["p_cam"], float) if ref.get("p_cam") else None
        if ref_pc is None and ep_ref.get("position") \
                and ep_ref.get("orientation"):
            T_ref = T_from(ep_ref["position"], ep_ref["orientation"])
            ref_pc = (np.linalg.inv(T_ref @ T_x)
                      @ np.array([*ref["p_base"], 1.0]))[:3]

        def _pass(tier, head, **extra):
            """收敛放行公共出口: 重锚Δ记账(含过大拒走保护)+日志+grec。
            dxy/dz/cxy/cz 取调用时刻的最新测量值。"""
            ren = None
            if self._taught_end_pos:
                # Δ锚 = 当前末端 − 前一动作示教末点(T2 修正 + 回放残余),
                # 传给紧随的 reanchor 动作平移其示教轨迹
                off = (np.array(self.end_pose()["position"])
                       - np.array(self._taught_end_pos, float))
                ren = [round(float(v) * 1000.0, 2) for v in off]
                # 保险: 重锚Δ过大 = 参考录制与当前示教/工装不配套
                # (典型: 先录参考后改示教,或工装被挪)。此时把整个
                # 下降轨迹平移几十毫米,极易怼进工装/桌面卡死 ——
                # 宁可停线要求重录参考,也不盲走(2026-10-22 16:40
                # B2 重锚-31mm 卡死即此)。0=关闭该保护。
                rmax = float(gate.get("reanchor_max_mm", 25.0))
                rlen = float(np.linalg.norm(off)) * 1000.0
                if rmax > 0 and rlen > rmax:
                    self.runner.runlog.event(
                        "gate_fail", self.arm, which=which,
                        reason="reanchor_over_limit",
                        reanchor_mm=ren, limit_mm=rmax,
                        duration_s=round(time.monotonic() - m0, 3))
                    raise ValueError(
                        f"闸门[{where}] 收敛但重锚Δ={ren}mm "
                        f"超上限 {rmax:.0f}mm —— 对准参考与当前示教/"
                        "工装不配套(先录参考后改示教?工装被挪?),"
                        "请在观察位重录参考后再跑")
                self._gate_offset = off
            self.say(head + (f",重锚Δ={ren}mm" if ren else ""))
            grec = {"tier": tier, "which": which,
                    "dxy_mm": round(dxy, 1), "dz_mm": round(dz, 1),
                    "resid_xy_mm": round(cxy, 1), "resid_z_mm": round(cz, 1),
                    "reanchor_mm": ren,
                    "duration_s": round(time.monotonic() - m0, 3)}
            grec.update(extra)
            self._cycle_gates.append(grec)
            self.runner.runlog.event("gate_result", self.arm, **grec)
            return grec

        d0 = None
        crit = "基座Δ"
        for step in range(int(gate["iterations"]) + 1):
            self.check_abort()
            # 与『回观察位并测』一致: 运动收敛后先等稳定再测(闸门原本漏了
            # 这步,到位即测会吃到残余振动/对焦呼吸)
            self.runner._sleep_abort(float(gate["settle_s"]))
            ep = self.end_pose()
            m = measure_marker_base(sess, finder, marker, size,
                                    T_from(ep["position"], ep["orientation"]),
                                    T_x, frames=int(gate["frames"]))
            d = np.array(m["p_base"]) - np.array(ref["p_base"])
            dxy = float(np.hypot(d[0], d[1])) * 1000
            dz = float(abs(d[2])) * 1000
            key_ok = m["key"] == ref.get("key")
            fu = int(m.get("frames_used") or 0)
            self.say(f"测量{step}: Δxy={dxy:.1f}mm Δz={dz:.1f}mm "
                     f"z_cam={m['z_cam_mm']:.0f}mm 帧{fu}/{gate['frames']}"
                     f" key={m['key']}"
                     + (" ⚠解码帧少,读数可能抖" if fu < 3 else "")
                     + ("" if key_ok else " ⚠码身份与参考不一致!"))
            self.runner.runlog.event(
                "gate_measure", self.arm, which=which, step=step,
                dxy_mm=round(dxy, 1), dz_mm=round(dz, 1),
                z_cam_mm=m["z_cam_mm"], key=m["key"],
                frames_used=m.get("frames_used"),
                spread_mm=m.get("spread_mm"))
            if not key_ok:
                self.runner.runlog.event(
                    "gate_fail", self.arm, which=which, step=step,
                    reason="key_mismatch", key=m["key"],
                    ref_key=ref.get("key"))
                raise ValueError(f"对准失败: 码身份变了(测到 {m['key']}, "
                                 f"参考 {ref.get('key')})—— 换码/挪码要重记参考")
            if step == 0:
                d0 = d
                cxy, cz, crit = dxy, dz, "基座Δ"
                ok = cxy <= gate["tol_xy_mm"] and cz <= gate["tol_z_mm"]
                if not ok and fu < int(gate["frames"]):
                    # 步0 复测: 解码帧不足时中值可能整批带偏(小码@0.4m
                    # 边际解码),单次读数超差不足以进修正循环 ——
                    # 2026-10-23 14:37 中间件 3/5 帧读出 Δxy 5.9mm 假
                    # 阳性,按它修正把臂真的挪走 5.8mm,视差循环从此追
                    # 噪声打转。静止复测一次: 达标即 T1 放行;仍超差取
                    # 两次中点降噪再修。臂静止,复测安全。
                    self.say(f"闸门[{where}]: 步0 超差但解码帧少({fu}/"
                             f"{gate['frames']})—— 静止复测一次再判")
                    m2 = measure_marker_base(
                        sess, finder, marker, size,
                        T_from(ep["position"], ep["orientation"]),
                        T_x, frames=int(gate["frames"]))
                    d2 = np.array(m2["p_base"]) - np.array(ref["p_base"])
                    dxy2 = float(np.hypot(d2[0], d2[1])) * 1000
                    dz2 = float(abs(d2[2])) * 1000
                    self.say(f"复测0: Δxy={dxy2:.1f}mm Δz={dz2:.1f}mm "
                             f"帧{int(m2.get('frames_used') or 0)}/"
                             f"{gate['frames']}")
                    self.runner.runlog.event(
                        "gate_measure", self.arm, which=which, step=0,
                        dxy_mm=round(dxy2, 1), dz_mm=round(dz2, 1),
                        z_cam_mm=m2["z_cam_mm"], key=m2["key"],
                        frames_used=m2.get("frames_used"),
                        spread_mm=m2.get("spread_mm"), recheck=True)
                    if dxy2 <= gate["tol_xy_mm"] and dz2 <= gate["tol_z_mm"]:
                        d, dxy, dz, ok = d2, dxy2, dz2, True
                    else:
                        d = (d + d2) / 2.0
                        dxy = float(np.hypot(d[0], d[1])) * 1000
                        dz = float(abs(d[2])) * 1000
                    d0, cxy, cz = d, dxy, dz
                corr = d                            # 首步: 基座系全量修正
            elif ref_pc is not None:
                # 修正后: 相机系相对参考录制的视差(对齐到位→≈0)
                r = np.array(m["p_cam"]) - ref_pc
                cxy = float(np.hypot(r[0], r[1])) * 1000
                cz = float(abs(r[2])) * 1000
                crit = "视差"
                ok = cxy <= gate["tol_xy_mm"] and cz <= gate["tol_z_mm"]
                # 剩余视差(向量,非点)换回基座系: 只乘旋转,平移不参与
                corr = (T_from(ep["position"], ep["orientation"])[:3, :3]
                        @ T_x[:3, :3] @ r)
            else:
                # 旧参考缺 end_pose: 退化为测量稳定性(码静止→Δ 应不变)
                dd = d - d0
                cxy = float(np.hypot(dd[0], dd[1])) * 1000
                cz = float(abs(dd[2])) * 1000
                crit = "稳定性"
                ok = cxy <= gate["tol_xy_mm"] and cz <= gate["tol_z_mm"]
                corr = dd
            if ok:
                head = (f"闸门[{where}]: ✓ "
                        f"{'T1 免修' if step == 0 else f'T2 第{step}步收敛'}"
                        f"({crit} {cxy:.1f}/{cz:.1f}mm"
                        + ("" if step == 0 else
                           f",基座Δ {dxy:.1f}/{dz:.1f}mm 属预期(码静止)")
                        + ")")
                return _pass("T1" if step == 0 else "T2", head)
            if (float(np.hypot(corr[0], corr[1])) * 1000 > gate["max_corr_mm"]
                    or float(abs(corr[2])) * 1000 > gate["max_corr_mm"]):
                self.runner.runlog.event(
                    "gate_fail", self.arm, which=which, step=step,
                    reason="corr_over_limit", crit=crit,
                    cxy_mm=round(cxy, 1), cz_mm=round(cz, 1),
                    corr_mm=[round(float(v) * 1000, 2) for v in corr],
                    duration_s=round(time.monotonic() - m0, 3))
                # 分型: 视差巨大但基座Δ很小 = 码没动,是观察位姿与录参考
                # 时不一致(该路点录参考后重示教过)—— 17:04 料仓 65mm
                # 视差 vs 基座Δ 4.5mm 即此,别再吓人说是码被挪
                hint = (f"基座Δ仅 {dxy:.1f}/{dz:.1f}mm(码没动)但视差巨大"
                        f" → 观察位姿与录参考时不一致(该路点重示教过?),"
                        f"在观察位重录参考即可"
                        if max(cxy, cz) >= max(20.0, 3.0 * max(dxy, dz))
                        else "疑似工装被撞/码被挪/手眼可疑,"
                             "请人工确认后重记参考")
                raise ValueError(
                    f"闸门[{where}] 拒绝修正: {crit}待修"
                    f"({cxy:.1f}/{cz:.1f}mm) > 上限 {gate['max_corr_mm']}mm"
                    f" —— {hint}")
            if step >= int(gate["iterations"]):
                break
            self.say(f"闸门[{where}]: T2 修正 {np.round(corr, 4).tolist()}m")
            self.runner.runlog.event(
                "gate_fix", self.arm, which=which, step=step,
                corr_mm=[round(float(v) * 1000, 2) for v in corr])
            try:
                self.move_delta(corr, f"align_fix{step}")
            except RobotError as e:
                msg = str(e)
                if ("几乎一致" not in msg and "无需运动" not in msg
                        and "near-zero trajectory" not in msg):
                    raise
                # 修正量低于运动学分辨率 —— IK 判定目标与当前位姿等价
                # 拒绝出轨迹;降级轴平移后厂商接口同样拒(单次平移须
                # 引起 ≥0.5° 关节变化,2026-10-23 13:39 观察位姿下
                # 2.3mm 修正仅 0.385° 被拒即此)—— 重测/重试也一样,
                # 按收敛放行。
                # 常见于 Δ 刚蹭过容差边缘(如 2.5mm vs 容差 2.0mm,
                # 换算关节仅 ~0.4°):测量噪声±1mm 的量级,不值得停线。
                # 只留量级:IK"最大关节变化 X°" / 厂商"max_delta_deg=X"
                why = (msg.split("near-zero trajectory:")[-1].strip()
                       if "near-zero trajectory" in msg
                       else msg.split("(")[-1].split(")")[0])
                return _pass(
                    "T2",
                    f"闸门[{where}]: ✓ 修正量低于运动分辨率({why})"
                    f" —— 按收敛放行(Δ {dxy:.1f}/{dz:.1f}mm ≈ 容差边缘)",
                    low_res=True)
        if (dxy <= gate["tol_xy_mm"] and dz <= gate["tol_z_mm"]
                and cxy <= 1.5 * gate["tol_xy_mm"]
                and cz <= 1.5 * gate["tol_z_mm"]):
            # 迭代用尽但基座Δ达标(码静止时与臂位姿无关的判据,抓取
            # 真正用的坐标)且视差未失控(<1.5×容差,只容噪声压线)→
            # 按噪声边缘放行,不停线。2026-10-23 14:37 中间件: 步0
            # 3/5 帧假阳性 5.9mm → 修正把臂挪走 → 视差追噪声反号震荡,
            # 3 步耗尽,视差 z 4.0x 恰压容差 4.0 停线,而基座Δ 1.0/1.0
            # 明明达标。视差 >1.5× 仍走停线,⑲"参考过期(65mm 视差)"
            # 分型不受影响。
            return _pass("T2", f"闸门[{where}]: ✓ T2 噪声边缘放行(基座Δ "
                         f"{dxy:.1f}/{dz:.1f}mm 达标,{crit} {cxy:.1f}/"
                         f"{cz:.1f}mm 压线,迭代用尽)", noise_edge=True)
        self.runner.runlog.event(
            "gate_fail", self.arm, which=which, reason="not_converged",
            crit=crit, cxy_mm=round(cxy, 1), cz_mm=round(cz, 1),
            dxy_mm=round(dxy, 1), dz_mm=round(dz, 1),
            duration_s=round(time.monotonic() - m0, 3))
        # 保险: 失败时把两个判据都摆出来,并区分"参考观感过期"与
        # "码/工装真被挪了" —— 前者重录参考即愈,后者要人工确认
        hint = ("基座Δ已达标但视差超差 → 参考录制时的相机观感与当前不"
                "一致(链路变动/老参考),重录参考即可"
                if dxy <= gate["tol_xy_mm"] and dz <= gate["tol_z_mm"]
                else "基座Δ也超差 → 疑似码/工装被挪,人工确认后重录参考")
        raise ValueError(f"闸门[{where}] {gate['iterations']} 步修正后仍超差"
                         f"({crit} {cxy:.1f}/{cz:.1f}mm,基座Δ "
                         f"{dxy:.1f}/{dz:.1f}mm) —— 停线。{hint}")

    # ------------------------------------------------------------ 左臂: 视觉吸取
    def scan_and_pick(self):
        """物料检测 + 吸取 —— 与『物料检测』页一步到位同款行为:
        ①回放扫描位→YOLO+深度定位 ②③按 pick 配置 motion_mode 二选一:
        direct=一键直达(扫描位→aim 一段直线,不可达退分段走廊)/
        corridor=升越顶→(差>2°)转对准姿态→水平平移→下降(不可达退层内)
        ④下压(沿工具系 Z+/吸嘴轴向) ⑤真空/保持沿用 pick 配置,
        吸完即止(不抬升,A1 回放直接接管)。
        每个子步独立 phase(scan/pick_hover/…/pick_grab):
        运行徽章、单步确认与 phase_times 都按子步呈现,不再笼统一个"pick"。"""
        pcfg = pick_store.load_config()
        if not pcfg.get("scan_pose"):
            raise ValueError("左臂扫描位未示教 —— 请在『物料检测』页"
                             "『存当前为扫描位』")
        # ① 物料检测: 回扫描位 → 取帧 → YOLO → 深度 → 基座系点位 → 分层排序
        self.set_phase("scan")
        self.say("① 物料检测: 回扫描位 → 取帧 → YOLO+深度 → 基座系定位")
        out = scan_once(self.robot, self.runner.pick_adapter, pcfg,
                        replay=True, say=self.say)
        rows = out["rows"]
        if not rows:
            raise ValueError("物料检测未发现可吸取目标 —— 请补料,"
                             "或到『物料检测』页检查检测配置/光照")
        row = rows[0]                                # 队首(高层优先)
        self.say(f"吸取目标 #{row['order']}(层{row['layer']}) "
                 f"{row['cls']} z={row['z_mm']:.0f}mm")
        self.runner.runlog.event(
            "scan", self.arm, rows=len(rows), target_order=row["order"],
            layer=row["layer"], cls=row["cls"],
            z_mm=round(float(row["z_mm"]), 1),
            p_base_mm=[round(float(v) * 1000.0, 1) for v in row["p_base"]])
        res = np.array(pcfg.get("residual_mm", [0, 0, 0]), float) / 1000.0
        per = (pcfg.get("per_bag_residual_mm") or {}).get(str(row["order"]))
        if per:
            res = res + np.array(per, float) / 1000.0
        aim_xy = np.array(row["p_base"][:2]) + res[:2]
        aim_z = row["p_base"][2] + pcfg["hover_mm"] / 1000.0 + res[2]
        cur = self.end_pose()
        horiz = float(np.linalg.norm(aim_xy - np.array(cur["position"][:2])))
        if horiz > pcfg["max_horiz_m"]:
            raise ValueError(f"水平行程 {horiz * 1000:.0f}mm 超上限"
                             f" {pcfg['max_horiz_m'] * 1000:.0f}mm")
        align = pcfg.get("align")
        if align:                                    # 深度护栏(台面高度)
            dz = abs(aim_z - float(align["z_ref_m"]))
            if dz > pcfg["max_dz_m"]:
                raise ValueError(f"深度基准差 {dz * 1000:.0f}mm 超上限 "
                                 f"{pcfg['max_dz_m'] * 1000:.0f}mm"
                                 "(台面高度变了?重新校正)")
        top_z = max(r["p_base"][2] for r in rows)
        cruise_z = max(top_z + pcfg["cruise_clearance_mm"] / 1000.0,
                       cur["position"][2])
        # ②③ 移到物料上方并下降: 沿用『物料检测』页 motion_mode 设置
        # (direct=一键直达一段直线;corridor=升越顶→转姿态→水平→降)
        cur_rpy = quat_to_rpy_deg(cur["orientation"])
        tgt_rpy = list(align["rpy_deg"]) if align else cur_rpy   # 对准姿态

        def _corridor():
            """分段走廊: 升越顶→(差>2°)转姿态→水平平移→降(不可达退层内)。"""
            self.set_phase("pick_hover")
            now = self.end_pose()          # 现读: direct 失败退回时臂已挪位
            rpy_now = quat_to_rpy_deg(now["orientation"])
            self.say(f"② 分段走廊: 越顶 z={cruise_z:.3f}m"
                     f"(最高面 {top_z:.3f}m+{pcfg['cruise_clearance_mm']:.0f}mm)"
                     f"|水平行程 {horiz * 1000:.0f}mm")
            self.move_target([now["position"][0], now["position"][1],
                              cruise_z], rpy_now, "lift")
            if align and max(abs(a - b) for a, b in
                             zip(tgt_rpy, rpy_now)) > 2.0:
                self.move_target(self.end_pose()["position"], tgt_rpy, "rotate")
            try:
                self.move_target([aim_xy[0], aim_xy[1], cruise_z],
                                 tgt_rpy, "horizontal")
            except RobotError as e:
                layer_z = row["p_base"][2] + pcfg["hover_mm"] / 1000.0
                self.say(f"越顶平移不可达({e});退层内走廊 z={layer_z:.3f}m")
                self.move_target([aim_xy[0], aim_xy[1], layer_z],
                                 tgt_rpy, "horizontal")
            self.set_phase("pick_descend")
            self.move_target([aim_xy[0], aim_xy[1], aim_z], tgt_rpy, "descend")

        self.set_phase("pick_hover")
        if pcfg.get("motion_mode") == "direct":
            # 一键直达: 扫描位→袋心 aim 单段直线,姿态同步转到示教对准
            # 姿态(贴两端点高度横穿≈袋顶;途中避障靠扫描位摆放+行程护栏)
            dist = float(np.linalg.norm(
                np.array([aim_xy[0], aim_xy[1], aim_z]) -
                np.array(cur["position"])))
            self.say(f"② 一键直达: 扫描位→aim 一段直线 {dist * 1000:.0f}mm"
                     "(贴袋顶高度横穿)")
            try:
                self.move_target([aim_xy[0], aim_xy[1], aim_z],
                                 tgt_rpy, "direct")
            except RobotError as e:
                self.say(f"直达不可达({e});退回分段走廊")
                _corridor()
        else:
            _corridor()
        # ④ 下压(沿工具系 Z+/吸嘴轴向,与页面一致)
        press = float(pcfg.get("press_mm") or 0.0)
        if press > 0:
            self.set_phase("pick_press")
            self.say(f"④ 下压: 沿吸嘴轴向(工具Z+) {press:.1f}mm")
            dvec = rpy_deg_to_R(*tgt_rpy) @ np.array(
                [0.0, 0.0, press / 1000.0])
            xy = [aim_xy[0] + float(dvec[0]), aim_xy[1] + float(dvec[1])]
            pz = aim_z + float(dvec[2])
            self.move_target([xy[0], xy[1], pz], tgt_rpy, "press")
        # ⑤ 吸取: 真空方式沿用『物料检测』页设置(manual=保持窗口内人工
        #    接真空;gripper=夹爪联动自动)。吸完即结束,不做垂直抬升
        #    —— 由 A1 回放从当前位置直接接管(用户 2026-10-22 要求)
        self.set_phase("pick_grab")
        self.runner.vacuum(self, "grab", vac=pcfg.get("vacuum"),
                           hold_s=float(pcfg.get("hold_s", 2.0)))

    # ------------------------------------------------------------ 循环体
    def _loop(self):
        if self.arm == "armleft":
            self._loop_left()
        else:
            self._loop_right()

    def _has_center_marks(self, names) -> bool:
        """相关动作任一路点带"中点"标记 → 中心互斥走路点级(标记段
        进前抢锁/走到非标记点交锁,由 play_action 管理);都不标=维持
        整段互斥(A1→A2 / B1→B2 一把锁,循环包裹)。"""
        acts = self.runner.cfg["arms"][self.arm]["actions"]
        return any(bool(w.get("center"))
                   for nm in names for w in acts[nm]["waypoints"])

    def _release_center_or_warn(self, name: str):
        """中心段收尾: 仍持锁(示教末点还是中点,没教退出点)则提前
        交锁保活并提示补示教 —— 否则右臂会被堵到本臂下轮回扫描。"""
        hand = self.runner.handoff
        if hand.center_owner == self.arm:
            self.say(f"⚠ {name} 末点仍是中点: 原位提前交锁"
                     f"(建议给 {name} 末段补一个非中点退出路点,"
                     f"或给下一动作首点标 ●中点 让锁带到出口)")
            hand.exit_center(self.arm)

    def _loop_left(self):
        # safety.center_serialize(首跑建议): 左臂吸取后先在取料区等
        # 中间件 EMPTY 再进中心,持锁贯穿 A1→闸门→A2 —— 消除"左臂停在
        # 中间件上方观察位等料时,右臂 B1/B2 进中心取料"的双臂同区窗口
        ser = bool((self.runner.cfg.get("safety") or {})
                   .get("center_serialize"))
        while True:
            self.check_abort()
            self._cycle_t0 = time.time()
            self._cycle_phases, self._cycle_actions, self._cycle_gates = \
                [], [], []
            self.post_grip("start")
            self.scan_and_pick()
            if ser and not self.single_shot:
                self.runner.handoff.wait_fixture("EMPTY", self)
            wp_center = self._has_center_marks(
                ("A1_transfer", "A2_place"))
            if not wp_center:
                self.runner.handoff.enter_center(self)
            try:
                self.play_action("A1_transfer")
                self.post_grip("A1_transfer")
                self.align_gate("align")
                if not ser and not self.single_shot:
                    self.runner.handoff.wait_fixture("EMPTY", self)
                self.play_action("A2_place", reanchor=True)   # 闸门Δ传导
                if wp_center:
                    self._release_center_or_warn("A2_place")
            finally:
                self.runner.handoff.exit_center(self.arm)
            self.runner.handoff.set_filled(self.arm)
            self.post_grip("A2_place")
            self.cycles += 1
            self._mark_cycle()
            if self.single_shot:
                self.say("◎ 单臂测试: 1 个循环完成,自动停止"
                         "(中间件已置 FILLED,实物留在中间件)")
                return
            if self.max_cycles and self.cycles >= self.max_cycles:
                self.say(f"◎ 已跑满 {self.max_cycles} 轮,自动停止")
                return

    def post_grip(self, slot: str):
        """动作间夹爪整形(通用): 槽位=某动作名或 "start"(循环首步前),
        该动作完成后把夹爪调到配置的开合目标值并等待 wait_s
        (二次夹紧/稳料/换夹姿态)。插槽任意开关,不限 B2→B3。"""
        g = ((self.runner.cfg["arms"][self.arm].get("post_grippers")
              or {}).get(slot)) or {}
        if not g.get("enabled"):
            return
        pos = float(g.get("position", 300))
        wait = float(g.get("wait_s", 1.0))
        self.say(f"夹爪整形[{slot}]: 开口→{pos:g},等 {wait:g}s")
        self.robot.set_gripper(self.side, pos,               # 失败抛错 → 停线
                               int(g.get("velocity", 500)))
        self.runner._sleep_abort(wait)

    def _loop_right(self):
        while True:
            self.check_abort()
            self._cycle_t0 = time.time()
            self._cycle_phases, self._cycle_actions, self._cycle_gates = \
                [], [], []
            self.post_grip("start")
            if len(self.runner.cfg["arms"]["armright"]["actions"]
                   ["B0_pre"]["waypoints"]) > 0:
                self.play_action("B0_pre")
            self.post_grip("B0_pre")
            if not self.single_shot:
                if self.cycles == 0:
                    self.say("待命: 等中间件首件(左臂放完置 FILLED)后"
                             "B1 进中心取料 —— 启动时中间件为空,"
                             "右臂首轮无活属正常(流水线灌入)")
                self.runner.handoff.wait_fixture("FILLED", self)
            wp_center = self._has_center_marks(
                ("B1_approach", "B2_pick"))
            # B3 首点也标了 ●中点 → 锁不在 B2 末原位交,带着走过夹爪整形
            # 和段间隙直达 B3 的退出点(B3 内部"已出中心,交锁"):消灭
            # "B2 原位交锁 → 整形 1s → B3 再抢锁"的缝 —— 17:12 左臂正是
            # 从这 1s 缝里先进中心,右臂被堵在取料位原地等锁十几秒。
            # b3_holds=False(B3 首点未标)维持原样:B2 末原位交锁+⚠。
            b3wps = self.runner.cfg["arms"][self.arm]["actions"][
                "B3_transfer"]["waypoints"]
            b3_holds = bool(b3wps) and bool(b3wps[0].get("center"))
            if not wp_center:
                self.runner.handoff.enter_center(self)
            try:
                self.play_action("B1_approach")
                self.post_grip("B1_approach")
                self.align_gate("align")
                self.play_action("B2_pick", reanchor=True)    # 闸门Δ传导
                if wp_center and not b3_holds:
                    self._release_center_or_warn("B2_pick")
            finally:
                if not b3_holds:   # 持锁进入 B3;异常时锁随本次运行一起销毁
                    self.runner.handoff.exit_center(self.arm)
            self.runner.handoff.set_empty(self.arm)
            self.post_grip("B2_pick")              # 可选:B2→B3 夹爪整形
            self.play_action("B3_transfer")
            self.post_grip("B3_transfer")
            self.align_gate("align2")
            self.play_action("B4_place", reanchor=True)       # 闸门Δ传导
            self.post_grip("B4_place")
            self.cycles += 1
            self._mark_cycle()
            if self.single_shot:
                self.say("◎ 单臂测试: 1 个循环完成,自动停止"
                         "(中间件已置 EMPTY,请确认料已入仓)")
                return
            if self.max_cycles and self.cycles >= self.max_cycles:
                self.say(f"◎ 已跑满 {self.max_cycles} 轮,自动停止"
                         f"(第 {self.cycles} 件已送完料仓)")
                return

    def _mark_cycle(self):
        if self.phase and self.phase != "cycle_gap":
            self.set_phase("cycle_gap")    # 收口末个动作,间隙计入下一轮
        dt = time.time() - self._cycle_t0
        self.cycle_ts.append(dt)
        del self.cycle_ts[:-20]
        rec = {"cycle": self.cycles,
               "t_start": flow_runlog.iso(self._cycle_t0),
               "t_end": flow_runlog.iso(),
               "duration_s": round(dt, 3),
               "avg_recent_s": round(float(np.mean(self.cycle_ts)), 3),
               "speed_pct": speed_store.get_speed(self.arm),
               "phases": self._cycle_phases,
               "actions": self._cycle_actions,
               "gates": self._cycle_gates}
        self.say(f"◎ 循环 #{self.cycles} 完成 {dt:.1f}s"
                 f"(近均值 {np.mean(self.cycle_ts):.1f}s,"
                 f"speed {rec['speed_pct']}%)")
        self.runner.runlog.event("cycle", self.arm, **rec)

    def status(self) -> dict:
        return {
            "phase": self.phase,
            "error": self.error,
            "cycles": self.cycles,
            "last_cycle_s": round(self.cycle_ts[-1], 1) if self.cycle_ts
            else None,
            "avg_cycle_s": round(float(np.mean(self.cycle_ts)), 1)
            if self.cycle_ts else None,
            "phase_times": self.phase_times,
            "log": list(self.log)[-40:],
        }


# ===========================================================================
# 编排器
# ===========================================================================
class FlowRunner:
    """app.py 单例。start() 起两条臂线程;stop() 段边界软停。"""

    def __init__(self):
        self.robot: TianlianRobot | None = None
        self.cameras = FlowCameras()
        self.finder = MarkerFinder()
        self.handoff = Handoff(self)
        self.runlog = flow_runlog.NullLogger()
        self._run_t0: float | None = None
        self.log: deque = deque(maxlen=120)
        self.abort_event = threading.Event()
        self._threads: list[threading.Thread] = []
        self._running = False
        self._failed: str | None = None
        self.step_mode = False
        self._step_label: dict[str, str] = {}       # arm -> 等待确认的步骤名
        self._step_confirmed = threading.Event()
        self.arms: dict[str, ArmLoop] = {}
        self.pick_adapter = None
        self.cfg: dict = {}

    # ------------------------------------------------------------ 生命周期
    def start(self, robot: TianlianRobot, *, step_mode: bool = False,
              only: str | None = None, rounds: int | None = None) -> dict:
        """only: None=双臂连续 / "left"|"right"=单臂测试(只跑 1 个循环
        自动停,不等另一臂的令牌 —— 检查也只查该臂的前置项)。
        rounds: None/0=连续不限轮 / N=每臂跑满 N 个循环后整线自动停
        (左臂放完第 N 件即停;右臂把第 N 件送完料仓才停)。"""
        if self._running:
            raise FlowError("流水线已在运行")
        if rounds is None or rounds == 0:
            rounds = None
        elif (not isinstance(rounds, int) or isinstance(rounds, bool)
                or not 1 <= rounds <= 999):
            raise FlowError("rounds 需为 1~999 的整数(0/None=连续不限轮)")
        only_arm = {"left": "armleft", "right": "armright",
                    None: None}.get(only)
        if only is not None and only_arm is None:
            raise FlowError(f'only 只能是 "left"/"right",收到 {only!r}')
        cfg = flow_store.load_config()
        rd = flow_store.readiness(cfg, only=only_arm)
        if rd["missing"]:
            raise FlowError("前置检查未通过: " + ";".join(rd["missing"]))

        self.robot = robot
        self.cfg = cfg
        self.step_mode = bool(step_mode)
        self.rounds = rounds
        self.abort_event.clear()
        self._failed = None
        self.runlog = flow_runlog.RunLogger(
            tag={"armleft": "left", "armright": "right"}.get(only_arm,
                                                             "dual"))
        self._run_t0 = time.time()
        try:
            self._start_arms(cfg, robot, only_arm, step_mode)
        except BaseException:
            try:
                self.runlog.close("启动失败,未起线程")
            except Exception:                       # noqa: BLE001
                pass
            self.runlog = flow_runlog.NullLogger()
            raise

    def _start_arms(self, cfg: dict, robot, only_arm, step_mode: bool):
        self.handoff = Handoff(self)
        self.handoff.fixture = "EMPTY"
        wanted = [only_arm] if only_arm else ["armleft", "armright"]
        self.arms = {a: ArmLoop(self, a) for a in wanted}
        for loop in self.arms.values():
            loop.single_shot = only_arm is not None
            loop.max_cycles = 1 if only_arm else self.rounds
        self.pick_adapter = self.cameras.as_pick_provider("left")
        # 预热相机: 左臂扫描路(带深度)在 start 时就要用,提前把租约拿到;
        # 右臂单测用不到左深度,只热右相机彩色(免深度租约)
        if only_arm == "armright":
            self.cameras.get(cfg["arms"]["armright"]["cam_role"])
        else:
            role = pick_store.load_config().get("cam_role", "left")
            self.cameras.get(role, need_depth=True)
        self._running = True
        self.note(f"流水线启动(mode={cfg['gate']['mode']}"
                  f"{', 单步确认' if step_mode else ''}"
                  + (f", 单臂测试:{only_arm} 仅1循环" if only_arm else
                     (f", {self.rounds}轮后自动停" if self.rounds else
                      ", 连续")) + ")")
        self._log_start(cfg, only_arm, step_mode)
        for arm in wanted:
            t = threading.Thread(target=self.arms[arm].run, daemon=True,
                                 name=f"flow-{arm}")
            t.start()
            self._threads.append(t)
        threading.Thread(target=self._watch_threads, daemon=True,
                         name="flow-watch").start()
        return {"ok": True, "gate_mode": cfg["gate"]["mode"],
                "step_mode": step_mode, "only": only_arm,
                "rounds": self.rounds}

    def _log_start(self, cfg: dict, only_arm, step_mode: bool):
        """启动配置快照进运行日志(节拍测算要知道当时跑的是什么)。"""
        sp = speed_store.get_speeds()
        self.runlog.event(
            "start", None, only=only_arm, step_mode=step_mode,
            rounds=self.rounds,
            gate=dict(cfg["gate"]), move=dict(cfg["move"]),
            safety=dict(cfg.get("safety") or {}), speeds=sp)
        for a in self.arms:
            slot = cfg["arms"][a]
            refs = {}
            for w in ("align", "align2"):
                s = slot.get(w) or {}
                r = s.get("ref")
                refs[w] = {"enabled": s.get("enabled", True),
                           **({"key": r.get("key"),
                               "created": r.get("created")} if r else {})}
            self.runlog.event(
                "arm_config", a, cam_role=slot.get("cam_role"),
                waypoints={k: len(v["waypoints"])
                           for k, v in slot["actions"].items()},
                vacuums={k: v.get("vacuum")
                         for k, v in slot["actions"].items()},
                refs=refs)

    def _watch_threads(self):
        """双臂线程都退出后收尾(异常停线时释放相机租约/复位状态)。"""
        for t in self._threads:
            t.join()
        self._finish()

    def stop(self) -> dict:
        if not self._running:
            return {"ok": True, "note": "流水线未在运行"}
        self.runlog.event("stop", None)
        self.abort_event.set()
        self._step_confirmed.set()                  # 放行单步等待
        for t in self._threads:
            t.join(timeout=10.0)
        self._finish()
        return {"ok": True, "note": "已请求软停(段边界生效)"}

    def _finish(self):
        if not self._running:
            return
        self._running = False
        self._threads = []
        try:
            self.cameras.shutdown()
        except Exception:                           # noqa: BLE001
            pass
        # 自然跑完(轮次到/单臂测试)=完成;软停/异常=停止
        self.note("流水线完成" if (not self.abort_event.is_set()
                                 and not self._failed) else "流水线停止")
        try:
            dur = (time.strftime("%H:%M:%S",
                                 time.gmtime(time.time() - self._run_t0))
                   if self._run_t0 else "")
            self.runlog.close(f"总时长 {dur}" if dur else "")
        except Exception:                           # noqa: BLE001
            pass
        self.runlog = flow_runlog.NullLogger()

    def abort(self, why: str):
        if not self._running:
            return
        self._failed = why
        self.abort_event.set()
        self._step_confirmed.set()
        self.note(f"‼ 异常停线: {why}")
        self.runlog.event("abort", None, why=why)

    # ------------------------------------------------------------ 单步确认
    def _step_gate(self, arm: ArmLoop):
        if not self.step_mode or not self._running:
            return
        label = f"{arm.arm}:{arm.phase}"
        self._step_label[arm.arm] = label
        arm.say(f"⏸ [单步] {arm.phase} 待确认")
        while self._running and not self._step_confirmed.wait(timeout=0.4):
            if self.abort_event.is_set():
                self._step_label.pop(arm.arm, None)
                raise _Abort()
        self._step_confirmed.clear()
        self._step_label.pop(arm.arm, None)

    def confirm_step(self):
        self._step_confirmed.set()
        return {"ok": True}

    # ------------------------------------------------------------ 真空联动
    def vacuum(self, arm: ArmLoop, event: str, vac: dict | None = None,
               hold_s: float | None = None):
        """event: grab/release。vac 缺省时取该臂配置(左臂吸取传 pick 配置)。
        gripper 模式=联动夹爪通道;manual=人工窗口(等 hold 秒)。
        hold_s 显式传入时覆盖闸门默认(吸取步沿用 pick 配置的 hold_s)。"""
        gate = self.cfg["gate"]
        hold = (float(hold_s) if hold_s is not None else
                (gate["hold_grab_s"] if event == "grab"
                 else gate["hold_open_s"]))
        if vac is None:
            slot = self.cfg["arms"][arm.arm]
            vac = (slot.get("vacuum_grab") if event == "grab"
                   else slot.get("vacuum_release")) or {"mode": "manual"}
        m0 = time.monotonic()
        if vac.get("mode") == "gripper":
            try:
                self.robot.set_gripper(
                    arm.side, float(vac.get("gripper_position", 0)),
                    int(vac.get("gripper_velocity", 500)))
                arm.say(f"真空{event}(夹爪联动 pos="
                        f"{vac.get('gripper_position')})")
            except RobotError as e:
                arm.say(f"夹爪联动失败(继续,注意物料!): {e}")
            self._sleep_abort(hold)
        else:
            arm.say(f"真空{event}: 手动模式 —— 现在"
                    f"{'接' if event == 'grab' else '断'}真空,等 {hold}s")
            self._sleep_abort(hold)
        self.runlog.event(
            "vacuum", arm.arm, event=event, mode=vac.get("mode"),
            gripper_position=vac.get("gripper_position"),
            hold_s=hold, duration_s=round(time.monotonic() - m0, 3))

    def _sleep_abort(self, s: float):
        t0 = time.time()
        while time.time() - t0 < s:
            if self.abort_event.is_set():
                raise _Abort()
            time.sleep(0.1)

    # ------------------------------------------------------------ 状态
    def note(self, msg: str):
        self.log.append(f"[{time.strftime('%H:%M:%S')}] {msg}")
        self.runlog.line(f"[flow] {msg}")

    @property
    def running(self) -> bool:
        return self._running and any(t.is_alive() for t in self._threads)

    def status(self) -> dict:
        if not self.arms:
            return {"running": False, "gate_mode": None}
        return {
            "running": self.running,
            "failed": self._failed,
            "gate_mode": self.cfg.get("gate", {}).get("mode"),
            "step_mode": self.step_mode,
            "waiting_step": dict(self._step_label) or None,
            "fixture": self.handoff.fixture,
            "center_owner": self.handoff.center_owner,
            "arms": {a: self.arms[a].status() for a in self.arms},
            "log": list(self.log)[-30:],
        }


# ===========================================================================
# 示教端点用的独立操作(app.py 调用,不依赖流水线在跑)
# ===========================================================================
def record_align_ref(robot: TianlianRobot, cameras: FlowCameras,
                     finder: MarkerFinder, arm: str, which: str) -> dict:
    """把当前臂位姿下看到的码位置记为该站对准参考(臂必须已在对准观察位)。"""
    import cv2
    cfg = flow_store.load_config()
    slot = cfg["arms"].get(arm, {}).get(which)
    if slot is None:
        raise ValueError(f"{arm} 没有 {which} 对准槽位")
    T_x = flow_store.load_handeye_T(arm)            # 右臂未标定在这里抛
    ep = (robot.get_end_poses().get(arm)) or {}
    pos, quat = ep.get("position"), ep.get("orientation")
    if not pos or not quat:
        raise RobotError("末端位姿读取失败(臂未上电?)")
    role = cfg["arms"][arm].get("cam_role",
                                "left" if arm == "armleft" else "right")
    sess = cameras.get(role)
    m = measure_marker_base(sess, finder, slot["marker"],
                            slot["marker_size_mm"] / 1000.0,
                            T_from(pos, quat), T_x,
                            frames=int(cfg["gate"]["frames"]))
    # 参考截图(带角点)供 UI 核对码是否拍全/拍正
    try:
        import base64
        frame = sess.grab_color(timeout_s=4.0)
        found = finder.find(frame["color"], slot["marker"])
        if found:
            pts = np.array(found["corners"], np.int32)
            cv2.polylines(frame["color"], [pts], True, (0, 255, 0), 2)
            cv2.circle(frame["color"],
                       (int(found["center"][0]), int(found["center"][1])),
                       4, (0, 0, 255), -1)
        ok, buf = cv2.imencode(".jpg", frame["color"],
                               [cv2.IMWRITE_JPEG_QUALITY, 70])
        snap = ("data:image/jpeg;base64,"
                + base64.b64encode(buf).decode()) if ok else None
    except Exception:                                # noqa: BLE001
        snap = None
    q = robot.get_joint_angles().get(arm) or []
    ref = {"p_base": m["p_base"], "key": m["key"], "z_cam_mm": m["z_cam_mm"],
           "p_cam": m["p_cam"],           # 相机系原始观感(PBVS 基准)
           "spread_mm": m.get("spread_mm"),
           "marker": slot["marker"], "marker_size_mm": slot["marker_size_mm"],
           "cam_role": role, "joints_deg": [round(float(v), 3) for v in q],
           "end_pose": {"position": [float(v) for v in pos],
                        "orientation": [float(v) for v in quat]},
           "snapshot": snap,
           "created": time.strftime("%Y-%m-%d %H:%M:%S")}
    flow_store.save_align_ref(arm, which, ref)
    return {"ok": True, "arm": arm, "which": which, "ref":
            {k: v for k, v in ref.items() if k != "snapshot"},
            "snapshot": snap,
            "hint": "参考已记录。改 A1/B1/B3 路点不影响参考(参考是码在"
                    "基座系的位置,与臂姿无关);挪码/换码必须重记。"}


def test_align_gate(robot: TianlianRobot, cameras: FlowCameras,
                    finder: MarkerFinder, arm: str, which: str) -> dict:
    """在当前位姿跑一遍闸门测量(不修正、不动臂),验证手眼/码/容差配置。"""
    cfg = flow_store.load_config()
    slot = cfg["arms"][arm].get(which) or {}
    ref = slot.get("ref")
    if not ref:
        raise ValueError("先记录对准参考")
    T_x = flow_store.load_handeye_T(arm)
    ep = (robot.get_end_poses().get(arm)) or {}
    pos, quat = ep.get("position"), ep.get("orientation")
    if not pos or not quat:
        raise RobotError("末端位姿读取失败")
    sess = cameras.get(cfg["arms"][arm].get(
        "cam_role", "left" if arm == "armleft" else "right"))
    m = measure_marker_base(sess, finder, slot["marker"],
                            slot["marker_size_mm"] / 1000.0,
                            T_from(pos, quat), T_x,
                            frames=int(cfg["gate"]["frames"]))
    d = (np.array(m["p_base"]) - np.array(ref["p_base"])) * 1000.0
    g = cfg["gate"]
    verdict = ("PASS(T1 免修)" if np.hypot(d[0], d[1]) <= g["tol_xy_mm"]
               and abs(d[2]) <= g["tol_z_mm"] else
               "NEED-FIX(会触发 T2 修正)" if max(abs(d[0]), abs(d[1]),
                                                  abs(d[2])) <=
               g["max_corr_mm"] else "REFUSE(超修正上限,停线)")
    return {"ok": True, "arm": arm, "which": which,
            "measured_p_base": m["p_base"], "ref_p_base": ref["p_base"],
            "delta_mm": [round(float(v), 2) for v in d],
            "key": m["key"], "ref_key": ref.get("key"),
            "z_cam_mm": m["z_cam_mm"], "verdict": verdict,
            "spread_mm": m.get("spread_mm"),
            "parallax_mm": ([round(float(v), 2) for v in
                             (np.array(m["p_cam"]) -
                              np.array(ref["p_cam"])) * 1000.0]
                            if ref.get("p_cam") else None)}


def goto_align_ref(robot: TianlianRobot, cameras: FlowCameras,
                   finder: MarkerFinder, arm: str, which: str) -> dict:
    """先回到记录参考时的观察位(ref.joints_deg 关节回放),稳定后再测 Δ。

    『测试』是在**当前位姿**测,臂被移开观察位后码不在视野属正常;
    这个入口把臂送回去再测 —— 网页『回观察位并测』按钮,动臂。
    移动与 ArmLoop._traj 同款:关节轨迹 + 段后验证 + 超差补走一次。"""
    cfg = flow_store.load_config()
    slot = cfg["arms"].get(arm, {}).get(which) or {}
    ref = slot.get("ref")
    if not ref:
        raise ValueError("先记录对准参考")
    joints = ref.get("joints_deg") or []
    if not joints:
        raise ValueError("参考里没存观察位关节(旧版参考?)重新记录一次")
    mv = cfg["move"]
    speed = speed_store.get_speed(arm)
    tol = float(mv["joint_tol_deg"])
    target = [float(v) for v in joints]
    for attempt in range(2):
        send = [target]
        if attempt:          # 补走从当前位重发,避免相同目标被服务器当重复吞掉
            cur0 = robot.get_joint_angles().get(arm) or []
            if len(cur0) == len(target):
                send = [cur0, target]
        robot.move_robotarm_traj(arm, send, 1, mv["move_timeout_s"], speed)
        time.sleep(0.3)                       # 收敛中,稍等复读
        cur = robot.get_joint_angles().get(arm) or []
        if len(cur) == len(target) and \
                max(abs(a - b) for a, b in zip(cur, target)) <= tol:
            break
        if attempt:
            raise RobotError(f"回观察位到位验证失败(关节差 > {tol}°,补走后"
                             "仍超差) —— 臂停在当前位,请人工检查")
    time.sleep(float(cfg["gate"]["settle_s"]))          # 稳定后再测
    out = test_align_gate(robot, cameras, finder, arm, which)
    out.update({"moved_to_ref": True, "speed_pct": speed})
    return out


class _GateProbeRunner:
    """一次性闸门壳的 runner 适配器(⚡回观察位并修正用)。

    复用 FlowRunner 的资源(cfg/相机/识别器/日志),但 abort_event 用
    自己的全新事件 —— 非运行期的单步验证没有"停线"语义,更不能吃上一轮
    流水线失败/⛔软停残留下来的 abort 标志(那玩意只在下次 start 才清,
    共享 runner 会让壳一进 check_abort 就抛 _Abort,API 还接不住 → 500)。"""

    def __init__(self, runner: "FlowRunner"):
        # cfg 走现盘重读(POST 改开关/码参数立即生效),不吃流水线启动时
        # 缓存的旧配置 —— 单步验证就该反映当前文件状态
        self.cfg = flow_store.load_config()
        self.cameras = runner.cameras
        self.finder = runner.finder
        self.runlog = runner.runlog
        self.abort_event = threading.Event()

    def _sleep_abort(self, s: float):
        t0 = time.time()
        while time.time() - t0 < s:
            time.sleep(0.1)

    def _step_gate(self, arm):
        pass


def fix_align_gate(robot: TianlianRobot, runner: "FlowRunner",
                   arm: str, which: str) -> dict:
    """单步验证 PBVS(网页『⚡回观察位并修正』): 回观察位 → **真跑闸门**。

    与流水线里的闸门同一条代码路径:一次性 ArmLoop 壳复用 align_gate
    (T1 免修 / T2 修正 / 视差复判 / 低分辨率放行全一致),修完臂停在
    **修正后的位置**不回示教点 —— 停在哪、残余多少,就是 PBVS 单步答卷。
    安全边界不变:修正仍受 max_corr 上限,超限照样拒绝(动码请重记参考);
    Δ锚留在这只壳里,不传导进任何后续动作。"""
    if which == "align2" and arm != "armright":
        raise ValueError("align2(料仓)只有右臂有")
    cfg = flow_store.load_config()
    if not (cfg["arms"].get(arm, {}).get(which) or {}).get("enabled", True):
        raise ValueError("该站 PBVS 对准已停用(流水线页『启用 PBVS 对准』"
                         "开关)—— 启用后再单步验证")
    out = goto_align_ref(robot, runner.cameras, runner.finder, arm, which)
    loop = ArmLoop.__new__(ArmLoop)          # 一次性壳,只为复用 align_gate
    loop.runner = _GateProbeRunner(runner)   # 独立 abort,不吃残留软停
    loop.robot = robot
    loop.arm, loop.side = arm, "left" if arm == "armleft" else "right"
    loop.log, loop.phase, loop.phase_times = [], None, {}
    loop._phase_t0 = time.monotonic()
    loop.cycles, loop._cycle_gates = 0, []
    loop._cycle_t0, loop._cycle_phases = None, []
    loop._gate_offset = loop._taught_end_pos = None
    loop.say = lambda m: loop.log.append(m)
    g = loop.align_gate(which)
    return {**out, "gate": g, "log": loop.log}
