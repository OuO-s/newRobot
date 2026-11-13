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
            对准闸门2 → B4 放入料仓(断真空) → 收回 → (B5 附加动作,可选)

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

  - 启动PBVS(gate.mode="startup",2026-11-09): 只在流程首个启用的
    闸门测一次(左右臂各测各的),收敛 Δ 立即重解成"平移后的关节"
    固化到本次运行(内存,不落盘),此后全部动作·全部轮次按平移关节
    回放,过程中的闸门一律跳过 —— 整机挪动/工装整体平移场景,
    一套示教一次找回。单站独立漂移不适用(用默认 verify 逐站修);
    左臂扫描吸取(scan_and_pick)自带视觉,不参与平移。

  - 裁剪机构方案(cfg.cutter.enabled,2026-11-11 流水线3): 中间件换成
    Jetson 裁剪机构(推杆+升降台,ws_control.py,cutter_client.py)。
    左臂每轮两吸两放两推: A1吸取位到位→自动视觉吸①→A2放料正上方
    (下降放料走示教路点)→(等右臂取走上轮两摞=EMPTY)→升降从最高点降
    middle_revs 圈到中间点→推杆联动1[异步,去程剪切①·回程升降到最低;
    同时本臂回去吸②]→等推回→A2放②→推杆联动2(剪切②·回程升降升到
    最高)→置 FILLED。A1=吸取位(到位即自动吸取,scan_and_pick
    from_current,不回放扫描/悬停位);A2=放料正上方(末点=对准观察位,
    闸门在 A2 段后,startup 首轮定标)。右臂循环编排不变: 等 FILLED→
    B1→B2 在最高点一次夹两摞→置 EMPTY→B3→B4入仓。
    左右工作区物理分离 → 中心互斥整体停用(●中点标记失效,⏱等待仍生效);
    启动时升降回零(升上限位钉停)一次,此后每轮第2推联动升限位自动重对齐。
    异常停线时给裁剪机构发全停(stop);软停则让当前动作自然走完。

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
from cutter_client import CutterAborted, CutterError, CutterFlow
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

    @property
    def center_enabled(self) -> bool:
        """中心互斥总开关。裁剪机构方案(cfg.cutter.enabled)左右臂工作区
        物理分离(左半边/右半边,2026-11-11 流水线3),中心锁整体停用 ——
        enter/wait 直接放行,路点"中点"标记随之失效(不抢不交)。"""
        return not bool((self._runner.cfg.get("cutter") or {}).get("enabled"))

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
        if not self.center_enabled:            # 裁剪方案: 半区分离,无中心锁
            return
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
        if not self.center_enabled:            # 裁剪方案: 直接放行
            return
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
        self.last_cycle_rec: dict | None = None   # 最近一次 cycle 事件(轮次汇总)
        # 重锚回放状态(见模块头): _taught_end_pos = 刚回放动作的示教
        # 末点(基座系),_gate_offset = 闸门收敛后的平移量(reanchor 动作消费)
        self._taught_end_pos: list | None = None
        self._gate_offset: np.ndarray | None = None
        # 启动PBVS(gate.mode=startup): 首个启用的闸门收敛后 Δ 定标,
        # 全臂动作路点平移重解成关节存 _shifted_wps(仅本次运行);
        # _startup_delta 非 None(含 Δ≈0)即后续闸门全跳过
        self._startup_delta: np.ndarray | None = None
        self._shifted_wps: dict[str, list] | None = None
        # 启动模式实测的基座系码位移 d=p_now−p_ref(比 Δ锚干净,见
        # _latch_startup_delta 的说明);每次闸门测量都会刷新
        self._startup_meas_d: np.ndarray | None = None
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
        if self.phase == phase:
            return            # 同相位重入(wait_fixture 0.5s 轮询)不改
        if self.phase:        # 起点 —— 否则长等待只记末次 0.5s,等待时长
            dur = round(now - self._phase_t0, 2)   # 被漏记(11-12 实跑踩过)
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
        # 桥缓存优先(100Hz,毫秒级);DDS 风暴期 :8000 读角一半概率 ~3s,
        # 每动作 2~4 次读角全是死等(2026-11-05 A2_place 42.5s 的次要来源)
        q = self.robot.get_joint_angles_fast(self.arm)
        if not q:
            raise RobotError("关节角读取失败")
        return [float(v) for v in q]

    # ------------------------------------------------------------ 运动
    def _mv_cfg(self) -> dict:
        return self.runner.cfg["move"]

    def play_action(self, name: str, reanchor: bool = False,
                    wp_range=None):
        """示教动作回放: 关节路点序列 + 段后验证补走 + 中途真空触发。

        动作配置 vacuum = {"event": "grab"|"release", "at": "start"|k|"end"},
        k=路点序号: 到第 k 个路点后触发(下降到位再断/开真空,然后走完剩余)。
        路点 wait_s=到点等待: 到该点(段后验证补走之后)先等 N 秒再走剩余段;
        真空触发点同点配了等待时,顺序=到点→等待→触发真空。

        wp_range=(a, b)=只回放第 a+1..b 个路点(b=None=到末点),用于动作
        中途分段(2026-11-12 裁剪流水线: 第2摞吸完先走到 A2 点2 等推杆
        回位,再走剩余下降放料段)。真空触发号按整动作绝对路点号解读,
        不落在本段的自动不触发;"end"只在末段、"start"只在首段生效;
        该段路点为空时静默跳过(不动挂起的闸门Δ,后续段还要用)。

        reanchor=True(闸门后的 A2/B2/B4 用): 若紧前的对准闸门留有平移
        Δ锚,各路点示教位姿 +Δ锚 后按笛卡尔逐点走,让视觉修正传导进
        下降/退出轨迹;Δ 缺失或 < reanchor_min_mm 时退回纯关节回放。
        """
        meta = flow_store.ACTIONS_META[self.arm][name]
        act = self.runner.cfg["arms"][self.arm]["actions"][name]
        wps = act["waypoints"]
        if not wps:
            # 空动作=跳过(2026-11-09:部分示教也要能跑测试;真空触发/
            # 段后对准都挂在本段路点上,一并自然失效)。臂不动。
            self.say(f"{name}: 无路点,跳过(真空/对准随段失效)")
            self._gate_offset = None            # 本段消费不了挂起的闸门Δ,作废
            self.runner.runlog.event("action_skip", self.arm, action=name,
                                     reason="no_waypoints")
            return
        shifted = (self._shifted_wps or {}).get(name)
        if shifted is not None:
            # 启动PBVS: 换成平移后的关节路点;回放/补走/等待/中点锁/
            # 真空时序机制全部不变 —— 仍是关节轨迹,只是刚体平移过
            wps = shifted
        base, tail = 0, True
        if wp_range is not None:
            a, b = wp_range
            base = int(a)
            wps = wps[base:] if b is None else wps[base:int(b)]
            tail = b is None
            if not wps:
                # 分段为空: 静默跳过 —— 不作废挂起的闸门Δ(它属整个
                # 动作,后续段仍要用;区别于"整动作无路点"的作废)
                self.say(f"{name}: 路点区间[{a}:{b if b is not None else '末'}]"
                         "为空,跳过分段")
                return
        if (meta.get("wait") == "center_free"
                and self.runner.handoff.center_owner != self.arm):
            # 自己已持锁(center_serialize 的 A1)时不再等 —— 防自等死锁
            self.runner.handoff.wait_center_free(self)
        if not reanchor and self._shifted_wps is None:
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
               "wp_range": (list(wp_range) if wp_range is not None else None),
               "mode": "reanchor" if off is not None else "joint",
               "startup_shift_mm": ([round(float(v) * 1000.0, 2)
                                     for v in self._startup_delta]
                                    if shifted is not None else None),
               "vacuum": vac, "pose_start": self._pose_snap()}
        try:
            cut = len(wps) - 1                   # 真空触发前走到的路点
            if vac:
                at = vac.get("at", "end")
                # 分段回放(wp_range)时 at 按整动作绝对路点号解读:
                # 不落在本段的(含 end 遇非末段/start 遇非首段)不在此触发
                if at == "start":
                    cut = -1 if base == 0 else len(wps)
                elif at == "end":
                    cut = (len(wps) - 1) if tail else len(wps)
                else:
                    k = int(at)
                    cut = k - base if base <= k < base + len(wps) \
                        else len(wps)
            fire = vac is not None and cut <= len(wps) - 1

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

                if not hand.center_enabled:
                    run(0, len(wlist))     # 裁剪方案: 中心锁停用,整段直走
                    return                 # (●中点标记失效,⏱等待仍生效)
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
            if fire:
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
            if off is not None and isinstance(e, RobotError):
                # 重锚段失败别只留一句"未到位":带上 Δ锚 与分型提示
                # (10-28 14:05/14:26 两次 B2·重锚p1 不可达,根因是中间件
                #  参考过期把假 Δz 灌进 B2,报错里根本看不出该重录参考)
                e.args = (f"{e} —— 重锚Δ="
                          f"{np.round(off * 1000.0, 1).tolist()}mm:"
                          "实物没挪而闸门Δ大=该站参考过期,重录参考"
                          "(⚡回观察位并测复核)再试;实物挪过或被挡="
                          "查工装位置与另一臂占位",)
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
        if gate.get("mode") == "startup":
            # 启动模式一次定标,PnP 的 z 噪声会原样进 Δ 影响全程:
            # 帧数至少 9(verify 逐站修可以容忍抖,一次性定标不行)
            gate = {**gate, "frames": max(int(gate["frames"]), 9)}
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
            if gate["mode"] == "startup":
                # 启动PBVS钉住"实测码位移"(基座系):比 Δ锚(当前末端−
                # 示教末点,含回放残余+T2 修正)更干净 —— 新鲜回放的
                # 残余在平移后的回放里同样会出现,算进 Δ 等于重复计入
                # (11-13 流水线2实测: 应用端 0.0~0.01° 精确,残差全在
                # Δ 估计本身)
                self._startup_meas_d = np.asarray(d, float).copy()
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
    def scan_and_pick(self, from_current: bool = False):
        """物料检测 + 吸取 —— 与『物料检测』页一步到位同款行为:
        ①回放扫描位→YOLO+深度定位 ②③按 pick 配置 motion_mode 二选一:
        direct=一键直达(扫描位→aim 一段直线,不可达退分段走廊)/
        corridor=升越顶→(差>2°)转对准姿态→水平平移→下降(不可达退层内)
        ④下压(沿工具系 Z+/吸嘴轴向) ⑤真空/保持沿用 pick 配置,
        吸完即止(不抬升,A1 回放直接接管)。
        每个子步独立 phase(scan/pick_hover/…/pick_grab):
        运行徽章、单步确认与 phase_times 都按子步呈现,不再笼统一个"pick"。
        from_current=True(流水线3 裁剪方案): A1 吸取位到位后调用 ——
        不回放扫描位/悬停位,稳一稳直接从当前位姿(A1 末点)取帧检测,
        A1 末点本身就是吸取悬停几何,后续对准/下压照常。"""
        pcfg = pick_store.load_config()
        if not from_current and not pcfg.get("scan_pose"):
            raise ValueError("左臂扫描位未示教 —— 请在『物料检测』页"
                             "『存当前为扫描位』")
        # ① 物料检测: 回扫描位 → 取帧 → YOLO → 深度 → 基座系点位 → 分层排序
        self.set_phase("scan")
        if from_current:
            self.say("① 物料检测: 从当前位(A1 吸取位)取帧 → YOLO+深度"
                     " → 基座系定位")
            time.sleep(pcfg.get("settle_s", 0.5))   # A1 到位后稳一稳再取帧
        else:
            self.say("① 物料检测: 回扫描位 → 取帧 → YOLO+深度 → 基座系定位")
        out = scan_once(self.robot, self.runner.pick_adapter, pcfg,
                        replay=not from_current, say=self.say)
        rows = out["rows"]
        if not rows:
            raise ValueError("物料检测未发现可吸取目标 —— 请补料,"
                             "或到『物料检测』页检查检测配置/光照")
        row = rows[0]                                # 队首(高层优先)
        # ①+ 悬停位关节回放(不过 IK): 扫描位求视野、悬停位求吸取几何,
        # 料仓在工作空间边缘时二者不可兼得(2026-11-09 扫描位→袋的笛卡尔
        # IK 解分支翻转被 45° 护栏拒)。悬停位示教于袋上方·同解枝,
        # 关节回放无跳解,后续小行程笛卡尔下探从这里出发。
        # (from_current 时跳过: A1 末点就是示教的吸取位,不再另回悬停位)
        hover_pose = pcfg.get("hover_pose")
        if hover_pose and not from_current:
            self.set_phase("pick_hovermove")
            self.say(f"①+ 关节回放悬停位"
                     f"({hover_pose.get('created', '?')} 示教)")
            self._traj([{"joints_deg": hover_pose["joints_deg"]}],
                       speed_store.get_speed(self.arm), "hover_replay")
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
            if self.runner.cutter_on():
                self._loop_left_cutter()
            else:
                self._loop_left()
        else:
            self._loop_right()          # 裁剪方案下仅中心锁自动失效,
                                        # 编排不变: 等 FILLED→B1→B2夹→
                                        # EMPTY→B3→B4(±B0/B5)

    def _end_of_cycle(self, note: str) -> bool:
        """循环收尾共法: 轮次/单臂测试判定,True=应停止循环。"""
        if self.single_shot:
            if self.cycles >= (self.max_cycles or 1):
                self.say(f"◎ 单臂测试: {self.cycles} 个循环完成,自动停止"
                         f"({note})")
                return True
            return False
        if self.max_cycles and self.cycles >= self.max_cycles:
            self.say(f"◎ 已跑满 {self.max_cycles} 轮,自动停止({note})")
            return True
        return False

    def _cutter_aborted(self, fn, *a, **kw):
        """裁剪动作包装: 软停期间的等待中断转 _Abort(不算失败),
        其余 CutterError 原样上抛 → 停线。"""
        try:
            return fn(*a, **kw)
        except CutterAborted:
            raise _Abort() from None

    def _spawn(self, fn, name):
        """起后台线程(裁剪编排的并行机构步,守护线程随进程退出)。
        本地测试把它替换成同步执行,时序断言才是确定的。"""
        threading.Thread(target=fn, daemon=True, name=name).start()

    def _wait_evt(self, evt, err_box=None):
        """等 threading.Event(0.2s 步进,软停可断);err_box 带错则上抛。
        主线程与后台机构线程共用同一等待语义。"""
        while not evt.is_set():
            self.check_abort()
            evt.wait(0.2)
        if err_box is not None and "err" in err_box:
            raise err_box["err"]

    def _cut_step(self, step: str, t0: float, *, cycle: int | None = None,
                  ok: bool = True, err: str | None = None):
        """裁剪机构步落 jsonl(kind=cutter,独立并行车道) + runner 环形账本
        (轮次汇总取数用)。机构步与臂动作并行,不挂臂名(arm=None)。"""
        t1 = time.time()
        rec = {"step": step, "cycle": self.cycles + 1 if cycle is None
               else cycle, "t0": t0, "t1": t1,
               "dur": round(t1 - t0, 3), "ok": ok, "err": err}
        try:
            self.runner._cut_steps.append(rec)
        except Exception:                                      # noqa: BLE001
            pass
        self.runner.runlog.event(
            "cutter", None, step=step, cycle=rec["cycle"],
            t_start=flow_runlog.iso(t0), t_end=flow_runlog.iso(t1),
            duration_s=rec["dur"], ok=ok, error=err)

    def _has_center_marks(self, names) -> bool:
        """相关动作任一路点带"中点"标记 → 中心互斥走路点级(标记段
        进前抢锁/走到非标记点交锁,由 play_action 管理);都不标=维持
        整段互斥(A1→A2 / B1→B2 一把锁,循环包裹)。"""
        acts = self.runner.cfg["arms"][self.arm]["actions"]
        return any(bool(w.get("center"))
                   for nm in names for w in acts[nm]["waypoints"])

    def _latch_startup_delta(self):
        """启动PBVS定标(gate.mode=startup): 流程首个启用的闸门收敛后,
        把 Δ锚 固化为本轮全程修正 —— 本臂所有非空动作的各路点
        end_pose+Δ 后以"该点示教关节"为起点重解 IK(同分支,45°跳解
        护栏照常),得到平移后的关节角;此后所有轮次按平移关节回放
        (回放/补走/等待/中点锁/真空时序全不变),过程中的闸门全跳过。

        适用前提: 偏差来源是整机挪动/工装整体平移(所有站同一 Δ)。
        单站独立漂移(只挪了一个工装)不适用 —— 用默认 verify 逐站修。
        Δ 来源优先用闸门实测的码位移 d(基座系,见 align_gate 里的
        _startup_meas_d),没有才退回 Δ锚;每个重解结果再用 FK 复核
        (平移后末端位移 vs Δ,残差>0.5mm=IK 解走了别的分支,停线)。
        重解失败(不可达/跳解)= 该点位偏移后到不了,立即停线: 此时臂
        还在观察位,安全;提示减小挪动量或重示教该动作。"""
        off = (self._startup_meas_d if self._startup_meas_d is not None
               else self._gate_offset)
        self._gate_offset = None
        self._startup_delta = np.zeros(3)
        self._shifted_wps = {}
        if off is None:
            self.say("启动PBVS: 闸门通过但无 Δ(未取到测量值),"
                     "后续按原示教回放")
            return
        mm = float(np.linalg.norm(off)) * 1000.0
        if mm < float(self._mv_cfg()["reanchor_min_mm"]):
            self.say(f"启动PBVS: Δ {mm:.2f}mm < "
                     f"{self._mv_cfg()['reanchor_min_mm']}mm 死区,"
                     "后续按原示教回放")
            return
        acts = self.runner.cfg["arms"][self.arm]["actions"]
        shifted, report, max_fk = {}, [], 0.0
        for name in flow_store.ACTIONS_META[self.arm]:
            wps = acts[name]["waypoints"]
            if not wps:
                continue
            new = []
            for i, w in enumerate(wps):
                ep = w.get("end_pose") or {}
                if not ep.get("position") or not ep.get("orientation"):
                    raise ValueError(
                        f"启动PBVS: {name} 点{i + 1} 缺 end_pose ——"
                        " 重新示教一遍该动作即可补全")
                tgt = [float(ep["position"][k]) + float(off[k])
                       for k in range(3)]
                try:
                    out = plan_cartesian_path(
                        self.arm, [float(v) for v in w["joints_deg"]],
                        op="target", target_pos_m=tgt,
                        target_rpy_deg=quat_to_rpy_deg(
                            [float(v) for v in ep["orientation"]]),
                        cur_pos_m=[float(v) for v in ep["position"]],
                        cur_quat=[float(v) for v in ep["orientation"]])
                    sol = (out.get("waypoints_deg") or [None])[-1]
                    if sol is None:
                        raise RobotError("逆解未返回轨迹")
                    # FK 复核: 平移后关节的末端位移应恰为 Δ(URDF 系。
                    # 上报系与 URDF 系只差固定纯平移安装偏移,平移量在
                    # 两系相等)。残差大 = IK 落到了别的分支/姿态变了,
                    # 照走会偏 —— 直接停线
                    fk0 = plan_cartesian_path(
                        self.arm, [float(v) for v in w["joints_deg"]],
                        op="fk")
                    fk1 = plan_cartesian_path(
                        self.arm, [float(v) for v in sol], op="fk")
                    rd = (np.array(fk1["pos_m"], float)
                          - np.array(fk0["pos_m"], float))
                    resid_mm = float(
                        np.linalg.norm(rd - np.asarray(off, float))) * 1000.0
                    if resid_mm > 0.5:
                        dmm = np.round(np.asarray(off, float) * 1000.0, 1)
                        raise ValueError(
                            f"启动PBVS: {name} 点{i + 1} 重解后 FK 复核残差 "
                            f"{resid_mm:.2f}mm(≠Δ {dmm.tolist()}mm) —— "
                            "IK 走了不同分支,减小挪动量或重示教该动作")
                    max_fk = max(max_fk, resid_mm)
                except RobotError as e:
                    msg = str(e)
                    if ("几乎一致" in msg or "无需运动" in msg
                            or "near-zero trajectory" in msg):
                        new.append(dict(w))   # 低于运动分辨率: 关节不变
                        continue
                    raise ValueError(
                        f"启动PBVS: {name} 点{i + 1} 平移 "
                        f"{np.round(off * 1000.0, 1).tolist()}mm 后重解失败"
                        f"({msg}) —— 该点位偏移后不可达或跳解,"
                        "减小挪动量或重示教该动作") from e
                nw = dict(w)
                nw["joints_deg"] = [float(v) for v in sol]
                nw["end_pose"] = {"position": tgt,
                                  "orientation": list(ep["orientation"])}
                new.append(nw)
            shifted[name] = new
            dj = max(max(abs(a - b) for a, b in zip(n["joints_deg"],
                                                    o["joints_deg"]))
                     for n, o in zip(new, wps))
            report.append((name, dj))
        self._startup_delta = np.asarray(off, float)
        self._shifted_wps = shifted
        src = "实测码位移" if self._startup_meas_d is not None else "Δ锚"
        self.say(f"启动PBVS: ✓ 定标 Δ={np.round(off * 1000.0, 1).tolist()}mm"
                 f"({src}),{len(shifted)} 个动作路点已平移"
                 f"(FK 复核最大残差 {max_fk:.2f}mm;各动作关节最大变化 "
                 + ", ".join(f"{n} {d:.1f}°" for n, d in report)
                 + "),本轮及后续轮次不再闸门")
        self.runner.runlog.event(
            "startup_fix", self.arm,
            delta_mm=[round(float(v) * 1000.0, 2) for v in off],
            delta_src=src, fk_resid_mm=round(max_fk, 3),
            actions={n: round(float(d), 2) for n, d in report})

    def _gate_or_skip(self, obs_action: str, which: str, where: str):
        """段后对准闸门: 观察位动作有路点才测(Δ 锚在观察位末点上,
        动作空=无观察位,测出的 Δ 是幻影,绝不能修);空则跳过并提示,
        后续 reanchor 回放自然退纯关节。
        gate.mode=startup: 流程首个真测了的闸门=启动PBVS,通过即把 Δ
        定标到全臂(见 _latch_startup_delta),此后所有闸门全跳过。"""
        if (self.runner.cfg["gate"].get("mode") == "startup"
                and self._shifted_wps is not None):
            if self._shifted_wps:
                self.say(f"闸门[{where}]: 跳过 —— 启动PBVS已定标"
                         f"(Δ={np.round(self._startup_delta * 1000.0, 1).tolist()}"
                         "mm 已应用于全部动作与轮次)")
            else:
                self.say(f"闸门[{where}]: 跳过 —— 启动PBVS未定标"
                         "(该臂 PBVS 停用或无参考),按原示教回放")
            self.runner.runlog.event("gate_skip", self.arm, which=which,
                                     reason="startup_done" if self._shifted_wps
                                     else "startup_nocal")
            return
        if self.runner.cfg["arms"][self.arm]["actions"][obs_action]["waypoints"]:
            res = self.align_gate(which)
            if (self.runner.cfg["gate"].get("mode") == "startup"
                    and res.get("tier") in ("T1", "T2")):
                self._latch_startup_delta()
        else:
            self.say(f"{obs_action} 空: 跳过{where}对准(无观察位)")
            self.runner.runlog.event("gate_skip", self.arm, which=which,
                                     reason=f"{obs_action}_empty")

    def _startup_pbvs(self, obs_action: str, which: str, where: str):
        """启动PBVS(流水线3 右臂专用入口): 调用方须已把臂停在观察位
        (右臂=就位 B2 点1 后、等 FILLED 之前 —— 裁剪还在切,测量零等待)。
        仅 gate.mode=startup 且尚未定标时测一次并定标到全臂;verify/off
        模式静默不测(裁剪方案原本就没有过程右臂闸门,维持原行为)。
        PBVS 停用/无参考: 置空定标标记,后续闸门按 startup_nocal 跳过,
        本轮按原示教回放 —— 不拦线,靠就绪检查与 UI 提示兜底。"""
        if (self.runner.cfg["gate"].get("mode") != "startup"
                or self._shifted_wps is not None):
            return
        slot = self.runner.cfg["arms"][self.arm].get(which) or {}
        if not slot.get("enabled", True):
            self.say(f"启动PBVS[{where}]: 该臂 PBVS 已停用 —— "
                     "本轮按原示教回放(要修正在『PBVS 参考』卡勾选启用)")
            self._shifted_wps = {}          # 空=已处理不惦记(见 _gate_or_skip)
            self.runner.runlog.event("gate_skip", self.arm, which=which,
                                     reason="startup_disabled")
            return
        if not slot.get("ref"):
            self.say(f"启动PBVS[{where}]: 未记录参考 —— 本轮按原示教回放"
                     "(要修正请先在『PBVS 参考』卡记录)")
            self._shifted_wps = {}
            self.runner.runlog.event("gate_skip", self.arm, which=which,
                                     reason="startup_noref")
            return
        self._gate_or_skip(obs_action, which, where)

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
                self._gate_or_skip("A1_transfer", "align", "中间件")
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
                if self.cycles >= (self.max_cycles or 1):
                    self.say(f"◎ 单臂测试: {self.cycles} 个循环完成,自动停止"
                             "(中间件已置 FILLED,实物留在中间件)")
                    return                       # 未到轮次 → 接着下一循环
            elif self.max_cycles and self.cycles >= self.max_cycles:
                self.say(f"◎ 已跑满 {self.max_cycles} 轮,自动停止")
                return

    def _loop_left_cutter(self):
        """裁剪机构版左臂循环(流水线3;2026-11-12 二次重排=左右臂完全
        独立,机构步全部并行化,左臂只在 A2 点2 等推杆):

        每轮 = 【轮首后台线程: (等上轮推②完成→)等右臂清空(EMPTY)→
        升降到中间点】‖ A1 吸取位 → 自动视觉吸取第1摞 → A2 到点2 →(等
        上一轮推②把托盘让出来)→ 断真空放第1摞 →【后台线程: 等升降到
        中间(通常早已到)→ 推①】‖ 同时 A1 吸第2摞 → A2 回点2 等推①回位
        (到点时已回=直接放)→ 断真空放第2摞 → 推②(联动升)异步发出
        【完成线程自动置 FILLED】→ 左臂不等,立刻进下一轮吸取。

        - 左臂全程不等右臂/令牌(单臂测试也不等): 唯一的等待=在 A2 点2
          等推杆回位 —— 现场已确认推杆扫过点2 不会碰到吸嘴,到位等是安全的。
        - 升降降中间点在轮首就发(与吸第1摞并行): 首轮也不晚拍 —— 此前
          排在"放完第1摞"后,首次启动时臂都到 A2 点2 了升降才开始降
          (11-12 复盘问题②;稳态因 EMPTY 等待叠在吸料下而看不出来)。
        - 推①的完整门禁 = 第1摞已放好 + 升降已在中间 + 右臂已清空,
          由 _push1 线程等 lift_done 事件保证,Jetson 不会被 busy 拒。
        - 右臂只看 FILLED(=推②完成+升降到最高),等令牌期间预先就位
          B2 点1(见 _loop_right)。
        A1=吸取位(到位即自动视觉吸取);A2 点2=等待位=释放位(现 2 点
        示教: 接近/放料位;≥3 点示教时 [2:] 段=下降/释放/抬起,真空按
        绝对路点号在段内触发,2 点示教由循环显式断真空)。闸门观察位=
        点2,startup 首轮定标。中心互斥整体停用。"""
        cut = self.runner.cutter                # CutterFlow(真机/SIM)
        ab = self.runner.abort_event.is_set     # 软停感知(裁剪等待可中断)
        hand = self.runner.handoff
        note = self.runner.note
        a2act = self.runner.cfg["arms"][self.arm]["actions"]["A2_place"]

        class _Quiet:
            """后台线程的 wait_fixture 替身: 不抢左臂相位显示。"""
            def set_phase(self, phase):
                pass

        home_done = False
        # 上一轮推②的完成状态(fin2 线程置位): 放下1摞前等托盘让位。
        # 注意不能用句柄直接 wait —— fin2 已把句柄收走,再 wait 会报
        # "不存在或已收尾"(15:07 实跑踩过)
        p2_done, p2_box = None, None
        while True:
            self.check_abort()
            self._cycle_t0 = time.time()
            self._cycle_phases, self._cycle_actions, self._cycle_gates = \
                [], [], []
            self.post_grip("start")
            if not home_done:
                # 启动回零(每次运行仅一次): 保证"从最高点降 N 圈=中间点"
                # 的基准成立 —— 之后每轮推②联动升到上限位自动重对齐
                self.set_phase("cutter_home")
                self.say("裁剪: 启动回零 —— 升降升到最高(撞上限位钉停)")
                t0 = time.time()
                try:
                    self._cutter_aborted(cut.lift_home_top, abort_cb=ab)
                except BaseException as e:      # 计时先于上抛: 失败也要留痕
                    self._cut_step("lift_home", t0, ok=False, err=str(e))
                    raise
                self._cut_step("lift_home", t0)
                home_done = True
            a2_simple = len(a2act["waypoints"]) <= 2   # 点2=释放位(现场确认)
            # ── 轮首就发升降线程: [等上轮推②完成→]右臂清空(EMPTY)→降到
            # 中间点 —— 与吸第1摞完全并行。11-12 复盘: 首轮曾把这一步排在
            # "放完第1摞"之后,臂都到 A2 点2 了升降还在最高点才开始降;
            # 提前到轮首后,臂到点时升降已在中间点(稳态本来就有此并行)。
            prev_done, prev_box = p2_done, p2_box
            lift_done, lift_box = threading.Event(), {}

            def _lift_mid():
                try:
                    if prev_done is not None:
                        self._wait_evt(prev_done, prev_box)
                    if not self.single_shot:
                        hand.wait_fixture("EMPTY", _Quiet())
                    note("裁剪: 升降到中间点(第1摞接料高度)"
                         "—— 与吸取并行,放好第1摞即推①")
                    t0 = time.time()
                    try:
                        self._cutter_aborted(cut.lift_middle, abort_cb=ab)
                    except BaseException as e:
                        self._cut_step("lift_mid", t0, ok=False, err=str(e))
                        raise
                    self._cut_step("lift_mid", t0)
                except BaseException as e:
                    lift_box["err"] = e
                finally:
                    lift_done.set()
            self._spawn(_lift_mid, f"cutter-liftmid-{self.cycles + 1}")
            # ── 第 1 摞: 吸取 → A2 点2 →(等上轮推②让位)→ 放 ──
            self.play_action("A1_transfer")            # A1=吸取位
            self.scan_and_pick(from_current=True)      # 到位后自动视觉吸取
            self.post_grip("A1_transfer")
            self.play_action("A2_place", wp_range=(0, 2))   # 到点2等/放
            if prev_done is not None:
                self.set_phase("cutter_wait_push2")
                self.say("裁剪: 等上一轮推②回位(托盘让位)再放第1摞"
                         "(通常吸料途中已完成,到点即过)")
                self._wait_evt(prev_done, prev_box)   # 推②失败=托盘没让位
            self._gate_or_skip("A2_place", "align", "裁剪放料区")
            if a2_simple:
                self.say("A2 点2 到位,断真空放第1摞")
                self.runner.vacuum(self, "release")
            # ── 放好①摞 → 发推①线程: 等升降到中间(通常早已到)即推 ──
            pbox, armed = {}, threading.Event()

            def _push1():
                try:
                    self._wait_evt(lift_done, lift_box)  # 升降失败在这里抛
                    note("裁剪: 第1摞已放、升降在中间点 —— 推①")
                    t0 = time.time()             # 受理即计时(排队等待不算)
                    pbox["tok"] = self._cutter_aborted(
                        cut.begin_push_cycle, "down", abort_cb=ab)
                    pbox["t0"] = t0
                except BaseException as e:        # 软停/机构故障: 交给主线程
                    pbox["err"] = e
                finally:
                    armed.set()
            self._spawn(_push1, f"cutter-push1-{self.cycles + 1}")
            self.play_action("A2_place", reanchor=True, wp_range=(2, None))
            # ── 第 2 摞: 吸取(与升降/推①并行)→ A2 点2 等推杆回位 → 放 ──
            self.play_action("A1_transfer")
            self.scan_and_pick(from_current=True)
            self.post_grip("A1_transfer")
            self.play_action("A2_place", wp_range=(0, 2))   # 回点2等推杆
            self.set_phase("cutter_wait_push1")
            self._wait_evt(armed, pbox)          # 推①可能尚未发出(右臂未清)
            self.say("裁剪: 等推①回位(回位=升降已在最低)再放第2摞"
                     "(到点时已回=直接放)")
            try:                                 # 回位=推①机构段收口,落时
                self._cutter_aborted(cut.wait, pbox["tok"], abort_cb=ab)
                self._cut_step("push1", pbox.get("t0", time.time()))
            except BaseException as e:
                if pbox.get("t0") is not None:
                    self._cut_step("push1", pbox["t0"], ok=False, err=str(e))
                raise
            self._gate_or_skip("A2_place", "align", "裁剪放料区")
            if a2_simple:
                self.say("A2 点2 到位,断真空放第2摞")
                self.runner.vacuum(self, "release")
            self.play_action("A2_place", reanchor=True, wp_range=(2, None))
            # ── 推②(联动升)异步: 完成线程置 FILLED,左臂不等立刻下一轮 ──
            cyc = self.cycles + 1            # 轮号在受理时定格(fin2 异步收口)
            t2 = time.time()
            try:
                tok = self._cutter_aborted(cut.begin_push_cycle, "up",
                                           abort_cb=ab)
            except BaseException as e:       # 受理即失败: 同步留痕后停线
                self._cut_step("push2", t2, cycle=cyc, ok=False, err=str(e))
                raise
            p2_done, p2_box = threading.Event(), {}

            def _fin2():
                try:
                    self._cutter_aborted(cut.wait, tok, abort_cb=ab)
                    self._cut_step("push2", t2, cycle=cyc)
                    hand.set_filled("armleft")
                except BaseException as e:   # 软停/故障: 下一轮托盘让位
                    self._cut_step("push2", t2, cycle=cyc, ok=False,
                                   err=str(e))
                    p2_box["err"] = e        # 等待处会 raise,不会带病叠放
                finally:
                    p2_done.set()
            self._spawn(_fin2, f"cutter-fin2-{self.cycles + 1}")
            self.say(f"裁剪: 第{self.cycles + 1}轮推②已发(去程剪切·"
                     "回程升降到最高,完成自动置 FILLED)"
                     " —— 左臂不等,继续吸下一摞")
            self.post_grip("A2_place")
            self.cycles += 1
            self._mark_cycle()
            if self._end_of_cycle(
                    f"第{self.cycles}轮两摞已放好、推②升降中,"
                    "单臂模式完成后请人工取走" if self.single_shot else
                    f"第{self.cycles}轮两摞已放好,推②完成后右臂自动取"):
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
                # 裁剪方案(流水线3): 等令牌期间先把整段接近行程走掉,
                # 就位 B2 点1(夹取接近位)—— 令牌一到直接进夹取段,
                # 不再"待命中→FILLED→从远端整段走来"(2026-11-12 视频
                # 复盘③: 视频里右臂全程远端待命,白串整段接近时间)。
                # 就位即启动PBVS定标(startup 模式首轮一次): 人就位/
                # 裁剪在切的这段空闲正好用来测码+重解全程路点,零节拍代价
                if self.runner.cutter is not None and self.runner.cfg[
                        "arms"][self.arm]["actions"]["B2_pick"]["waypoints"]:
                    self.play_action("B2_pick", wp_range=(0, 1))
                    self.say("右臂: 已就位 B2 点1,先启动PBVS定标"
                             "再等裁剪完成(FILLED)")
                    self._startup_pbvs("B2_pick", "align", "裁剪取料位")
                self.runner.handoff.wait_fixture("FILLED", self)
            elif self.runner.cutter is not None and self.runner.cfg[
                    "arms"][self.arm]["actions"]["B2_pick"]["waypoints"]:
                # 单臂右测(裁剪): 同样先就位 B2 点1 做启动PBVS定标
                self.play_action("B2_pick", wp_range=(0, 1))
                self._startup_pbvs("B2_pick", "align", "裁剪取料位")
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
                if self.runner.cutter_on():
                    # 裁剪方案: 右臂 PBVS 只有取料位一处(启动时已定标),
                    # B1/料仓两处过程闸门不在本流程中
                    self.runner.runlog.event(
                        "gate_skip", self.arm, which="align",
                        reason="cutter_single_pbvs")
                else:
                    self._gate_or_skip("B1_approach", "align", "中间件")
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
            if self.runner.cutter_on():
                self.runner.runlog.event(
                    "gate_skip", self.arm, which="align2",
                    reason="cutter_single_pbvs")
            else:
                self._gate_or_skip("B3_transfer", "align2", "料仓")
            self.play_action("B4_place", reanchor=True)       # 闸门Δ传导
            self.post_grip("B4_place")
            # B5 附加动作(可选): B4 后·循环末前,空路点=整段跳过(同 B0)。
            # 纯示教回放,不接闸门Δ(B4 已消费);其夹爪整形插槽也只在
            # B5 实际执行时才生效。
            if self.runner.cfg["arms"][self.arm]["actions"]["B5_extra"]["waypoints"]:
                self.play_action("B5_extra")
                self.post_grip("B5_extra")
            self.cycles += 1
            self._mark_cycle()
            if self.single_shot:
                if self.cycles >= (self.max_cycles or 1):
                    self.say(f"◎ 单臂测试: {self.cycles} 个循环完成,自动停止"
                             "(中间件已置 EMPTY,请确认料已入仓)")
                    return                       # 未到轮次 → 接着下一循环
            elif self.max_cycles and self.cycles >= self.max_cycles:
                self.say(f"◎ 已跑满 {self.max_cycles} 轮,自动停止"
                         f"(第 {self.cycles} 件已送完料仓)")
                return

    def _mark_cycle(self):
        if self.phase and self.phase != "cycle_gap":
            self.set_phase("cycle_gap")    # 收口末个动作,间隙计入下一轮
        dt = time.time() - self._cycle_t0
        self.cycle_ts.append(dt)
        del self.cycle_ts[:-20]
        # 令牌等待/净工作分解(轮次节拍口径,右臂最常用):
        #   wait_s = 全部 wait_*(令牌/中心锁)相位合计;
        #   work_s = 末个 wait_* 之后的相位合计 —— 右臂即"FILLED 到手
        #   → 夹完放完回归"的净时长(就位段在等待前,不算);
        #   picks = 本循环视觉吸取次数(流水线3 左臂=2)。
        wait_s = sum(d for p, d in self._cycle_phases
                     if str(p).startswith("wait_"))
        work_s = 0.0
        for p, d in reversed(self._cycle_phases):
            if str(p).startswith("wait_"):
                break
            work_s += d
        # 本轮实际下发过的速度(每动作取一次全局值):顶栏中途改速/动作级
        # 覆盖都会体现为多值,如 [30, 50] → "30→50%"
        used = sorted({a.get("speed_pct") for a in self._cycle_actions
                       if isinstance(a.get("speed_pct"), (int, float))})
        spd_txt = "→".join(f"{v:g}%" for v in used) \
            or f"{speed_store.get_speed(self.arm)}%"
        rec = {"cycle": self.cycles,
               "t_start": flow_runlog.iso(self._cycle_t0),
               "t_end": flow_runlog.iso(),
               "duration_s": round(dt, 3),
               "wait_s": round(wait_s, 1),
               "work_s": round(work_s, 1),
               "picks": sum(1 for p, _ in self._cycle_phases if p == "scan"),
               "avg_recent_s": round(float(np.mean(self.cycle_ts)), 3),
               "speed_pct": speed_store.get_speed(self.arm),
               "speeds_used": used,
               "phases": self._cycle_phases,
               "actions": self._cycle_actions,
               "gates": self._cycle_gates}
        self.say(f"◎ 循环 #{self.cycles} 完成 {dt:.1f}s"
                 f"(近均值 {np.mean(self.cycle_ts):.1f}s,"
                 f"speed {spd_txt})")
        self.runner.runlog.event("cycle", self.arm, **rec)
        self.last_cycle_rec = rec
        self.runner._on_cycle_done(self)   # 双臂都到 N → 轮次事件(见下)

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


def _set_vacuum_valve(runner, on: bool, vac: dict):
    """左臂吸盘电磁阀 —— **2026-11-13 硬件已接,实装**。

    链路: Jetson ws:8765 "valve" 指令 → /ssd/control/dist/valve_min.py
    (USB-485 @9600 BESTEP 双路继电器)→ AirTAC 4V210-08 单电控阀。
    通电=吸,断电=放(物料自由落体)。经 CutterFlow.valve() 走裁剪
    机构现成 ws 通道,即时执行不受 busy 门禁(推杆动作期间也要能放料)。
    参数: valve_port(C/U,默认C)、valve_timeout_s 随 vacuum_grab/
    vacuum_release 配置透传(update_config 已放行 valve_*)。
    失败抛错停线: 该吸没吸=带不动料,该放没放=物料挂在吸嘴上撞机构。"""
    cutter = getattr(runner, "cutter", None)
    if cutter is None:
        raise ValueError(
            "电磁阀需要裁剪机构通道(cutter 未启用) —— 勾选"
            "「启用裁剪编排」或把真空模式改回 manual")
    cutter.valve(on, str(vac.get("valve_port", "C")))


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
        self.cutter: CutterFlow | None = None   # 裁剪方案启用时构建
        # 轮次口径(2026-11-12,流水线3): 一整轮 = 左臂吸放两摞 + 右臂夹完
        # 放完回归位置;轮完成 = 两臂 cycle#N 都收口(右臂是节拍尾部)。
        # 轮 N 起点 = 上一轮完成时刻(第 1 轮 = 启动)→ duration 即
        # "启动→首轮完成"(冷启动)与之后的稳态轮周期。
        self.round_stats: list[dict] = []       # 每轮一条(网页/日志共用)
        self._rounds_done = 0
        self._round_prev_end: float | None = None
        self._cut_steps: deque = deque(maxlen=80)   # 机构步账本(轮次取数)
        self._round_lock = threading.Lock()

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
        # 裁剪机构方案: 构建客户端(懒连接,SIM=sim 不连真机);
        # 左臂循环切 _loop_left_cutter,中心互斥整体停用
        self.cutter = None
        if bool((cfg.get("cutter") or {}).get("enabled")):
            self.cutter = CutterFlow(cfg.get("cutter") or {}, log=self.note)
            self.note(f"裁剪机构已接入: {self.cutter.url}"
                      + (" [SIM]" if self.cutter.sim else ""))
        self.step_mode = bool(step_mode)
        self.rounds = rounds
        self.abort_event.clear()
        self._failed = None
        self.round_stats, self._rounds_done = [], 0
        self._round_prev_end = None
        self._cut_steps.clear()
        self.runlog = flow_runlog.RunLogger(
            tag={"armleft": "left", "armright": "right"}.get(only_arm,
                                                             "dual"))
        # 多方案:运行日志落方案名(多方案并存后,翻日志先看是哪套)
        try:
            self.runlog.line(f"流水线方案: {flow_store.active_flow()}")
            self.runlog.event("pipeline", name=flow_store.active_flow())
        except Exception:                               # noqa: BLE001
            pass
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

    def cutter_on(self) -> bool:
        """裁剪机构方案(本臂循环编排分支/中心锁旁路都看它)。"""
        return bool((self.cfg.get("cutter") or {}).get("enabled"))

    def _start_arms(self, cfg: dict, robot, only_arm, step_mode: bool):
        self.handoff = Handoff(self)
        self.handoff.fixture = "EMPTY"
        wanted = [only_arm] if only_arm else ["armleft", "armright"]
        self.arms = {a: ArmLoop(self, a) for a in wanted}
        for loop in self.arms.values():
            loop.single_shot = only_arm is not None
            # 单臂测试也吃轮次:填 N=跑 N 循环,留空/0=保持 1 循环旧语义
            loop.max_cycles = (self.rounds or 1) if only_arm else self.rounds
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
                  + (f", 单臂测试:{only_arm} 仅"
                     f"{(self.rounds or 1)}循环" if only_arm else
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

    # ------------------------------------------------------------ 轮次口径
    def _on_cycle_done(self, loop: "ArmLoop"):
        """某臂收口一个循环后调: 双臂都到第 N 个 → 第 N 轮完成,落轮次
        事件。统计失败绝不影响流水线(全吞)。"""
        try:
            if len(self.arms) < 2 or loop.single_shot:
                return                 # 单臂测试/未双臂: 无"轮"概念
            with self._round_lock:
                n = min(a.cycles for a in self.arms.values())
                while self._rounds_done < n:
                    self._rounds_done += 1
                    self._emit_round()
        except Exception:                                          # noqa: BLE001
            pass

    def _emit_round(self):
        """第 self._rounds_done 轮完成: 起止/分解落 jsonl(kind=round)+网页。
        分解 = 左臂(吸N次)/右臂(等FILLED·净夹放回归)/机构步(推①②·升降),
        全部取自两臂 last_cycle_rec 与 _cut_steps(按轮号对账)。"""
        n = self._rounds_done
        t_end = time.time()
        t_start = self._round_prev_end or (self._run_t0 or t_end)
        dur = t_end - t_start
        arms = {}
        for a, lp in self.arms.items():
            c = lp.last_cycle_rec or {}
            if c.get("cycle") == n:
                arms[a] = {k: c.get(k) for k in
                           ("duration_s", "wait_s", "work_s", "picks",
                            "speeds_used")}
        cut: dict[str, float] = {}
        for s in self._cut_steps:    # 同轮同名步只留首现(每轮各一次)
            if s.get("cycle") == n and s["step"] not in cut:
                cut[s["step"]] = s.get("dur")
        rec = {"round": n, "cold": n == 1,
               "t_start": flow_runlog.iso(t_start),
               "t_end": flow_runlog.iso(t_end),
               "duration_s": round(dur, 1),
               "arms": arms, "cutter": cut}
        self._round_prev_end = t_end
        self.round_stats.append(rec)
        self.runlog.event("round", None, **rec)
        f = lambda v: f"{v:.1f}s" if isinstance(v, (int, float)) else "?"  # noqa: E731
        l, r = arms.get("armleft") or {}, arms.get("armright") or {}
        msg = (f"◎ 第{n}轮完成: "
               + (f"冷启动(启动→首轮完成) {dur:.1f}s" if n == 1
                  else f"轮周期 {dur:.1f}s"))
        if r:
            msg += f" | 右臂净夹放回归 {f(r.get('work_s'))}"
            if r.get("wait_s") is not None:
                msg += f"(等FILLED {f(r['wait_s'])})"
        if l:
            msg += (f" | 左臂 {f(l.get('duration_s'))}"
                    f"(吸{l.get('picks')}次"
                    + (f",等待{f(l['wait_s'])}" if l.get("wait_s") else "")
                    + ")")
        for step in ("push2", "push1", "lift_mid"):
            if cut.get(step) is not None:
                msg += f" | { {'push2': '推②', 'push1': '推①', 'lift_mid': '升降'}[step] } {f(cut[step])}"
        self.note(msg)

    def _rounds_brief(self) -> dict | None:
        """status() 用的轮次摘要: 完成/冷启动/稳态均值/最近一轮。"""
        if not self.round_stats:
            return None
        steady = [r["duration_s"] for r in self.round_stats[1:]]
        return {"done": self._rounds_done,
                "first_s": self.round_stats[0].get("duration_s"),
                "last_s": self.round_stats[-1].get("duration_s"),
                "avg_steady_s": round(sum(steady) / len(steady), 1)
                if steady else None,
                "last": self.round_stats[-1]}

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
        if self.cutter is not None:
            try:
                self.cutter.close()         # 断开 ws(不动机构,动作已收尾)
            except Exception:                       # noqa: BLE001
                pass
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
        # 裁剪机构: 异常停线时后台全停(推杆/升降随时可停,下次动作前
        # 服务端自动清急停);失败不让它拖着走完
        if self.cutter is not None:
            threading.Thread(target=self._cutter_safe_stop, daemon=True,
                             name="cutter-estop").start()

    def _cutter_safe_stop(self):
        try:
            self.cutter.stop()
        except Exception:                               # noqa: BLE001
            pass

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
        gripper 模式=联动夹爪通道;valve=电磁阀直控(11-13 已接硬件,经
    Jetson ws 即时指令,不受推杆 busy 门禁);
        manual=人工窗口(等 hold 秒) —— 左臂吸盘现为人手控,grab/release
        都走 manual,到点留出人工开关真空的时间窗(2026-11-12 现场确认)。
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
        if vac.get("mode") == "valve":
            _set_vacuum_valve(self, event == "grab", vac)
            arm.say(f"真空{event}(电磁阀 {'通' if event == 'grab' else '断'})")
            self._sleep_abort(hold)
        elif vac.get("mode") == "gripper":
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
            "cutter": ({"url": self.cutter.url, "sim": self.cutter.sim}
                       if self.cutter is not None else None),
            "rounds": self._rounds_brief(),
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
