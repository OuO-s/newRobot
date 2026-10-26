# -*- coding: utf-8 -*-
"""双臂流水线运行日志: 每次启动一册,人读 .log + 机读 .jsonl。
================================================================
目录 logs/flow/(与 webapp 同级,整目录可拷):
  run_<时间>[_<tag>].log    人读时间线(动作起止·闸门·真空·交接·循环)
  run_<时间>[_<tag>].jsonl  机读事件流,每行一条,供脚本测算节拍/速度

事件类型(kind):
  start/arm_config     启动配置快照(闸门/速度/各臂路点数/真空/参考)
  phase                臂 phase 切换(含耗时归属)
  action               示教动作回放: t_start/t_end/duration_s/speed_pct/
                       waypoints/mode(joint|reanchor)/vacuum/起止位姿快照
  move                 单段运动(move_kind=traj 关节段 / cartesian IK 直线段):
                       duration_s/speed_pct/attempts/到位残差
  gate_start/          对准闸门: 每步测量(gate_measure)、修正(gate_fix)、
  gate_measure/        结果(gate_result: tier/dxy/dz/耗时)、失败(gate_fail)
  gate_fix/gate_result/gate_fail
  scan                 物料检测结果(行数/目标/基座系点位)
  vacuum               真空抓/放(模式/参数/保持时长)
  handoff              中间件令牌 EMPTY/FILLED 与中心区锁进出
  cycle                单臂一轮完整记录: 起止时间/duration_s/phases 分解/
                       actions 明细/gates 明细/速度%
  stop/abort/arm_end   收尾与异常

写失败一律吞掉 —— 日志绝不让流水线崩。jsonl 用 default=str 兜住
numpy 标量等非 JSON 类型。

导出(网页『⬇ 导出 CSV』/ GET /api/flow/logs/export):
  list_runs()  日志册清单(新→旧,含起止时间/循环数/臂);
  export_csv() 一册 jsonl → Excel 友好 CSV(UTF-8 BOM):
    timeline=动作/运动段/循环/闸门/真空/交接(默认)
    cycles=仅循环汇总  actions=动作+循环+真空+闸门结果  all=全部含 phase。
    行列: 记录时间/类型/臂/循环/名称/开始时间/结束时间/耗时s/速度%/
    档位状态/明细 —— move 与 gate_result 的开始时间由 记录时间−耗时 反推。

客户视图运行报告(网页『📈 运行报告』/ GET /api/flow/logs/preview):
  aggregate_report(start,end) 按时间段(默认当天)把全部日志册聚合成
    轮次 × A/B 大步(起止/耗时,进行中标记)+ 汇总(总轮数/累计/平均);
  export_report_csv(start,end) 同口径 CSV(中文表头,UTF-8 BOM)。
  纯读现有 jsonl,不新增落盘;明细日志照常写,供内部排查。
"""
from __future__ import annotations

import csv
import io
import json
import re
import threading
import time
from datetime import datetime, timedelta
from pathlib import Path

LOG_ROOT = Path(__file__).resolve().parent / "logs" / "flow"


def iso(t: float | None = None) -> str:
    """秒级时间戳 → 'YYYY-MM-DD HH:MM:SS.mmm'(日志通用时间格式)。"""
    t = time.time() if t is None else t
    return datetime.fromtimestamp(t).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]


class RunLogger:
    """一次流水线运行(或一次试跑)的日志册。线程安全。"""

    def __init__(self, tag: str = ""):
        LOG_ROOT.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        base = f"run_{stamp}" + (f"_{tag}" if tag else "")
        n = 0
        while (LOG_ROOT / f"{base}.jsonl").exists():
            n += 1
            base = f"run_{stamp}" + (f"_{tag}" if tag else "") + f"_{n}"
        self.path_log = LOG_ROOT / f"{base}.log"
        self.path_jsonl = LOG_ROOT / f"{base}.jsonl"
        self._txt = open(self.path_log, "a", encoding="utf-8")
        self._jl = open(self.path_jsonl, "a", encoding="utf-8")
        self._lock = threading.Lock()
        self.t_start = time.time()
        self.line(f"===== 流水线运行日志 {base} 开始 {iso(self.t_start)} =====")

    def line(self, msg: str):
        """人读行(带时间戳,立即落盘)。"""
        with self._lock:
            try:
                t = time.time()
                self._txt.write(f"[{time.strftime('%H:%M:%S', time.localtime(t))}"
                                f".{int(t % 1 * 1000):03d}] {msg}\n")
                self._txt.flush()
            except Exception:                              # noqa: BLE001
                pass

    def event(self, kind: str, arm: str | None = None, **fields):
        """机读事件(一行 JSON,立即落盘)。"""
        rec = {"ts": round(time.time(), 3), "time": iso(),
               "kind": kind, "arm": arm}
        rec.update(fields)
        with self._lock:
            try:
                self._jl.write(json.dumps(rec, ensure_ascii=False,
                                          default=str) + "\n")
                self._jl.flush()
            except Exception:                              # noqa: BLE001
                pass

    def close(self, note: str = ""):
        with self._lock:
            try:
                self._txt.write(f"[{time.strftime('%H:%M:%S')}] "
                                f"===== 结束 {iso()}"
                                f"{' ' + note if note else ''} =====\n")
            except Exception:                              # noqa: BLE001
                pass
            for f in (self._jl, self._txt):
                try:
                    f.close()
                except Exception:                          # noqa: BLE001
                    pass


class NullLogger:
    """流水线没在跑时的占位 —— flow 侧不用判空。"""

    def line(self, msg: str):                              # noqa: D401
        pass

    def event(self, kind: str, arm: str | None = None, **fields):
        pass

    def close(self, note: str = ""):
        pass


# ===========================================================================
# 导出: 日志册清单 + 一册 jsonl → CSV(节拍测算用)
# ===========================================================================
_KIND_CN = {"action": "动作", "move": "运动段", "cycle": "循环",
            "gate_start": "闸门开始", "gate_measure": "闸门测量",
            "gate_fix": "闸门修正", "gate_result": "闸门结果",
            "gate_fail": "闸门失败", "scan": "物料检测", "vacuum": "真空",
            "handoff": "交接", "start": "启动", "stop": "停止",
            "abort": "异常停线", "arm_end": "臂结束", "phase": "阶段",
            "arm_config": "臂配置"}

_SCOPES = {
    "timeline": ("action", "move", "cycle", "gate_start", "gate_measure",
                 "gate_fix", "gate_result", "gate_fail", "scan", "vacuum",
                 "handoff", "start", "stop", "abort", "arm_end"),
    "cycles": ("cycle",),
    "actions": ("action", "cycle", "vacuum", "gate_result"),
    "all": None,                       # None = 不筛(含 phase)
}

_CSV_COLS = ["记录时间", "类型", "臂", "循环", "名称", "开始时间", "结束时间",
             "耗时s", "速度%", "档位/状态", "明细"]


def _jsonl_path(run: str) -> Path:
    """run 册名(stem)→ jsonl 路径;名字不合法/不存在抛 ValueError/FileNotFoundError。"""
    if not re.fullmatch(r"[A-Za-z0-9_\-]+", run or "") or \
            not run.startswith("run_"):
        raise ValueError(f"非法运行名: {run!r}")
    p = LOG_ROOT / f"{run}.jsonl"
    if not p.is_file():
        raise FileNotFoundError(f"运行册不存在: {run}")
    return p


def _load_events(run: str) -> list[dict]:
    out = []
    for line in _jsonl_path(run).read_text(encoding="utf-8").splitlines():
        try:
            out.append(json.loads(line))
        except Exception:                                  # noqa: BLE001
            continue                       # 半行(崩溃瞬间)跳过
    return out


def list_runs() -> list[dict]:
    """logs/flow/ 全部日志册(新→旧): 名字/起止时间/循环数/臂,供网页选择。"""
    out: list[dict] = []
    if not LOG_ROOT.is_dir():
        return out
    for p in sorted(LOG_ROOT.glob("run_*.jsonl"), reverse=True):
        n_cycle, arms, started, ended = 0, [], None, None
        try:
            for r in _load_events(p.stem):
                started = started or r.get("time")
                ended = r.get("time") or ended
                if r.get("kind") == "cycle":
                    n_cycle += 1
                if r.get("arm"):
                    arms.append(r["arm"])
        except Exception:                                  # noqa: BLE001
            pass
        out.append({"name": p.stem, "started": started, "ended": ended,
                    "cycles": n_cycle, "arms": sorted(set(arms)),
                    "bytes": p.stat().st_size,
                    "mtime": datetime.fromtimestamp(
                        p.stat().st_mtime).strftime("%Y-%m-%d %H:%M:%S")})
    return out


def _t_shift(iso_s: str | None, dt_s: float) -> str:
    """ISO 时间平移 ±秒(move/gate 的开始时间 = 记录时间 − 耗时)。"""
    try:
        t = datetime.strptime(iso_s, "%Y-%m-%d %H:%M:%S.%f")
        return (t + timedelta(seconds=dt_s)).strftime(
            "%Y-%m-%d %H:%M:%S.%f")[:-3]
    except Exception:                                      # noqa: BLE001
        return ""


def _row(r: dict) -> list:
    """一条事件 → CSV 行(各 kind 的名称/起止/明细见模块头)。"""
    k = r.get("kind")
    name = status = detail = ""
    t0 = t1 = dur = spd = cyc = ""
    if k == "action":
        vac = r.get("vacuum") or {}
        name = str(r.get("action"))
        cyc = r.get("cycle")
        t0, t1 = r.get("t_start"), r.get("t_end")
        dur, spd = r.get("duration_s"), r.get("speed_pct")
        status = f'{r.get("status")}/{r.get("mode")}'
        detail = f'路点={r.get("waypoints")}'
        if vac:
            detail += f';真空={vac.get("event")}@{vac.get("at", "end")}'
        if r.get("error"):
            detail += f';{r["error"]}'
    elif k == "move":
        name = str(r.get("name"))
        dur, spd = r.get("duration_s"), r.get("speed_pct")
        t1 = r.get("time")
        t0 = _t_shift(t1, -float(dur or 0))
        status = f'{r.get("move_kind")}' + (
            f' 补走{r.get("attempts")}' if (r.get("attempts") or 1) > 1 else "")
        if r.get("move_kind") == "traj":
            detail = f'关节残差={r.get("max_err_deg")}°'
        else:
            detail = (f'Δxy={r.get("dxy_mm")}mm '
                      f'Δz={r.get("dz_mm")}mm')
        if not r.get("ok", True):
            status += " ⚠未到位"
    elif k == "cycle":
        name = f'循环#{r.get("cycle")}'
        t0, t1 = r.get("t_start"), r.get("t_end")
        dur, spd = r.get("duration_s"), r.get("speed_pct")
        detail = ";".join(f"{p}={d}s" for p, d in r.get("phases") or [])
        detail += f';近均值={r.get("avg_recent_s")}s'
        if r.get("gates"):
            detail += ";闸门=" + ",".join(
                f'{g.get("which")}:{g.get("tier")}' for g in r["gates"])
    elif k == "gate_result":
        name = f'闸门[{r.get("which")}]'
        dur = r.get("duration_s")
        t1 = r.get("time")
        t0 = _t_shift(t1, -float(dur or 0))
        status = str(r.get("tier"))
        detail = (f'Δxy={r.get("dxy_mm")} Δz={r.get("dz_mm")}mm '
                  f'残差={r.get("resid_xy_mm")}/{r.get("resid_z_mm")}mm '
                  f'重锚={r.get("reanchor_mm")}mm')
    elif k == "gate_measure":
        name = f'闸门[{r.get("which")}]#{r.get("step")}'
        detail = (f'Δxy={r.get("dxy_mm")} Δz={r.get("dz_mm")}mm '
                  f'z_cam={r.get("z_cam_mm")}mm key={r.get("key")}')
    elif k == "gate_fix":
        name = f'闸门[{r.get("which")}]#{r.get("step")}'
        detail = f'修正={r.get("corr_mm")}mm'
    elif k == "gate_fail":
        name = f'闸门[{r.get("which")}]'
        status = str(r.get("reason"))
        detail = (f'{r.get("crit", "")} {r.get("cxy_mm")}/{r.get("cz_mm")}'
                  f'mm corr={r.get("corr_mm")}')
    elif k == "gate_start":
        name = f'闸门[{r.get("which")}]'
        status = str(r.get("mode"))
        detail = f'容差={r.get("tol_xy_mm")}/{r.get("tol_z_mm")}mm'
    elif k == "scan":
        name = f'目标#{r.get("target_order")}'
        detail = (f'{r.get("cls")} 层{r.get("layer")} z={r.get("z_mm")}mm '
                  f'共{r.get("rows")}行 p_base={r.get("p_base_mm")}mm')
    elif k == "vacuum":
        name = f'{r.get("event")}'
        dur = r.get("duration_s")
        detail = (f'{r.get("mode")} pos={r.get("gripper_position")} '
                  f'hold={r.get("hold_s")}s')
    elif k == "handoff":
        name = (f'中间件={r.get("fixture")}' if r.get("fixture")
                else f'中心锁={r.get("center")}')
    elif k == "start":
        g = r.get("gate") or {}
        detail = (f'gate={g.get("mode")} only={r.get("only")} '
                  f'speeds={r.get("speeds")}')
    elif k == "arm_end":
        status = str(r.get("status"))
        detail = str(r.get("error") or "")
    elif k == "abort":
        detail = str(r.get("why") or "")
    elif k == "phase":
        name = str(r.get("phase"))
    return [r.get("time"), _KIND_CN.get(k, k), r.get("arm") or "", cyc, name,
            t0, t1, dur, spd, status, detail]


# ===========================================================================
# 客户视图: 按时间段聚合轮次/A·B 大步(预览 + 导出)
#   数据全部来自现有明细 jsonl(cycle 事件给每臂每轮起止,phase 事件给
#   大步边界),不新增落盘 —— 明细日志照常写、照常留作内部排查。
# ===========================================================================
# 每臂的 phase → 大步映射(客户只看 A/B 大粒度)。标签是键:'$固定名'
# 或 'arm|动作id' —— 后者运行时解析为 flow_config 里该动作的**可编辑
# 提示名**(用户在流水线页改提示,报表自动跟随);固定名只有视觉吸取/
# 对准/等待三样(没有对应动作可取名)。
_ARM_STEPS = {
    "armleft": (
        ("$wait", ("wait_empty", "wait_center")),
        ("$pick", ("scan", "pick_hover", "pick_descend", "pick_press",
                   "pick_grab", "pick_retreat")),
        ("armleft|A1_transfer", ("A1_transfer",)),
        ("$gate_align", ("gate_align",)),
        ("armleft|A2_place", ("A2_place",)),
    ),
    "armright": (
        ("$wait", ("wait_filled", "wait_center")),
        ("armright|B0_pre", ("B0_pre",)),
        ("armright|B1_approach", ("B1_approach",)),
        ("$gate_align", ("gate_align",)),
        ("armright|B2_pick", ("B2_pick",)),
        ("armright|B3_transfer", ("B3_transfer",)),
        ("$gate_align2", ("gate_align2",)),
        ("armright|B4_place", ("B4_place",)),
    ),
}
_ARM_LABEL = {"armleft": "A", "armright": "B"}
_FIXED_STEP = {"$wait": "等待/交接", "$pick": "视觉吸取(自动)",
               "$gate_align": "对准·中间件", "$gate_align2": "对准·料仓"}
_TITLES_CACHE: tuple[float, dict] = (0.0, {})


def _step_labels() -> dict:
    """步骤键 → 显示名。动作步取 flow_config 当前提示名(按文件 mtime
    缓存);flow_store 不可用时退回动作 id,报表绝不因它报错。"""
    global _TITLES_CACHE
    out = dict(_FIXED_STEP)
    try:
        import flow_store
        m = flow_store.CONFIG_PATH.stat().st_mtime
        if _TITLES_CACHE[0] != m:
            cfg = flow_store.load_config()
            _TITLES_CACHE = (m, {
                f"{arm}|{aid}": (s.get("title") or aid).strip()
                for arm, slot in cfg.get("arms", {}).items()
                for aid, s in (slot.get("actions") or {}).items()})
        out.update(_TITLES_CACHE[1])
    except Exception:                                    # noqa: BLE001
        pass
    return out


_PHASE_STEP = {arm: {p: i for i, (_, ps) in enumerate(steps) for p in ps}
               for arm, steps in _ARM_STEPS.items()}

_DT_FORMATS = ("%Y-%m-%d %H:%M:%S", "%Y-%m-%d %H:%M", "%Y-%m-%d")


def parse_when(s: str, *, is_end: bool = False) -> datetime:
    """'YYYY-MM-DD [HH:MM[:SS]]' → datetime。空返回 None(由调用方补缺省)。
    is_end: 纯日期按当天 23:59:59 收口(选一天=看整天);否则 00:00。"""
    s = (s or "").strip()
    if not s:
        return None
    for fmt in _DT_FORMATS:
        try:
            dt = datetime.strptime(s, fmt)
            if fmt == "%Y-%m-%d" and is_end:
                return dt.replace(hour=23, minute=59, second=59)
            return dt
        except ValueError:
            continue
    raise ValueError(f"时间格式应为 YYYY-MM-DD 或 YYYY-MM-DD HH:MM[:SS],"
                     f"收到 {s!r}")


def resolve_range(start: str = "", end: str = "") -> tuple[datetime, datetime]:
    """时间段解析: start 缺省=今天 00:00;end 缺省=现在。"""
    now = datetime.now()
    dt0 = parse_when(start) or now.replace(hour=0, minute=0, second=0,
                                           microsecond=0)
    dt1 = parse_when(end, is_end=True) or now
    if dt1 < dt0:
        raise ValueError(f"结束时间({dt1})早于开始时间({dt0})")
    return dt0, dt1


def _iso_dt(s: str | None) -> datetime | None:
    try:
        return datetime.strptime(s, "%Y-%m-%d %H:%M:%S.%f")
    except (TypeError, ValueError):
        try:
            return datetime.strptime(s, "%Y-%m-%d %H:%M:%S")
        except (TypeError, ValueError):
            return None


def _steps_from_phases(ph_recs: list[dict], arm: str,
                       t_end: float | None) -> list[dict]:
    """一个循环内的 phase 事件序列 → 大步列表(同名相邻 phase 合并;
    t_end=None=进行中,末步不封口)。phase→大步映射外的 phase
    (cycle_gap/aborted/…)封住当前步。"""
    steps: list[dict] = []
    cur: dict | None = None
    labels = _step_labels()
    for r in ph_recs:
        idx = _PHASE_STEP.get(arm, {}).get(r.get("phase"))
        if idx is None:
            if cur is not None:
                cur["t_end"] = r.get("ts")
                steps.append(cur)
                cur = None
            continue
        name = labels.get(_ARM_STEPS[arm][idx][0],
                          _ARM_STEPS[arm][idx][0].split("|")[-1])
        if cur is not None and cur["name"] == name:
            continue                       # 同大步内的子步切换,不切段
        if cur is not None:
            cur["t_end"] = r.get("ts")
            steps.append(cur)
        cur = {"name": name, "t_start": r.get("ts"), "t_end": None}
    if cur is not None:
        cur["t_end"] = t_end               # None → 进行中
        steps.append(cur)
    out = []
    for s in steps:
        t0, t1 = s["t_start"], s["t_end"]
        if t1 is not None and t1 - t0 <= 0.05:
            continue        # 轮末同一毫秒开又封的幽灵步(下轮首相位串门)
        out.append({
            "arm": arm, "arm_label": _ARM_LABEL.get(arm, arm),
            "name": s["name"],
            "start": datetime.fromtimestamp(t0).strftime(
                "%Y-%m-%d %H:%M:%S") if t0 else None,
            "end": datetime.fromtimestamp(t1).strftime(
                "%Y-%m-%d %H:%M:%S") if t1 else None,
            "duration_s": round(t1 - t0, 1) if (t0 and t1) else None,
            "ongoing": t1 is None,
        })
    return out


def _rounds_of_run(events: list[dict]) -> tuple[list[dict], dict | None]:
    """一册事件 → (完成轮列表, 进行中轮或 None)。轮 N = 各臂 cycle#N
    的并:A 的第 N 件从取料到入中间件,B 的第 N 件从中间件到入料仓。"""
    cyc: dict[str, dict[int, dict]] = {}
    ph_by_arm: dict[str, list[dict]] = {}
    for r in events:
        k, arm = r.get("kind"), r.get("arm")
        if arm not in _ARM_STEPS or "ts" not in r:
            continue
        if k == "cycle":
            try:
                cyc.setdefault(arm, {})[int(r.get("cycle"))] = r
            except (TypeError, ValueError):
                continue
        elif k == "phase":
            ph_by_arm.setdefault(arm, []).append(r)
    if not cyc:
        return [], None                       # 试跑册(无循环)不进客户视图
    for phs in ph_by_arm.values():
        phs.sort(key=lambda r: r["ts"])
    rounds = []
    for n in sorted({i for d in cyc.values() for i in d}):
        recs = {a: d[n] for a, d in cyc.items() if n in d}
        dts = [( _iso_dt(c.get("t_start")), _iso_dt(c.get("t_end")))
               for c in recs.values()]
        dts = [(a, b) for a, b in dts if a and b]
        if not dts:
            continue
        t0 = min(a.timestamp() for a, _ in dts)
        t1 = max(b.timestamp() for _, b in dts)
        steps = []
        for arm in ("armleft", "armright"):
            c = recs.get(arm)
            if c is None:
                continue
            ct0, ct1 = _iso_dt(c.get("t_start")), _iso_dt(c.get("t_end"))
            if not (ct0 and ct1):
                continue
            phs = [r for r in ph_by_arm.get(arm, [])
                   if ct0.timestamp() - 0.5 <= r["ts"] <= ct1.timestamp() + 0.05]
            steps += _steps_from_phases(phs, arm, ct1.timestamp())
        rounds.append({
            "round": n, "t_start": t0, "t_end": t1,
            "start": datetime.fromtimestamp(t0).strftime("%Y-%m-%d %H:%M:%S"),
            "end": datetime.fromtimestamp(t1).strftime("%Y-%m-%d %H:%M:%S"),
            "duration_s": round(t1 - t0, 1), "ongoing": False,
            "arms": sorted(recs), "steps": steps,
            "_cycles": {a: c.get("duration_s") for a, c in recs.items()},
        })
    # 进行中: 最新一册里,某臂最后一条 cycle 之后还有 phase(未收口的循环)。
    # 没收过 cycle 的臂(如右臂首轮一直等 FILLED)全部 phase 算进行中第 1 轮。
    pend = {}
    for arm, phs in ph_by_arm.items():
        if cyc.get(arm):
            last = max(cyc[arm])
            last_end = _iso_dt(cyc[arm][last].get("t_end"))
            if not last_end:
                continue
            cut = last_end.timestamp() + 0.05
        else:
            last, cut = 0, None
        trail = [r for r in phs
                 if (cut is None or r["ts"] > cut)
                 and r.get("phase") in _PHASE_STEP.get(arm, {})]
        if not trail:
            continue
        st = _steps_from_phases(trail, arm, None)
        if st:
            pend[arm] = (last + 1, st)
    pending = None
    if pend:
        n = min(pn for pn, _ in pend.values())
        steps = [s for arm in ("armleft", "armright")
                 if arm in pend for s in pend[arm][1]]
        starts = [s["start"] for s in steps if s.get("start")]
        t0 = min(starts) if starts else None
        pending = {
            "round": n, "t_start": None, "start": t0, "end": None,
            "duration_s": None, "ongoing": True,
            "arms": sorted(pend), "steps": steps,
        }
    return rounds, pending


def aggregate_report(start: str = "", end: str = "",
                     *, active: bool = False) -> dict:
    """时间段内全部日志册 → 客户视图报告(纯读现有 jsonl,不写盘)。
    active=True(流水线在跑)时,最新一册末尾未收口的循环计为「进行中」。"""
    dt0, dt1 = resolve_range(start, end)
    t0, t1 = dt0.timestamp(), dt1.timestamp()
    runs = []                                  # (首事件ts, 册名, 事件)
    if LOG_ROOT.is_dir():
        for p in LOG_ROOT.glob("run_*.jsonl"):
            try:
                ev = _load_events(p.stem)
            except Exception:                              # noqa: BLE001
                continue
            ts = min((r.get("ts") or 0) for r in ev) if ev else 0
            runs.append((ts, p.stem, ev))
    runs.sort(key=lambda x: x[0])
    rounds, arm_cyc = [], {"armleft": [], "armright": []}
    sess = 0                       # 次 = 一次点启动(一册日志);内含多轮
    for i, (_, name, ev) in enumerate(runs):
        rs, pending = _rounds_of_run(ev)
        tag = re.sub(r"^run_\d{8}_\d{6}_?", "", name) or "run"
        added = []
        for r in rs:
            r["run"] = f'{r["start"][:16]} {tag}'.strip()
            cyc_durs = r.pop("_cycles") or {}   # 内部字段不进客户视图
            if t0 <= r["t_start"] <= t1:
                added.append(r)
                for a, dur in cyc_durs.items():
                    if isinstance(dur, (int, float)):
                        arm_cyc.setdefault(a, []).append(dur)
        if pending and active and i == len(runs) - 1:
            if t0 <= (_iso_dt(pending["start"]).timestamp()
                      if _iso_dt(pending["start"]) else t0) <= t1:
                pending["run"] = f'{(pending["start"] or "")[:16]} {tag}'.strip()
                added.append(pending)
        if added:
            sess += 1
            for r in added:
                r["session"] = sess
            rounds.extend(added)
    done = [r for r in rounds if not r["ongoing"]]
    total = round(sum(r["duration_s"] for r in done), 1) if done else None
    summary = {
        "range": {"start": dt0.strftime("%Y-%m-%d %H:%M:%S"),
                  "end": dt1.strftime("%Y-%m-%d %H:%M:%S")},
        "sessions_total": sess,
        "rounds_total": len(done),
        "ongoing_rounds": len(rounds) - len(done),
        "first_start": done[0]["start"] if done else None,
        "last_end": done[-1]["end"] if done else None,
        "total_s": total,
        "avg_s": round(total / len(done), 1)
        if (total is not None and done) else None,
        "per_arm": {a: {"label": _ARM_LABEL[a] + "臂", "rounds": len(v),
                        "avg_cycle_s": round(sum(v) / len(v), 1)}
                    for a, v in arm_cyc.items() if v},
    }
    return {"rounds": rounds, "summary": summary}


def export_report_csv(start: str = "", end: str = "",
                      *, active: bool = False) -> tuple[str, str]:
    """时间段 → 客户视图聚合 CSV(UTF-8 BOM,每轮小计行 + 末尾总计行)。"""
    rep = aggregate_report(start, end, active=active)
    buf = io.StringIO()
    w = csv.writer(buf, lineterminator="\n")
    w.writerow(["次", "轮次", "臂", "大步", "开始时间", "结束时间", "耗时(秒)"])
    for r in rep["rounds"]:
        for s in r["steps"]:
            w.writerow([f'第{r.get("session", 1)}次', f'第{r["round"]}轮',
                        s["arm_label"] + "臂", s["name"],
                        s["start"], s["end"] or "进行中",
                        "" if s["duration_s"] is None else s["duration_s"]])
        w.writerow([f'第{r.get("session", 1)}次', f'第{r["round"]}轮',
                    "", "本轮小计", r["start"], r["end"] or "进行中",
                    "" if r["duration_s"] is None else r["duration_s"]])
    s = rep["summary"]
    if s["rounds_total"]:
        w.writerow(["总计",
                    f'{s["rounds_total"]}轮'
                    + ("(另含进行中)" if s["ongoing_rounds"] else ""),
                    f'共{s["sessions_total"]}次 · 平均每轮 {s["avg_s"]}s',
                    s["first_start"], s["last_end"], s["total_s"]])
    else:
        w.writerow(["(该时间段无流水线记录)"])
    d0 = s["range"]["start"][:10].replace("-", "")
    d1 = s["range"]["end"][:10].replace("-", "")
    return "﻿" + buf.getvalue(), f"flow_rounds_{d0}-{d1}.csv"


def export_csv(run: str = "", scope: str = "timeline") -> tuple[str, str]:
    """一册日志 → (csv 文本, 下载文件名)。run 空=最新一册;scope 见 _SCOPES。"""
    kinds = _SCOPES.get(scope)
    if kinds is None and scope != "all":
        raise ValueError(f'scope 只能是 {" / ".join(_SCOPES)},收到 {scope!r}')
    if not run:
        runs = list_runs()
        if not runs:
            raise FileNotFoundError("还没有任何运行日志(先启动一次流水线/试跑)")
        run = runs[0]["name"]
    buf = io.StringIO()
    w = csv.writer(buf, lineterminator="\n")
    w.writerow(_CSV_COLS)
    for r in _load_events(run):
        if kinds is not None and r.get("kind") not in kinds:
            continue
        w.writerow(_row(r))
    fname = f'{run}_{scope}_{time.strftime("%Y%m%d_%H%M%S")}.csv'
    return "﻿" + buf.getvalue(), fname      # BOM: Excel 中文不乱码
