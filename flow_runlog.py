# -*- coding: utf-8 -*-
"""双臂流水线运行日志: 每次启动一册,人读 .log + 机读 .jsonl。
================================================================
目录 logs/flow/(与 webapp 同级,整目录可拷):
  run_<时间>[_<tag>].log    人读时间线(动作起止·闸门·真空·交接·循环)
  run_<时间>[_<tag>].jsonl  机读事件流,每行一条,供脚本测算节拍/速度

事件类型(kind):
  start/arm_config     启动配置快照(闸门/速度/各臂路点数/真空/参考)
  pipeline             本次运行的流水线方案名(多方案并存后翻日志先看它)
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
  cutter               裁剪机构步(独立并行车道,arm=None): step=lift_home/
                       lift_mid/push1/push2, t_start/t_end/duration_s/cycle
  cycle                单臂一轮完整记录: 起止时间/duration_s/wait_s(令牌
                       等待)/work_s(末次等待后的净工作,右臂=夹完放完回归)/
                       picks(视觉吸取次数)/phases 分解/actions 明细/gates
                       明细/速度%(speeds_used=本轮实际下发过的速度,中途
                       改速多值 [30,50])
  round                整线一整轮(流水线3 口径: 左臂吸放两摞+右臂夹完放完
                       回归): 起止(轮 N 起点=上一轮完成,第 1 轮=启动)/
                       duration_s(冷启动|稳态轮周期)/arms 分解/cutter 分解
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

客户视图运行报告(独立页 /report,入口在第9 Tab『📈 运行报告』/
GET /api/flow/logs/preview):
  轮次口径(2026-11-12 起,匹配流水线3): 一整轮 = 左臂吸放两摞 + 右臂夹完
  放完回归位置;轮的完成 = 两臂 cycle#N 都收口(右臂是节拍尾部);轮 N 的
  起点 = 上一轮完成时刻,第 1 轮 = 本册启动 → duration_s 即
  「启动→首轮完成」(冷启动)与其后的稳态轮周期。流水线左右重叠,左臂
  cycle N 会伸进上一轮窗口,属正常并行(整程甘特不裁剪,正为此)。
  aggregate_report(start,end) 按时间段聚合全部日志册:
    runs   按册分组(一次启动 = 甘特一行): 各臂+机构**全部大步不裁剪**
           blocks(带 key=关键步标记,供页面标注)+ FILLED 时刻表 + 该册
           轮列表(每轮 起止/轮周期/左右臂分解/机构步/速度);
    rounds 跨册平铺(CSV/表格用,无 steps/blocks 明细);
    summary 汇总(冷启动/稳态均值/右臂净工作均值/左臂一轮均值)。
  export_report_csv(start,end) 同口径 CSV(中文表头,UTF-8 BOM,一行一轮:
  轮周期/左右臂分解/机构步/速度)。
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
            "gate_fail": "闸门失败", "gate_skip": "闸门跳过",
            "scan": "物料检测", "vacuum": "真空",
            "handoff": "交接", "start": "启动", "stop": "停止",
            "abort": "异常停线", "arm_end": "臂结束", "phase": "阶段",
            "arm_config": "臂配置", "pipeline": "方案",
            "cutter": "裁剪机构", "round": "轮次",
            "startup_fix": "启动定标", "action_skip": "动作跳过"}

_CUT_CN = {"lift_home": "升降回零", "lift_mid": "升降→中间",
           "push1": "推①剪切", "push2": "推②剪切·升"}

_SCOPES = {
    "timeline": ("action", "move", "cycle", "gate_start", "gate_measure",
                 "gate_fix", "gate_result", "gate_fail", "scan", "vacuum",
                 "handoff", "start", "stop", "abort", "arm_end",
                 "cutter", "round", "pipeline"),
    "cycles": ("cycle", "round"),
    "actions": ("action", "cycle", "vacuum", "gate_result", "cutter",
                "round"),
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


def _spd_txt(rec: dict | None) -> str:
    """一条 cycle 记录 → 该轮实际速度文本: '30%' / 中途改速 '30→50%'。
    speeds_used(本轮各动作实际下发值)优先,旧册无此字段退 speed_pct。"""
    if not rec:
        return ""
    used = rec.get("speeds_used")
    if used:
        return "→".join(f"{v:g}%" for v in used)
    v = rec.get("speed_pct")
    return f"{v:g}%" if isinstance(v, (int, float)) else ""


def _round_speed_txt(speeds: dict[str, str]) -> str:
    """{armleft: '30%', armright: '40%'} → 'A臂 30% / B臂 40%'(缺臂跳过)。"""
    parts = [f"{_ARM_LABEL[a]}臂 {s}" for a in ("armleft", "armright")
             if (s := speeds.get(a))]
    return " / ".join(parts)


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
        dur = r.get("duration_s")
        cyc = r.get("cycle")
        spd = _spd_txt(r) or r.get("speed_pct")   # 30 / 30→50(中途改速)
        detail = ";".join(f"{p}={d}s" for p, d in r.get("phases") or [])
        detail += f';等待={r.get("wait_s")}s 净工作={r.get("work_s")}s'
        detail += f';吸取={r.get("picks")}次;近均值={r.get("avg_recent_s")}s'
        if r.get("gates"):
            detail += ";闸门=" + ",".join(
                f'{g.get("which")}:{g.get("tier")}' for g in r["gates"])
    elif k == "round":
        name = f'第{r.get("round")}轮' + ("(冷启动)" if r.get("cold") else "")
        cyc = r.get("round")
        t0, t1 = r.get("t_start"), r.get("t_end")
        dur = r.get("duration_s")
        a = r.get("arms") or {}
        l = a.get("armleft") or {}
        rt = a.get("armright") or {}
        detail = (f'左臂={l.get("duration_s")}s(吸{l.get("picks")}次'
                  f',等{l.get("wait_s")}s)'
                  if l else "")
        if rt:
            detail += (f';右臂={rt.get("duration_s")}s'
                       f'(等FILLED={rt.get("wait_s")},'
                       f'净夹放回归={rt.get("work_s")}s)')
        c = r.get("cutter") or {}
        detail += "".join(f';{ _CUT_CN.get(s, s) }={v}s'
                          for s, v in c.items())
    elif k == "cutter":
        s = str(r.get("step"))
        name = f'裁剪·{_CUT_CN.get(s, s)}'
        cyc = r.get("cycle")
        t0, t1 = r.get("t_start"), r.get("t_end")
        dur = r.get("duration_s")
        status = "ok" if r.get("ok", True) else f'✗ {r.get("error") or ""}'
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
        ("$cut_home", ("cutter_home",)),
        ("$wait", ("wait_empty", "wait_center", "cutter_wait_push1",
                   "cutter_wait_push2")),
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
        ("armright|B5_extra", ("B5_extra",)),
    ),
}
_ARM_LABEL = {"armleft": "A", "armright": "B"}
_FIXED_STEP = {"$wait": "等待/交接", "$pick": "视觉吸取(自动)",
               "$gate_align": "对准·中间件", "$gate_align2": "对准·料仓",
               "$cut_home": "升降回零"}
_TITLES_CACHE: tuple[float, dict] = (0.0, {})


def _step_labels() -> dict:
    """步骤键 → 显示名。动作步取激活流水线方案当前提示名(按文件 mtime
    缓存);flow_store 不可用时退回动作 id,报表绝不因它报错。"""
    global _TITLES_CACHE
    out = dict(_FIXED_STEP)
    try:
        import flow_store
        m = flow_store.active_path().stat().st_mtime
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
    (cycle_gap/aborted/…)封住当前步。wait=True 的步($wait 等待/交接)
    甘特图渲染成灰色等待块;key=True(示教动作/视觉吸取/机构步)为关键步,
    报告页甘特优先标注名字。"""
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
        key = _ARM_STEPS[arm][idx][0]
        name = labels.get(key, key.split("|")[-1])
        if cur is not None and cur["name"] == name:
            continue                       # 同大步内的子步切换,不切段
        if cur is not None:
            cur["t_end"] = r.get("ts")
            steps.append(cur)
        cur = {"name": name, "t_start": r.get("ts"), "t_end": None,
               "wait": key == "$wait",
               "key": key == "$pick" or not key.startswith("$")}
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
            "name": s["name"], "wait": s.get("wait", False),
            "key": s.get("key", False),
            "start": datetime.fromtimestamp(t0).strftime(
                "%Y-%m-%d %H:%M:%S") if t0 else None,
            "end": datetime.fromtimestamp(t1).strftime(
                "%Y-%m-%d %H:%M:%S") if t1 else None,
            "t_start": t0, "t_end": t1,
            "duration_s": round(t1 - t0, 1) if (t0 and t1) else None,
            "ongoing": t1 is None,
        })
    return out


def _cyc_split(c: dict) -> tuple:
    """cycle 事件 → (wait_s, work_s, picks)。新册带显式字段;旧册从
    phases 序列补算(口径同 flow.py::_mark_cycle),历史日志同样出报表。"""
    ph = c.get("phases") or []
    wait, work = c.get("wait_s"), c.get("work_s")
    if wait is None:
        wait = round(sum(d for p, d in ph if str(p).startswith("wait_")), 1)
    if work is None:
        w = 0.0
        for p, d in reversed(ph):
            if str(p).startswith("wait_"):
                break
            w += d
        work = round(w, 1)
    picks = c.get("picks")
    if picks is None:
        picks = sum(1 for p, _ in ph if p == "scan")
    return wait, work, picks


def _clip_blocks(steps: list[dict], t0: float, t1: float | None) -> list[dict]:
    """大步 → 甘特块(裁剪到轮窗口 [t0,t1];t1=None=进行中不裁右沿)。"""
    out = []
    for s in steps:
        b0, b1 = s.get("t_start"), s.get("t_end")
        if b0 is None:
            continue
        if t1 is not None:
            b1 = t1 if b1 is None else min(b1, t1)
            b0 = max(b0, t0)
            if b1 - b0 <= 0.05:
                continue          # 完全在窗口外(上一轮的尾巴)
        out.append({"n": s["name"], "t0": round(b0, 3),
                    "t1": round(b1, 3) if b1 is not None else None,
                    "wait": s.get("wait", False),
                    "arm": s.get("arm")})
    return out


def _run_blocks(rounds: list[dict], pending: dict | None,
                cut_steps: list[dict], filled_ts: list[float],
                events: list[dict], start_ts: float | None) -> dict:
    """整册一图的数据(一次启动 = 甘特一行): 各臂全部大步**不裁剪**合并 +
    机构全部步 + FILLED 时刻表 + 册时间范围。轮窗口裁剪会切掉并行溢出
    (左臂伸进上一轮的段),整程视图必须用未裁剪块才能看清两臂时间连接。"""
    blocks: dict[str, list] = {"armleft": [], "armright": [], "cutter": []}

    def _add(steps: list[dict]):
        for s in steps:
            arm = s.get("arm")
            t0, t1 = s.get("t_start"), s.get("t_end")
            if arm not in blocks or t0 is None:
                continue
            blocks[arm].append({
                "n": s["name"], "t0": round(t0, 3),
                "t1": round(t1, 3) if t1 is not None else None,
                "wait": s.get("wait", False), "arm": arm,
                "key": s.get("key", False),
                "dur": round(t1 - t0, 1) if t1 is not None else None})

    for r in rounds:
        _add(r.get("steps") or [])
    if pending:
        _add(pending.get("steps") or [])
    blocks["cutter"] = [
        {"n": _CUT_CN.get(s["step"], s["step"]), "t0": round(s["t0"], 3),
         "t1": round(s["t1"], 3), "wait": False, "arm": "cutter",
         "key": True, "dur": s.get("dur"), "cycle": s.get("cycle")}
        for s in cut_steps]
    for arr in blocks.values():
        arr.sort(key=lambda b: b["t0"])
    t0 = start_ts
    if t0 is None:
        heads = [b["t0"] for arr in blocks.values() for b in arr]
        t0 = min(heads) if heads else None
    tails = [b["t1"] for arr in blocks.values() for b in arr
             if b["t1"] is not None]
    tails += [r["t_end"] for r in rounds if r.get("t_end")]
    return {"blocks": blocks, "filled_ts": sorted(filled_ts),
            "t0": t0, "t1": max(tails) if tails else None,
            "t_last": max((r.get("ts") or 0) for r in events) if events
            else None}


def _rounds_of_run(events: list[dict]) -> tuple[list[dict], dict | None,
                                                dict]:
    """一册事件 → (完成轮列表, 进行中轮或 None, 整册甘特数据)。

    轮次口径(2026-11-12 起,匹配流水线3): 一整轮 = 左臂吸放两摞 + 右臂
    夹完放完回归位置;轮的**完成** = 各臂 cycle#N 都收口(右臂是节拍
    尾部,实际决定轮末);轮 N 的**起点** = 上一轮完成时刻,第 1 轮 =
    本册启动事件 → duration_s 即「启动→首轮完成」(冷启动)与其后的
    稳态轮周期。左右重叠(左臂 cycle N 伸进上一轮窗口)是流水线3 的
    常态,甘特块按轮窗口裁剪呈现。单臂试跑册: 按该臂循环链算,单臂标记。"""
    cyc: dict[str, dict[int, dict]] = {}
    ph_by_arm: dict[str, list[dict]] = {}
    spd_by_arm: dict[str, list[tuple]] = {}   # [(ts, speed_pct)] 进行中轮取速度
    cut_steps: list[dict] = []                # 机构步(新册)
    filled_ts: list[float] = []               # FILLED 令牌时刻(甘特标记线)
    start_ts: float | None = None
    for r in events:
        k = r.get("kind")
        if k == "start" and start_ts is None and r.get("ts"):
            start_ts = r["ts"]
        elif k == "cutter":
            t0, t1 = _iso_dt(r.get("t_start")), _iso_dt(r.get("t_end"))
            if t0 and t1:
                cut_steps.append({"step": r.get("step"),
                                  "cycle": r.get("cycle"),
                                  "t0": t0.timestamp(), "t1": t1.timestamp(),
                                  "dur": r.get("duration_s")})
            continue
        elif k == "handoff" and r.get("fixture") == "FILLED" and r.get("ts"):
            filled_ts.append(r["ts"])
        arm = r.get("arm")
        if arm not in _ARM_STEPS or "ts" not in r:
            continue
        if k == "cycle":
            try:
                cyc.setdefault(arm, {})[int(r.get("cycle"))] = r
            except (TypeError, ValueError):
                continue
        elif k == "phase":
            ph_by_arm.setdefault(arm, []).append(r)
        elif k in ("action", "move") and r.get("speed_pct") is not None:
            spd_by_arm.setdefault(arm, []).append((r["ts"], r["speed_pct"]))
    if not cyc:
        return [], None, {}                   # 试跑册(无循环)不进客户视图
    for phs in ph_by_arm.values():
        phs.sort(key=lambda r: r["ts"])
    dual = len(cyc) == 2
    rounds: list[dict] = []
    prev_end: float | None = None
    for n in sorted({i for d in cyc.values() for i in d}):
        recs = {a: d[n] for a, d in cyc.items() if n in d}
        win = {}                             # arm -> (cycle起, cycle止)
        for a, c in recs.items():
            t0 = _iso_dt(c.get("t_start"))
            t1 = _iso_dt(c.get("t_end")) or _iso_dt(c.get("time"))
            if t0 and t1:
                win[a] = (t0.timestamp(), t1.timestamp())
        if not win:
            continue
        t1r = max(w[1] for w in win.values())
        if dual:                             # 轮起点=上轮完成/册启动
            t0r = prev_end if prev_end is not None \
                else (start_ts or min(w[0] for w in win.values()))
        else:                                # 单臂册: 该臂循环即轮
            t0r = min(w[0] for w in win.values())
        prev_end = t1r
        steps, left, right = [], None, None
        blocks: dict[str, list] = {}
        for arm in ("armleft", "armright"):
            c = recs.get(arm)
            if c is None or arm not in win:
                continue
            ct0, ct1 = win[arm]
            phs = [r for r in ph_by_arm.get(arm, [])
                   if ct0 - 0.5 <= r["ts"] <= ct1 + 0.05]
            st = _steps_from_phases(phs, arm, ct1)
            steps += st
            blocks[arm] = _clip_blocks(st, t0r, t1r)
            wait, work, picks = _cyc_split(c)
            ent = {"cycle_s": c.get("duration_s"), "wait_s": wait,
                   "work_s": work, "picks": picks, "speed": _spd_txt(c)}
            if arm == "armleft":
                left = ent
            else:
                right = ent
        cut: dict[str, float] = {}
        for s in cut_steps:                  # 同轮同名步取首现(每轮各一次)
            if s.get("cycle") == n and s["step"] not in cut:
                cut[s["step"]] = s["dur"]
        blocks["cutter"] = [{"n": _CUT_CN.get(s["step"], s["step"]),
                             "t0": round(s["t0"], 3), "t1": round(s["t1"], 3),
                             "wait": False, "arm": "cutter"}
                            for s in cut_steps if s.get("cycle") == n]
        filled = None
        if (right is not None and "armright" in win):
            cands = [t for t in filled_ts
                     if win["armright"][0] < t <= win["armright"][1]]
            filled = cands[0] if cands else None
        speeds = {a: _spd_txt(c) for a, c in recs.items()}
        rounds.append({
            "round": n, "cold": n == 1 and dual,
            "single_arm": None if dual else next(iter(cyc)),
            "t_start": t0r, "t_end": t1r,
            "start": datetime.fromtimestamp(t0r).strftime(
                "%Y-%m-%d %H:%M:%S"),
            "end": datetime.fromtimestamp(t1r).strftime(
                "%Y-%m-%d %H:%M:%S"),
            "duration_s": round(t1r - t0r, 1), "ongoing": False,
            "arms": sorted(recs), "steps": steps, "blocks": blocks,
            "left": left, "right": right, "cutter": cut,
            "filled_ts": filled,
            "speeds": speeds, "speed_txt": _round_speed_txt(speeds),
        })
    # 进行中: 某臂最后一条 cycle 之后还有 phase(未收口的循环);没收过
    # cycle 的臂(如右臂首轮一直等 FILLED)全部 phase 算进行中第 1 轮。
    pend = {}
    for arm, phs in ph_by_arm.items():
        if cyc.get(arm):
            last = max(cyc[arm])
            last_end = _iso_dt(cyc[arm][last].get("t_end"))
            if not last_end:
                continue
            cut_at = last_end.timestamp() + 0.05
        else:
            last, cut_at = 0, None
        trail = [r for r in phs
                 if (cut_at is None or r["ts"] > cut_at)
                 and r.get("phase") in _PHASE_STEP.get(arm, {})]
        if not trail:
            continue
        st = _steps_from_phases(trail, arm, None)
        if st:
            used = sorted({v for t, v in spd_by_arm.get(arm, [])
                           if cut_at is None or t > cut_at})
            pend[arm] = (last + 1, st,
                         "→".join(f"{v:g}%" for v in used))
    pending = None
    if pend:
        n = min(p[0] for p in pend.values())
        steps = [s for arm in ("armleft", "armright")
                 if arm in pend for s in pend[arm][1]]
        t_ev = max((r.get("ts") or 0) for r in events) if events else None
        t0p = rounds[-1]["t_end"] if rounds else None   # =上一完成轮末
        if t0p is None:
            ss = [s["t_start"] for s in steps if s.get("t_start")]
            t0p = min(ss) if ss else None
        blocks = {a: _clip_blocks(p[1], t0p or 0, t_ev)
                  for a, p in pend.items()}
        speeds = {a: p[2] for a, p in pend.items() if p[2]}
        pending = {
            "round": n, "cold": False,
            "single_arm": None if len(pend) == 2 else next(iter(pend)),
            "t_start": t0p,
            "start": datetime.fromtimestamp(t0p).strftime(
                "%Y-%m-%d %H:%M:%S") if t0p else None,
            "end": None, "duration_s": None, "ongoing": True,
            "arms": sorted(pend), "steps": steps, "blocks": blocks,
            "left": None, "right": None,
            "cutter": {s["step"]: s["dur"] for s in cut_steps
                       if s.get("cycle") == n},
            "filled_ts": next((t for t in filled_ts
                               if t0p is not None and t > t0p), None),
            "speeds": speeds, "speed_txt": _round_speed_txt(speeds),
        }
    return (rounds, pending,
            _run_blocks(rounds, pending, cut_steps, filled_ts, events,
                        start_ts))


def aggregate_report(start: str = "", end: str = "",
                     *, active: bool = False) -> dict:
    """时间段内全部日志册 → 客户视图报告(纯读现有 jsonl,不写盘)。
    返回 runs(按册分组,整程甘特数据)/rounds(跨册平铺)/summary。
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
    g_runs: list[dict] = []        # 按册分组: 一次启动 = 甘特一行
    sess = 0                       # 次 = 一次点启动(一册日志);内含多轮
    for i, (_, name, ev) in enumerate(runs):
        rs, pending, extra = _rounds_of_run(ev)
        tag = re.sub(r"^run_\d{8}_\d{6}_?", "", name) or "run"
        added = []
        for r in rs:
            r["run"] = f'{r["start"][:16]} {tag}'.strip()
            if t0 <= r["t_start"] <= t1:
                added.append(r)
                for a, key in (("armleft", "left"), ("armright", "right")):
                    e = r.get(key) or {}
                    if isinstance(e.get("cycle_s"), (int, float)):
                        arm_cyc.setdefault(a, []).append(e["cycle_s"])
        if pending and active and i == len(runs) - 1:
            if t0 <= (pending.get("t_start")
                      or (_iso_dt(pending["start"]).timestamp()
                          if _iso_dt(pending["start"]) else t0)) <= t1:
                pending["run"] = f'{(pending["start"] or "")[:16]} {tag}'.strip()
                added.append(pending)
        if added:
            sess += 1
            for r in added:
                r["session"] = sess
                r.pop("steps", None)     # 明细块并入册级 blocks,轮记录瘦身
                r.pop("blocks", None)
            g_runs.append({
                "session": sess, "name": name, "tag": tag,
                "run": added[0]["run"], "start": added[0]["start"],
                "end": added[-1]["end"], "t_start": added[0]["t_start"],
                "t_end": added[-1]["t_end"], "rounds": added,
                "blocks": extra["blocks"], "filled_ts": extra["filled_ts"],
                "t_last": extra["t_last"], "pending": added[-1]["ongoing"],
            })
            rounds.extend(added)
    done = [r for r in rounds if not r["ongoing"]]
    total = round(sum(r["duration_s"] for r in done), 1) if done else None
    _mean = lambda v: round(sum(v) / len(v), 1) if v else None      # noqa: E731
    colds = [r["duration_s"] for r in done if r.get("cold")]
    steady = [r["duration_s"] for r in done
              if not r.get("cold") and not r.get("single_arm")]
    rworks = [r["right"]["work_s"] for r in done
              if r.get("right")
              and isinstance(r["right"].get("work_s"), (int, float))]
    lcycs = [r["left"]["cycle_s"] for r in done
             if r.get("left")
             and isinstance(r["left"].get("cycle_s"), (int, float))]
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
        # 轮次口径指标(2026-11-12): 冷启动=启动→首轮完成;稳态=其后轮周期;
        # 右臂净工作=FILLED 到手→夹完放完回归;左臂一轮=吸放两摞全程。
        "cold_s": colds[-1] if colds else None,
        "cold_avg_s": _mean(colds),
        "steady_rounds": len(steady),
        "steady_avg_s": _mean(steady),
        "right_work_avg_s": _mean(rworks),
        "left_cycle_avg_s": _mean(lcycs),
        "per_arm": {a: {"label": _ARM_LABEL[a] + "臂", "rounds": len(v),
                        "avg_cycle_s": round(sum(v) / len(v), 1)}
                    for a, v in arm_cyc.items() if v},
    }
    return {"runs": g_runs, "rounds": rounds, "summary": summary}


def export_report_csv(start: str = "", end: str = "",
                      *, active: bool = False) -> tuple[str, str]:
    """时间段 → 客户视图聚合 CSV(UTF-8 BOM,一行一轮 + 末尾总计行)。
    列 = 轮周期/左右臂分解(wait/净工作)/机构步/速度;冷启动在性质列。"""
    rep = aggregate_report(start, end, active=active)
    buf = io.StringIO()
    w = csv.writer(buf, lineterminator="\n")
    w.writerow(["次", "轮次", "性质", "开始时间", "结束时间", "轮周期(秒)",
                "左臂(秒)", "吸取次数", "左臂等待(秒)",
                "右臂(秒)", "右臂等待(秒)", "右臂净夹放回归(秒)",
                "推①(秒)", "推②(秒)", "升降(秒)", "速度"])
    for r in rep["rounds"]:
        l, rt = r.get("left") or {}, r.get("right") or {}
        c = r.get("cutter") or {}
        if r.get("ongoing"):
            kind = "进行中"
        elif r.get("single_arm"):
            kind = f'单臂{_ARM_LABEL.get(r["single_arm"], "")}'
        elif r.get("cold"):
            kind = "冷启动"
        else:
            kind = "稳态"
        w.writerow([f'第{r.get("session", 1)}次', f'第{r["round"]}轮', kind,
                    r["start"], r["end"] or "进行中",
                    "" if r["duration_s"] is None else r["duration_s"],
                    l.get("cycle_s", ""), l.get("picks", ""),
                    l.get("wait_s", ""),
                    rt.get("cycle_s", ""), rt.get("wait_s", ""),
                    rt.get("work_s", ""),
                    c.get("push1", ""), c.get("push2", ""),
                    c.get("lift_mid", c.get("lift_home", "")),
                    r.get("speed_txt", "")])
    s = rep["summary"]
    if s["rounds_total"]:
        w.writerow(["总计",
                    f'{s["rounds_total"]}轮'
                    + ("(另含进行中)" if s["ongoing_rounds"] else ""),
                    f'冷启动{s["cold_s"] if s["cold_s"] is not None else "-"}s'
                    f' · 稳态均{s["steady_avg_s"] if s["steady_avg_s"] is not None else "-"}s'
                    f'({s["steady_rounds"]}轮)',
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
