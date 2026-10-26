# -*- coding: utf-8 -*-
"""双臂流水线的持久化: 配置 / 动作路点 / 对准参考 / 手眼绑定。
================================================================
目录 flow_data/(与 webapp 同级,整目录可拷):
  flow_config.json            流水线参数 + 各臂动作路点 + 对准参考
  handeye_armright.yaml       右臂手眼标定(格式同 pick_data 左臂 yaml,
                              未标定时不存在 → 右臂对准闸门不可用)

设计要点(见 doc/双臂协同上下料工作流方案_2026-10-20.md):
  - 动作 = 关节路点序列(用户在网页上"记录当前位"逐点添加,随时可改,
    改完无需动任何代码);
  - 对准参考 = 示教时码在基座系的位置 p_base(经手眼 X 换算);运行时
    重测 p_base_now,Δ = now − ref 即基座系修正量 —— 差分设计使
    marker_size 的系统比例误差在 Δ 中抵消(示教与运行同距离)。
"""
from __future__ import annotations

import json
import os
import tempfile
import time
from pathlib import Path

import numpy as np
import yaml

FLOW_DIR = Path(__file__).resolve().parent / "flow_data"
CONFIG_PATH = FLOW_DIR / "flow_config.json"
# 左臂手眼直接复用一步到位模块的标定文件;右臂是独立文件(待标定)
HANDEYE_PATHS = {
    "armleft": FLOW_DIR.parent / "pick_data" / "handeye_result_tianlian_left.yaml",
    "armright": FLOW_DIR / "handeye_armright.yaml",
}
HANDEYE_ROLE = {"armleft": "left", "armright": "right"}   # 标定绑定的相机角色

# 动作槽位: 固定骨架,用户只往里灌路点。wait 是进动作前的等待条件,
# center=动作期间持有中心共享区(双臂互斥的硬联锁)。
ACTIONS_META = {
    "armleft": {
        "A1_transfer": {"title": "A1 转移: 吸料后 → 中间件上方(末点=对准观察位)",
                        "wait": "center_free", "center": False},
        "A2_place":    {"title": "A2 放中间件: 对准后下降放料 → 放开真空 → 退出中心",
                        "wait": "fixture_empty", "center": True},
    },
    "armright": {
        "B0_pre":      {"title": "B0 预备(可选): 收回位 → 中心外预备位(不进中心)",
                        "wait": None, "center": False},
        "B1_approach": {"title": "B1 到中间件: 预备位 → 中间件上方(末点=对准观察位)",
                        "wait": "fixture_filled", "center": True},
        "B2_pick":     {"title": "B2 取料: 对准后下降取料 → 吸住 → 退出中心",
                        "wait": None, "center": True},
        "B3_transfer": {"title": "B3 转移料仓: → 料仓上方(末点=对准观察位)",
                        "wait": None, "center": False},
        "B4_place":    {"title": "B4 放料仓: 下降放料 → 放开真空 → 收回",
                        "wait": None, "center": False},
    },
}

ARM_KEYS = ("armleft", "armright")

# 动作间夹爪整形插槽: 键=动作名(该动作完成后执行)/"start"(循环首步前)。
# 用户可自行开关任意插槽,不限 B2→B3。
POST_GRIP_SLOTS = {
    "armleft": {
        "start":       "循环首·吸取前",
        "A1_transfer": "A1 后·对准前",
        "A2_place":    "A2 后·循环末",
    },
    "armright": {
        "start":       "循环首·B0 前",
        "B0_pre":      "B0 后·B1 前",
        "B1_approach": "B1 后·对准前",
        "B2_pick":     "B2 后·B3 前",
        "B3_transfer": "B3 后·对准(料仓)前",
        "B4_place":    "B4 后·循环末",
    },
}

POST_GRIP_DEFAULT = {"enabled": False, "position": 300.0,
                     "velocity": 500.0, "wait_s": 1.0}

DEFAULTS: dict = {
    "arms": {
        "armleft": {
            "cam_role": "left",
            "align": {
                "enabled": True,
                "marker": {"type": "aruco", "id": 0},   # 中间件码(左臂视角)
                "marker_size_mm": 100.0,                # 实测码边,只影响绝对 Z,不影响 Δ
                "ref": None,            # 示教时记录,见 flow.py record_align_ref
            },
            "vacuum_release": {"mode": "gripper",       # 放料时真空断开方式
                               "gripper_position": 600,
                               "gripper_velocity": 500},
            # 动作间夹爪整形(通用,插槽见 POST_GRIP_SLOTS;原 b23_gripper
            # = post_grippers.B2_pick,load_config 自动迁移)
            "post_grippers": {k: dict(POST_GRIP_DEFAULT)
                              for k in POST_GRIP_SLOTS["armleft"]},
        },
        "armright": {
            "cam_role": "right",
            "align": {
                "enabled": True,
                "marker": {"type": "aruco", "id": 1},   # 中间件码(右臂视角)
                "marker_size_mm": 100.0,
                "ref": None,
            },
            # 右臂在料仓还要对一次: 第二组 marker/ref,B3 末点用
            "align2": {
                "enabled": True,
                "marker": {"type": "aruco", "id": 2},   # 料仓码
                "marker_size_mm": 100.0,
                "ref": None,
            },
            "vacuum_grab": {"mode": "gripper",          # 取料时真空建立方式
                            "gripper_position": 0,
                            "gripper_velocity": 500},
            "vacuum_release": {"mode": "gripper",
                               "gripper_position": 600,
                               "gripper_velocity": 500},
            # 动作间夹爪整形(通用,插槽见 POST_GRIP_SLOTS;原 b23_gripper
            # = post_grippers.B2_pick,load_config 自动迁移)
            "post_grippers": {k: dict(POST_GRIP_DEFAULT)
                              for k in POST_GRIP_SLOTS["armright"]},
        },
    },
    # 三档闸门: verify=每轮测一次,达标即过(T1,默认);off=纯回放(T0,极限节拍,
    # 靠事件兜底);修正量超 max_corr 视为工装被撞,停机报警。
    "gate": {
        "mode": "verify",
        "tol_xy_mm": 2.0,
        "tol_z_mm": 4.0,          # 小码@0.4m 的 PnP z 抖动 ±4mm,3 会压线
        "max_corr_mm": 40.0,
        "iterations": 3,          # 最大修正步数(每步=测一次+修一次)
        "frames": 5,              # 每次测量取几帧中值(多帧压 PnP 抖动)
        "settle_s": 0.8,          # 到观察位后等稳定
        "reanchor_max_mm": 25.0,  # 重锚Δ超此值=参考与示教不配套,停线(0=关)
        "hold_open_s": 1.5,       # 真空断开后等物料脱附
        "hold_grab_s": 1.5,       # 真空建立等待
    },
    "move": {
        "speed": 15,              # 动作回放/修正移动速度%
        "move_timeout_s": 60.0,
        "joint_tol_deg": 1.5,     # 路点回放段后关节验证容差
        "xy_res_tol_mm": 8.0,     # 修正段后 FK 验证容差(同一步到位)
        "z_res_tol_mm": 5.0,
        "reanchor_min_mm": 0.5,   # 闸门后重锚平移生效阈值,小于则关节回放
        "place_xy_tol_mm": 2.5,   # 重锚放置段段后 FK 验证容差(比修正段紧)
        "place_z_tol_mm": 3.0,
    },
    "safety": {
        # 中心区串行(首次双臂联调建议开): 左臂吸取后先等中间件 EMPTY
        # 再进中心,并持锁贯穿 A1→闸门→A2 —— 消除"左臂停在中间件上方
        # 观察位等料时,右臂 B1/B2 进中心取料"的双臂同区并行窗口。
        "center_serialize": False,
    },
    "stats": {"cycles": 0},       # 流水线累计循环数
}


def _deep_merge(base: dict, patch: dict) -> dict:
    out = dict(base)
    for k, v in (patch or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _deep_merge(out[k], v)
        else:
            out[k] = v
    return out


def _fill_actions(cfg: dict) -> dict:
    """保证每个动作槽位存在(带默认空路点),老配置升级用。"""
    for arm, acts in ACTIONS_META.items():
        slot = cfg["arms"].setdefault(arm, {})
        actions = slot.setdefault("actions", {})
        for name, meta in acts.items():
            a = actions.setdefault(name, {})
            a.setdefault("title", meta["title"])
            a.setdefault("waypoints", [])        # [{joints_deg, end_pose, created}]
            a.setdefault("speed", None)          # None=用全局 move.speed
            a.setdefault("vacuum", None)         # {"mode":"on|off","at":"start|end"}
    return cfg


def load_config() -> dict:
    cfg = json.loads(json.dumps(DEFAULTS))       # 深拷贝默认
    if CONFIG_PATH.is_file():
        try:
            data = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
            if isinstance(data, dict):
                cfg = _deep_merge(cfg, data)
        except Exception:                              # noqa: BLE001
            pass                                       # 损坏则回默认
    return _migrate_b23(_fill_actions(cfg))


def _migrate_b23(cfg: dict) -> dict:
    """旧 b23_gripper(只锁 B2→B3) → 通用 post_grippers。
    读到就丢弃旧键;若旧键启用且 B2 插槽还是默认值,则把参数搬过去。
    仅供 load_config 使用,不单独落盘(下次任意 save 即清除旧键)。"""
    for arm in ARM_KEYS:
        slot = cfg["arms"].get(arm) or {}
        b23 = slot.pop("b23_gripper", None)
        if not (b23 and b23.get("enabled")):
            continue
        pg = slot.setdefault("post_grippers", {})
        cur = pg.get("B2_pick")
        if cur is None or cur == POST_GRIP_DEFAULT:
            pg["B2_pick"] = {
                "enabled": True,
                "position": float(b23.get("gripper_position", 300)),
                "velocity": float(b23.get("gripper_velocity", 500)),
                "wait_s": float(b23.get("wait_s", 1.0)),
            }
    return cfg


def save_config(cfg: dict) -> None:
    FLOW_DIR.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=str(FLOW_DIR), suffix=".tmp")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            json.dump(cfg, f, ensure_ascii=False, indent=2)
        os.replace(tmp, CONFIG_PATH)
    except Exception:                                   # noqa: BLE001
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise


def update_config(patch: dict) -> dict:
    """浅合并补丁(仅参数区 gate/move 与 marker/marker_size_mm/
    cam_role/真空配置),动作路点与参考必须走示教端点,不接受此处修改。"""
    cfg = load_config()
    if "gate" in patch:
        g = patch["gate"]
        if g.get("mode") not in (None, "verify", "off"):
            raise ValueError("gate.mode 只能是 verify/off")
        for k in ("tol_xy_mm", "tol_z_mm", "max_corr_mm", "reanchor_max_mm"):
            if k in g:
                g[k] = float(min(max(float(g[k]), 0.0), 100.0))
        for k in ("iterations", "frames"):
            if k in g:
                g[k] = int(min(max(int(g[k]), 1), 5))
        for k in ("settle_s", "hold_open_s", "hold_grab_s"):
            if k in g:
                g[k] = float(min(max(float(g[k]), 0.0), 10.0))
        cfg["gate"] = _deep_merge(cfg["gate"], g)
    if "move" in patch:
        m = patch["move"]
        if "speed" in m:
            m["speed"] = int(min(max(int(m["speed"]), 1), 100))
        for k in ("move_timeout_s",):
            if k in m:
                m[k] = float(min(max(float(m[k]), 10.0), 300.0))
        for k in ("joint_tol_deg", "xy_res_tol_mm", "z_res_tol_mm",
                  "reanchor_min_mm", "place_xy_tol_mm", "place_z_tol_mm"):
            if k in m:
                m[k] = float(min(max(float(m[k]), 0.5), 50.0))
        cfg["move"] = _deep_merge(cfg["move"], m)
    if "safety" in patch:
        s = patch["safety"] or {}
        if "center_serialize" in s:
            cfg.setdefault("safety", {})["center_serialize"] = bool(
                s["center_serialize"])
    for arm in ARM_KEYS:
        p = patch.get(arm)
        if not p:
            continue
        slot = cfg["arms"][arm]
        if "cam_role" in p:
            if p["cam_role"] not in ("left", "head", "right"):
                raise ValueError("cam_role 只能是 left/head/right")
            slot["cam_role"] = p["cam_role"]
        for ak in ("align", "align2"):
            if ak in p and slot.get(ak):
                a = p[ak]
                if "marker" in a:
                    mk = a["marker"]
                    if mk.get("type") not in ("aruco", "qr"):
                        raise ValueError("marker.type 只能是 aruco/qr")
                    if mk["type"] == "aruco":
                        mk["id"] = int(min(max(int(mk.get("id", 0)), 0), 49))
                    else:                                  # qr: data 留空=任意一张
                        mk["data"] = str(mk.get("data", ""))[:64]
                    slot[ak]["marker"] = mk
                if "marker_size_mm" in a:
                    slot[ak]["marker_size_mm"] = float(
                        min(max(float(a["marker_size_mm"]), 20.0), 500.0))
                if "enabled" in a:
                    slot[ak]["enabled"] = bool(a["enabled"])
        for vk in ("vacuum_grab", "vacuum_release"):
            if vk in p and slot.get(vk):
                v = p[vk]
                slot[vk] = {
                    "mode": v.get("mode", "manual") if v.get("mode") in
                    ("manual", "gripper") else "manual",
                    "gripper_position": float(v.get("gripper_position", 0)),
                    "gripper_velocity": max(500, float(v.get("gripper_velocity",
                                                             500))),
                }
        if "post_grippers" in p:
            pg_in = p["post_grippers"] or {}
            pg = slot.setdefault("post_grippers", {})
            for name in (POST_GRIP_SLOTS.get(arm) or {}):
                if name not in pg_in:
                    continue
                g = pg_in[name] or {}
                pg[name] = {
                    "enabled": bool(g.get("enabled", False)),
                    "position": float(min(max(float(g.get(
                        "position", 300)), 0.0), 1000.0)),
                    "velocity": float(min(max(float(g.get(
                        "velocity", 500)), 100.0), 1000.0)),
                    "wait_s": float(min(max(float(g.get("wait_s", 1.0)),
                                            0.0), 10.0)),
                }
    save_config(cfg)
    return cfg


# ---------------------------------------------------------------- 手眼
def load_handeye_T(arm: str):
    """读 T_ee_camera(4x4)。缺文件/坏文件抛 FileNotFoundError/ValueError。"""
    path = HANDEYE_PATHS[arm]
    if not path.is_file():
        raise FileNotFoundError(
            f"{arm} 手眼标定文件不存在: {path}"
            + ("(左臂用 pick_data 的标定;右臂尚未标定 —— 右臂对准前必须先标)"
               if arm == "armright" else ""))
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    T = np.asarray(data.get("T_ee_camera"), dtype=float)
    if T.shape != (4, 4) or not np.isfinite(T).all():
        raise ValueError("T_ee_camera 不是合法 4x4 矩阵")
    return T


def handeye_info(arm: str) -> dict:
    path = HANDEYE_PATHS[arm]
    if not path.is_file():
        return {"ok": False, "arm": arm, "path": str(path),
                "hint": "右臂未标定:对准闸门需要的像素→基座系变换只有左臂有"
                        "(标定流程见流水线页说明)"}
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
        rs = data.get("residual_stats") or {}
        return {"ok": True, "arm": arm, "path": str(path),
                "time": data.get("time"),
                "method": (rs.get("method_used")
                           or ",".join((data.get("methods") or {}).keys())),
                "pos_std_m": rs.get("pos_std_m"),
                "calibrated_role": HANDEYE_ROLE[arm]}
    except Exception as e:                                     # noqa: BLE001
        return {"ok": False, "arm": arm, "error": str(e)}


# ---------------------------------------------------------------- 动作
def record_waypoint(arm: str, action: str, joints_deg: list,
                    end_pose: dict) -> dict:
    cfg = load_config()
    a = _action(cfg, arm, action)
    wp = {"joints_deg": [round(float(v), 3) for v in joints_deg],
          "end_pose": end_pose,
          "created": time.strftime("%Y-%m-%d %H:%M:%S")}
    a["waypoints"].append(wp)
    save_config(cfg)
    return wp


def delete_waypoint(arm: str, action: str, index: int) -> None:
    cfg = load_config()
    a = _action(cfg, arm, action)
    wps = a["waypoints"]
    if not (0 <= index < len(wps)):
        raise ValueError(f"路点序号 {index} 越界(共 {len(wps)} 个)")
    wps.pop(index)
    save_config(cfg)


def overwrite_waypoint(arm: str, action: str, index: int, joints_deg: list,
                       end_pose: dict) -> dict:
    """用当前位姿覆盖第 index 个路点(其余路点与顺序不动)。
    created 保留原示教时间,另记 updated。"""
    cfg = load_config()
    wps = _action(cfg, arm, action)["waypoints"]
    if not (0 <= index < len(wps)):
        raise ValueError(f"路点序号 {index} 越界(共 {len(wps)} 个)")
    wp = {"joints_deg": [round(float(v), 3) for v in joints_deg],
          "end_pose": end_pose,
          "created": wps[index].get("created"),
          "updated": time.strftime("%Y-%m-%d %H:%M:%S")}
    wps[index] = wp
    save_config(cfg)
    return wp


def mark_center(arm: str, action: str, index: int, center: bool) -> dict:
    """设置/取消路点的"中点"标记(双臂中心互斥的路点级细化)。

    center=True: 该点在中心区(中间件上方重叠体积)内 —— 回放时走向它
    之前抢中心锁,走到其后第一个非标记点(退出段走完)才交锁。
    取消=移除键,保持旧配置文件干净(缺键=非中点)。"""
    cfg = load_config()
    wps = _action(cfg, arm, action)["waypoints"]
    if not (0 <= index < len(wps)):
        raise ValueError(f"路点序号 {index} 越界(共 {len(wps)} 个)")
    if center:
        wps[index]["center"] = True
    else:
        wps[index].pop("center", None)
    save_config(cfg)
    return wps[index]


def set_waypoint_wait(arm: str, action: str, index: int, seconds: float) -> dict:
    """设置/取消路点的"到点等待"(回放到该点后停留 N 秒再走剩余段)。

    用途: 吸/放前稳一稳、等真空建立、等抖动衰减等。
    seconds<=0: 取消(移除键,保持旧配置文件干净,缺键=不等)。"""
    cfg = load_config()
    wps = _action(cfg, arm, action)["waypoints"]
    if not (0 <= index < len(wps)):
        raise ValueError(f"路点序号 {index} 越界(共 {len(wps)} 个)")
    if seconds > 0:
        if seconds > 60:
            raise ValueError("到点等待需在 0~60s(0=取消)")
        wps[index]["wait_s"] = round(float(seconds), 2)
    else:
        wps[index].pop("wait_s", None)
    save_config(cfg)
    return wps[index]


def move_waypoint(arm: str, action: str, index: int, offset: int) -> None:
    """与相邻路点交换位置(offset=-1 前移 / +1 后移)。"""
    if offset not in (-1, 1):
        raise ValueError("offset 只能是 -1/+1")
    cfg = load_config()
    wps = _action(cfg, arm, action)["waypoints"]
    if not (0 <= index < len(wps)):
        raise ValueError(f"路点序号 {index} 越界(共 {len(wps)} 个)")
    j = index + offset
    if not (0 <= j < len(wps)):
        raise ValueError(f"点 {index + 1} 已在序列{'头' if offset < 0 else '尾'},"
                         f"无法再{'前移' if offset < 0 else '后移'}")
    wps[index], wps[j] = wps[j], wps[index]
    save_config(cfg)


def clear_action(arm: str, action: str) -> None:
    cfg = load_config()
    _action(cfg, arm, action)["waypoints"] = []
    if arm == "armleft" and action == "A2_place":
        pass        # 对准参考不随路点清除(观察位在 A1 末点)
    save_config(cfg)


def _action(cfg: dict, arm: str, action: str) -> dict:
    if arm not in ARM_KEYS:
        raise ValueError(f"arm 必须是 {'/'.join(ARM_KEYS)}")
    acts = ACTIONS_META.get(arm) or {}
    if action not in acts:
        raise ValueError(f"未知动作槽位: {action}(可选 {list(acts)})")
    return cfg["arms"][arm]["actions"][action]


def set_action_title(arm: str, action: str, title: str) -> str:
    """改动作槽位的步骤说明(网页上 A几/B几 后面那行提示,工艺变了可自行改)。"""
    t = str(title).strip()
    if not t:
        raise ValueError("步骤说明不能为空")
    t = t[:60]                                  # 一行内显示,截断保平安
    cfg = load_config()
    _action(cfg, arm, action)["title"] = t
    save_config(cfg)
    return t


def set_action_vacuum(arm: str, action: str, event: str | None,
                      at: str = "end") -> dict:
    """配置动作中途的真空触发。event: grab/release/None(清除);
    at: "start" | "end" | 路点序号字符串("2"=到第 3 个路点后)。"""
    cfg = load_config()
    a = _action(cfg, arm, action)
    if event in (None, "", "none"):
        a["vacuum"] = None
    else:
        if event not in ("grab", "release"):
            raise ValueError("event 只能是 grab/release/none")
        if at not in ("start", "end") and not str(at).isdigit():
            raise ValueError('at 只能是 "start"/"end"/路点序号')
        a["vacuum"] = {"event": event, "at": at}
    save_config(cfg)
    return a


def readiness(cfg: dict | None = None, only: str | None = None) -> dict:
    """启动前置检查(网页就绪清单与 FlowRunner.start 共用)。

    only: None=双臂全查 / "armleft"|"armright"=单臂测试只查该臂。
    missing = 阻断项(不齐不给跑);warnings = 提示项(能跑但有风险)。
    """
    cfg = cfg or load_config()
    missing: list[str] = []
    warnings: list[str] = []
    la, ra = cfg["arms"]["armleft"], cfg["arms"]["armright"]
    want_l, want_r = only in (None, "armleft"), only in (None, "armright")
    import pick_store as _ps
    if want_l and not _ps.load_config().get("scan_pose"):
        missing.append("左臂扫描位未示教(物料检测页『存当前为扫描位』)")
    if want_l:
        for name in ("A1_transfer", "A2_place"):
            if not la["actions"][name]["waypoints"]:
                missing.append(f"左臂 {name} 无路点")
    if want_r:
        for name in ("B1_approach", "B2_pick", "B3_transfer", "B4_place"):
            if not ra["actions"][name]["waypoints"]:
                missing.append(f"右臂 {name} 无路点")
    gate_on = cfg["gate"]["mode"] != "off"
    for arm, slot, which, where in (
            ("armleft", la, "align", "中间件"),
            ("armright", ra, "align", "中间件"),
            ("armright", ra, "align2", "料仓")):
        if arm == "armleft" and not want_l:
            continue
        if arm == "armright" and not want_r:
            continue
        if not slot.get(which):
            continue
        if gate_on and slot[which].get("enabled", True):
            if not slot[which].get("ref"):
                missing.append(f"{arm} {where}对准参考未记录")
            if not HANDEYE_PATHS[arm].is_file():
                missing.append(f"{arm} 手眼标定缺失"
                               + ("(右臂待标定,见页内说明)" if arm == "armright"
                                  else ""))
            if slot[which]["marker"].get("type") == "qr" and \
                    not _has_pyzbar():
                warnings.append(f"{arm} {where}用 QR 码但本进程无 pyzbar,"
                                "将检不到码(建议改 ArUco)")
        elif gate_on:
            warnings.append(f"{arm} {where}PBVS 对准已停用:该站按示教关节"
                            "回放,不测码不修正(页内『启用 PBVS 对准』开关)")
    for arm, name, want in (("armleft", "A2_place", "release"),
                            ("armright", "B2_pick", "grab"),
                            ("armright", "B4_place", "release")):
        if (arm == "armleft" and not want_l) or \
                (arm == "armright" and not want_r):
            continue
        vac = cfg["arms"][arm]["actions"][name].get("vacuum")
        if not vac:
            warnings.append(f"{arm} {name} 未配置真空触发({ '吸取' if want == 'grab' else '释放'}"
                            "会漏 —— 页内『动作真空触发』设置)")
        elif vac.get("event") != want:
            warnings.append(f"{arm} {name} 真空触发是 {vac.get('event')},"
                            f"流程预期 {want}")
    if want_r and not ra["actions"]["B0_pre"]["waypoints"]:
        warnings.append("右臂 B0 预备位为空(可选):配好后取料衔接更快")
    # —— 路点级"中点"标记的完整性提示(有标记才查) ——
    for arm, name, want in (("armleft", "A2_place", want_l),
                            ("armright", "B2_pick", want_r)):
        if not want:
            continue
        wps = cfg["arms"][arm]["actions"][name]["waypoints"]
        if wps and wps[-1].get("center"):
            warnings.append(f"{arm} {name} 末点仍是中点:放完在原位提前交锁"
                            "(建议末段补一个非中点退出路点)")
    # —— 双臂同区/码身份的风险提示(几何/配置类,不阻断启动) ——
    if want_l and len(la["actions"]["A2_place"]["waypoints"]) == 1:
        warnings.append("左臂 A2_place 只有 1 个路点:中心锁在放料位释放,"
                        "右臂 B1 可能随即进中心,而左臂要等下一轮回扫描位"
                        "才离开 —— 建议给 A2 补一个抬升/退出路点,"
                        "或开启 safety.center_serialize")
    if want_r:
        k1 = ((ra.get("align") or {}).get("ref") or {}).get("key")
        k2 = ((ra.get("align2") or {}).get("ref") or {}).get("key")
        if k1 and k2 and k1 == k2:
            warnings.append(f"中间件码与料仓码身份相同({k1}):QR data 留空"
                            "时闸门只认'第一张看到的码',无法区分两张码"
                            "——建议料仓换一张不同 data 的 QR 并重记参考")
    if only is None and not (cfg.get("safety") or {}).get(
            "center_serialize"):
        warnings.append("双臂并行模式:左臂会停在中间件上方观察位等右臂"
                        "B1/B2 取料 —— 首次联调建议开 safety.center_serialize"
                        " 或用单步确认模式慢速验证")
    return {"ok": not missing, "missing": missing, "warnings": warnings}


def _has_pyzbar() -> bool:
    try:
        import pyzbar  # noqa: F401
        return True
    except ImportError:
        # 与 flow_align 同法: 复用 upper_computer env 的纯 Python pyzbar
        import sys
        sys.path.append(
            "/home/user/miniconda3/envs/upper_computer/lib/python3.11/site-packages")
        try:
            import pyzbar  # noqa: F401
            return True
        except ImportError:
            return False


def action_hash(cfg: dict, arm: str, action: str) -> str:
    """动作路点内容的短 hash —— 对准参考绑定它,改动作可提示复验。"""
    import hashlib
    wps = cfg["arms"][arm]["actions"][action]["waypoints"]
    blob = json.dumps(wps, sort_keys=True).encode()
    return hashlib.sha1(blob).hexdigest()[:8]


def save_align_ref(arm: str, which: str, ref: dict) -> None:
    """which: align(中间件) / align2(料仓,仅右臂)。"""
    cfg = load_config()
    key = "align" if which == "align" else "align2"
    slot = cfg["arms"].get(arm)
    if slot is None or key not in slot:
        raise ValueError(f"{arm} 没有 {key} 槽位")
    slot[key]["ref"] = ref
    save_config(cfg)
