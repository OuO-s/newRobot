#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""扫描位→吸取 可达性离线验证(不动臂)。

每次重示教扫描位后运行:
    /usr/bin/python3 verify_scan_pose.py

读 pick_config.json(扫描位/残差/hover/走廊参数) + last_scan.json(袋位),
按 pick_flow 的走廊序列逐段 dry-IK:
    lift → rotate → horizontal → descend
(先试 direct 一段直线,不通走走廊 —— 与实跑逻辑一致),
每段报告首尾/相邻关节变化与 45° 护栏判定。
"""
import json
import math
import os
import sys

os.environ.setdefault("IK_URDF_MODEL", "y2_p_dt_710vg")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import ik_helper  # noqa: E402

ik_helper.JUMP_GUARD_MAX_DQ_DEG = 1e9    # 放开护栏看全程
ik_helper.NEAR_ZERO_TRAJ_DEG = -1.0
GUARD = 45.0

HERE = os.path.dirname(os.path.abspath(__file__))
cfg = json.loads(open(os.path.join(HERE, "pick_data", "pick_config.json"),
                      encoding="utf-8").read())
scan = json.loads(open(os.path.join(HERE, "pick_data", "last_scan.json"),
                       encoding="utf-8").read())
rows = scan.get("rows") or []
if not rows or not cfg.get("scan_pose"):
    print("缺扫描位或上次扫描结果;先跑一次扫描/示教")
    sys.exit(1)

row = next((r for r in rows if r.get("order") == 1), rows[0])
res = np.array(cfg.get("residual_mm", [0, 0, 0]), float) / 1000.0
per = (cfg.get("per_bag_residual_mm") or {}).get(str(row["order"]))
if per:
    res = res + np.array(per, float) / 1000.0
aim_xy = np.array(row["p_base"][:2]) + res[:2]
aim_z = row["p_base"][2] + cfg["hover_mm"] / 1000.0 + res[2]
align = cfg.get("align") or {}
align_rpy = list(align.get("rpy_deg", [0, 0, 0]))

sp = cfg["scan_pose"]
q = [float(v) for v in sp["joints_deg"]]
scan_pos = sp["end_pose"]["position"]
scan_quat = sp["end_pose"]["orientation"]
kin, *_ = ik_helper._get_kin("armleft", q)


def quat_of(T):
    R = T[:3, :3]
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    w = np.sqrt(tr + 1) / 2
    return [(R[2, 1] - R[1, 2]) / (4 * w), (R[0, 2] - R[2, 0]) / (4 * w),
            (R[1, 0] - R[0, 1]) / (4 * w), w]


if "--live" in sys.argv:
    # 用臂当前关节角当扫描位候选(FK 出上报系位姿,两系已证一致)
    import urllib.request
    with urllib.request.urlopen(
            "http://127.0.0.1:8001/api/arm/joints", timeout=5) as r:
        jj = json.loads(r.read())
    if not jj.get("ok"):
        print(f"读臂关节失败: {jj}")
        sys.exit(1)
    q = [float(v) for v in jj["deg"]]
    T_live = np.asarray(kin.fkine(q), dtype=float)
    scan_pos = T_live[:3, 3].tolist()
    scan_quat = quat_of(T_live)


def plan(q_cur, cur_pos, cur_quat, tgt, rpy):
    return ik_helper.handle_request({
        "arm": "armleft", "q_deg": q_cur, "op": "target",
        "target_pos_m": list(map(float, tgt)),
        "target_rpy_deg": list(map(float, rpy)),
        "cur_pos_m": list(map(float, cur_pos)),
        "cur_quat": list(map(float, cur_quat))})


# 扫描位自身 rpy(ZYX,与 pick_flow.rpy_deg_to_R 互逆;用于 lift 段保持原姿态)
R = ik_helper._quat_to_R(*scan_quat)
scan_rpy = [math.degrees(math.atan2(R[2, 1], R[2, 2])),
            math.degrees(-math.asin(max(-1, min(1, R[2, 0])))),
            math.degrees(math.atan2(R[1, 0], R[0, 0]))]

top_z = max(r["p_base"][2] for r in rows)
cruise_z = max(top_z + cfg["cruise_clearance_mm"] / 1000.0, scan_pos[2])

live = "--live" in sys.argv
hp = cfg.get("hover_pose")
if live and hp is None:
    hp = {"joints_deg": q, "end_pose": {"position": scan_pos,
                                        "orientation": scan_quat},
          "created": "(live)"}

print(f"{'【LIVE 臂当前位姿】' if live else '扫描位'} "
      f"{'(挪到位验证通过后点存!)' if live else '示教于 ' + str(sp.get('created'))} "
      f"joints={np.round(q,1).tolist()}")
print(f"扫描位 pos={np.round(scan_pos,3).tolist()} rpy="
      f"{np.round(scan_rpy,1).tolist()}")
if hp and not live:
    print(f"悬停位 {hp.get('created')} joints="
          f"{np.round([float(v) for v in hp['joints_deg']],1).tolist()}")
print(f"aim(含残差/hover)=[{aim_xy[0]:.3f},{aim_xy[1]:.3f},{aim_z:.3f}] "
      f"对准rpy={align_rpy}")

if hp:
    # —— 悬停位模式: 扫描→(关节腿,不过 IK,仅报行程)→悬停位→小段走廊 ——
    q_h = [float(v) for v in hp["joints_deg"]]
    leg = max(abs(b - a) for a, b in zip(q, q_h))
    hot = " ".join(f"J{i+1}{b-a:+.0f}" for i, (a, b) in enumerate(zip(q, q_h))
                   if abs(b - a) > 45)
    print(f"关节腿 扫描位→悬停位: 最大 {leg:.1f}° "
          f"{'(大行程,' + hot + ',低速走·首跑盯防)' if hot else ''}")
    T_h = np.asarray(kin.fkine(q_h), dtype=float)
    h_pos = T_h[:3, 3].tolist()
    h_quat = quat_of(T_h)
    h_rpy = [math.degrees(math.atan2(R[2, 1], R[2, 2])),
             math.degrees(-math.asin(max(-1, min(1, R[2, 0])))),
             math.degrees(math.atan2(R[1, 0], R[0, 0]))]
    cruise_z = max(top_z + cfg["cruise_clearance_mm"] / 1000.0, h_pos[2])
    print(f"悬停位 pos={np.round(h_pos,3).tolist()} rpy="
          f"{np.round(h_rpy,1).tolist()} | 越顶走廊 z={cruise_z:.3f}\n")
    q, scan_pos, scan_quat, scan_rpy = q_h, h_pos, h_quat, h_rpy

SEGS = [
    ("lift",       [scan_pos[0], scan_pos[1], cruise_z], scan_rpy),
    ("rotate",     [scan_pos[0], scan_pos[1], cruise_z], align_rpy),
    ("horizontal", [aim_xy[0], aim_xy[1], cruise_z],     align_rpy),
    ("descend",    [aim_xy[0], aim_xy[1], aim_z],        align_rpy),
]

# direct 一段直线(实跑优先尝试;悬停位模式下=悬停位→aim)
o = plan(q, scan_pos, scan_quat, [aim_xy[0], aim_xy[1], aim_z], align_rpy)
if o.get("ok"):
    t = o["waypoints_deg"]
    dq = max(abs(b - a) for a, b in zip(t[0], t[-1]))
    st = ik_helper._max_step_dq_deg(t)
    ok = dq <= GUARD and st <= GUARD
    print(f"direct {'悬停位' if hp else '扫描位'}→aim 一段直线: "
          f"{dq:.1f}°/{st:.1f}° "
          f"{'✓ 全程可走' if ok else '✗ 会跳解,将退走廊'}")
else:
    print(f"direct 不可达({o.get('error')}),走走廊")

print()
all_ok = True
q_cur, cur_pos, cur_quat = q, scan_pos, scan_quat
for name, tgt, rpy in SEGS:
    o = plan(q_cur, cur_pos, cur_quat, tgt, rpy)
    if not o.get("ok"):
        print(f"{name:10s} ✗ {o.get('error')}")
        all_ok = False
        break
    t = o["waypoints_deg"]
    dq = [b - a for a, b in zip(t[0], t[-1])]
    st = ik_helper._max_step_dq_deg(t)
    ok = max(abs(d) for d in dq) <= GUARD and st <= GUARD
    all_ok = all_ok and ok
    hot = " ".join(f"J{i+1}{d:+.0f}" for i, d in enumerate(dq)
                   if abs(d) > 25)
    print(f"{name:10s} →{np.round(tgt,3).tolist()} "
          f"{max(abs(d) for d in dq):5.1f}°/{st:5.1f}° "
          f"{'✓' if ok else '✗'}  {('大幅:' + hot) if hot else ''}")
    q_cur = t[-1]
    Tc = np.asarray(kin.fkine(q_cur), dtype=float)
    cur_pos, cur_quat = Tc[:3, 3].tolist(), quat_of(Tc)

print(f"\n结论: {'✓ 当前位姿可作悬停位(点『存当前为悬停位』)' if live and all_ok else '✓ 吸取链全程通过护栏,可实跑(建议先⚡干跑)' if all_ok else '✗ 未通过 —— ' + ('继续挪,再喊我验' if live else '重示教悬停位/扫描位再验')}")
