# -*- coding: utf-8 -*-
"""一步到位: 检测 → 手眼定位 → 走廊运动 → 到位吸取。
================================================================
几何全程用 :8000 上报系:
  T_base_ee = get_end_poses()(position + orientation[x,y,z,w])
  p_base    = T_base_ee @ T_ee_camera @ p_cam        (手眼约定同 3_verify_handeye)
运动段经 plan_cartesian_path(op="target", 带 cur_pos_m/cur_quat)——
ik_helper 此时把 target 解释为上报系,与几何自洽。

分段纪律(移植旧⑭ suction_bag.py,2026-10 实测教训):
  先竖直升走廊 → 原地转姿态 → 水平平移 → 垂直下降;
  水平移动只发生在越顶(最高袋面+clearance)或层内(袋面+hover)高度,
  绝不在中间高度横移;每段先规划(=dry-IK)失败即原地停,段后读真实
  位姿验证(xy ≤8mm / z ≤5mm,超差原地停人工接管)。

对准校正(align)可缺省: 缺省时不做原地转姿态、不做深度基准护栏,
日志给告警 —— 首跑就是这样(悬停在 袋+hover 处 → 人工微调 → 校正),
校正后残差/姿态/基准才生效。
"""
from __future__ import annotations

import base64
import json
import math
import threading
import time
import urllib.request
from collections import deque

import cv2
import numpy as np

import pick_store
import speed_store
from cartesian import plan_cartesian_path
from depth_provider import DepthError, DepthProvider
from robot_client import RobotError, TianlianRobot

POSE_PORT = 7100


class _Abort(Exception):
    """软停(段边界)。"""


class _PlanFail(RobotError):
    """规划(逆解)失败 —— 唯一允许降级重试(越顶→层内)的失败类型。"""


# ===========================================================================
# 旋转 / 几何(约定与 ik_helper._rpy_to_R / _quat_to_R 完全一致)
# ===========================================================================
def quat_to_R(x, y, z, w) -> np.ndarray:
    q = np.array([x, y, z, w], dtype=float)
    q = q / np.linalg.norm(q)
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rpy_deg_to_R(roll_deg, pitch_deg, yaw_deg) -> np.ndarray:
    r, p, y = (math.radians(v) for v in (roll_deg, pitch_deg, yaw_deg))
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def R_to_rpy_deg(R) -> list:
    """ZYX 欧拉角提取(rpy_deg_to_R 的逆),输出度。"""
    sy = max(-1.0, min(1.0, -R[2, 0]))
    pitch = math.asin(sy)
    if abs(sy) > 0.99999:                            # 万向锁:roll 取 0
        roll = 0.0
        yaw = math.atan2(-R[0, 1], R[1, 1] * (1 if sy > 0 else -1))
    else:
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
    return [math.degrees(a) for a in (roll, pitch, yaw)]


def T_from(pos, quat) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = quat_to_R(*quat)
    T[:3, 3] = [float(v) for v in pos]
    return T


def quat_to_rpy_deg(quat) -> list:
    return R_to_rpy_deg(quat_to_R(*quat))


# ===========================================================================
# 检测 → 基座系点位
# ===========================================================================
def bag_points(detections, T_base_ee, T_ee_cam, session,
               ghost_min_z_mm=200.0):
    """detections(:7100 原始) → 每袋 {p_cam 隐含, p_base, z_mm, ...}。

    取点: 框内最近深度面的 **3D 点云质心**(box_face_points)优先 ——
    框质心像素横移是场景相关误差的主项(两次校正残差 X 分量差 8.2mm,
    Y/Z 稳定),整面点云质心直接给"真实面中心"。失败退质心圆中值
    (pixel_to_3d, r=14);再退检测框中值(box_depth_3d);再失败/
    近距鬼影(z<ghost_min_z)拒收。
    """
    rows, rejected = [], []
    for i, det in enumerate(detections, 1):
        cen = det.get("centroid")
        p_cam = None
        face = session.box_face_points(det.get("box") or [])
        row = {"index": i, "cls": det.get("cls"), "conf": det.get("conf"),
               "centroid": cen,
               "action": det.get("action")}
        if face is not None:
            p_cam = face["p_cam"]
            row["face_frac"] = face["frac"]
            row["aim"] = "face3d"
        elif cen:
            p_cam = session.pixel_to_3d(cen[0], cen[1], radius=14)
            if p_cam is None:
                p_cam = session.box_depth_3d(det.get("box") or [])
        if p_cam is None:
            row["reason"] = "深度无效"
            rejected.append(row)
            continue
        z_mm = float(p_cam[2]) * 1000.0
        if z_mm < ghost_min_z_mm:
            row["reason"] = f"鬼影z={z_mm:.0f}mm<{ghost_min_z_mm:.0f}"
            rejected.append(row)
            continue
        p_base = (T_base_ee @ T_ee_cam @
                  np.array([p_cam[0], p_cam[1], p_cam[2], 1.0]))[:3]
        row["z_mm"] = round(z_mm, 1)
        row["p_base"] = [round(float(v), 5) for v in p_base]
        rows.append(row)
    return rows, rejected


def order_bags(rows, flange_xy, layer_mm=25.0):
    """编号规则(2026-10-20 多袋实测修正):按**基座系高度 p_base[2]**聚类
    分层(相邻差 >layer_mm 开新层),最高层先吸(自上而下);层内按
    e_left = ẑ×d̂(d̂ = 法兰→袋群均值 水平方向)投影从左到右。
    ⚠ 旧版沿袭⑭用相机深度 z_mm 分层——那是俯视桌面堆叠的假设(离相机近
    =堆顶);腕相机正视货架时各层深度几乎相同(实测 348/349/349mm 全进
    L0),层内 xy 投影差被毫米级噪声淹没 → 编号每次扫描随机洗牌,实测
    "吸1去了3、吸3去了1"。层号 0 = 最高层。"""
    if not rows:
        return []
    hs = sorted((float(r["p_base"][2]) for r in rows), reverse=True)
    bounds = []
    for a, b in zip(hs, hs[1:]):
        if a - b > layer_mm / 1000.0:
            bounds.append((a + b) / 2.0)

    def layer_of(h):
        return sum(1 for m in bounds if h < m)

    mean_xy = np.mean([[r["p_base"][0], r["p_base"][1]] for r in rows],
                      axis=0)
    d = (mean_xy - np.asarray(flange_xy, float))[:2]
    n = np.linalg.norm(d)
    e_left = (np.array([-d[1], d[0]]) / n) if n > 1e-6 else np.array([0.0, 1.0])
    for r in rows:
        r["layer"] = layer_of(float(r["p_base"][2]))
        xy = np.array([r["p_base"][0], r["p_base"][1]])
        r["_proj"] = float(xy @ e_left)
    out = sorted(rows, key=lambda r: (r["layer"], -r["_proj"]))
    for k, r in enumerate(out, 1):
        r["order"] = k
    return out


def detect_via_pose_service(color_bgr, conf=0.5, timeout=30.0) -> dict:
    """color 帧 → JPEG → POST :7100/detect → {detections, image_b64,...}。"""
    ok, buf = cv2.imencode(".jpg", color_bgr,
                           [cv2.IMWRITE_JPEG_QUALITY, 90])
    if not ok:
        raise DepthError("JPEG 编码失败")
    url = f"http://127.0.0.1:{POSE_PORT}/detect?conf={conf}"
    req = urllib.request.Request(url, data=buf.tobytes(),
                                 headers={"Content-Type":
                                          "application/octet-stream"},
                                 method="POST")
    with urllib.request.urlopen(req, timeout=timeout) as r:
        payload = json.loads(r.read())
    if not payload.get("ok"):
        raise DepthError(f"检测失败: {payload.get('error')}")
    return payload


# ===========================================================================
# 扫描(供 /api/pick/scan 同步调用,也被 PickJob 复用)
# ===========================================================================
def scan_once(robot: TianlianRobot, provider: DepthProvider, cfg: dict,
              *, replay: bool = True, say=None) -> dict:
    """回放扫描位(可选)→ 取帧 → 检测 → 深度 → 基座系点位 → 分层排序。

    返回 {rows, rejected, image_b64, synced, end_pose};rows 已写入
    order/layer 并存入 last_scan。say 可选日志回调。
    """
    say = say or (lambda m: None)
    scan_pose = cfg.get("scan_pose")
    if replay:
        if not scan_pose:
            raise ValueError("未示教扫描位 —— 先『存当前为扫描位』")
        robot.move_robotarm_traj(
            self_arm(cfg), [scan_pose["joints_deg"]], 1,
            cfg["move_timeout_s"], speed_store.get_speed(self_arm(cfg)))
        time.sleep(cfg["settle_s"])
    role = cfg.get("cam_role", "left")
    provider.start_session(role)
    session = provider.session
    pair = session.grab_pair(timeout_s=10.0)
    say(f"取帧 {pair['color'].shape[1]}x{pair['color'].shape[0]} "
        f"synced={pair['synced']}")
    poses = robot.get_end_poses().get(self_arm(cfg)) or {}
    pos, quat = poses.get("position"), poses.get("orientation")
    if not pos or not quat:
        raise RobotError("末端位姿数据不完整(get_end_poses)")
    ep = {"position": [float(v) for v in pos],
          "orientation": [float(v) for v in quat]}
    T_base_ee = T_from(ep["position"], ep["orientation"])
    T_ee_cam = pick_store.load_handeye_T()
    payload = detect_via_pose_service(pair["color"], conf=cfg.get("conf", 0.5))
    dets = payload.get("detections") or []
    say(f"检测 {len(dets)} 个目标({payload.get('latency_ms')}ms)")
    rows, rejected = bag_points(dets, T_base_ee, T_ee_cam, session,
                                ghost_min_z_mm=cfg["ghost_min_z_mm"])
    for rj in rejected:
        say(f"拒收 #{rj['index']}({rj['cls']}): {rj['reason']}")
    if not rows:
        raise DepthError("无可吸目标(检测为空或深度全部无效)")
    rows = order_bags(rows, ep["position"][:2], layer_mm=cfg["layer_mm"])
    # 用最终编号(#order L层)重画标注图::7100 原图上的序号是检测器枚举
    # 顺序,与高度分层编号是两套号(实测 2 袋时图上 #1 画在低袋、表格 #1
    # 是高袋,按图点表就"吸错袋")。图/表/任务必须同一套号。
    img = pair["color"].copy()
    for r in rows:
        box = (dets[r["index"] - 1].get("box") or [0, 0, 0, 0])
        x1, y1 = int(box[0]), int(box[1])
        x2, y2 = int(box[2]), int(box[3])
        label = f"#{r['order']} L{r['layer']}"
        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
        ty = max(y1 - 6, 12)
        cv2.putText(img, label, (x1, ty), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 0, 0), 3)
        cv2.putText(img, label, (x1, ty), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 255, 0), 1)
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 85])
    image_b64 = base64.b64encode(buf).decode() if ok else None
    pick_store.save_last_scan({
        "rows": [{k: v for k, v in r.items() if not k.startswith("_")}
                 for r in rows],
        "rejected": rejected,
        "T_base_ee": ep,
        "cam_role": role,
        "conf": cfg.get("conf", 0.5),
        "image_b64": image_b64,
    })
    return {"rows": [{k: v for k, v in r.items() if not k.startswith("_")}
                     for r in rows],
            "rejected": rejected,
            "image_b64": image_b64,
            "synced": pair["synced"], "end_pose": ep}


def self_arm(cfg) -> str:
    return cfg.get("arm", "armleft")


# ===========================================================================
# 吸取任务
# ===========================================================================
class PickJob:
    """一次 一步到位 任务。app.py 单例持有,threading.Thread 跑 run()。"""

    def __init__(self, robot: TianlianRobot, provider: DepthProvider,
                 cfg: dict, *, bag_index=None, dry_run=False,
                 use_last_scan=False, press_mm=None, hold_s=None):
        self.robot = robot
        self.provider = provider
        self.cfg = cfg
        self.arm = cfg.get("arm", "armleft")
        self.bag_index = bag_index
        self.dry_run = bool(dry_run)
        self.use_last_scan = bool(use_last_scan)
        self.press_mm = press_mm
        self.hold_s = hold_s
        self.abort_event = threading.Event()
        self.phase = "init"
        self.pct = 0
        self.log: deque = deque(maxlen=300)
        self.error: str | None = None
        self.result: dict | None = None
        self.started_at = time.time()
        self._done = False
        self._T_ee_cam = pick_store.load_handeye_T()

    # ------------------------------------------------------------- 基础
    def _say(self, msg: str):
        self.log.append(f"[{time.strftime('%H:%M:%S')}] {msg}")

    def _check_abort(self):
        if self.abort_event.is_set():
            raise _Abort()

    def _set_phase(self, phase: str, pct: int):
        self.phase = phase
        self.pct = pct
        self._say(f"—— {phase} ——")

    def _sleep(self, s: float):
        t0 = time.time()
        while time.time() - t0 < s:
            self._check_abort()
            time.sleep(0.1)

    def _end_pose(self) -> dict:
        p = (self.robot.get_end_poses().get(self.arm)) or {}
        pos, quat = p.get("position"), p.get("orientation")
        if not pos or not quat:
            raise RobotError("末端位姿数据不完整(get_end_poses)")
        return {"position": [float(v) for v in pos],
                "orientation": [float(v) for v in quat]}

    def _q_deg(self) -> list:
        q = (self.robot.get_joint_angles().get(self.arm)) or []
        if not q:
            raise RobotError("关节角读取失败")
        return [float(v) for v in q]

    # ------------------------------------------------------------- 主流程
    def run(self):
        try:
            self._run_inner()
        except _Abort:
            self.phase = "aborted"
            self._say("⛔ 已在段边界停止(软停),臂停在当前位")
        except (RobotError, DepthError, ValueError, FileNotFoundError) as e:
            self.phase = "failed"
            self.error = str(e)
            self._say(f"✗ 失败: {e}(臂停在当前位,人工接管)")
        except Exception as e:                        # noqa: BLE001
            self.phase = "failed"
            self.error = f"{type(e).__name__}: {e}"
            self._say(f"✗ 异常: {self.error}")
        finally:
            self._done = True

    def _run_inner(self):
        cfg = self.cfg
        press_mm = cfg["press_mm"] if self.press_mm is None else \
            max(0.0, min(float(self.press_mm), cfg["max_press_mm"]))
        hold_s = cfg["hold_s"] if self.hold_s is None else float(self.hold_s)
        hover = cfg["hover_mm"] / 1000.0
        res = np.array(cfg.get("residual_mm", [0, 0, 0]), float) / 1000.0

        # ---- ① preflight ----
        self._set_phase("preflight", 5)
        scan_pose = cfg.get("scan_pose")
        rows = None
        if self.use_last_scan:
            last = pick_store.load_last_scan()
            if last and last.get("rows"):
                rows = last["rows"]
                self._say(f"使用上次扫描({len(rows)} 袋, "
                          f"{last.get('saved_at')})")
            else:
                self._say("无可用 last_scan,改走现场扫描")
        if rows is None and not scan_pose:
            raise ValueError("未示教扫描位 —— 请先手动移到货架全景视角,"
                             "『存当前为扫描位』后重试")
        align = cfg.get("align")
        if not self.dry_run and not align:
            self._say("⚠ 未做对准校正: 首跑悬停可能偏,到位后请人工微调"
                      "并『校正吸取位』")
        self._check_abort()

        # ---- ② 回放扫描位(干跑不动臂) ----
        if rows is None and not self.dry_run:
            self._set_phase("replay", 15)
            self._say(f"回放扫描位({len(scan_pose['joints_deg'])} 关节)")
            self.robot.move_robotarm_traj(
                self.arm, [scan_pose["joints_deg"]], 1,
                cfg["move_timeout_s"], speed_store.get_speed(self.arm))
            self._sleep(cfg["settle_s"])

        # ---- ③ 扫描 ----
        if rows is None:
            self._set_phase("scan", 30)
            out = scan_once(self.robot, self.provider, cfg,
                            replay=False, say=self._say)
            rows = out["rows"]
        for r in rows[:12]:
            self._say(f"#{r['order']} L{r['layer']} {r['cls']} "
                      f"conf={r['conf']:.2f} z={r['z_mm']:.0f}mm "
                      f"p=[{r['p_base'][0]:.3f},{r['p_base'][1]:.3f},"
                      f"{r['p_base'][2]:.3f}]m")

        # ---- ④ 选袋 + 残差 ----
        self._set_phase("select", 40)
        row = next((r for r in rows if r.get("order") == self.bag_index),
                   None) if self.bag_index else rows[0]
        if row is None:
            raise ValueError(f"找不到 #{self.bag_index}(共 {len(rows)} 袋)")
        per = (cfg.get("per_bag_residual_mm") or {}).get(str(row["order"]))
        res_all = res + (np.array(per, float) / 1000.0 if per
                         else np.zeros(3))
        aim_xy = np.array(row["p_base"][:2]) + res_all[:2]
        aim_z = row["p_base"][2] + hover + res_all[2]
        self._say(f"目标 #{row['order']}(层{row['layer']}) "
                  f"aim=[{aim_xy[0]:.3f},{aim_xy[1]:.3f},{aim_z:.3f}]m "
                  f"残差={np.round(res_all * 1000, 1).tolist()}mm")

        # ---- ⑤ 护栏 ----
        self._set_phase("guards", 45)
        cur = self._end_pose()
        horiz = float(np.linalg.norm(aim_xy - np.array(cur["position"][:2])))
        if horiz > cfg["max_horiz_m"]:
            raise ValueError(f"水平行程 {horiz * 1000:.0f}mm 超上限 "
                             f"{cfg['max_horiz_m'] * 1000:.0f}mm")
        if align:
            dz = abs(aim_z - float(align["z_ref_m"]))
            if dz > cfg["max_dz_m"]:
                raise ValueError(f"深度基准差 {dz * 1000:.0f}mm 超上限 "
                                 f"{cfg['max_dz_m'] * 1000:.0f}mm"
                                 f"(台面高度变了?重新校正)")
        top_z = max(r["p_base"][2] for r in rows)
        cruise_z = max(top_z + cfg["cruise_clearance_mm"] / 1000.0,
                       cur["position"][2])
        self._say(f"越顶走廊 z={cruise_z:.3f}m(最高面 {top_z:.3f}m+"
                  f"{cfg['cruise_clearance_mm']:.0f}mm)|水平行程 "
                  f"{horiz * 1000:.0f}mm|{'干跑' if self.dry_run else '实跑'}")

        # ---- ⑥ 分段运动(升 → 转 → 平移 → 降) ----
        align_rpy = list(align["rpy_deg"]) if align else None

        def plan_seg(pos, rpy):
            """规划一段(=dry-IK)。失败抛 RobotError,臂不动。"""
            cur2 = self._end_pose()
            out = plan_cartesian_path(
                self.arm, self._q_deg(), op="target",
                target_pos_m=[float(v) for v in pos],
                target_rpy_deg=[float(v) for v in rpy],
                cur_pos_m=cur2["position"], cur_quat=cur2["orientation"])
            traj = out.get("waypoints_deg") or []
            if not traj:
                raise RobotError("逆解未返回轨迹")
            return traj

        def run_seg(name, pos, rpy, pct, retries=1):
            """执行一段。fd_arm_server 大行程会"流发过快→停在半途但报成功"
            (stream_dt 0.004 灌 0.01 时间轴 = 指令速度 ×2.5,到位判据仅 2°/关节),
            故验证失败先等 0.6s 复读(排除收敛中),再重规划补走一次。"""
            self._check_abort()
            self._set_phase(name, pct)
            try:
                traj = plan_seg(pos, rpy)
            except RobotError as e:
                raise _PlanFail(str(e)) from e
            self._say(f"{name}: →[{pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f}]m "
                      f"rpy={np.round(rpy, 1).tolist()}({len(traj)} 路点)")
            if self.dry_run:
                return
            last_dxy = last_dz = 0.0
            for attempt in range(retries + 1):
                self._check_abort()
                if attempt:
                    self._say(f"{name}: 未到位,从当前位重规划补走"
                              f"(第 {attempt} 次重试)")
                    try:
                        traj = plan_seg(pos, rpy)
                    except RobotError as e:
                        raise _PlanFail(str(e)) from e
                self.robot.move_robotarm_traj(
                    self.arm, traj, 1,
                    cfg["move_timeout_s"], speed_store.get_speed(self.arm))
                now = self._end_pose()["position"]
                dxy = math.hypot(now[0] - pos[0], now[1] - pos[1])
                dz = abs(now[2] - pos[2])
                if (dxy * 1000 > cfg["xy_res_tol_mm"] or
                        dz * 1000 > cfg["z_res_tol_mm"]) and \
                        attempt < retries:
                    time.sleep(0.6)              # 可能仍在收敛,复读一次
                    now = self._end_pose()["position"]
                    dxy = math.hypot(now[0] - pos[0], now[1] - pos[1])
                    dz = abs(now[2] - pos[2])
                last_dxy, last_dz = dxy, dz
                if dxy * 1000 <= cfg["xy_res_tol_mm"] and \
                        dz * 1000 <= cfg["z_res_tol_mm"]:
                    break
            self._say(f"{name}: 到位 Δxy={last_dxy * 1000:.1f}mm "
                      f"Δz={last_dz * 1000:.1f}mm"
                      + ("" if attempt == 0 else f"(重试 {attempt} 次后)"))
            if last_dxy * 1000 > cfg["xy_res_tol_mm"] or \
                    last_dz * 1000 > cfg["z_res_tol_mm"]:
                raise RobotError(
                    f"{name} 段后验证超差(Δxy={last_dxy * 1000:.1f}mm,"
                    f"Δz={last_dz * 1000:.1f}mm,重试后仍不收敛)—— "
                    f"原地停,请人工核对")

        cur_rpy = quat_to_rpy_deg(cur["orientation"])
        tgt_rpy = align_rpy or cur_rpy

        def _corridor():
            """分段走廊:升→转→水平(越顶/层内)→降。"""
            run_seg("lift", [cur["position"][0], cur["position"][1],
                             cruise_z], cur_rpy, 50)
            if align_rpy and max(abs(a - b) for a, b in
                                 zip(align_rpy, cur_rpy)) > 2.0:
                pos_now = self._end_pose()["position"]
                run_seg("rotate", pos_now, align_rpy, 58)
            try:
                run_seg("horizontal", [aim_xy[0], aim_xy[1], cruise_z],
                        tgt_rpy, 66)
            except _PlanFail as e:
                layer_z = row["p_base"][2] + hover
                self._say(f"越顶平移规划不可达({e});退层内走廊 "
                          f"z={layer_z:.3f}m")
                run_seg("horizontal", [aim_xy[0], aim_xy[1], layer_z],
                        tgt_rpy, 70)
            run_seg("descend", [aim_xy[0], aim_xy[1], aim_z], tgt_rpy, 80)

        if cfg.get("motion_mode") == "direct":
            # 一段直达:扫描位→袋心 aim 单段直线,姿态同步转到示教对准
            # 姿态。注意直线贴两端点高度横穿(≈袋顶),途中他袋/货架
            # 避障由扫描位摆放 + max_horiz 行程护栏保证。
            dist = math.hypot(math.hypot(aim_xy[0] - cur["position"][0],
                                         aim_xy[1] - cur["position"][1]),
                              aim_z - cur["position"][2])
            self._say(f"direct: 扫描位→aim 一段直线 {dist * 1000:.0f}mm"
                      "(贴袋顶高度横穿,注意途中避障)")
            try:
                run_seg("direct", [aim_xy[0], aim_xy[1], aim_z], tgt_rpy, 80)
            except _PlanFail as e:
                self._say(f"直达规划不可达({e});退回分段走廊")
                _corridor()
        else:
            _corridor()

        # ---- ⑦ 下压(可选) ----
        # 下压沿工具系 Z+(吸嘴轴向;2026-10-20 用户实测正确方向。吸嘴沿法兰
        # Z+ 伸出约 92mm——对准残差 y=+92mm 即其长度,示教姿态下 Z+ ≈ 基座
        # -Y 水平指向袋面,descend 末吸口已贴袋面,下压=沿 Z+ 推入袋内)。
        if press_mm > 0:
            prpy = align_rpy or cur_rpy
            dvec = rpy_deg_to_R(*prpy) @ np.array(
                [0.0, 0.0, press_mm / 1000.0])
            if self.dry_run:
                self._say(f"press(干跑): 沿工具Z+ {press_mm:.1f}mm → 基座Δ"
                          f"[{dvec[0]*1000:.1f},{dvec[1]*1000:.1f},"
                          f"{dvec[2]*1000:.1f}]mm")
            else:
                run_seg("press", [aim_xy[0] + dvec[0], aim_xy[1] + dvec[1],
                                  aim_z + dvec[2]], prpy, 90)

        # ---- ⑧ 保持(外部接真空窗口) ----
        if not self.dry_run:
            self._set_phase("hold", 92)
            vac = cfg.get("vacuum") or {}
            if vac.get("mode") == "gripper":
                side = "left" if self.arm == "armleft" else "right"
                try:
                    self.robot.set_gripper(
                        side, float(vac.get("gripper_position", 0)),
                        int(vac.get("gripper_velocity", 500)))
                    self._say("已联动夹爪")
                except RobotError as e:
                    self._say(f"夹爪联动失败(继续): {e}")
            self._say(f"到位保持 {hold_s}s —— 需要真空请现在接")
            self._sleep(hold_s)

        # ---- ⑨ 完成 ----
        self._set_phase("done", 100)
        final = self._end_pose()["position"]
        self.result = {
            "bag": {k: row[k] for k in ("order", "layer", "cls", "conf",
                                        "z_mm", "p_base") if k in row},
            "aim_m": [round(float(v), 4) for v in
                      (aim_xy[0], aim_xy[1], aim_z)],
            "final_pos_m": [round(float(v), 4) for v in final],
            "dry_run": self.dry_run, "press_mm": press_mm,
            "duration_s": round(time.time() - self.started_at, 1),
        }
        self._say(f"✓ 完成({'干跑' if self.dry_run else '到位'},目标 "
                  f"#{row['order']},总耗时 {self.result['duration_s']}s)")

    # ------------------------------------------------------------- 状态
    def status(self) -> dict:
        return {
            "running": not self._done,
            "phase": self.phase, "pct": self.pct,
            "log": list(self.log)[-60:],
            "error": self.error, "result": self.result,
        }


# ===========================================================================
# 对准校正(app.py 的 teach/align 端点调用)
# ===========================================================================
def compute_align_residual(cfg: dict, bag_order: int,
                           aligned_pos: list) -> dict:
    """人工微调到正确吸附点后,算全局残差与对准姿态基准。

    aligned_pos: 微调后的法兰位姿 position(上报系,米)。
    残差 = 对准点 − (袋 p_base + [0,0,hover]),同时吸收吸嘴-法兰 z 差、
    手眼 t 误差、镜头漂移等一切恒定项(扫描位固定时成立)。
    """
    last = pick_store.load_last_scan()
    if not last or not last.get("rows"):
        raise ValueError("没有最近扫描结果 —— 先扫描一次再校正")
    row = next((r for r in last["rows"]
                if r.get("order") == bag_order), None)
    if row is None:
        raise ValueError(f"上次扫描里没有 #{bag_order}")
    hover = cfg["hover_mm"] / 1000.0
    bag = np.array(row["p_base"], float)
    aligned = np.array(aligned_pos, float)
    res = aligned - (bag + np.array([0.0, 0.0, hover]))
    res_mm = [round(float(v) * 1000, 2) for v in res]
    return {"residual_mm": res_mm,
            "z_ref_m": round(float(aligned[2]), 4),
            "bag_order": bag_order,
            "note": f"袋#{bag_order} 袋心={np.round(bag, 3).tolist()}m "
                    f"对准法兰={np.round(aligned, 3).tolist()}m"}
