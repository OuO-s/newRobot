#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""末端笛卡尔位姿逆解辅助进程。

必须用系统 Python 3.10 运行(GUI 的 conda 3.11 无法导入 tl_kinematics
的 C 扩展):

    echo '{"op":"rotate",...}' | /usr/bin/python3 ik_helper.py          # 单次
    /usr/bin/python3 ik_helper.py --server                              # 常驻

从 stdin 读 JSON 请求,stdout 输出 JSON 结果(单次模式整段读入,server
模式按行一问一答)。职责:根据当前关节角构造目标位姿(相对平移/相对
旋转/绝对位姿),用 tl_kinematics 的直线+姿态插值规划关节轨迹,由上位
机经 /move_robotarm 下发执行。

server 模式(2026-11-05):URDF+运动学链按臂缓存只加载一次——单次模式
每请求要 ~350ms(URDF 解析占大头),笛卡尔点动 150ms 一拍喂不起;
常驻后纯求解仅需毫秒级。

请求 op:
  translate {"arm","q_deg","deltas_m":[dx,dy,dz],"frame":"base|tool"}
  rotate    {"arm","q_deg","axis":"rx|ry|rz","angle_deg","frame":"base|tool"}
  target    {"arm","q_deg","target_pos_m":[x,y,z],"target_rpy_deg":[r,p,y],
             "cur_pos_m":[x,y,z],"cur_quat":[x,y,z,w]}   # 当前上报位姿,用于基座偏移换算
"""

import glob
import json
import math
import os
import sys

try:
    import numpy as np
    from tl_kinematics import RobotKinematics
    _DEPS_OK = True
    _DEPS_ERR = ""
except Exception as _exc:  # numpy / tl_kinematics 缺失
    _DEPS_OK = False
    _DEPS_ERR = str(_exc)

FD_WS = os.path.expanduser(os.environ.get("FD_WS", "~/fd_ws"))
URDF_DIRS = (
    os.path.join(FD_WS, "install", "tl_robot_description", "share",
                 "tl_robot_description", "urdf"),
    os.path.join(FD_WS, "src", "standard", "fd_arm", "tcb_ros2_ws", "src",
                 "tl_robot_description", "urdf"),
)
NEAR_ZERO_TRAJ_DEG = 0.5     # 与 fd_arm_http 阈值一致
JUMP_GUARD_MAX_DQ_DEG = 45.0  # 单段直线 IK 关节变化护栏(首尾+相邻路点)
MAX_TOTAL_DQ_DEG = 135.0     # 自适应分段允许的整段最大重构(90° 合法吸取
                             # 工位实测需 ~90°;258° 真翻转仍拦)
MAX_SUBDIV_SEGMENTS = 8      # 分段链式规划的最大段数
REACH_POS_TOL_M = 5e-3       # 末端到达容差(与 fd_arm_api 近成功容差一致)
REACH_ROT_TOL_RAD = 5e-2
JOINT_LIMIT_EPS_DEG = 1e-2
NUM_POINTS = 6               # fd_arm_api straightline_num_points 默认值
# 直线 IK 种子敏感失败的错误前缀(2026-11-18 实测: 流水线3 右臂 B3 点1
# 平移 40mm,首解发散,同目标换邻近种子即解,半径 ~380mm 远未到臂展边界):
# 这些失败换路点密度/换种子常可解出,值得重试;关节限位/跳解护栏类
# 失败不在此列(后者有自适应分段专用通道)。
SOLVER_RETRY_PREFIXES = (
    "直线规划失败", "规划结果为空", "规划结果含非有限值", "目标位姿不可达",
)


def _load_yaml(path):
    try:
        import yaml
        with open(path, "r", encoding="utf-8") as f:
            return yaml.safe_load(f) or {}
    except Exception:
        return {}


def _resolve_urdf(startup, profile, model):
    """按 fd_arm 启动配置解析 URDF 路径。

    环境变量 IK_URDF_MODEL 优先(不带 _description 后缀的型号名):
    本机 startup.yaml 的 urdf_model=y2_p_ds_705vg 与真机(Y2-P-DT)不符——
    :8000 上报系与 robot_profile.yaml 实际都按 y2_p_dt_710vg 计算(2026-10-20
    双 URDF 对拍: DT 偏差 0.0mm, DS 跨构型漂移 16mm/大构型差 100mm),
    DS 规划会让大行程笛卡尔目标整体跑偏。webapp 启动时置
    IK_URDF_MODEL=y2_p_dt_710vg 钉死正确模型。
    """
    urdf_model = (os.environ.get("IK_URDF_MODEL") or "").strip() \
        or (startup.get("urdf_model") or "").strip()
    for d in URDF_DIRS:
        if not os.path.isdir(d):
            continue
        if urdf_model:
            exact = os.path.join(d, urdf_model + "_description.urdf")
            if os.path.isfile(exact):
                return exact
            hits = sorted(glob.glob(os.path.join(d, f"*{urdf_model}*.urdf")))
            if hits:
                return hits[0]
        rel = ((profile.get("models") or {}).get(model) or {}).get("urdf_path")
        if rel:
            p = os.path.normpath(os.path.join(os.path.dirname(d), rel))
            if os.path.isfile(p):
                return p
    return None


def _resolve_frames(profile, arm, dof):
    """解析运动学链 base/end frame;解析失败按双臂命名约定推导。"""
    side = "left" if arm.endswith("left") else "right"
    frames = (((profile.get("models") or {}).get(
        profile.get("_model") or "") or {}).get("frames") or {})
    entry = ((frames.get("dual") or {}).get(side)) or (
        (frames.get("single") or {}))
    if entry.get("base_frame") and entry.get("end_frame"):
        return entry["base_frame"], entry["end_frame"]
    prefix = "" if profile.get("_arm_mode") == "single" else f"{side}_"
    return f"{prefix}tl_robot_link0", f"{prefix}tl_robot_link{dof}"


def _skew_vec(R):
    """旋转矩阵 → 轴角向量(R_err 的旋转向量)。"""
    return np.array([R[2, 1], R[0, 2], R[1, 0]])


def _rot_about(axis_unit, theta):
    """罗德里格斯公式:绕单位轴旋转 theta(弧度)。"""
    K = np.array([[0.0, -axis_unit[2], axis_unit[1]],
                  [axis_unit[2], 0.0, -axis_unit[0]],
                  [-axis_unit[1], axis_unit[0], 0.0]])
    return np.eye(3) + math.sin(theta) * K + (1.0 - math.cos(theta)) * (K @ K)


def _rpy_to_R(roll_deg, pitch_deg, yaw_deg):
    """ZYX 欧拉角(度) → 旋转矩阵,与 app.quat_to_rpy_deg 约定一致。"""
    r, p, y = (math.radians(v) for v in (roll_deg, pitch_deg, yaw_deg))
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def _quat_to_R(x, y, z, w):
    q = np.array([x, y, z, w], dtype=float)
    q = q / np.linalg.norm(q)
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _R_to_quat(R):
    """旋转矩阵 → [x,y,z,w](与 op=fk 分支同款)。"""
    tr = float(R[0, 0] + R[1, 1] + R[2, 2])
    if tr > 0.0:
        w = math.sqrt(tr + 1.0) / 2.0
        return np.array([(R[2, 1] - R[1, 2]) / (4 * w),
                         (R[0, 2] - R[2, 0]) / (4 * w),
                         (R[1, 0] - R[0, 1]) / (4 * w), w])
    i = max(range(3), key=lambda k: R[k, k])
    j, k = (i + 1) % 3, (i + 2) % 3
    t = math.sqrt(max(0.0, 1.0 + R[i, i] - R[j, j] - R[k, k]))
    q = np.zeros(4)
    q[i] = t / 2.0
    q[j] = (R[j, i] + R[i, j]) / (2 * t)
    q[k] = (R[k, i] + R[i, k]) / (2 * t)
    q[3] = (R[k, j] - R[j, k]) / (2 * t)
    return q


def _T_interp(T0, T1, s):
    """SE(3) 插值(位置线性 + 姿态四元数 slerp)—— 自适应分段的子目标。"""
    T = np.eye(4)
    T[:3, 3] = T0[:3, 3] + s * (T1[:3, 3] - T0[:3, 3])
    q0, q1 = _R_to_quat(T0[:3, :3]), _R_to_quat(T1[:3, :3])
    d = float(np.dot(q0, q1))
    if d < 0.0:
        q1, d = -q1, -d
    if d > 0.9995:
        q = q0 + s * (q1 - q0)
        q /= np.linalg.norm(q)
    else:
        th = math.acos(min(1.0, d))
        q = (math.sin((1 - s) * th) * q0 + math.sin(s * th) * q1) / math.sin(th)
    T[:3, :3] = _quat_to_R(float(q[0]), float(q[1]), float(q[2]), float(q[3]))
    return T


def _inv_T(T):
    """齐次刚体变换求逆。"""
    out = np.eye(4)
    out[:3, :3] = T[:3, :3].T
    out[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return out


def _max_dq_deg(traj):
    if not traj or len(traj) < 2:
        return 0.0
    n = min(len(traj[0]), len(traj[-1]))
    return max(abs(float(traj[-1][i]) - float(traj[0][i])) for i in range(n))


def _max_step_dq_deg(traj):
    """相邻路点最大关节变化(轨迹中途翻转到别的解分支再翻回时,
    首尾差近 0,只有相邻差能暴露)。"""
    m = 0.0
    for a, b in zip(traj, traj[1:]):
        n = min(len(a), len(b))
        m = max(m, max(abs(float(b[i]) - float(a[i])) for i in range(n)))
    return m


_KIN_CACHE: dict = {}     # (arm, dof) → RobotKinematics(server 模式复用,URDF 只解析一次)


def _get_kin(arm, q_deg):
    """取(或建)该臂的运动学链;返回 (kin, urdf, base_frame, end_frame)。
    加载失败抛 RuntimeError。C 扩展对象不能挂自定义属性,元数据随元组返回。"""
    key = (arm, len(q_deg))
    hit = _KIN_CACHE.get(key)
    if hit is not None:
        return hit
    startup = _load_yaml(os.path.join(FD_WS, "data", "param", "startup.yaml"))
    profile = _load_yaml(os.path.join(FD_WS, "data", "param",
                                      "robot_profile.yaml"))
    model = (startup.get("model") or "").strip()
    profile["_model"] = model
    profile["_arm_mode"] = (profile.get("models") or {}).get(
        model, {}).get("arm_mode") or (profile.get("arm_mode") or "dual")
    urdf = _resolve_urdf(startup, profile, model)
    if not urdf:
        raise RuntimeError(
            "未找到 URDF(检查 ~/fd_ws/data/param/startup.yaml "
            "的 urdf_model 与 tl_robot_description 安装)")
    base_frame, end_frame = _resolve_frames(profile, arm, len(q_deg))
    kin = RobotKinematics(urdf, base_frame, end_frame)
    hit = (kin, urdf, base_frame, end_frame)
    _KIN_CACHE[key] = hit
    return hit


def handle_request(req: dict) -> dict:
    """单请求处理:dict 进 dict 出(单次/server 两种模式共用)。"""
    def fail(msg, unavailable=False):
        return {"ok": False, "error": msg, "unavailable": unavailable}

    if not _DEPS_OK:
        return fail(f"系统 Python 缺少 numpy/tl_kinematics: {_DEPS_ERR}",
                    unavailable=True)

    arm = req.get("arm") or ""
    q_deg = [float(v) for v in (req.get("q_deg") or [])]
    op = req.get("op")
    if arm not in ("armleft", "armright") or not q_deg:
        return fail("缺少 arm / q_deg")
    if op not in ("translate", "rotate", "target", "fk"):
        return fail(f"未知 op: {op}")

    try:
        kin, urdf, base_frame, end_frame = _get_kin(arm, q_deg)
    except RuntimeError as exc:
        return fail(str(exc), unavailable=True)
    except Exception as exc:
        return fail(f"运动学初始化失败: {exc}", unavailable=True)

    if kin.num_joints > len(q_deg):
        return fail(f"关节角数量 {len(q_deg)} 少于 URDF 自由度 "
                    f"{kin.num_joints}")
    q_deg = q_deg[:kin.num_joints]

    T_cur = np.asarray(kin.fkine(q_deg), dtype=float)

    if op == "fk":
        # 纯正解:关节角 → 末端位姿(fkine/URDF 系)。:8000 上报系与它的
        # 固定安装偏移(实测 ~80mm)由调用方标定后叠加 —— 风暴期免掉
        # get_end_pose 的 0.6~3s 服务往返。
        R = T_cur[:3, :3]
        tr = R[0, 0] + R[1, 1] + R[2, 2]
        if tr > 0.0:                      # 常规分支(w 最大)
            w = math.sqrt(tr + 1.0) / 2.0
            x = (R[2, 1] - R[1, 2]) / (4.0 * w)
            y = (R[0, 2] - R[2, 0]) / (4.0 * w)
            z = (R[1, 0] - R[0, 1]) / (4.0 * w)
        else:                             # 180° 旋转分支,按对角元取轴
            i = max(range(3), key=lambda k: R[k, k])
            j, k = (i + 1) % 3, (i + 2) % 3
            t = math.sqrt(max(0.0, 1.0 + R[i, i] - R[j, j] - R[k, k]))
            q4 = [0.0, 0.0, 0.0]
            q4[i] = t / 2.0
            q4[j] = (R[j, i] + R[i, j]) / (2.0 * t)
            q4[k] = (R[k, i] + R[i, k]) / (2.0 * t)
            x, y, z = q4
            w = (R[k, j] - R[j, k]) / (2.0 * t)
        return {"ok": True, "pos_m": [float(v) for v in T_cur[:3, 3]],
                "quat": [float(v) for v in (x, y, z, w)], "frame": "urdf"}

    T_target = T_cur.copy()
    if op == "translate":
        deltas = req.get("deltas_m")
        if not deltas or len(deltas) != 3:
            return fail("translate 模式需要 deltas_m=[dx,dy,dz]")
        frame = (req.get("frame") or "base").lower()
        if frame not in ("base", "tool"):
            return fail(f"未知坐标系: {frame}")
        d = np.array([float(v) for v in deltas], dtype=float)
        # base 系:沿基座轴平移;tool 系:沿末端当前姿态的轴平移
        T_target[:3, 3] += (np.eye(3) if frame == "base"
                            else T_cur[:3, :3]) @ d
    elif op == "rotate":
        axis = (req.get("axis") or "").lower()
        axis_map = {"rx": (1.0, 0.0, 0.0), "ry": (0.0, 1.0, 0.0),
                    "rz": (0.0, 0.0, 1.0)}
        if axis not in axis_map:
            return fail(f"未知旋转轴: {axis}")
        theta = math.radians(float(req.get("angle_deg") or 0.0))
        frame = (req.get("frame") or "base").lower()
        if frame not in ("base", "tool"):
            return fail(f"未知坐标系: {frame}")
        R_delta = _rot_about(np.array(axis_map[axis]), theta)
        # base 系:绕基座轴旋转;tool 系:绕末端当前轴旋转(右乘)。
        T_target[:3, :3] = (R_delta @ T_cur[:3, :3] if frame == "base"
                            else T_cur[:3, :3] @ R_delta)
    else:
        pos = req.get("target_pos_m")
        rpy = req.get("target_rpy_deg")
        if not pos or len(pos) != 3 or not rpy or len(rpy) != 3:
            return fail("target 模式需要 target_pos_m / target_rpy_deg")
        T_target[:3, 3] = [float(v) for v in pos]
        T_target[:3, :3] = _rpy_to_R(*(float(v) for v in rpy))
        # 界面显示/录入的是 :8000 上报位姿,其基座原点与 URDF 链存在固定
        # 安装偏移(实测 ~80mm)。用"当前上报位姿 vs 当前 fkine"的刚体
        # 变换把目标换算到 fkine 系,保证"取当前值→移动"零位移。
        cur_pos = req.get("cur_pos_m")
        if cur_pos and len(cur_pos) == 3 and all(
                v is not None for v in cur_pos):
            T_rep = np.eye(4)
            T_rep[:3, 3] = [float(v) for v in cur_pos]
            cur_quat = req.get("cur_quat")
            if cur_quat and len(cur_quat) == 4 and all(
                    v is not None for v in cur_quat):
                T_rep[:3, :3] = _quat_to_R(*(float(v) for v in cur_quat))
            T_offset = T_rep @ _inv_T(T_cur)   # 上报系 = T_offset @ fkine系
            T_target = _inv_T(T_offset) @ T_target

    # ---------- 直线规划 + 自适应分段(2026-11-09) ----------
    lower = list(kin.joint_lower or [])
    upper = list(kin.joint_upper or [])

    def _seg_plan(T_from, T_to, q_from, num_points=None):
        """一段直线:规划+全套校验。返回 (traj|None, err|None, max_dq)。
        err="NEAR_ZERO" = 起终位姿几乎一致(整段无需运动),由上层定夺。"""
        try:
            traj = kin.straightline(T_target=T_to, T_current=T_from,
                                    q_initial=q_from,
                                    num_points=num_points or NUM_POINTS)
        except Exception as exc:
            return None, f"直线规划失败: {exc}", 0.0
        traj = [[float(v) for v in pt] for pt in (traj or [])]
        if not traj or any(len(pt) != kin.num_joints for pt in traj):
            return None, "规划结果为空或维度不符", 0.0
        for pt in traj:
            if not all(math.isfinite(v) for v in pt):
                return None, "规划结果含非有限值(目标位姿可能不可达)", 0.0
        if len(lower) >= kin.num_joints and len(upper) >= kin.num_joints:
            for i, pt in enumerate(traj):
                for j in range(kin.num_joints):
                    if upper[j] > lower[j] and not (
                            lower[j] - JOINT_LIMIT_EPS_DEG <= pt[j]
                            <= upper[j] + JOINT_LIMIT_EPS_DEG):
                        return None, (f"轨迹点 {i} 关节 {j + 1}={pt[j]:.2f}° "
                                      f"超出限位 [{lower[j]:.1f}, "
                                      f"{upper[j]:.1f}]"), 0.0
        max_dq = _max_dq_deg(traj)
        if max_dq < NEAR_ZERO_TRAJ_DEG:
            return None, "NEAR_ZERO", max_dq
        # 跳解护栏(单段): 腕部奇异附近(如料仓观察位 J5≈-2°,2026-10-23)
        # 数值 IK 会收敛到等价解分支——首尾关节翻转 70~258° 而末端位姿与
        # 目标一致,FK 验证拦不住;跳解轨迹下发后大行程翻转走不完,以
        # arrival timeout 收场。正常单段(闸门修正/重锚)关节变化实测
        # ≤20°;首尾超 45° 但**相邻路点无翻转**的连续大重构(2026-11-09
        # 吸取工位:扫描位→袋面 J1 合法转 ~90°,解随深度单调连续)走
        # 下方自适应分段,每段仍受全部护栏约束。
        if max_dq > JUMP_GUARD_MAX_DQ_DEG:
            return None, (f"跳解护栏: 关节最大变化 {max_dq:.1f}° > "
                          f"{JUMP_GUARD_MAX_DQ_DEG:.0f}°(解分支翻转)"), \
                max_dq
        if _max_step_dq_deg(traj) > JUMP_GUARD_MAX_DQ_DEG:
            return None, (f"跳解护栏: 相邻路点关节最大变化 "
                          f"{_max_step_dq_deg(traj):.1f}° > "
                          f"{JUMP_GUARD_MAX_DQ_DEG:.0f}°(轨迹中途翻转)"), \
                max_dq
        T_end = np.asarray(kin.fkine(traj[-1]), dtype=float)
        pos_err = float(np.linalg.norm(T_end[:3, 3] - T_to[:3, 3]))
        rot_err = float(np.linalg.norm(
            _skew_vec(T_to[:3, :3] @ T_end[:3, :3].T)))
        if pos_err > REACH_POS_TOL_M or rot_err > REACH_ROT_TOL_RAD:
            return None, (f"目标位姿不可达(末端误差 pos={pos_err * 1000:.1f}"
                          f"mm rot={math.degrees(rot_err):.2f}°)"), max_dq
        return traj, None, max_dq

    traj, err, max_dq = _seg_plan(T_cur, T_target, q_deg)
    if err == "NEAR_ZERO":
        return fail(f"目标与当前位姿几乎一致(最大关节变化 {max_dq:.3f}°),"
                    "无需运动")
    subdiv, note = 0, ""
    # 种子敏感重试(2026-11-18): 首解失败且属求解器类错误时,先加密路点
    # (步进 IK 步长更短)再小扰动种子(逃离局部极小)重试。每轮结果仍过
    # 全部护栏(限位/跳解/相邻步长/FK 到达)——扰动只换求解起点,不放松
    # 任何校验,解出跳分支会被护栏拒绝。
    if err is not None and err.startswith(SOLVER_RETRY_PREFIXES):
        first_err = err
        rng = np.random.default_rng(20261118)
        ladder = [(None, NUM_POINTS * 2), (None, NUM_POINTS * 4),
                  (3.0, None), (5.0, None), (8.0, None)]
        for i, (amp, npts) in enumerate(ladder, 1):
            q_try = (list(q_deg) if amp is None else
                     [v + float(d) for v, d in zip(
                         q_deg, rng.uniform(-amp, amp, kin.num_joints))])
            t, e, dq = _seg_plan(T_cur, T_target, q_try, num_points=npts)
            if e is None:
                traj, err, max_dq = t, None, dq
                how = (f"种子扰动±{amp:.0f}°" if amp is not None
                       else f"路点加密至{npts}")
                note = (f"直线IK首解失败({first_err[:14]}…),"
                        f"重试第{i}次成功({how})")
                break
            err = e
    if err is not None and err.startswith("跳解护栏: 关节最大变化") \
            and max_dq <= MAX_TOTAL_DQ_DEG:
        # 首尾超限但量级在合法重构范围内 → 按几何等分链式细分重试:
        # 每段独立过护栏(首尾+相邻)+FK 到位校验,链式取实际解推进;
        # 中途翻转/任段不可达/超总重构上限则维持原拒绝。
        for k in (2, 3, 4, 6, MAX_SUBDIV_SEGMENTS):
            if k * JUMP_GUARD_MAX_DQ_DEG < max_dq:
                continue                      # 段数不够把每段压进 45°
            q_i, T_i, chain, broken = list(q_deg), T_cur, [], None
            for i in range(1, k + 1):
                T_sub = _T_interp(T_cur, T_target, i / k)
                t, e, _ = _seg_plan(T_i, T_sub, q_i)
                if e == "NEAR_ZERO":
                    continue                  # 该子段已到位,跳过
                if e is not None:
                    broken = e
                    break
                chain.extend(t if not chain else t[1:])
                q_i = list(t[-1])
                T_i = np.asarray(kin.fkine(q_i), dtype=float)
            if broken is not None or not chain:
                continue
            T_end = np.asarray(kin.fkine(chain[-1]), dtype=float)
            pos_err = float(np.linalg.norm(T_end[:3, 3] - T_target[:3, 3]))
            rot_err = float(np.linalg.norm(
                _skew_vec(T_target[:3, :3] @ T_end[:3, :3].T)))
            if pos_err <= REACH_POS_TOL_M and rot_err <= REACH_ROT_TOL_RAD:
                traj, err, subdiv = chain, None, k
                note = (f"整段重构 {max_dq:.0f}° 超单段护栏,"
                        f"分 {k} 段链式规划")
                break
    if err is not None:
        return fail(err + ("(自适应分段后仍不可行,拒绝下发)"
                           if err.startswith("跳解护栏") else ""))

    max_dq = _max_dq_deg(traj)
    return {"ok": True, "waypoints_deg": traj,
            "max_dq_deg": round(max_dq, 3),
            "subdivisions": subdiv, "note": note,
            "urdf": urdf, "base_frame": base_frame,
            "end_frame": end_frame, "num_points": len(traj)}


def _serve() -> int:
    """常驻模式:按行读请求,一行一个 JSON 响应。stderr 只留给致命错误。"""
    while True:
        line = sys.stdin.readline()
        if not line:
            return 0                       # 对端关闭,正常退出
        line = line.strip()
        if not line:
            continue
        try:
            req = json.loads(line)
        except json.JSONDecodeError as exc:
            out = {"ok": False, "error": f"请求 JSON 无效: {exc}"}
        else:
            try:
                out = handle_request(req)
            except Exception as exc:       # 兜底:不让 server 挂掉
                out = {"ok": False, "error": f"内部异常: {exc}"}
        sys.stdout.write(json.dumps(out, ensure_ascii=False) + "\n")
        sys.stdout.flush()


def main() -> int:
    if "--server" in sys.argv[1:]:
        return _serve()
    try:
        req = json.loads(sys.stdin.read() or "{}")
    except json.JSONDecodeError as exc:
        print(json.dumps({"ok": False, "error": f"请求 JSON 无效: {exc}"}))
        return 1
    out = handle_request(req)
    print(json.dumps(out, ensure_ascii=False))
    return 0 if out.get("ok") else 1


if __name__ == "__main__":
    sys.exit(main())
