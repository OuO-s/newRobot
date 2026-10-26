#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""末端笛卡尔位姿逆解辅助进程。

必须用系统 Python 3.10 运行(GUI 的 conda 3.11 无法导入 tl_kinematics
的 C 扩展):

    echo '{"op":"rotate",...}' | /usr/bin/python3 ik_helper.py

从 stdin 读 JSON 请求,stdout 输出单行 JSON 结果。职责:根据当前关节角
构造目标位姿(相对旋转 / 绝对位姿),用 tl_kinematics 的直线+姿态插值
规划关节轨迹,由上位机经 /move_robotarm 下发执行。

请求 op:
  rotate  {"arm","q_deg","axis":"rx|ry|rz","angle_deg","frame":"base|tool"}
  target  {"arm","q_deg","target_pos_m":[x,y,z],"target_rpy_deg":[r,p,y],
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
REACH_POS_TOL_M = 5e-3       # 末端到达容差(与 fd_arm_api 近成功容差一致)
REACH_ROT_TOL_RAD = 5e-2
JOINT_LIMIT_EPS_DEG = 1e-2
NUM_POINTS = 6               # fd_arm_api straightline_num_points 默认值


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


def main() -> int:
    try:
        req = json.loads(sys.stdin.read() or "{}")
    except json.JSONDecodeError as exc:
        print(json.dumps({"ok": False, "error": f"请求 JSON 无效: {exc}"}))
        return 1

    def fail(msg, unavailable=False):
        print(json.dumps({"ok": False, "error": msg,
                          "unavailable": unavailable}, ensure_ascii=False))
        return 1

    if not _DEPS_OK:
        return fail(f"系统 Python 缺少 numpy/tl_kinematics: {_DEPS_ERR}",
                    unavailable=True)

    arm = req.get("arm") or ""
    q_deg = [float(v) for v in (req.get("q_deg") or [])]
    op = req.get("op")
    if arm not in ("armleft", "armright") or not q_deg:
        return fail("缺少 arm / q_deg")
    if op not in ("rotate", "target"):
        return fail(f"未知 op: {op}")

    startup = _load_yaml(os.path.join(FD_WS, "data", "param", "startup.yaml"))
    profile = _load_yaml(os.path.join(FD_WS, "data", "param",
                                      "robot_profile.yaml"))
    model = (startup.get("model") or "").strip()
    profile["_model"] = model
    profile["_arm_mode"] = (profile.get("models") or {}).get(
        model, {}).get("arm_mode") or (profile.get("arm_mode") or "dual")

    urdf = _resolve_urdf(startup, profile, model)
    if not urdf:
        return fail("未找到 URDF(检查 ~/fd_ws/data/param/startup.yaml "
                    "的 urdf_model 与 tl_robot_description 安装)",
                    unavailable=True)

    dof = len(q_deg)
    base_frame, end_frame = _resolve_frames(profile, arm, dof)
    try:
        kin = RobotKinematics(urdf, base_frame, end_frame)
    except Exception as exc:
        return fail(f"运动学初始化失败({base_frame}→{end_frame}): {exc}",
                    unavailable=True)
    if kin.num_joints > len(q_deg):
        return fail(f"关节角数量 {len(q_deg)} 少于 URDF 自由度 "
                    f"{kin.num_joints}")
    q_deg = q_deg[:kin.num_joints]

    T_cur = np.asarray(kin.fkine(q_deg), dtype=float)
    T_target = T_cur.copy()
    if op == "rotate":
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

    try:
        traj = kin.straightline(T_target=T_target, T_current=T_cur,
                                q_initial=q_deg, num_points=NUM_POINTS)
    except Exception as exc:
        return fail(f"直线规划失败: {exc}")
    traj = [[float(v) for v in pt] for pt in (traj or [])]
    if not traj or any(len(pt) != kin.num_joints for pt in traj):
        return fail("规划结果为空或维度不符")

    for pt in traj:
        if not all(math.isfinite(v) for v in pt):
            return fail("规划结果含非有限值(目标位姿可能不可达)")

    lower = list(kin.joint_lower or [])
    upper = list(kin.joint_upper or [])
    if len(lower) >= kin.num_joints and len(upper) >= kin.num_joints:
        for i, pt in enumerate(traj):
            for j in range(kin.num_joints):
                if upper[j] > lower[j] and not (
                        lower[j] - JOINT_LIMIT_EPS_DEG <= pt[j]
                        <= upper[j] + JOINT_LIMIT_EPS_DEG):
                    return fail(f"轨迹点 {i} 关节 {j + 1}={pt[j]:.2f}° "
                                f"超出限位 [{lower[j]:.1f}, {upper[j]:.1f}]")

    max_dq = _max_dq_deg(traj)
    if max_dq < NEAR_ZERO_TRAJ_DEG:
        return fail(f"目标与当前位姿几乎一致(最大关节变化 {max_dq:.3f}°),"
                    "无需运动")
    # 跳解护栏: 腕部奇异附近(如料仓观察位 J5≈-2°,2026-10-23)数值 IK
    # 会收敛到等价解分支 —— 首尾关节翻转 70~258°,而末端位姿与目标一致,
    # 下方 pos_err/rot_err 验证拦不住;跳解轨迹下发后大行程翻转走不完,
    # 以 arrival timeout 收场。正常单段(闸门修正/重锚/吸取)关节变化
    # 实测 ≤20°,45° 足够宽松又远低于任何跳解。
    if max_dq > JUMP_GUARD_MAX_DQ_DEG:
        return fail(f"跳解护栏: 关节最大变化 {max_dq:.1f}° > "
                    f"{JUMP_GUARD_MAX_DQ_DEG:.0f}°(解分支翻转),拒绝下发")
    if _max_step_dq_deg(traj) > JUMP_GUARD_MAX_DQ_DEG:
        return fail(f"跳解护栏: 相邻路点关节最大变化 "
                    f"{_max_step_dq_deg(traj):.1f}° > "
                    f"{JUMP_GUARD_MAX_DQ_DEG:.0f}°(轨迹中途翻转),拒绝下发")

    T_end = np.asarray(kin.fkine(traj[-1]), dtype=float)
    pos_err = float(np.linalg.norm(T_end[:3, 3] - T_target[:3, 3]))
    rot_err = float(np.linalg.norm(
        _skew_vec(T_target[:3, :3] @ T_end[:3, :3].T)))
    if pos_err > REACH_POS_TOL_M or rot_err > REACH_ROT_TOL_RAD:
        return fail(f"目标位姿不可达(末端误差 pos={pos_err * 1000:.1f}mm "
                    f"rot={math.degrees(rot_err):.2f}°)")

    print(json.dumps({"ok": True, "waypoints_deg": traj,
                      "max_dq_deg": round(max_dq, 3),
                      "urdf": urdf, "base_frame": base_frame,
                      "end_frame": end_frame, "num_points": len(traj)},
                     ensure_ascii=False))
    return 0


if __name__ == "__main__":
    sys.exit(main())
