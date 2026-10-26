# -*- coding: utf-8 -*-
"""示教数据模型与 JSON 持久化。

两种示教文件:
  joint_teach     关节示教:每个路点记录 N 个关节角(度)+ 可选末端执行器目标
  cartesian_teach 笛卡尔示教:每个路点记录末端位姿(position + orientation)+ 可选末端
"""
from __future__ import annotations

import json
import time
from pathlib import Path

TEACH_DIR = Path(__file__).resolve().parent / "teach_data"


def _stamp_name(prefix: str) -> str:
    return f"{prefix}_{time.strftime('%Y%m%d_%H%M%S')}"


class TeachFile:
    """示教工程在内存中的表示与存取。"""

    def __init__(self, kind: str, arm: str = "armleft"):
        assert kind in ("joint_teach", "cartesian_teach")
        self.kind = kind
        self.arm = arm                  # armleft / armright
        self.name = _stamp_name(prefix="关节示教" if kind == "joint_teach" else "笛卡尔示教")
        self.speed = 15                 # 回放速度百分比 1-100
        self.created = time.strftime("%Y-%m-%d %H:%M:%S")
        self.points: list[dict] = []    # 元素结构见 add_*

    # ------------------------------------------------------------ 路点操作
    def add_joint_point(self, joints_deg: list[float], gripper: float | None = None,
                        note: str = "") -> None:
        self.points.append({
            "joints_deg": [round(float(v), 3) for v in joints_deg],
            "gripper": gripper,
            "note": note,
        })

    def add_cartesian_point(self, position: list[float], orientation: list[float],
                            gripper: float | None = None, note: str = "") -> None:
        self.points.append({
            "position": [round(float(v), 4) for v in position],
            "orientation": [round(float(v), 4) for v in orientation],
            "gripper": gripper,
            "note": note,
        })

    def set_gripper_at(self, index: int, value: float | None) -> None:
        self.points[index]["gripper"] = value

    def move_point(self, index: int, offset: int) -> None:
        """上下移动路点,返回后调用方刷新表格。"""
        new_index = index + offset
        if not 0 <= new_index < len(self.points):
            return
        self.points[index], self.points[new_index] = \
            self.points[new_index], self.points[index]

    def remove_point(self, index: int) -> None:
        del self.points[index]

    def clear(self) -> None:
        self.points.clear()

    # ---------------------------------------------------------------- 存取
    def save(self, path: str | Path) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "kind": self.kind,
            "name": self.name,
            "arm": self.arm,
            "speed": self.speed,
            "created": self.created,
            "points": self.points,
        }
        path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")

    @classmethod
    def load(cls, path: str | Path) -> "TeachFile":
        payload = json.loads(Path(path).read_text(encoding="utf-8"))
        kind = payload.get("kind")
        if kind not in ("joint_teach", "cartesian_teach"):
            raise ValueError(f"未知示教类型: {kind}")
        obj = cls(kind, payload.get("arm", "armleft"))
        obj.name = payload.get("name", Path(path).stem)
        obj.speed = int(payload.get("speed", 15))
        obj.created = payload.get("created", "")
        obj.points = payload.get("points", [])
        return obj

    # ------------------------------------------------------------ 便捷属性
    @property
    def is_joint(self) -> bool:
        return self.kind == "joint_teach"

    @property
    def has_gripper(self) -> bool:
        return any(p.get("gripper") is not None for p in self.points)

    def joints_list(self) -> list[list[float]]:
        return [p["joints_deg"] for p in self.points]
