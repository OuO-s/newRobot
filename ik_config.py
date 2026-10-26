# -*- coding: utf-8 -*-
"""ik_helper 运行配置。

子进程方式运行逆解(避开 conda 3.11 缺 C 扩展 / GUI 进程没法 import tl_kinematics)。
"""
from pathlib import Path

# tl_kinematics 链只在系统 Python 3.10 可导入;webapp 也是任意 3.10+ 即可运行
# 如有需要可改 virtualenv python
SYS_PYTHON = "/usr/bin/python3"

# ik_helper.py 与本文件同目录,子进程启动时用绝对路径调起
IK_HELPER = Path(__file__).resolve().with_name("ik_helper.py")
