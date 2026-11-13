# -*- coding: utf-8 -*-
"""
cutter_client.py — 裁剪机构(推杆+升降台)websocket 客户端
================================================================
对接 Jetson 10.245.169.248 /ssd/control/ws_control.py
(ws://<jetson>:8765, JSON over WebSocket, 无鉴权仅限内网)。

协议摘要(完整见 Jetson 端源码头注释):
  请求: {"id":N, "cmd":"push","mode":"cycle|left|right"[,"speed"]}
        {"id":N, "cmd":"lift","dir":"down|up"[,"revs":圈数][,"speed"]}
        {"id":N, "cmd":"cycle"[,"lift_dir":"down|up"]}   ← 联动
        {"id":N, "cmd":"stop"} / {"id":N, "cmd":"status"}
  响应: hello(连上即发,带状态) → ack(受理) → done(动作完成,带 ok+legs)
        / error(busy|bad_cmd|bad_args|bad_json)
  服务器每次只执行一个动作,不排队;动作独立于连接执行到底并把结果
  留底在 last_done —— 断线后可 reconnect + status 对账恢复。

线程模型: websockets 是异步库,这里起一条后台事件循环线程,对外全部
同步阻塞接口(流水线臂线程直接调);所有发送经锁串行化。

CutterFlow = 面向流水线的高层封装(升降回中/推切联动/异步推切),
支持 sim=True 纯本地模拟(不连真机,写代码调试用)。
"""
from __future__ import annotations

import asyncio
import json
import threading
import time

try:                                   # websockets>=13 新客户端实现
    from websockets.asyncio.client import connect as _ws_connect
except ImportError:                    # 旧版
    from websockets import connect as _ws_connect  # type: ignore

import websockets


class CutterError(RuntimeError):
    """裁剪机构通讯/执行失败(面向用户的消息)。"""


class CutterAborted(CutterError):
    """等待动作完成期间流水线被软停/停线 —— 转成 _Abort,不算失败。"""


# 每条腿怎么停的中文分型(排障用,与 Jetson 端 ws_test_client 一致)
LEG_CN = {
    "limit_neg": "撞负向限位钉停(左/下)",
    "limit_pos": "撞正向限位钉停(右/上)",
    "position": "走完脉冲自然停",
    "aborted": "被stop中止",
    "timeout": "等待超时!!",
    "comms": "通讯丢失!!",
    "error": "指令失败!!",
}


def _legs_cn(done: dict) -> str:
    return " ".join(f"{l.get('leg')}:{LEG_CN.get(l.get('result'), l.get('result'))}"
                    for l in done.get("legs", [])) or "?"


class CutterWS:
    """单连接 websocket 客户端(线程安全,懒连接,断线由调用方恢复)。"""

    def __init__(self):
        self._lock = threading.RLock()
        self._loop: asyncio.AbstractEventLoop | None = None
        self._ws = None
        self._send_lock: asyncio.Lock | None = None   # loop 内串行化发送
        self._url: str | None = None
        self._seq = 0
        self._pending: dict[int, dict] = {}           # id → {ev,msg,acked}
        self.hello: dict | None = None                # 最近一次连接的 hello

    # ------------------------------------------------------------ 连接管理
    def _ensure_loop(self):
        if self._loop is not None and not self._loop.is_closed():
            return
        started = threading.Event()

        def _run():
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            self._loop = loop
            started.set()
            try:
                loop.run_forever()
            finally:
                loop.close()

        threading.Thread(target=_run, daemon=True,
                         name="cutter-ws-loop").start()
        if not started.wait(3.0):
            raise CutterError("裁剪 websocket 后台线程起不来")

    @property
    def connected(self) -> bool:
        return self._ws is not None

    def connect(self, url: str, timeout: float = 5.0) -> dict:
        """连接(同 URL 已连=幂等返回 hello),返回服务端 hello 状态。"""
        with self._lock:
            if self._ws is not None and url == self._url:
                return self.hello or {}
            self._ensure_loop()
            if self._ws is not None:                  # 换 URL:先弃旧连接
                self._close_ws_quiet()
            fut = asyncio.run_coroutine_threadsafe(
                self._connect(url, timeout), self._loop)
            try:
                ws, hello = fut.result(timeout + 2.0)
            except CutterError:
                raise
            except Exception as e:                    # noqa: BLE001
                raise CutterError(f"裁剪机构连接失败 {url} ({e})") from e
            self._ws, self._url, self.hello = ws, url, hello
            self._send_lock = asyncio.Lock()
            asyncio.run_coroutine_threadsafe(self._reader(ws), self._loop)
            return hello

    async def _connect(self, url: str, timeout: float):
        try:
            ws = await _ws_connect(url, open_timeout=timeout,
                                   ping_interval=20, ping_timeout=20,
                                   close_timeout=1, max_size=65536)
            hello = json.loads(await asyncio.wait_for(ws.recv(), timeout))
            if hello.get("type") != "hello":
                raise CutterError(f"首包不是 hello: {hello}")
            return ws, hello
        except CutterError:
            raise
        except Exception as e:                        # noqa: BLE001
            raise CutterError(f"裁剪机构不可达 {url} ({type(e).__name__})") from e

    async def _reader(self, ws):
        """收消息线程任务: 按 id 分发;连接断开时清掉 self._ws。"""
        try:
            async for raw in ws:
                try:
                    msg = json.loads(raw)
                except ValueError:
                    continue
                p = self._pending.get(msg.get("id"))
                if p is None:
                    continue
                t = msg.get("type")
                if t == "ack":
                    p["acked"] = True
                elif t in ("done", "error", "status"):
                    p["msg"] = msg
                    p["ev"].set()
        except Exception:                             # noqa: BLE001
            pass                                      # 断线:wait() 会走恢复
        finally:
            if self._ws is ws:
                self._ws = None

    def _close_ws_quiet(self):
        ws = self._ws
        self._ws = None
        if ws is not None and self._loop is not None:
            try:
                asyncio.run_coroutine_threadsafe(ws.close(), self._loop) \
                    .result(1.0)
            except Exception:                         # noqa: BLE001
                pass

    def close(self):
        with self._lock:
            self._close_ws_quiet()
            self._url = None

    # ------------------------------------------------------------ 收发
    def _register(self) -> dict:
        self._seq += 1
        p = {"ev": threading.Event(), "msg": None, "acked": False}
        self._pending[self._seq] = p
        return self._seq, p

    def _send(self, payload: dict):
        if self._ws is None:
            raise CutterError("裁剪机构未连接")

        async def _do():
            assert self._send_lock is not None
            async with self._send_lock:
                await self._ws.send(json.dumps(payload, ensure_ascii=False))

        try:
            asyncio.run_coroutine_threadsafe(_do(), self._loop) \
                .result(3.0)
        except CutterError:
            raise
        except Exception as e:                        # noqa: BLE001
            raise CutterError(f"裁剪指令发送失败(连接断开?) {e}") from e

    def _request(self, cmd: str, args: dict, timeout: float,
                 ack_only: bool = False, abort_cb=None) -> dict:
        """发一条指令并等回复。ack_only=True: 收到 ack 即返回(stop 用)。
        否则等 done/error/status 之一,返回该消息。"""
        with self._lock:
            rid, p = self._register()
            try:
                self._send({"id": rid, "cmd": cmd, **args})
                deadline = time.time() + timeout
                while True:
                    if abort_cb and abort_cb():
                        raise CutterAborted("软停:等待裁剪机构期间中断")
                    if p["ev"].wait(0.1):
                        return p["msg"]
                    if p["acked"] and ack_only:
                        return {"type": "ack", "id": rid, "cmd": cmd}
                    if time.time() > deadline:
                        raise CutterError(
                            f"裁剪指令 {cmd} 等待响应超时 {timeout:g}s"
                            "(连接断开或服务器无应答)")
            finally:
                self._pending.pop(rid, None)

    # ------------------------------------------------------------ 公开接口
    def status(self, url: str, timeout: float = 3.0) -> dict:
        with self._lock:
            self.connect(url, timeout=min(timeout, 5.0))
            out = self._request("status", {}, timeout)
        if out.get("type") == "error":
            raise CutterError(f"status 失败: {out}")
        return out

    def stop(self, url: str, timeout: float = 3.0) -> dict:
        with self._lock:
            self.connect(url, timeout=min(timeout, 5.0))
            out = self._request("stop", {}, timeout, ack_only=True)
        return out

    def run(self, url: str, cmd: str, timeout: float, args: dict | None = None,
            abort_cb=None) -> dict:
        """发运动指令并阻塞到 done。返回 done 消息(ok 由调用方检查)。"""
        args = args or {}
        with self._lock:
            self.connect(url, timeout=5.0)
            out = self._request(cmd, args, timeout, abort_cb=abort_cb)
        if out.get("type") == "error":
            raise CutterError(f"裁剪指令 {cmd} 被拒: {out.get('code')}"
                              f"({out.get('hint') or out.get('msg') or ''})")
        return out

    def begin(self, url: str, cmd: str, args: dict | None = None,
              ack_timeout: float = 4.0, abort_cb=None) -> int:
        """发运动指令,等到 ack(受理)即返回 —— 完成消息由 wait() 收。"""
        args = args or {}
        with self._lock:
            self.connect(url, timeout=5.0)
            rid, p = self._register()
            try:
                self._send({"id": rid, "cmd": cmd, **args})
                deadline = time.time() + ack_timeout
                while True:
                    if abort_cb and abort_cb():
                        raise CutterAborted("软停:等待裁剪机构受理期间中断")
                    if p["ev"].wait(0.05):
                        msg = p["msg"]
                        if msg.get("type") == "error":
                            raise CutterError(
                                f"裁剪指令 {cmd} 被拒: {msg.get('code')}")
                        return rid                     # 受理前就完成了(少见)
                    if p["acked"]:
                        return rid
                    if time.time() > deadline:
                        raise CutterError(f"裁剪指令 {cmd} 无受理响应"
                                          "(连接断开?)")
            except BaseException:
                self._pending.pop(rid, None)
                raise

    def wait(self, url: str, handle: int, timeout: float,
             abort_cb=None) -> dict:
        """等 begin() 那条指令的 done。断线时重连对账恢复。"""
        with self._lock:
            p = self._pending.get(handle)
            if p is None:
                raise CutterError(f"裁剪动作#{handle} 不存在或已收尾")
            deadline = time.time() + timeout
            while True:
                if abort_cb and abort_cb():
                    raise CutterAborted("软停:等待裁剪动作完成期间中断")
                if p["ev"].wait(0.25):
                    msg = p["msg"]
                    if msg.get("type") == "error":
                        raise CutterError(f"裁剪动作#{handle} 失败: "
                                          f"{msg.get('code')}")
                    self._pending.pop(handle, None)
                    return msg
                if self._ws is None:                  # 连接断了 → 恢复
                    return self._recover(url, handle,
                                         max(10.0, deadline - time.time()),
                                         abort_cb)
                if time.time() > deadline:
                    raise CutterError(f"裁剪动作#{handle} 超时 {timeout:g}s"
                                      " —— 机构状态未知,停线人工确认")

    def _recover(self, url: str, handle: int, timeout: float,
                 abort_cb=None) -> dict:
        """断线恢复: 重连 → 轮询 busy → 用 last_done 对账本条动作结果。
        (服务器动作独立于连接执行完并留底,所以断线≠结果丢失)"""
        self._pending.pop(handle, None)
        t0 = time.time()
        said = False
        while time.time() - t0 < timeout:
            if abort_cb and abort_cb():
                raise CutterAborted("软停:裁剪机构断线恢复期间中断")
            try:
                st = self.status(url, timeout=3.0)
            except CutterError:
                if not said:
                    said = True
                time.sleep(1.0)                       # 服务端还没起来,再试
                continue
            if not st.get("busy"):
                ld = st.get("last_done") or {}
                if ld.get("id") == handle:
                    return ld
                raise CutterError(
                    f"裁剪动作#{handle} 结果丢失(last_done.id="
                    f"{ld.get('id')}) —— 人工确认机构状态后再启动")
            time.sleep(0.5)
        raise CutterError("裁剪机构断线恢复超时(动作可能仍在执行)"
                          " —— 人工确认后再启动")


# ============================================================
# 流水线语义高层封装
# ============================================================
class CutterFlow:
    """cfg = flow_config 顶层 "cutter" 节; log = callable(str) 记流水线日志。

    高层动作(全部阻塞到 Jetson 回 done,ok=False 抛错停线):
      lift_home_top()     升降升到最高(撞上限位钉停)—— 回零/基准
      lift_lowest()       升降降到最低(撞下限位钉停)
      lift_middle()       升降从最高点下降 middle_revs 圈 = 中间点
                          (与手动台 double_control_linux.py 按键4同义:
                          LIFT_DEF_REVS=11 默认;每轮第2推联动升到上限位
                          重新对齐基准,圈数走脉冲自然停,无累积误差)
      lift_highest()      同 lift_home_top(手动按钮用)
      lift_revs(dir,n)    按圈数步进(标定中间点用)
      push_cycle(lift_dir) 推杆往返(去左限位=剪切,回右限位);
                          lift_dir="down"/"up" = 回程同时升降降/升到限位
                          (手动台按键7/8 = 联动1/联动2)
      begin_push_cycle(lift_dir) + wait(tok)
                          异步版: 受理即返回,期间臂可去吸下一摞
      valve(on, port)    吸盘电磁阀通/断(独立继电器串口,不受 busy 门禁)
    时序对应(2026-11-11 流水线3 编排):
      [启动回零: 升到最高] →
      第1摞放下 → (等右臂取走上轮) → 升降↓middle_revs→中间点 →
      begin_push_cycle("down")(剪切第1摞,回程升降降到最低)
      [左臂同时吸第2摞回放料区上方] → wait(推回+升降已到最低)
      → 放第2摞 → push_cycle("up")(剪切第2摞,回程升降升到最高)
      → 置 FILLED,右臂从最高点夹取两摞
    """

    def __init__(self, cfg: dict, log=None):
        self.cfg = cfg or {}
        self.log = log or (lambda s: None)
        self.sim = bool(self.cfg.get("sim"))
        self.url = self.cfg.get("ws_url") or "ws://10.245.169.248:8765"
        self._sim_t0 = None
        if not self.sim:
            self.ws = CutterWS()
        else:
            self.ws = None

    # ------------------------------------------------------------ 基础
    @property
    def _t_push(self) -> float:
        return float(self.cfg.get("push_timeout_s", 90.0))

    @property
    def _t_lift(self) -> float:
        return float(self.cfg.get("lift_timeout_s", 60.0))

    @property
    def _t_valve(self) -> float:
        return float(self.cfg.get("valve_timeout_s", 8.0))

    def _check(self, done: dict, what: str) -> dict:
        if not done.get("ok", False):
            raise CutterError(f"裁剪[{what}] 未安全完成: {_legs_cn(done)}"
                              f"(raw={done})")
        self.log(f"裁剪[{what}] 完成 {done.get('ms')}ms · {_legs_cn(done)}")
        return done

    def _sim_sleep(self, what: str, s: float):
        self.log(f"裁剪[SIM][{what}] 模拟执行 {s:g}s")
        time.sleep(s)

    # ------------------------------------------------------------ 状态/急停
    def status(self) -> dict:
        if self.sim:
            return {"type": "status", "busy": False, "sim": True}
        return self.ws.status(self.url)

    def stop(self) -> dict:
        """全停/中止当前动作(随时可发)。失败不抛(急停路径)。"""
        if self.sim:
            self.log("裁剪[SIM] stop")
            return {"type": "ack", "cmd": "stop", "sim": True}
        try:
            out = self.ws.stop(self.url)
            self.log("裁剪: 已发全停(stop)")
            return out
        except CutterError as e:
            self.log(f"裁剪: stop 发送失败({e})")
            return {"type": "error", "error": str(e)}

    def close(self):
        if self.ws is not None:
            self.ws.close()

    # ------------------------------------------------------------ 电磁阀(吸盘)
    def valve(self, on: bool, port: str = "C", abort_cb=None) -> dict:
        """吸盘电磁阀通/断电(2026-11-13 硬件已接)。

        走 Jetson ws "valve" 指令 → dist/valve_min.py → USB-485 继电器
        (BESTEP 双路,port C=通道1/U=通道2)→ AirTAC 4V210-08 单电控阀。
        通电=阀换向(吸),断电=弹簧复位(放,物料自由落体)。
        即时执行不受 busy 门禁(推杆动作期间也要能放料);继电器无应答
        → done.ok=false → 这里抛 CutterError → 流水线停线(该吸没吸/
        该放没放都不能静默继续)。"""
        if self.sim:
            self.log(f"裁剪[SIM] 电磁阀{'通电(吸)' if on else '断电(放)'}"
                     f" {port}通道")
            return {"ok": True, "sim": True, "on": bool(on)}
        return self._check(
            self.ws.run(self.url, "valve", self._t_valve,
                        {"on": bool(on), "port": str(port).upper()},
                        abort_cb=abort_cb),
            f"电磁阀{'通电(吸)' if on else '断电(放)'}")

    # ------------------------------------------------------------ 运动
    def lift_lowest(self, abort_cb=None) -> dict:
        if self.sim:
            self._sim_sleep("升降→最低", 2.0)
            return {"ok": True, "sim": True}
        return self._check(
            self.ws.run(self.url, "lift", self._t_lift, {"dir": "down"},
                        abort_cb=abort_cb), "升降→最低")

    def lift_highest(self, abort_cb=None) -> dict:
        """升降升到最高(撞上限位钉停)—— 手动按钮/回零用。"""
        if self.sim:
            self._sim_sleep("升降→最高", 2.5)
            return {"ok": True, "sim": True}
        return self._check(
            self.ws.run(self.url, "lift", self._t_lift, {"dir": "up"},
                        abort_cb=abort_cb), "升降→最高")

    lift_home_top = lift_highest          # 语义别名: 回零=升到上限位

    def lift_revs(self, direction: str, revs: float, abort_cb=None) -> dict:
        """按圈数走(自然停,不撞限位)—— 标定中间点用手动步进。"""
        if self.sim:
            self._sim_sleep(f"升降{direction} {revs:g}圈", 1.0)
            return {"ok": True, "sim": True}
        return self._check(
            self.ws.run(self.url, "lift", self._t_lift,
                        {"dir": direction, "revs": abs(float(revs))},
                        abort_cb=abort_cb), f"升降{direction}{revs:g}圈")

    def lift_middle(self, abort_cb=None) -> dict:
        """升降 → 中间点 = 从最高点下降 middle_revs 圈(脉冲自然停)。
        基准保证: 每轮第2推 push_cycle("up") 回程把升降撞上限位钉停,
        之后右臂才来取料(EMPTY),所以本动作出发时必在最高点 —— 与手动台
        「按键6 升到上限位 → 按键4 下降 N 圈」同一套用法。"""
        revs = float(self.cfg.get("middle_revs", 11.0))
        if self.sim:
            self._sim_sleep(f"升降↓{revs:g}圈→中间点", 1.5)
            return {"ok": True, "sim": True}
        done = self.ws.run(self.url, "lift", self._t_lift,
                           {"dir": "down", "revs": revs}, abort_cb=abort_cb)
        out = self._check(done, f"升降↓{revs:g}圈→中间点")
        for leg in done.get("legs", []):        # 圈数走完应是"自然停"
            if leg.get("result") == "limit_neg":
                self.log(f"⚠ 升降到中间点时撞了下限位钉停:"
                         f"middle_revs={revs:g} 圈过大或基准不在最高点"
                         " —— 核对圈数后重跑")
        return out

    def _push_speed(self):
        """推杆速度 r/min(cfg.push_speed_rpm; 0/缺省=不带,服务端默认1200)。
        只作用于推杆(cycle 的 speed 也只写站1),升降腿恒走服务端默认300
        (Jetson 实测 >300 堵转,勿动)。"""
        try:
            v = float(self.cfg.get("push_speed_rpm") or 0)
        except (TypeError, ValueError):
            return None
        return v if v >= 10 else None

    def push_cycle(self, lift_dir: str | None = None,
                   abort_cb=None) -> dict:
        """推杆往返一次(去=剪切,回=复位)。lift_dir 给 down/up 时,
        回程同时升降降/升到限位(Jetson 端 cycle 联动)。"""
        args = {} if lift_dir is None else {"lift_dir": lift_dir}
        spd = self._push_speed()
        if spd is not None:
            args["speed"] = spd
        name = "推杆往返" if lift_dir is None else f"推杆往返·联动升降{lift_dir}"
        if self.sim:
            self._sim_sleep(f"{name}@{spd:g}rpm" if spd else name, 3.0)
            return {"ok": True, "sim": True}
        return self._check(
            self.ws.run(self.url, "cycle" if lift_dir else "push",
                        self._t_push, args, abort_cb=abort_cb), name)

    def begin_push_cycle(self, lift_dir: str | None = None,
                         abort_cb=None) -> int:
        """异步版推杆往返: 受理(ack)即返回句柄,完成用 wait() 收。
        流水线用它实现『推杆剪切的同时左臂回去吸第 2 摞』。"""
        args = {} if lift_dir is None else {"lift_dir": lift_dir}
        spd = self._push_speed()
        if spd is not None:
            args["speed"] = spd
        if self.sim:
            self.log(f"裁剪[SIM] 推杆往返(异步) 受理"
                     + (f" @{spd:g}rpm" if spd else ""))
            return -1
        return self.ws.begin(self.url, "cycle" if lift_dir else "push",
                             args, abort_cb=abort_cb)

    def wait(self, handle: int, abort_cb=None) -> dict:
        if self.sim:
            self._sim_sleep("推杆往返完成", 3.0)
            return {"ok": True, "sim": True}
        return self._check(
            self.ws.wait(self.url, handle, self._t_push, abort_cb=abort_cb),
            "推杆往返(异步)")


def probe(url: str, timeout: float = 2.0) -> dict:
    """一次性连通性检查(readiness 用): 连上+收 hello+查一次状态。"""
    ws = CutterWS()
    try:
        return ws.status(url, timeout=timeout)
    finally:
        ws.close()
