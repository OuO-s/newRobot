# 裁剪机构接入 — 机器人侧集成指南

目标目录: `/home/user/tianlian/tianlian_webapp/`（FastAPI, app.py + static/app.js）
桥接服务: `http://10.245.169.20:8100`（Windows 电脑, CH340→RS485→驱动器, Modbus-RTU）

共 5 处改动。**cutter_client.py 是新文件直接放进去**，其余 4 处按下面的锚点插入。

---

## 1. 新文件 `cutter_client.py` ✅ 已写好（本目录同名文件），直接拷贝到 webapp 根目录

## 2. `flow_store.py` — DEFAULTS 加 cutter 节 + 读写助手

`DEFAULTS`（约 :88，与 gate/move 平级）追加：

```python
"cutter": {
    "base_url": "http://10.245.169.20:8100",
    "default_turns": -4,       # 默认反转4圈
    "rpm": 240,
    "move_timeout_s": 60,
    "poll_s": 0.3,
},
```

再加（镜像现有 set_action_vacuum 的写法）：

```python
def set_action_cutter(title, cutter_cfg):
    """cutter_cfg = {"turns": -4} | None(=删除触发器)"""
    cfg = load_config()
    act = _find_action(cfg, title)          # ← 复用你们现有的动作查找函数
    if cutter_cfg is None:
        act.pop("cutter", None)
    else:
        act["cutter"] = {"turns": float(cutter_cfg.get("turns", -4))}
    save_config(cfg)
    return act
```

## 3. `flow.py` — play_action 触发块（镜像 vacuum, 约 :443 触发点）

在 vacuum 触发逻辑旁边（同一个“到点→wait_s→触发”顺序）加：

```python
from cutter_client import CutterClient, CutterError

# --- cutter 触发: 语义同 vacuum.at, act.get("cutter") = {"turns": -4} ---
cutter_trig = act.get("cutter")
if cutter_trig and _hit_trigger(cutter_trig.get("at", "end"), wp_idx, total):  # ← 复用 vacuum 的触发点计算
    ccfg = flow_store.load_config().get("cutter", {})
    client = CutterClient(ccfg.get("base_url", "http://10.245.169.20:8100"))
    client.trigger(ccfg, cutter_trig)   # 阻塞等到位; 失败抛 CutterError → 停线
```

急停/abort 路径（现有 estop 处理处）兜底加：

```python
try:
    CutterClient(cfg.get("cutter", {}).get("base_url", "")).stop()
except Exception:
    pass
```

## 4. `app.py` — 手动测试端点（放其它 /api/flow/* 附近）

```python
from cutter_client import CutterClient, CutterError

def _cutter():  # 从配置取 base_url
    return CutterClient(flow_store.load_config().get("cutter", {}).get(
        "base_url", "http://10.245.169.20:8100"))

@app.get("/api/cutter/health")
def cutter_health():
    return _cutter().health()

@app.get("/api/cutter/status")
def cutter_status():
    return _cutter().status()

@app.post("/api/cutter/move")
def cutter_move(turns: float = -4):
    return _cutter().move_and_wait(turns)   # 手动点=等待到位

@app.post("/api/cutter/stop")
def cutter_stop():
    return _cutter().stop()

@app.post("/api/flow/action/cutter")
def flow_action_cutter(title: str, body: dict = Body(...)):
    """设置/删除动作的裁剪触发器 body: {"turns": -4} 或 {} = 删除"""
    return flow_store.set_action_cutter(title, body or None)
```

## 5. `static/app.js` — 动作面板配置 UI

在 vacuum 触发器配置行（action 面板里）旁边镜像加一行：

```javascript
// HTML(index.html 动作面板): <label>✂️ 裁剪 <input id="actCutterTurns" type="number" value="-4" size="4"> 圈
//                            <button id="btnActCutter">挂载/更新</button> <button id="btnActCutterDel">删除</button></label>
async function setActionCutter(del) {
  const title = currentActionTitle();              // ← 复用面板现有的动作选择
  const body = del ? {} : { turns: parseFloat(document.getElementById('actCutterTurns').value || -4) };
  await fetch(`/api/flow/action/cutter?title=${encodeURIComponent(title)}`, {
    method: 'POST', headers: {'Content-Type': 'application/json'},
    body: del ? '{}' : JSON.stringify(body) });
}
// 载入动作时回显: act.cutter?.turns -> actCutterTurns.value
```

顶栏 base_url 配置（可选, 将来换 IP 网页一行改）：
```javascript
// 保存: POST /api/flow/config {"cutter": {...cfg, base_url: 新值}}  (走你们现有的 save_config 通道)
```

---

## 验证顺序

1. `curl http://10.245.169.20:8100/api/cutter/health` ✅（已通）
2. 集成后在网页手动测: `POST /api/cutter/move?turns=-4`（SIM 模式, 立即到位）
3. 示教 B5_extra 到裁剪位 → 挂 cutter 触发器 → 跑一遍流水线
4. **真机前**: Windows 侧插上 CH340 → 重启 cutter_bridge（自动连真实串口）→ health 里
   `"simulated": false` 才是真机; `true` 一律是模拟。

## 注意

- 桥接 SIM 模式一切返回成功但机构不动，接真机前务必确认 `simulated: false`
- Windows 电脑重启后需要重新启动桥接: `.venv\Scripts\python.exe cutter_bridge.py`
  （后续可做成开机自启/计划任务）
- 驱动器拨码须为 SW1=ON, SW6/SW7=OFF（RS-485 模式）, A-B 线序正确
