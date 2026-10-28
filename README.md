# tianlian_webapp — 天链机器人远程控制(独立 Web 模块)

把双臂机器人 / 升降 / 相机 / 思岚底盘 / 示教 / 物料检测 的常用控制搬到浏览器里,
局域网任意终端(包括手机)开 http://<host>:8001/ 即可远程操作。

不依赖 PySide6 / ROS2,纯 Python 3.10+ + FastAPI。

> 本模块与 `upper_computer/` 桌面 GUI **互不依赖**。源码直接拷贝即可
> 独立运行(连同同名依赖文件),也可整体作为新项目的一部分使用。
> 若 `upper_computer/robot_client.py` 后续有 bug 修复,需手动同步
> 到本目录同名文件。

## 九个 Tab

| Tab | 功能 |
| --- | --- |
| **📊 状态** | 1Hz SSE 推送:急停 / 升降 / 腰头 / 双臂 7 关节角 / 末端位姿;双臂上下电、升降上下/停/回零/清错、腰头移动、IK 可用性 |
| **🧭 笛卡尔移动** | 当前末端位姿;X/Y/Z 平移点动(基座/工具系,Rx/Ry/Rz 旋转点动需 IK);目标点位姿输入(自动填当前位姿);夹爪开合 |
| **📍 关节示教** | 记录当前 7 关节角为路点,上下移/删除;保存/加载/删除工程 JSON;▶ 一键回放 |
| **🛤️ 笛卡尔示教** | 记录当前末端位姿(pos+quat)为路点;保存/加载/删除工程;▶ 一键回放(逐段 IK) |
| **💾 点位复现** | 把当前关节角保存为命名点位(机器人本机);列表 ▶ 直达 / 删除;双击列表项也可直达 |
| **🔍 物料检测** | 启停 `pose_service.py` 子进程(yolo11_env GPU 推理);取相机最新帧送推理;返回检测结果 + 标注图;支持连续检测 |
| **📷 相机** | 左/头/右 三路勾选启停;MJPEG 流(浏览器原生 `<img>`);抓拍下载;1–20 fps 可调。取帧默认走 `camera_bridge.py`(ROS2 话题直订阅,**原生 640×480 无畸变**),每路显示帧来源/分辨率 |
| **🚗 底盘** | 独立连接;栅格地图 PNG(点击导航,叠加机器人/激光/桩);位姿+激光实时刷新;移动到 / 旋转 / 回桩 / 取消 / 4 向遥控;**星标(POI)增删改+CSV 导入,一键前往**;**🛤️ 沿轨导航独立设置**(自动规划 / 沿轨·停等 / 绕行 / 忽略障碍直行,对星标前往/巡逻队列/点选/坐标统一生效,up* 点位可设不带朝向);**巡逻队列循环跑点**(CSV 日志/导出,后端执行);电量/健康/动作/速度;💾存/📂重载 stcm;📍手动重定位 |
| **🦾 双臂流水线** | 左臂 YOLO 吸料 → 中间件,右臂 中间件取料 → 料仓 的整线编排:左臂吸料五子步全视觉自动(①物料检测 → ②移到物料上方 → ③下降+下压 → ④吸取·手动真空窗口 → ⑤抬升,只需示教 A1/A2)、动作路点网页示教(固定槽位 A1/A2/B0–B4)+**逐点编辑(覆盖/删除/排序)**+**路点级"中点"标记**(标记段进前抢中心锁/走到非标记点才交,不标=整段互斥)+**路点⏱到点等待**(到该点停留 N 秒再走剩余段,点徽章输秒数,0=取消;真空触发点同点配等待时=到点→等→触发)、**步骤说明可编辑**、**动作间夹爪整形(任意插槽,不限 B2→B3)**、三档对准闸门(纯回放/单帧校验/一步修正,**逐站启用/停用**开关)+重锚回放、双臂令牌协同、**轮次**(N 轮后整线自动停,留空=连续;右臂会把第 N 件送完料仓才停)、单臂测试(左/右各跑 1 循环自动停,不等令牌)、单步确认联调、真空中途触发。详见 `doc/双臂协同上下料工作流方案_2026-10-20.md` |

顶部 ⛔ 急停 = 机械臂下电 + 升降停 + 底盘取消动作。

### 同网段直连底盘(电脑连手机热点,不切换 WiFi)

机器人 wlan0 与电脑同在手机热点网段,`~/tianlian/forward_slamware_1448.py`(名字带 1448,实际双端口)把底盘透传到该网段。**手动启动、重启不自启**:

```bash
nohup /usr/bin/python3 /home/user/tianlian/forward_slamware_1448.py >/tmp/fwd1448.log 2>&1 &
```

| 用途 | 地址 |
|---|---|
| RoboStudio(思岚) | `<机器人 wlan0 IP>:1445` ← RoboStudio 专用 JSON 通道,**不是 1448** |
| 底盘 REST/SDK(HTTP) | `<机器人 wlan0 IP>:1448` |
| 网页控制台 / SSH | `:8001` / `:22` |

- wlan0 IP 为 DHCP(当前 `10.245.169.128`),热点重连后可能变,`ip addr show wlan0` 查看
- 日志 `/tmp/fwd1448.log` 记录每连接双向前几段报文,可辨协议/排障
- 重启转发器时勿用 `pkill -f`(会匹配自身命令行自杀),先 `ss -tlnp` 取 PID 再 kill

### 🛤️ 沿轨导航(底盘虚拟轨道,2026-10-27)

RoboStudio 里画的**虚拟轨道存在底盘上**,导航时给 `move_to` 带 `mode` 即由底盘
规划器原生贴线走(与 RoboStudio「前往(虚拟轨道且含Yaw角)」同一机制):

| mode | 行为 |
|---|---|
| 0 | 自动规划(默认,避障) |
| 1 | 强制沿轨,遇障碍**停下等待** |
| 2 | 轨道优先,遇障碍**绕行** |
| 3 | 强制沿轨 + **忽略动态障碍直行**(官方 flag `avoid_path_ignoring_dynamic_obstacles`,规划时无视激光观测层障碍点,静态地图障碍仍保护;⚠️ 真障碍也照走,仅确认现场无实物时用) |

- 网页星标与巡逻之间有**「🛤️ 沿轨导航(独立设置)」卡片**(选择会记住),作用于
  星标前往 / 巡逻队列 / 地图点选 / 坐标导航,统一生效
- 网页自建轨道(画轨编辑器/密点贴线/巡逻参照轨道)已于 2026-10-27 整体移除,
  数据文件 `chassis_tracks.json` 与后端 /api/chassis/tracks 保留(休眠)
- **速度比例(speed_ratio,10-27 加)**:卡片内下拉 100/75/50/30/15%(记住,选
  100% 时不下发)——官方 move_options.speed_ratio(插件>=4.5.4,本机固件 5.2.1 ✓),
  本次导航最大速度=全局上限×比例,单次生效不动系统参数;作用于星标前往/巡逻/
  点选/坐标,与全局「最大速度」(系统参数,重启失效)互补
- **up\* 不带朝向**:卡片内勾选框(记住)——名字以 up 开头的点位(如 up1~up5
  岔轨出口拐弯点)在任何模式下前往都不下发 yaw,到点即止不原地转向;
  作用于星标前往与巡逻队列,001~005 等非 up 点照常带朝向
- 沿轨遇障停等后动作可能报 done 但未到点(遇障放弃);到位朝向/定位异常时
  优先怀疑定位漂移,用 📍重定位或 RoboStudio 重定位后再试
- 岔轨"半途刹停但报 done"已定性(2026-10-27):静态地图干净、无虚拟墙,
  实时激光在岔轨线上有**位置固定但逐帧闪烁**的回波(8~92 点/帧剧烈波动,
  10 帧采样同一点 7/10 帧出现)——疑似透明/悬挂物(玻璃、亚克力、软门帘)
  对激光的假象,高度约离地 15~30cm(雷达扫描平面);mode 3 即为绕开该假象
  所加,激光假回波解码格式见记忆(1445 协议节)
- 只读动作监视器 `~/tianlian/chassis_watch.py`(动作/路径/位姿抓包,
  日志 `/tmp/chassis_watch.log`),协议细节见记忆 `tianlian-chassis-control-channels`

## 文件清单

| 文件 | 作用 |
| --- | --- |
| `app.py` | FastAPI 入口(72 个路由) |
| `services.py` | 纯 Python 后台轮询线程(状态/相机/底盘) |
| `robot_client.py` | 天链 :5000 / :8000 HTTP 封装(从 upper_computer 同步) |
| `camera_bridge.py` | **子进程**:ROS2 彩色话题 → 本地 :7101 快照/MJPEG(系统 3.10 + ROS2;与 GUI 同款同端口,外部已跑则复用) |
| `slamtec_client.py` | 思岚 Slamware REST 封装(从 upper_computer 同步) |
| `ik_helper.py` | 末端位姿逆解子进程(从 upper_computer 同步) |
| `pose_service.py` | YOLO11 物料检测服务(从 upper_computer 同步) |
| `ik_config.py` | `SYS_PYTHON` / `IK_HELPER` 路径配置 |
| `cartesian.py` | `plan_cartesian_path` + `cartesian_delta_move`(IK 折线) |
| `teach_store.py` | 示教工程 JSON 持久化(从 upper_computer 同步) |
| `flow_store.py` | 双臂流水线持久化:`flow_data/flow_config.json`(参数+动作路点+对准参考)、手眼文件绑定 |
| `speed_store.py` | **全局速度(双臂分别)**持久化:`speed_data/global_speed.json`。顶栏唯一速度入口,所有臂运动下发时取值,改动即时生效 |
| `resume_store.py` | **会话自恢复**持久化:`resume_data/session.json` 记住上次整机连接与开着的相机,启动时自动重连+恢复预览(见下) |
| `flow_align.py` | 对准闸门测量:ArUco/QR 找码 + IPPE_SQUARE PnP + 多帧中值 → 码在基座系位置(QR 解码靠 pyzbar,导入期自动尾部追加 upper_computer env 的 site-packages 复用;流水线页三组码均可选 ArUco/QR,QR 内容留空=认画面里第一张) |
| `flow.py` | 流水线编排器:双臂线程、中间件令牌 EMPTY/FILLED、中心区互斥、三档闸门、重锚回放(闸门修正 Δ 传导进 A2/B2/B4 的下降/退出轨迹,否则按示教关节角回放会把修正弹回示教点)、左臂吸取步与『物料检测』页一步到位同款且**逐子步上报 phase**(①物料检测 scan → ②移到物料上方并下降 pick_hover/pick_descend —— 路径沿用『物料检测』页 motion_mode(一键直达一段直线/分段走廊)→ ③下压 pick_press → ④吸取·真空窗口 pick_grab,吸完 A1 回放直接接管,运行徽章/单步确认/耗时统计都按子步呈现)、B2→B3 夹爪整形步、单步确认、`safety.center_serialize`(左臂先等中间件 EMPTY 再持锁贯穿 A1→闸门→A2,消除双臂同在中间件上方的窗口)、**路点级"中点"标记**(任一路点标了 ●中点 即从整段互斥切到路点级:走向中点前抢锁、带到其后第一个非标记点(退出段走完)才交,闸门/跨动作自然衔接;Handoff.center_wp_level 区分锁级别防整段锁被提前释放) |
| `flow_runlog.py` | **流水线运行日志**:每次启动/试跑在 `logs/flow/` 生成 `run_*.log`(人读时间线)+ `run_*.jsonl`(机读事件流)。动作含开始/结束时间·耗时·速度%·路点数·起止位姿,运动段含耗时与到位残差,闸门含每步测量/修正/档位,循环含各 phase 耗时分解 —— 供节拍测算与排查 |
| `depth_provider.py` | fd_cameras_hub 相机会话封装(深度租约 5s 心跳 / 纯彩色无租约) |
| `static/{index.html, app.js, style.css}` | 浏览器端 9 Tab |
| `requirements.txt` | fastapi / uvicorn / requests / Pillow |
| `run.sh` | 一键启动脚本 |

## 安装与运行

```bash
cd tianlian_webapp
python3 -m pip install -r requirements.txt
./run.sh                          # 监听 0.0.0.0:8001
# 或:
HOST=0.0.0.0 PORT=8001 ./run.sh
```

## 开机自启(已配置)

通过 crontab `@reboot` 挂载,**无需 sudo**(本机 sudo 需密码、systemd 用户服务无 linger 会在登出时被杀,故选 cron):

```bash
$ crontab -l
@reboot /home/user/tianlian/tianlian_webapp/start_server.sh start
```

`start_server.sh` 是一个轻量 supervisor:

- 前台循环拉起 uvicorn,进程崩溃/被杀 **3 秒后自动重启**
- pid 文件 `.webapp.pid`;日志 `logs/webapp.log`(超 10MB 自动截断)
- 手动操作:

```bash
./start_server.sh start     # 启动(已在运行则提示)
./start_server.sh stop      # 停止(supervisor + uvicorn 一起收掉)
./start_server.sh restart   # 改完代码后重启生效
./start_server.sh status    # 状态 + 最近日志
```

- 启动前会 `source /opt/ros/humble/setup.bash` + `~/fd_ws/install/setup.bash`
  ——uvicorn 进程内的 depth_provider(物料扫描取流/深度租约)要导
  rclpy/fd_msgs,不 source 会报 `No module named 'rclpy'`。

**重启自动恢复(会话自愈)**:webapp 一启动,后台线程读取
`resume_data/session.json` 自动重连上次的整机(等网关就绪,5s×60 次重试),
并恢复上次开着的相机预览;每次成功连接、开关相机都会更新记录。显式断开
不清记录(下次重启仍自动恢复);用户先手动连接则以手动为准。重试与恢复
痕迹打在 `logs/webapp.log`(`[auto-resume]` 前缀)。

> 走 systemd 的备选方案(若有 sudo):把 `tianlian-webapp.service`
> 放进 `/etc/systemd/system/` 后 `systemctl enable --now`,效果相同
> 且日志进 journalctl。当前环境没有免密 sudo,故用 cron。

> ik_helper 需 `/usr/bin/python3` (系统 3.10) 能 import `tl_kinematics`
> (位于 `~/fd_ws`)。若该环境不可用,`/api/ik/availability` 会返回
> `available=false` 并附原因;但其他接口(状态/相机/底盘/示教)仍正常工作,
> 仅笛卡尔旋转/目标移动/笛卡尔示教回放会失败并返回 503。

> 物料检测需 `yolo11_env` venv(旧工程 Jetson torch 环境,路径常量
> `POSE_YOLO_ENV_PY` = `/home/user/tianlian/old/tianlian/yolo11_env/bin/python`)。
> 换机器时改 `app.py` 顶部常量;启动时还会自动补 `cusparselt` 的
> `LD_LIBRARY_PATH`(只补这一个,勿加 venv 自带 cudnn)。

## API 一览(节选)

- `POST /api/connect` `{host, web_port, arm_port}` — 连接整机(成功后写入 resume_data,重启自动恢复)
- `GET  /api/status` / `/api/status/stream` — 状态 JSON / SSE
- `POST /api/camera/{start,stop}` / `GET /api/camera/stream?cam=left&fps=10` / `GET /api/camera/{snapshot,meta}?cam=left` — 相机(meta 含帧来源 ros=原生 / firmware=固件兜底)
- `POST /api/chassis/connect` `{host, port}` / `GET /api/chassis/{pose,laser,map}` / `POST /api/chassis/{move_to,joystick,rotate_to,go_home,cancel}` — 底盘
- `GET  /api/pose/list?arm=armleft` / `POST /api/pose/{save,goto,delete}` — 点位
- `POST /api/cartesian/{jog,rotate,move_to}` / `GET /api/cartesian/current?arm=...` / `POST /api/gripper` — 笛卡尔
- `GET/DELETE /api/teach/{kind}/list|load|delete` / `POST /api/teach/{save,joint/play,cart/play}` — 示教
- `POST /api/pose_service/{start,stop,detect}` / `GET /api/pose_service/health` — 物料检测
- `GET  /api/ik/availability` / `GET /api/health` — 探测
- `GET/POST /api/flow/config` — 流水线参数 + 就绪检查(readiness)
- `POST /api/flow/teach/{waypoint,waypoint/delete,waypoint/overwrite,waypoint/move,clear,align_ref}` — 动作路点示教与逐点编辑(覆盖=当前位姿替换第k点 / delete=删第k点 / move=前移后移一位)
- `POST /api/flow/action/{play,play/stop,vacuum}` / `GET /api/flow/action/play/status` — 单动作试跑与真空中途触发
- `POST /api/flow/align/test` — 对准闸门只测不修(返回 PASS/NEED-FIX/REFUSE)
- `POST /api/flow/align/goto` — 回到记录参考时的观察位(动臂,关节回放+段后验证)再测 Δ
- `POST /api/flow/align/fix` — 回观察位并**真跑闸门**(测 Δ→超容差实际修正→复测收敛,≤max_corr;修完停在修正后的位置)—— 单步验证 PBVS,与流水线闸门同一条代码路径
- `POST /api/flow/{start,stop,step/confirm}` / `GET /api/flow/status` — 整线启停(支持单步确认模式)
- `GET /api/flow/logs` — 运行日志册清单(新→旧:起止时间/循环数/臂)
- `GET /api/flow/logs/preview?start=&end=` — 运行报告(按时间段聚合成轮次 × A/B 大步,含每轮/累计/平均耗时;start/end 支持 `YYYY-MM-DD[ HH:MM[:SS]]`,缺省=当天;流水线在跑时末轮标"进行中")
- `GET /api/flow/logs/export?start=&end=` — 运行报告导出 CSV(与预览同口径,中文表头 + 每轮小计 + 总计,UTF-8 BOM,Excel 直开;网页『📈 运行报告』同款)

## 流水线运行日志与节拍测算(logs/flow/)

每次启动(含单臂测试/单步模式)或单动作试跑,都会生成一册
`run_<时间>[_dual|left|right|play_armxxx].log` + `.jsonl`:

- `.log` 人读时间线:每动作起止与耗时·速度%、闸门测量/修正/档位、真空事件、令牌交接、循环小结;
- `.jsonl` 机读事件流,一行一条,`kind` 见 `flow_runlog.py` 模块头(action/move/gate_*/scan/vacuum/handoff/cycle/…)。

网页运行卡有「📈 运行报告」:按时间段(默认当天)预览每轮 A/B 大步
耗时,一键导出同口径 CSV(客户视图,无速度%等内部字段);上面的明细
jsonl 仍照常落盘,供内部逐动作测算节拍(每轮循环耗时 + 各动作分解 + 当时速度):

```bash
python3 - <<'EOF'
import json, glob
rows = [json.loads(l) for l in open(sorted(glob.glob('logs/flow/run_*.jsonl'))[-1])]
for r in rows:
    if r["kind"] == "cycle":
        print(f'{r["arm"]} #{r["cycle"]} {r["duration_s"]}s '
              f'(actions: ' + ', '.join(f'{a["action"]}={a["duration_s"]}s@{a["speed_pct"]}%'
                                        for a in r["actions"]) + ')')
EOF
```

## 已知限制

1. 单用户单进程;多浏览器同时操作会互相覆盖。
2. 相机取帧默认走 `camera_bridge.py` 子进程(ROS2 彩色话题直订阅,
   原生 640×480,~21 fps;连接整机/开相机时自动拉起,日志
   `logs/camera_bridge.log`,需本机有 ROS2 Humble)。桥接不可用时
   自动回退 :5000 固件抓拍——**注意固件会把 640×480 硬拉成 1920×1080
   (4:3→16:9),画面有几何畸变**,仅保底用,前端会标注"固件(有拉伸畸变)"。
   防御:bridge 内置看门狗(全路干涸 >45s 即重建订阅自愈 hub 重建发布端
   导致的失配),webapp 侧发现 bridge 进程失联会自动重拉;
   画面变形排查见 `../doc/相机画面畸变_排查与自愈_2026-10-21.md`。
   webapp 重启后会自动重连整机并恢复开着的相机预览(见「开机自启」),
   浏览器刷新即可,无需重新连接/勾选。
3. 地图点击导航使用图中心为原点的简单换算(精度有限,适合"回桩附近"
   场景;精确导航请直接输入 X/Y)。
4. ik_helper 的可达性探测会真起一次子进程(~1–3s),不要高频调用。
5. 笛卡尔示教回放为"逐段 IK + 串行 move_robotarm_traj",长轨迹耗时长;
   UI 无进度条,等接口返回即完成。
