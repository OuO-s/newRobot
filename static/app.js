// 天链机器人远程控制 - 前端逻辑(原生 JS,8 Tab)
'use strict';

const API = {
  health:        () => fetch('/api/health').then(r => r.json()),
  connect:       (host, wp, ap) => post('/api/connect', { host, web_port: wp, arm_port: ap }),
  disconnect:    () => post('/api/disconnect', {}),
  estop:         () => post('/api/arm/emergency_stop', {}),
  speedGet:      () => fetch('/api/speed').then(r => r.json()),
  speedSet:      (b) => post('/api/speed', b),
  armPowerOn:    (name, sp) => post('/api/arm/power_on', { robot_name: name, speed: sp }),
  armPowerOff:   (name) => post('/api/arm/power_off', { robot_name: name }),
  jointJog:      (b) => post('/api/arm/joint_jog', b),
  jogReset:      (arm) => post(`/api/arm/jog_reset?arm=${encodeURIComponent(arm)}`, {}),
  status:        () => fetch('/api/status').then(r => r.json()),
  statusStream:  () => new EventSource('/api/status/stream'),
  liftInfo:      () => fetch('/api/lift/info').then(r => r.json()),
  liftMove:      (d, sp, dir) => post('/api/lift/move', { distance: d, speed: sp, direction: dir }),
  liftStop:      () => post('/api/lift/stop', {}),
  liftHoming:    () => post('/api/lift/homing', {}),
  liftClearErr:  () => post('/api/lift/clear_error', {}),
  waistMove:     (a) => post('/api/waist/move', { angle: a }),
  headMove:      (a) => post('/api/head/move', { angle: a }),
  cartCurrent:   (arm) => fetch(`/api/cartesian/current?arm=${arm}`).then(r => r.json()),
  cartJog:       (b) => post('/api/cartesian/jog', b),
  cartRotate:    (b) => post('/api/cartesian/rotate', b),
  cartMoveTo:    (b) => post('/api/cartesian/move_to', b),
  gripper:       (side, pos, sp) => post('/api/gripper', { side, position: pos, speed: sp }),
  teachList:     (kind) => fetch(`/api/teach/${kind}/list`).then(r => r.json()),
  teachLoad:     (kind, name) => fetch(`/api/teach/${kind}/load?name=${encodeURIComponent(name)}`).then(r => r.json()),
  teachDelete:   (kind, name) => fetch(`/api/teach/${kind}/delete?name=${encodeURIComponent(name)}`, { method: 'DELETE' }).then(r => r.json()),
  teachSave:     (b) => post('/api/teach/save', b),
  teachPlayJ:    (b) => post('/api/teach/joint/play', b),
  teachPlayC:    (b) => post('/api/teach/cart/play', b),
  poseList:      (arm) => fetch(`/api/pose/list?arm=${arm}`).then(r => r.json()),
  poseSave:      (b) => post('/api/pose/save', b),
  poseGoto:      (b) => post('/api/pose/goto', b),
  poseDelete:    (b) => post('/api/pose/delete', b),
  poseSvcHealth: () => fetch('/api/pose_service/health').then(r => r.json()),
  cameraMeta:    () => fetch('/api/camera/meta').then(r => r.json()),
  poseSvcStart:  () => post('/api/pose_service/start', {}),
  poseSvcStop:   () => post('/api/pose_service/stop', {}),
  poseDetect:    (b) => post('/api/pose_service/detect', b),
  pkConfigGet:   () => fetch('/api/pick/config').then(r => r.json()),
  pkConfigSet:   (b) => post('/api/pick/config', b),
  pkDepthHealth: () => fetch('/api/pick/depth/health').then(r => r.json()),
  pkTeachScan:   () => post('/api/pick/teach/scan_pose', {}),
  pkTeachAlign:  (b) => post('/api/pick/teach/align', b),
  pkScan:        (b) => post('/api/pick/scan', b),
  pkStart:       (b) => post('/api/pick/start', b),
  pkStatus:      () => fetch('/api/pick/status').then(r => r.json()),
  pkStop:        () => post('/api/pick/stop', {}),
  flConfigGet:   () => fetch('/api/flow/config').then(r => r.json()),
  flConfigSet:   (b) => post('/api/flow/config', b),
  flWpAdd:       (arm, action) => post('/api/flow/teach/waypoint', { arm, action }),
  flWpDelLast:   (arm, action, n) => post('/api/flow/teach/waypoint/delete', { arm, action, index: n }),
  flWpOverwrite: (arm, action, index) => post('/api/flow/teach/waypoint/overwrite', { arm, action, index }),
  flTitle:       (arm, action, title) => post('/api/flow/action/title', { arm, action, title }),
  flWpMove:      (arm, action, index, offset) => post('/api/flow/teach/waypoint/move', { arm, action, index, offset }),
  flCenterMark:  (arm, action, index, center) => post('/api/flow/teach/center_mark', { arm, action, index, center }),
  flWpWait:      (arm, action, index, seconds) => post('/api/flow/teach/waypoint/wait', { arm, action, index, seconds }),
  flWpClear:     (arm, action) => post('/api/flow/teach/clear', { arm, action }),
  flVac:         (b) => post('/api/flow/action/vacuum', b),
  flPlay:        (arm, action) => post('/api/flow/action/play', { arm, action }),
  flPlayStop:    () => post('/api/flow/action/play/stop', {}),
  flPlayStatus:  () => fetch('/api/flow/action/play/status').then(r => r.json()),
  flRefRecord:   (arm, which) => post('/api/flow/teach/align_ref', { arm, which }),
  flAlignTest:   (arm, which) => post('/api/flow/align/test', { arm, which }),
  flAlignGoto:   (arm, which) => post('/api/flow/align/goto', { arm, which }),
  flAlignFix:    (arm, which) => post('/api/flow/align/fix', { arm, which }),
  flStart:       (b) => post('/api/flow/start', b),
  flStop:        () => post('/api/flow/stop', {}),
  flStepOk:      () => post('/api/flow/step/confirm', {}),
  flStatus:      () => fetch('/api/flow/status').then(r => r.json()),
  flRpt:         async (start, end) => {
    const r = await fetch(`/api/flow/logs/preview?start=${encodeURIComponent(start)}&end=${encodeURIComponent(end)}`);
    const d = await r.json().catch(() => ({}));
    if (!r.ok) throw new Error(d.detail || r.statusText);
    return d;
  },
  ikAvail:       () => fetch('/api/ik/availability').then(r => r.json()),
  camStart:      (cam, ms) => post('/api/camera/start', { cam, interval_ms: ms }),
  camStop:       (cam) => post('/api/camera/stop?cam=' + encodeURIComponent(cam)),
  camStreamUrl:  (cam, fps) => `/api/camera/stream?cam=${cam}&fps=${fps}`,
  camSnapUrl:    (cam) => `/api/camera/snapshot?cam=${cam}`,
  chConn:        (host, port) => post('/api/chassis/connect', { host, port }),
  chDisc:        () => post('/api/chassis/disconnect', {}),
  chPose:        () => fetch('/api/chassis/pose').then(r => r.json()),
  chLaser:       () => fetch('/api/chassis/laser').then(r => r.json()),
  chMap:         (refresh) => fetch(`/api/chassis/map?refresh=${refresh?'true':'false'}`).then(r => r.blob()),
  chMoveTo:      (x, y, yaw) => post('/api/chassis/move_to', { x, y, yaw }),
  chRotate:      (a) => post('/api/chassis/rotate_to', { angle_rad: a }),
  chHome:        (dock) => post('/api/chassis/go_home', { dock }),
  chCancel:      () => post('/api/chassis/cancel', {}),
  chJoystick:    (dir, dur) => post('/api/chassis/joystick', { direction: dir, duration_ms: dur }),
};

async function post(url, body) {
  const r = await fetch(url, { method: 'POST',
                                headers: { 'Content-Type': 'application/json' },
                                body: JSON.stringify(body) });
  let data = {};
  try { data = await r.json(); } catch {}
  if (!r.ok) throw new Error(data.detail || r.statusText);
  return data;
}

// ===== Toast =====
const $toast = document.getElementById('toast');
let _toastTimer = null;
function toast(msg, type = '') {
  $toast.textContent = msg;
  $toast.className = 'toast show ' + type;
  clearTimeout(_toastTimer);
  _toastTimer = setTimeout(() => $toast.className = 'toast', 2400);
}

function fmt(v, n = 3) {
  if (v === null || v === undefined) return '-';
  if (typeof v !== 'number') return String(v);
  return v.toFixed(n);
}
function fmtArr(a, n = 3) {
  if (!Array.isArray(a)) return '-';
  return '[' + a.map(v => fmt(v, n)).join(', ') + ']';
}
function fmtPose(p) {
  if (!p || typeof p !== 'object') return '-';
  const pos = p.position || p.pos;
  const quat = p.orientation || p.quat;
  const rpy = p.rpy;
  let s = '';
  if (Array.isArray(pos))  s += `pos   : ${fmtArr(pos, 4)}\n`;
  if (Array.isArray(quat)) s += `quat  : ${fmtArr(quat, 4)}\n`;
  if (Array.isArray(rpy))  s += `rpy°  : ${fmtArr(rpy, 2)}`;
  return s || '-';
}
// 笛卡尔 Tab 专用:平移显示成 mm(0.1mm 分辨率),便于微调对准
function fmtPoseMm(p) {
  if (!p || typeof p !== 'object') return '-';
  const pos = p.position || p.pos;
  const quat = p.orientation || p.quat;
  const rpy = p.rpy;
  let s = '';
  if (Array.isArray(pos))  s += `pos mm: ${fmtArr(pos.map(v => v * 1000), 1)}\n`;
  if (Array.isArray(quat)) s += `quat  : ${fmtArr(quat, 4)}\n`;
  if (Array.isArray(rpy))  s += `rpy°  : ${fmtArr(rpy, 2)}`;
  return s || '-';
}

// ===== Tabs =====
document.querySelectorAll('.tab').forEach(t => {
  t.addEventListener('click', () => {
    document.querySelectorAll('.tab').forEach(x => x.classList.remove('active'));
    document.querySelectorAll('.panel').forEach(x => x.classList.remove('active'));
    t.classList.add('active');
    document.getElementById('tab-' + t.dataset.tab).classList.add('active');
    if (t.dataset.tab === 'chassis') refreshMap();
    if (t.dataset.tab === 'cart') { cartRefreshPose(); refreshEffector(); }
    if (t.dataset.tab === 'cart-teach') ctRefreshCurrent();
  });
});

// ===== Topbar (connect + estop) =====
const $connStatus = document.getElementById('connStatus');
const $btnConnect = document.getElementById('btnConnect');
const $btnEstop = document.getElementById('btnEstop');

$btnConnect.addEventListener('click', async () => {
  const host = document.getElementById('robotHost').value.trim();
  if (!host) { toast('请输入 IP', 'error'); return; }
  $btnConnect.disabled = true;
  try {
    const r = await API.connect(host, 5000, 8000);
    toast('连接成功: arm=' + (r.arm_ok ? 'ok' : '不可达'), 'success');
    $connStatus.textContent = '已连接';
    $connStatus.className = 'badge on';
    localStorage.setItem('robotHost', host);
    startStatusStream();
    refreshIkBadge();
  } catch (e) {
    toast('连接失败: ' + e.message, 'error');
  } finally {
    $btnConnect.disabled = false;
  }
});

document.getElementById('btnDisconnect').addEventListener('click', async () => {
  try { await API.disconnect(); } catch {}
  $connStatus.textContent = '未连接';
  $connStatus.className = 'badge off';
  stopStatusStream();
  stopAllCameras();
});

$btnEstop.addEventListener('click', async () => {
  if (!confirm('确认全局急停?')) return;
  try { await API.estop(); toast('已发送急停', 'success'); }
  catch (e) { toast('急停失败: ' + e.message, 'error'); }
});

const savedHost = localStorage.getItem('robotHost');
if (savedHost) document.getElementById('robotHost').value = savedHost;

// ===== 全局速度(双臂分别) =====
// 顶栏唯一入口;点动/笛卡尔/示教回放/点位直达/吸取/流水线的臂运动速度
// 全部由后端按下发时的全局速度解析(请求不显式带 speed 即可)。
// 后端持久化 → 多浏览器标签共享;改动即时生效(下一拍/下一段即用新值)。
const GSPD = { armleft: 15, armright: 15 };
let _gspdTimer = null;

function gspd(arm) { return GSPD[arm] || 15; }

function gspdSync(range, num, arm, v) {
  v = Math.max(1, Math.min(100, Math.round(+v) || 15));
  range.value = v;
  num.value = v;
  GSPD[arm] = v;
  clearTimeout(_gspdTimer);                    // 拖动过程合并成一次保存
  _gspdTimer = setTimeout(() => {
    API.speedSet({ [arm]: v })
      .catch(e => toast('全局速度保存失败: ' + e.message, 'error'));
  }, 250);
}

function gspdBind(rangeId, numId, arm) {
  const range = document.getElementById(rangeId);
  const num = document.getElementById(numId);
  range.addEventListener('input', () => gspdSync(range, num, arm, range.value));
  num.addEventListener('change', () => gspdSync(range, num, arm, num.value));
}

async function gspdInit() {
  try { Object.assign(GSPD, await API.speedGet()); } catch {}
  for (const [r, n, arm] of [['gspdL', 'gspdLn', 'armleft'],
                             ['gspdR', 'gspdRn', 'armright']]) {
    document.getElementById(r).value = gspd(arm);
    document.getElementById(n).value = gspd(arm);
  }
}
gspdBind('gspdL', 'gspdLn', 'armleft');
gspdBind('gspdR', 'gspdRn', 'armright');
gspdInit();

// ===== Status SSE =====
let sse = null;
function startStatusStream() {
  stopStatusStream();
  sse = API.statusStream();
  sse.onmessage = (e) => { try { renderStatus(JSON.parse(e.data)); } catch {} };
  sse.onerror = () => {
    $connStatus.textContent = '连接丢失';
    $connStatus.className = 'badge warn';
  };
}
function stopStatusStream() { if (sse) { sse.close(); sse = null; } }

const $em = document.getElementById('em');
const $lift = document.getElementById('lift');
const $waist = document.getElementById('waist');
const $head = document.getElementById('head');
const $cx = document.getElementById('cx');
const $cy = document.getElementById('cy');
const $cyaw = document.getElementById('cyaw');
const $joints = document.getElementById('joints');
const $epLeft = document.getElementById('epLeft');
const $epRight = document.getElementById('epRight');

function renderStatus(d) {
  $em.textContent = d.emergency ?? '-';
  $lift.textContent = d.lift ?? '-';
  $waist.textContent = d.waist ?? '-';
  $head.textContent = d.head ?? '-';
  $cx.textContent = fmt(d.chassis_pose?.x);
  $cy.textContent = fmt(d.chassis_pose?.y);
  $cyaw.textContent = fmt(d.chassis_pose?.yaw, 3);
  const j = d.joints;
  if (j && typeof j === 'object') {
    lastJoints.armleft = j.armleft;
    lastJoints.armright = j.armright;
    const row = (arm) => {
      const arr = j[arm];
      if (!Array.isArray(arr)) return '<td colspan="7" class="muted">-</td>';
      return arr.slice(0, 7).map(v => `<td>${fmt(v, 2)}</td>`).join('');
    };
    $joints.innerHTML =
      `<tr><td>左臂</td>${row('armleft')}</tr>` +
      `<tr><td>右臂</td>${row('armright')}</tr>`;
    syncJogFromStatus();
  }
  const ep = d.end_poses;
  if (ep && typeof ep === 'object') {
    $epLeft.textContent  = fmtPose(ep.armleft);
    $epRight.textContent = fmtPose(ep.armright);
  }
}

// ===== Status tab actions =====
document.querySelectorAll('[data-action]').forEach(btn => {
  btn.addEventListener('click', async () => {
    const a = btn.dataset.action;
    try {
      switch (a) {
        case 'arm-power-on':  await API.armPowerOn('both', 15); toast('已上电', 'success'); break;
        case 'arm-power-off': await API.armPowerOff('both');     toast('已下电', 'success'); break;
        case 'lift-up': {
          const d = +document.getElementById('liftUpDist').value || 0.05;
          await API.liftMove(d, 0.02, 0);
          break;
        }
        case 'lift-down': {
          const d = +document.getElementById('liftDnDist').value || 0.05;
          await API.liftMove(d, 0.02, 1);
          break;
        }
        case 'lift-stop':   await API.liftStop();   toast('升降停', 'success'); break;
        case 'lift-homing': await API.liftHoming(); toast('回零中...', 'success'); break;
        case 'lift-clear':  await API.liftClearErr(); toast('已清错', 'success'); break;
        case 'waist-move':  await API.waistMove(+document.getElementById('waistAng').value); break;
        case 'head-move':   await API.headMove(+document.getElementById('headAng').value); break;
      }
    } catch (e) { toast('失败: ' + e.message, 'error'); }
  });
});

async function refreshIkBadge() {
  const el = document.getElementById('ikAvail');
  try {
    const r = await API.ikAvail();
    el.textContent = r.available ? '✅ IK 子进程可用' : `⚠️ IK 不可用: ${r.reason}`;
  } catch (e) {
    el.textContent = 'IK 状态获取失败';
  }
}

// ===== 关节点动(按住持续平滑移动) =====
// 每个关节一行,− / + 按钮按住时按固定周期连续下发单点轨迹。
// 状态:JOBS[joint] = {timer, dir, seq} —— dir=+1/-1,seq 用于丢弃过期响应
const JOBS = Array.from({length: 7}, () => ({ timer: null, dir: 0, seq: 0, inflight: false }));

const JOINT_NAMES = ['J1 基座回转', 'J2 大臂', 'J3 小臂', 'J4 腕转', 'J5 腕摆', 'J6 腕转', 'J7 末端'];
let lastJoints = { armleft: null, armright: null };

function buildJogTable() {
  const body = document.getElementById('jogBody');
  if (!body) return;
  body.innerHTML = JOINT_NAMES.map((nm, i) => `
    <tr data-j="${i}">
      <td>${nm}</td>
      <td class="cur" data-cur="${i}">-</td>
      <td><button class="jog-btn" data-jog="${i}" data-dir="-1">−</button></td>
      <td style="color:#4a5260">|</td>
      <td><button class="jog-btn" data-jog="${i}" data-dir="1">+</button></td>
      <td>
        <input class="jog-abs" data-abs="${i}" type="number" step="0.5" placeholder="0.0" />
        <button class="jog-goto" data-goto="${i}">→</button>
      </td>
    </tr>`).join('');

  // 按住 / 松开
  body.querySelectorAll('.jog-btn').forEach(btn => {
    const j = +btn.dataset.jog;
    const dir = +btn.dataset.dir;
    const stop = (e) => { e.preventDefault(); stopJog(j, btn); };
    btn.addEventListener('pointerdown', (e) => {
      e.preventDefault();
      btn.setPointerCapture?.(e.pointerId);   // 捕获指针,移出按钮也能收到 up
      startJog(j, dir, btn);
    });
    btn.addEventListener('pointerup', stop);
    btn.addEventListener('pointercancel', stop);
    btn.addEventListener('lostpointercapture', stop);
    // 键盘可达性:空格/回车按住同样连续点动
    btn.addEventListener('keydown', (e) => {
      if ((e.key === ' ' || e.key === 'Enter') && !e.repeat) { e.preventDefault(); startJog(j, dir, btn); }
    });
    btn.addEventListener('keyup', (e) => {
      if (e.key === ' ' || e.key === 'Enter') stopJog(j, btn);
    });
    btn.addEventListener('blur', () => stopJog(j, btn));
  });

  // 绝对定位
  body.querySelectorAll('[data-goto]').forEach(btn => {
    btn.addEventListener('click', () => gotoJoint(+btn.dataset.goto));
  });
}

function jogParams() {
  return {
    arm: document.getElementById('jogArm').value,
    step: +document.getElementById('jogStep').value || 2,
    period: +document.getElementById('jogPeriod').value || 150,
  };
}

function startJog(j, dir, btn) {
  const st = JOBS[j];
  if (st.timer) return;                 // 已按住,不重复起
  st.dir = dir;
  st.seq = 0;
  btn.classList.add('holding');
  const p = jogParams();
  // 丢掉后端游标,让本次按住从实时反馈起步(不会接上次按住的目标继续走)
  API.jogReset(p.arm).catch(() => {});
  // 立即发第一帧,避免按下到起跳有延迟
  tickJog(j);
  st.timer = setInterval(() => tickJog(j), p.period);
}

function stopJog(j, btn) {
  const st = JOBS[j];
  if (!st.timer) return;
  clearInterval(st.timer);
  st.timer = null;
  st.dir = 0;
  if (btn) btn.classList.remove('holding');
  // 松手即断开游标接管:下次按住从实时反馈起步,而不是接着上次累加
  try { API.jogReset(jogParams().arm).catch(() => {}); } catch {}
}

// 连续点动的节流:上一次请求还在飞就跳过本拍,避免请求堆积
function tickJog(j) {
  const st = JOBS[j];
  if (!st.timer || !st.dir || st.inflight) return;
  const p = jogParams();
  const zeroBase = document.getElementById('jogZeroOrigin').checked;
  st.inflight = true;
  const seq = ++st.seq;
  API.jointJog({
    arm: p.arm, joint: j, delta: st.dir * p.step,
    absolute: zeroBase ? 0 : null,     // 速度:后端按该臂全局速度解析
  }).then(r => {
    st.inflight = false;
    if (seq !== st.seq) return;          // 过期响应,丢弃
    const cell = document.querySelector(`[data-cur="${j}"]`);
    if (cell) cell.textContent = fmt(r.commanded_deg, 2);
    const s = document.getElementById('jogStatus');
    if (r.at_limit) s.textContent = `⚠️ J${j+1} 已达软限位 ±175°,换方向试试`;
    else s.textContent = `J${j+1} 命令 ${fmt(r.commanded_deg,2)}° / 反馈 ${fmt(r.actual_deg,2)}° (滞后 ${fmt(r.lag_deg,1)}°)`;
  }).catch(e => {
    st.inflight = false;
    stopJog(j);                          // 出错立即停,避免一直刷错误
    document.getElementById('jogStatus').textContent = '点动失败: ' + e.message;
  });
}

async function gotoJoint(j) {
  const inp = document.querySelector(`[data-abs="${j}"]`);
  const v = parseFloat(inp.value);
  if (Number.isNaN(v)) { toast('请输入目标角度', 'error'); return; }
  const p = jogParams();
  try {
    const r = await API.jointJog({ arm: p.arm, joint: j, delta: 0,
                                   absolute: v });
    const cell = document.querySelector(`[data-cur="${j}"]`);
    if (cell) cell.textContent = fmt(r.commanded_deg, 2);
    document.getElementById('jogStatus').textContent =
      `J${j+1} → ${fmt(r.commanded_deg, 1)}°`;
  } catch (e) { toast('定位失败: ' + e.message, 'error'); }
}

document.getElementById('jogArm')?.addEventListener('change', syncJogFromStatus);
// 松开所有(切页 / 失焦 / 关窗)都停,避免机械臂一直走
function stopAllJogs() {
  JOBS.forEach((_, j) => stopJog(j));
  stopCartJog();
}
window.addEventListener('blur', stopAllJogs);
document.querySelectorAll('.tab').forEach(t =>
  t.addEventListener('click', stopAllJogs));

// 用 SSE 状态刷新点动表的"当前角度"列
function syncJogFromStatus() {
  const arm = document.getElementById('jogArm').value;
  const arr = lastJoints[arm];
  if (!Array.isArray(arr)) return;
  arr.slice(0, 7).forEach((v, j) => {
    const cell = document.querySelector(`[data-cur="${j}"]`);
    // 点动中不覆盖(避免和点动响应打架)
    if (cell && !JOBS[j].timer) cell.textContent = fmt(v, 2);
  });
}

// ===== 笛卡尔移动 =====
async function cartRefreshPose() {
  const arm = document.getElementById('cartArm').value;
  try {
    const p = await API.cartCurrent(arm);
    document.getElementById('cartCurrentPose').textContent = fmtPoseMm(p);
  } catch (e) {
    document.getElementById('cartCurrentPose').textContent = '获取失败: ' + e.message;
  }
}
document.getElementById('cartArm').addEventListener('change', cartRefreshPose);
document.getElementById('cartRefreshPose').addEventListener('click', cartRefreshPose);

function cartParams() {
  return {
    arm: document.getElementById('cartArm').value,
    frame: document.getElementById('cartFrame').value,
    step_m: (+document.getElementById('cartStep').value || 20) / 1000,   // 输入 mm,接口用米
  };
}

// 平滑连续移动:勾选后按住方向键,按固定周期连续下发增量,松开即停。
// 后端无笛卡尔游标,每拍增量相对当时的实际位姿,因此采用「上一拍完成才发
// 下一拍」的串行节流(inflight),保证增量不叠加错位;代价是松手后最多
// 还会走完一个步距(步距越小越跟手)。旋转每拍要起 IK 子进程,周期会被
// 求解耗时自然拉长,同样成立。
const CART_JOB = { timer: null, key: null, seq: 0, inflight: false };
const CART_PERIOD_MS = 150;   // 与关节点动默认发送周期一致

function cartSmoothOn() {
  return !!document.getElementById('cartSmooth')?.checked;
}

function cartTick() {
  const job = CART_JOB;
  if (!job.timer || !job.key || job.inflight) return;
  const key = job.key;
  const sign = key.endsWith('+') ? 1 : -1;
  const axis = key.slice(0, -1);
  const isRot = axis.length === 2;
  const p = cartParams();
  const req = isRot
    ? API.cartRotate({ arm: p.arm, axis,
                       angle_deg: sign * (+document.getElementById('cartRotStep').value || 5),
                       frame: p.frame })            // 速度:后端按该臂全局速度解析
    : API.cartJog({ arm: p.arm, axis, distance: sign * p.step_m,
                    frame: p.frame, step_m: p.step_m });
  job.inflight = true;
  const seq = ++job.seq;
  req.then(() => {
    job.inflight = false;
    if (seq !== job.seq || !job.timer) return;   // 过期响应,丢弃
    cartRefreshPose();
  }).catch(e => {
    job.inflight = false;
    stopCartJog();                               // 出错立即停,避免一直刷错误
    toast((isRot ? '旋转失败: ' : '平移失败: ') + e.message, 'error');
  });
}

function startCartJog(key, btn) {
  stopCartJog();                  // 同一时间只保留一个连续任务(多指按下取后者)
  CART_JOB.key = key;
  btn.classList.add('holding');
  cartTick();                     // 立即发第一帧,按下即动
  CART_JOB.timer = setInterval(cartTick, CART_PERIOD_MS);
}

function stopCartJog() {
  if (CART_JOB.timer) clearInterval(CART_JOB.timer);
  CART_JOB.timer = null;
  CART_JOB.key = null;
  CART_JOB.seq++;                // 让在途响应过期
  document.querySelectorAll('[data-cart].holding')
    .forEach(b => b.classList.remove('holding'));
}

// 单击单步(未勾选平滑移动时的原行为)
async function cartStepOnce(key) {
  const sign = key.endsWith('+') ? 1 : -1;
  const axis = key.slice(0, -1);
  const isRot = axis.length === 2;
  const p = cartParams();
  if (isRot) {
    const step = +document.getElementById('cartRotStep').value || 5;
    try {
      await API.cartRotate({ arm: p.arm, axis, angle_deg: sign * step,
                             frame: p.frame });
      cartRefreshPose();
    } catch (e) { toast('旋转失败: ' + e.message, 'error'); }
  } else {
    try {
      await API.cartJog({ arm: p.arm, axis, distance: sign * p.step_m,
                          frame: p.frame, step_m: p.step_m });
      cartRefreshPose();
    } catch (e) { toast('平移失败: ' + e.message, 'error'); }
  }
}

document.querySelectorAll('[data-cart]').forEach(btn => {
  const key = btn.dataset.cart;
  const stop = (e) => { e.preventDefault(); stopCartJog(); };
  btn.addEventListener('pointerdown', (e) => {
    if (!cartSmoothOn()) return;             // 未勾选:走 click 单步
    e.preventDefault();
    btn.setPointerCapture?.(e.pointerId);    // 捕获指针,移出按钮也能收到 up
    startCartJog(key, btn);
  });
  btn.addEventListener('pointerup', stop);
  btn.addEventListener('pointercancel', stop);
  btn.addEventListener('lostpointercapture', stop);
  // 键盘可达性:空格/回车按住同样连续移动
  btn.addEventListener('keydown', (e) => {
    if ((e.key === ' ' || e.key === 'Enter') && !e.repeat && cartSmoothOn()) {
      e.preventDefault();
      startCartJog(key, btn);
    }
  });
  btn.addEventListener('keyup', (e) => {
    if (e.key === ' ' || e.key === 'Enter') stopCartJog();
  });
  btn.addEventListener('blur', () => stopCartJog());
  btn.addEventListener('click', () => {
    if (cartSmoothOn()) return;              // 平滑模式由 pointerdown 接管,忽略合成 click
    cartStepOnce(key);
  });
});
// 按住中途切开关:立即停,新模式下次按下生效
document.getElementById('cartSmooth')?.addEventListener('change', stopCartJog);

document.getElementById('cartGoto').addEventListener('click', async () => {
  const p = cartParams();
  const pos = [(+document.getElementById('tgtX').value) / 1000,
               (+document.getElementById('tgtY').value) / 1000,
               (+document.getElementById('tgtZ').value) / 1000];
  const rpy = [+document.getElementById('tgtR').value,
               +document.getElementById('tgtP').value,
               +document.getElementById('tgtYaw').value];
  if (pos.some(Number.isNaN)) { toast('X/Y/Z 必填', 'error'); return; }
  try {
    await API.cartMoveTo({ arm: p.arm, pos, rpy });
    toast('已下发目标位姿', 'success');
    cartRefreshPose();
  } catch (e) { toast('目标移动失败: ' + e.message, 'error'); }
});

document.getElementById('cartFillCurrent').addEventListener('click', async () => {
  const arm = document.getElementById('cartArm').value;
  try {
    const p = await API.cartCurrent(arm);
    const pos = p.position || p.pos || [];
    const rpy = p.rpy || [];
    if (pos[0] !== undefined) document.getElementById('tgtX').value = (pos[0] * 1000).toFixed(1);
    if (pos[1] !== undefined) document.getElementById('tgtY').value = (pos[1] * 1000).toFixed(1);
    if (pos[2] !== undefined) document.getElementById('tgtZ').value = (pos[2] * 1000).toFixed(1);
    if (rpy[0] !== undefined) document.getElementById('tgtR').value = rpy[0].toFixed(2);
    if (rpy[1] !== undefined) document.getElementById('tgtP').value = rpy[1].toFixed(2);
    if (rpy[2] !== undefined) document.getElementById('tgtYaw').value = rpy[2].toFixed(2);
  } catch (e) { toast('获取位姿失败: ' + e.message, 'error'); }
});

document.querySelectorAll('[data-grip]').forEach(btn => {
  btn.addEventListener('click', async () => {
    const arm = document.getElementById('cartArm').value;
    const pos = btn.dataset.grip === 'open'
      ? +document.getElementById('gripOpen').value
      : +document.getElementById('gripClose').value;
    const speed = +document.getElementById('gripSpeed').value || 500;
    try { await API.gripper(arm, pos, speed); toast('夹爪 ' + btn.dataset.grip, 'success'); }
    catch (e) { toast('夹爪失败: ' + e.message, 'error'); }
    refreshEffector();
  });
});

// 末端探测:各臂当前挂的执行器类型(null=机器人未检测到,按钮会失败)
const EE_NAMES = { gripper: '二指夹爪', hand: '灵巧手' };
async function refreshEffector() {
  const el = document.getElementById('gripInfo');
  if (!el) return;
  try {
    const r = await fetch('/api/effector').then(x => x.json());
    el.textContent = Object.entries(r).map(([arm, info]) => {
      const nm = arm === 'armleft' ? '左臂' : '右臂';
      if (!info.type) return `${nm}: 未检测到末端 ✗`;
      return `${nm}: ${EE_NAMES[info.type] || info.type}` +
             (info.open !== undefined && info.open !== null ? ` (开口 ${Math.round(info.open)})` : '');
    }).join(' · ');
  } catch { el.textContent = '末端状态未知(未连接?)'; }
}

// ===== 关节示教 =====
const jt = { points: [], selected: -1 };

async function jtGetJoints() {
  const arm = document.getElementById('jtArm').value;
  try {
    const r = await API.cartCurrent(arm);  // 顺带取一下
  } catch {}
  // 关节角取自 status 缓存 / 接口;这里直接另发一个 status 请求
  const s = await API.status();
  return s.joints?.[arm] || [];
}

document.getElementById('jtRecord').addEventListener('click', async () => {
  try {
    const j = await jtGetJoints();
    if (!j.length) { toast('未取到关节角', 'error'); return; }
    jt.points.push({ joints_deg: j.slice(0, 7), gripper: null, note: '' });
    jt.selected = jt.points.length - 1;
    renderJt();
  } catch (e) { toast('记录失败: ' + e.message, 'error'); }
});

document.getElementById('jtClear').addEventListener('click', () => {
  if (!jt.points.length) return;
  if (!confirm('清空所有路点?')) return;
  jt.points = []; jt.selected = -1; renderJt();
});
document.getElementById('jtUp').addEventListener('click', () => jtMove(-1));
document.getElementById('jtDown').addEventListener('click', () => jtMove(1));
document.getElementById('jtDel').addEventListener('click', () => {
  if (jt.selected < 0) return;
  jt.points.splice(jt.selected, 1);
  jt.selected = Math.min(jt.selected, jt.points.length - 1);
  renderJt();
});
function jtMove(off) {
  if (jt.selected < 0) return;
  const j = jt.selected + off;
  if (j < 0 || j >= jt.points.length) return;
  [jt.points[jt.selected], jt.points[j]] = [jt.points[j], jt.points[jt.selected]];
  jt.selected = j; renderJt();
}

function renderJt() {
  const body = document.getElementById('jtBody');
  if (!jt.points.length) {
    body.innerHTML = '<tr><td colspan="5" class="muted">暂无路点</td></tr>';
    return;
  }
  body.innerHTML = jt.points.map((p, i) => {
    const j = (p.joints_deg || []).map(v => fmt(v, 1)).join(' ');
    const g = p.gripper === null || p.gripper === undefined ? '-' : p.gripper;
    const sel = i === jt.selected ? ' class="selected"' : '';
    return `<tr${sel} data-i="${i}">
      <td>${i+1}</td>
      <td>${j}</td>
      <td>${g}</td>
      <td>${p.note || ''}</td>
      <td class="action"></td>
    </tr>`;
  }).join('');
  body.querySelectorAll('tr').forEach(tr => {
    tr.addEventListener('click', () => {
      jt.selected = +tr.dataset.i; renderJt();
    });
  });
}

document.getElementById('jtSave').addEventListener('click', async () => {
  const name = document.getElementById('jtName').value.trim();
  if (!name) { toast('工程名必填', 'error'); return; }
  try {
    await API.teachSave({
      kind: 'joint_teach', name,
      arm: document.getElementById('jtArm').value,
      points: jt.points,
    });
    toast('已保存: ' + name, 'success');
    jtRefresh();
  } catch (e) { toast('保存失败: ' + e.message, 'error'); }
});

document.getElementById('jtPlay').addEventListener('click', async () => {
  if (!jt.points.length) { toast('无路点', 'error'); return; }
  try {
    await API.teachPlayJ({
      name: document.getElementById('jtName').value.trim(),
      arm: document.getElementById('jtArm').value,
    });
    toast('回放完成', 'success');
  } catch (e) { toast('回放失败: ' + e.message, 'error'); }
});

document.getElementById('jtLoad').addEventListener('click', async () => {
  const name = document.getElementById('jtLoadSel').value;
  if (!name) { toast('请选择工程', 'error'); return; }
  try {
    const t = await API.teachLoad('joint_teach', name);
    document.getElementById('jtName').value = t.name;
    document.getElementById('jtArm').value = t.arm;
    jt.points = t.points || [];
    jt.selected = jt.points.length - 1;
    renderJt();
    document.getElementById('jtStatus').textContent =
      `已加载: ${t.name} (${jt.points.length} 路点, 创建于 ${t.created})`;
  } catch (e) { toast('加载失败: ' + e.message, 'error'); }
});

document.getElementById('jtDelFile').addEventListener('click', async () => {
  const name = document.getElementById('jtLoadSel').value;
  if (!name) { toast('请选择工程', 'error'); return; }
  if (!confirm('删除工程 ' + name + '?')) return;
  try { await API.teachDelete('joint_teach', name); toast('已删除', 'success'); jtRefresh(); }
  catch (e) { toast('删除失败: ' + e.message, 'error'); }
});

async function jtRefresh() {
  try {
    const r = await API.teachList('joint_teach');
    const sel = document.getElementById('jtLoadSel');
    sel.innerHTML = '<option value="">(加载已有工程)</option>' +
      r.items.map(it => `<option value="${it.file}">${it.name} (${it.n_points}路点)</option>`).join('');
  } catch (e) { console.warn(e); }
}
document.getElementById('jtRefresh').addEventListener('click', jtRefresh);

// ===== 笛卡尔示教 =====
const ct = { points: [], selected: -1 };

async function ctRefreshCurrent() {
  const arm = document.getElementById('ctArm').value;
  try {
    const p = await API.cartCurrent(arm);
    document.getElementById('ctCurrentPose').textContent = fmtPose(p);
  } catch (e) {
    document.getElementById('ctCurrentPose').textContent = '获取失败: ' + e.message;
  }
}
document.getElementById('ctArm').addEventListener('change', ctRefreshCurrent);

document.getElementById('ctRecord').addEventListener('click', async () => {
  const arm = document.getElementById('ctArm').value;
  try {
    const p = await API.cartCurrent(arm);
    const pos = p.position || p.pos;
    const ori = p.orientation || p.quat;
    if (!pos || !ori) { toast('末端位姿数据不完整', 'error'); return; }
    ct.points.push({ position: [...pos], orientation: [...ori],
                     gripper: null, note: '' });
    ct.selected = ct.points.length - 1;
    renderCt();
  } catch (e) { toast('记录失败: ' + e.message, 'error'); }
});

document.getElementById('ctClear').addEventListener('click', () => {
  if (!ct.points.length) return;
  if (!confirm('清空所有路点?')) return;
  ct.points = []; ct.selected = -1; renderCt();
});
document.getElementById('ctUp').addEventListener('click', () => ctMove(-1));
document.getElementById('ctDown').addEventListener('click', () => ctMove(1));
document.getElementById('ctDel').addEventListener('click', () => {
  if (ct.selected < 0) return;
  ct.points.splice(ct.selected, 1);
  ct.selected = Math.min(ct.selected, ct.points.length - 1);
  renderCt();
});
function ctMove(off) {
  if (ct.selected < 0) return;
  const j = ct.selected + off;
  if (j < 0 || j >= ct.points.length) return;
  [ct.points[ct.selected], ct.points[j]] = [ct.points[j], ct.points[ct.selected]];
  ct.selected = j; renderCt();
}

function renderCt() {
  const body = document.getElementById('ctBody');
  if (!ct.points.length) {
    body.innerHTML = '<tr><td colspan="6" class="muted">暂无路点</td></tr>';
    return;
  }
  body.innerHTML = ct.points.map((p, i) => {
    const sel = i === ct.selected ? ' class="selected"' : '';
    const pos = (p.position || []).map(v => fmt(v, 3)).join(' ');
    const ori = (p.orientation || []).map(v => fmt(v, 3)).join(' ');
    const g = p.gripper === null || p.gripper === undefined ? '-' : p.gripper;
    return `<tr${sel} data-i="${i}">
      <td>${i+1}</td>
      <td>${pos}</td>
      <td>${ori}</td>
      <td>${g}</td>
      <td>${p.note || ''}</td>
      <td class="action"></td>
    </tr>`;
  }).join('');
  body.querySelectorAll('tr').forEach(tr => {
    tr.addEventListener('click', () => { ct.selected = +tr.dataset.i; renderCt(); });
  });
}

document.getElementById('ctSave').addEventListener('click', async () => {
  const name = document.getElementById('ctName').value.trim();
  if (!name) { toast('工程名必填', 'error'); return; }
  try {
    await API.teachSave({
      kind: 'cartesian_teach', name,
      arm: document.getElementById('ctArm').value,
      points: ct.points,
    });
    toast('已保存: ' + name, 'success');
    ctRefresh();
  } catch (e) { toast('保存失败: ' + e.message, 'error'); }
});

document.getElementById('ctPlay').addEventListener('click', async () => {
  if (!ct.points.length) { toast('无路点', 'error'); return; }
  try {
    await API.teachPlayC({
      name: document.getElementById('ctName').value.trim(),
      arm: document.getElementById('ctArm').value,
    });
    toast('回放完成', 'success');
  } catch (e) { toast('回放失败: ' + e.message, 'error'); }
});

document.getElementById('ctLoad').addEventListener('click', async () => {
  const name = document.getElementById('ctLoadSel').value;
  if (!name) { toast('请选择工程', 'error'); return; }
  try {
    const t = await API.teachLoad('cartesian_teach', name);
    document.getElementById('ctName').value = t.name;
    document.getElementById('ctArm').value = t.arm;
    ct.points = t.points || [];
    ct.selected = ct.points.length - 1;
    renderCt();
    document.getElementById('ctStatus').textContent =
      `已加载: ${t.name} (${ct.points.length} 路点, 创建于 ${t.created})`;
  } catch (e) { toast('加载失败: ' + e.message, 'error'); }
});

document.getElementById('ctDelFile').addEventListener('click', async () => {
  const name = document.getElementById('ctLoadSel').value;
  if (!name) { toast('请选择工程', 'error'); return; }
  if (!confirm('删除工程 ' + name + '?')) return;
  try { await API.teachDelete('cartesian_teach', name); toast('已删除', 'success'); ctRefresh(); }
  catch (e) { toast('删除失败: ' + e.message, 'error'); }
});

async function ctRefresh() {
  try {
    const r = await API.teachList('cartesian_teach');
    const sel = document.getElementById('ctLoadSel');
    sel.innerHTML = '<option value="">(加载已有工程)</option>' +
      r.items.map(it => `<option value="${it.file}">${it.name} (${it.n_points}路点)</option>`).join('');
  } catch (e) { console.warn(e); }
}
document.getElementById('ctRefresh').addEventListener('click', ctRefresh);

// ===== 点位复现 =====
const pl = { arm: 'armleft' };

async function plRefresh() {
  const arm = document.getElementById('plArm').value;
  const list = document.getElementById('plList');
  list.innerHTML = '<li class="muted">加载中...</li>';
  try {
    const r = await API.poseList(arm);
    const names = r.names || [];
    if (!names.length) {
      list.innerHTML = '<li class="muted">该臂暂无保存点位</li>';
      if (r.warning) toast(r.warning, 'error');
      return;
    }
    list.innerHTML =
      (r.warning ? `<li class="muted warn-line">⚠ ${r.warning}</li>` : '') +
      names.map(n => `
      <li>
        <div>
          <div class="pl-name">${n}</div>
          <div class="pl-meta">arm: ${arm}</div>
        </div>
        <div class="pl-actions">
          <button data-pl-goto="${n}">▶ 直达</button>
          <button data-pl-del="${n}">删除</button>
        </div>
      </li>
    `).join('');
    list.querySelectorAll('[data-pl-goto]').forEach(b =>
      b.addEventListener('click', (e) => { e.stopPropagation(); plGoto(b.dataset.plGoto); }));
    list.querySelectorAll('[data-pl-del]').forEach(b =>
      b.addEventListener('click', (e) => { e.stopPropagation(); plDel(b.dataset.plDel); }));
    list.querySelectorAll('.pl-name').forEach(pn => {
      pn.closest('li').addEventListener('dblclick', () => plGoto(pn.textContent));
    });
    if (r.warning) toast(r.warning, 'error');
  } catch (e) { list.innerHTML = '<li class="muted">加载失败: ' + e.message + '</li>'; }
}

async function plGoto(name) {
  if (!confirm('直达点位 ' + name + '?')) return;
  const arm = document.getElementById('plArm').value;
  try { await API.poseGoto({ arm, name }); toast('已下发: ' + name, 'success'); }
  catch (e) { toast('失败: ' + e.message, 'error'); }
}

async function plDel(name) {
  if (!confirm('删除点位 ' + name + '?')) return;
  const arm = document.getElementById('plArm').value;
  try { await API.poseDelete({ arm, name }); toast('已删除', 'success'); plRefresh(); }
  catch (e) { toast('失败: ' + e.message, 'error'); }
}

document.getElementById('plArm').addEventListener('change', plRefresh);
document.getElementById('plRefresh').addEventListener('click', plRefresh);
document.getElementById('plSave').addEventListener('click', async () => {
  const arm = document.getElementById('plArm').value;
  const name = document.getElementById('plName').value.trim();
  if (!name) { toast('点位名必填', 'error'); return; }
  try { await API.poseSave({ arm, name }); toast('已保存: ' + name, 'success'); plRefresh(); }
  catch (e) { toast('保存失败: ' + e.message, 'error'); }
});

// ===== 物料检测 =====
async function pdCheck() {
  try {
    const r = await API.poseSvcHealth();
    const el = document.getElementById('pdSvcStatus');
    if (r.alive) {
      el.textContent = '运行中';
      el.className = 'badge on';
    } else {
      el.textContent = '未启动' + (r.reason ? ` (${r.reason})` : '');
      el.className = 'badge off';
    }
  } catch (e) {
    document.getElementById('pdSvcStatus').textContent = '探测失败';
  }
}

document.getElementById('pdSvcStart').addEventListener('click', async () => {
  try { await API.poseSvcStart(); toast('启动中,稍候...', 'success'); setTimeout(pdCheck, 3000); }
  catch (e) { toast('启动失败: ' + e.message, 'error'); }
});

document.getElementById('pdSvcStop').addEventListener('click', async () => {
  try { await API.poseSvcStop(); toast('已停止', 'success'); pdCheck(); }
  catch (e) { toast('停止失败: ' + e.message, 'error'); }
});

let pdTimer = null;
document.getElementById('pdLoop').addEventListener('change', (e) => {
  if (e.target.checked) {
    const ms = (+document.getElementById('pdInterval').value || 1.5) * 1000;
    pdTimer = setInterval(pdDetectOnce, ms);
  } else {
    clearInterval(pdTimer); pdTimer = null;
  }
});
document.getElementById('pdInterval').addEventListener('change', () => {
  if (document.getElementById('pdLoop').checked) {
    clearInterval(pdTimer);
    const ms = (+document.getElementById('pdInterval').value || 1.5) * 1000;
    pdTimer = setInterval(pdDetectOnce, ms);
  }
});

document.getElementById('pdDetect').addEventListener('click', pdDetectOnce);
async function pdDetectOnce() {
  const cam = document.getElementById('pdCam').value;
  const conf = +document.getElementById('pdConf').value || 0.25;
  try {
    const r = await API.poseDetect({ cam, conf });
    if (r.image_b64) {
      document.getElementById('pdImg').src = 'data:image/jpeg;base64,' + r.image_b64;
    }
    document.getElementById('pdJson').textContent =
      JSON.stringify(r.detections || [], null, 2);
  } catch (e) { toast('检测失败: ' + e.message, 'error'); }
}

// 启动相机后才能检测:轮询健康
setInterval(pdCheck, 5000);
pdCheck();

// ===== 一步到位(吸取) =====
let pkPolling = false;

function pkBadge(id, ok, text) {
  const el = document.getElementById(id);
  el.textContent = text;
  el.className = 'badge ' + (ok ? 'on' : 'off');
}

async function pkRefreshCfg() {
  try {
    const r = await API.pkConfigGet();
    const c = r.config || {};
    document.getElementById('pkRole').value = c.cam_role || 'left';
    document.getElementById('pkConf').value = c.conf ?? 0.5;
    document.getElementById('pkMotion').value = c.motion_mode || 'direct';
    document.getElementById('pkPress').value = c.press_mm ?? 0;
    document.getElementById('pkHold').value = c.hold_s ?? 2;
    document.getElementById('pkVacuum').value = (c.vacuum || {}).mode || 'manual';
    pkBadge('pkScanBadge', !!c.scan_pose,
            c.scan_pose ? `扫描位:${c.scan_pose.created}` : '扫描位:未示教');
    const al = c.align;
    pkBadge('pkAlignBadge', !!al,
            al ? `对准:残差(${(c.residual_mm || []).join(',')})mm` : '对准:未校正');
    const he = r.handeye || {};
    pkBadge('pkHandeyeBadge', !!he.ok, he.ok ? `手眼:${he.time || ''}` : '手眼:缺文件');
    if (r.role_warning) {
      document.getElementById('pkScanInfo').textContent = '⚠ ' + r.role_warning;
    }
  } catch (e) { /* 配置接口异常不阻塞页面 */ }
}

document.getElementById('pkSaveCfg').addEventListener('click', async () => {
  const body = {
    cam_role: document.getElementById('pkRole').value,
    conf: +document.getElementById('pkConf').value,
    motion_mode: document.getElementById('pkMotion').value,
    press_mm: +document.getElementById('pkPress').value,
    hold_s: +document.getElementById('pkHold').value,
    vacuum: { mode: document.getElementById('pkVacuum').value },
  };
  try {
    await API.pkConfigSet(body);
    toast('设置已保存', 'success');
  } catch (e) { toast('保存失败: ' + e.message, 'error'); }
});

// —— 隐藏残差微调:三连击「对准」徽章呼出(无显式入口,防随意修改)——
(function () {
  const badge = document.getElementById('pkAlignBadge');
  const panel = document.getElementById('pkResidPanel');
  if (!badge || !panel) return;
  let cur = [0, 0, 0], clicks = 0, t0 = 0;
  const render = () => panel.querySelectorAll('[data-ax]').forEach(el => {
    el.querySelector('.pk-rv').textContent =
      (+cur['xyz'.indexOf(el.dataset.ax)]).toFixed(2);
  });
  panel.querySelectorAll('.pk-rb').forEach(b =>
    b.addEventListener('click', async () => {
      const i = 'xyz'.indexOf(b.closest('[data-ax]').dataset.ax);
      cur[i] = +(cur[i] + parseFloat(b.dataset.d)).toFixed(3);
      render();
      try {
        await API.pkConfigSet({ residual_mm: cur });
        pkRefreshCfg();               // 同步徽章显示
      } catch (e) { toast('残差保存失败: ' + e.message, 'error'); }
    }));
  document.getElementById('pkResidClose').addEventListener('click',
    () => panel.style.display = 'none');
  badge.addEventListener('click', () => {
    const now = Date.now();
    clicks = now - t0 < 1200 ? clicks + 1 : 1;
    t0 = now;
    if (clicks < 3) return;
    clicks = 0;
    if (panel.style.display === 'block') { panel.style.display = 'none'; return; }
    API.pkConfigGet().then(r => { cur = r.config.residual_mm || [0, 0, 0]; render(); });
    panel.style.display = 'block';
  });
})();

document.getElementById('pkTeachScan').addEventListener('click', async () => {
  if (!confirm('把当前臂位姿存为扫描位?(不动臂)')) return;
  try {
    const r = await API.pkTeachScan();
    toast('扫描位已存: ' + r.scan_pose.created, 'success');
    pkRefreshCfg();
  } catch (e) { toast('存扫描位失败: ' + e.message, 'error'); }
});

function pkRenderRows(rows) {
  const tb = document.querySelector('#pkBagTable tbody');
  tb.innerHTML = '';
  rows.forEach(row => {
    const tr = document.createElement('tr');
    const p = row.p_base || [];
    const mm = p.map(v => (v * 1000).toFixed(0));
    tr.innerHTML =
      `<td>#${row.order}</td><td>L${row.layer}</td><td>${row.cls}</td>` +
      `<td>${(row.conf ?? 0).toFixed(2)}</td><td>${row.z_mm ?? '-'}</td>` +
      `<td>(${mm[0]}, ${mm[1]}, ${mm[2]})</td>` +
      `<td class="action">` +
      `<button class="pk-dry" data-n="${row.order}">干跑</button> ` +
      `<button class="pk-go primary" data-n="${row.order}">吸取</button></td>`;
    tr.querySelector('.pk-dry').addEventListener('click', () => pkStart(row.order, true));
    tr.querySelector('.pk-go').addEventListener('click', () => {
      if (confirm(`实吸 #${row.order}?臂将移到袋上方并下降到位(先确认已干跑看过计划)`)) {
        pkStart(row.order, false);
      }
    });
    tb.appendChild(tr);
  });
  document.getElementById('pkBagTable').style.display = '';
}

document.getElementById('pkScan').addEventListener('click', async () => {
  const replay = !document.getElementById('pkScanNoReplay').checked;
  const info = document.getElementById('pkScanInfo');
  info.textContent = replay ? '扫描中(先回放扫描位,~10s)…' : '扫描中…';
  try {
    const r = await API.pkScan({ replay });
    pkRenderRows(r.rows || []);
    if (r.image_b64) {
      document.getElementById('pdImg').src = 'data:image/jpeg;base64,' + r.image_b64;
    }
    info.textContent = `扫描完成:${(r.rows || []).length} 袋` +
                       (r.synced === false ? '(⚠ 深度帧未同步)' : '');
    toast(`扫描到 ${(r.rows || []).length} 袋`, 'success');
  } catch (e) {
    info.textContent = '扫描失败: ' + e.message;
    toast('扫描失败: ' + e.message, 'error');
  }
});

async function pkStart(bagIndex, dryRun) {
  try {
    await API.pkStart({ bag_index: bagIndex, dry_run: dryRun });
    toast(dryRun ? `干跑 #${bagIndex} 已开始` : `吸取 #${bagIndex} 已开始`, 'success');
    pkPollStatus();
  } catch (e) { toast((dryRun ? '干跑' : '吸取') + '失败: ' + e.message, 'error'); }
}

function pkPollStatus() {
  if (pkPolling) return;
  pkPolling = true;
  const timer = setInterval(async () => {
    let s;
    try { s = await API.pkStatus(); } catch (e) { return; }
    const phaseEl = document.getElementById('pkPhase');
    const running = s.running;
    phaseEl.textContent = `${s.phase} ${s.pct ?? 0}%`;
    phaseEl.className = 'badge ' + (running ? 'warn'
      : (s.phase === 'done' ? 'on' : (s.phase === 'idle' ? 'off' : 'danger')));
    document.getElementById('pkBar').style.width = (s.pct || 0) + '%';
    const logEl = document.getElementById('pkLog');
    if (Array.isArray(s.log)) logEl.textContent = s.log.join('\n');
    if (!running) {
      clearInterval(timer);
      pkPolling = false;
      if (s.phase === 'failed') toast('吸取任务失败: ' + (s.error || ''), 'error');
      else if (s.phase === 'done') toast('任务完成 ✓', 'success');
      else if (s.phase === 'aborted') toast('已停止(段边界)', 'success');
      pkRefreshCfg();
    }
  }, 1000);
}

document.getElementById('pkStop').addEventListener('click', async () => {
  try { await API.pkStop(); toast('已请求软停', 'success'); }
  catch (e) { toast('停止失败: ' + e.message, 'error'); }
});

document.getElementById('pkTeachAlign').addEventListener('click', async () => {
  const n = +document.getElementById('pkAlignBag').value;
  const perBag = document.getElementById('pkAlignPerBag').checked;
  if (!confirm(`校正袋 #${n}?当前臂位姿将被存为正确吸附点` +
               (perBag ? '(仅此袋)' : '(全局残差)'))) return;
  try {
    const r = await API.pkTeachAlign({ bag_order: n, per_bag_only: perBag });
    toast(`校正完成: 残差(${r.residual_mm.join(', ')})mm → ${r.stored_as}`, 'success');
    pkRefreshCfg();
  } catch (e) { toast('校正失败: ' + e.message, 'error'); }
});

// 深度健康徽章轮询(5s; 只有会话启动后才有数据)
setInterval(async () => {
  try {
    const h = await API.pkDepthHealth();
    const el = document.getElementById('pkDepthBadge');
    if (h.running) {
      el.textContent = `深度:${h.role}${h.lease ? '' : '(无租约)'} ` +
                       `彩${h.color_age_s ?? '-'}s/深${h.depth_age_s ?? '-'}s`;
      el.className = 'badge ' + (h.lease && h.color_age_s !== null && h.color_age_s < 3
        ? 'on' : 'warn');
    } else {
      el.textContent = '深度:未启动';
      el.className = 'badge off';
    }
  } catch (e) { /* ignore */ }
}, 5000);

pkRefreshCfg();
pkPollStatus();      // 页面加载即拉一次(可能上次任务还在)

// ===== Camera =====
const camImgs = {}, camMeta = {};
let camMetaTimer = null;   // 帧来源/分辨率指示刷新
document.querySelectorAll('.cam-cell').forEach(cell => {
  camImgs[cell.dataset.cam] = cell.querySelector('.cam-img');
  camMeta[cell.dataset.cam] = cell.querySelector('.cam-meta');
});
document.getElementById('btnCamApply').addEventListener('click', applyCam);
document.getElementById('btnCamSnap').addEventListener('click', async () => {
  const cam = document.querySelector('.cam:checked')?.value || 'left';
  try {
    const r = await fetch(API.camSnapUrl(cam));
    if (!r.ok) throw new Error(await r.text());
    const blob = await r.blob();
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url; a.download = `${cam}_${Date.now()}.jpg`;
    a.click(); URL.revokeObjectURL(url);
    toast('已保存抓拍', 'success');
  } catch (e) { toast('抓拍失败: ' + e.message, 'error'); }
});

async function applyCam() {
  const fps = Math.max(1, Math.min(20, +document.getElementById('camFps').value || 8));
  const checked = [...document.querySelectorAll('.cam:checked')].map(c => c.value);
  await stopAllCameras();
  for (const cam of checked) {
    try {
      await API.camStart(cam, Math.round(1000 / fps));
      camImgs[cam].src = API.camStreamUrl(cam, fps);
      camMeta[cam].textContent = `${cam}: 拉流中 (fps=${fps})`;
    } catch (e) { camMeta[cam].textContent = `${cam}: 启动失败 - ${e.message}`; }
  }
  document.getElementById('camInfo').textContent =
    checked.length ? `已启动 ${checked.join('/')} @ ${fps}fps` : '全部停止';
  // 帧来源指示:ros=ROS 话题原生(无畸变) / none=无信号占位(桥接无帧)
  clearInterval(camMetaTimer);
  camMetaTimer = setInterval(refreshCamMeta, 2000);
  refreshCamMeta();
}
async function refreshCamMeta() {
  try {
    const m = await API.cameraMeta();
    for (const [cam, info] of Object.entries(m.cams || {})) {
      if (!camMeta[cam] || !info.ts) continue;
      const res = info.w ? ` ${info.w}×${info.h}` : '';
      const src = info.src === 'ros' ? 'ROS原生'
                : info.src === 'none' ? '无信号' : info.src;
      camMeta[cam].textContent =
        `${cam}: ${src}${res} ${info.kb || '?'}KB` +
        (info.fps !== undefined ? ` 话题${info.fps}fps` : '');
    }
    if (m.bridge === false) {
      document.getElementById('camInfo').textContent +=
        ' ⚠ 相机桥接未运行,正在自动拉起,画面为无信号占位';
    }
  } catch {}
}
async function stopAllCameras() {
  clearInterval(camMetaTimer);
  camMetaTimer = null;
  for (const cam of ['left', 'head', 'right']) {
    try { await API.camStop(cam); } catch {}
    camImgs[cam].src = '';
    camMeta[cam].textContent = '';
  }
}

// ===== Chassis =====
const $chStatus = document.getElementById('chStatus');
const $mapImg = document.getElementById('mapImg');

document.getElementById('btnChConn').addEventListener('click', async () => {
  const host = document.getElementById('chassisHost').value.trim();
  const port = +document.getElementById('chassisPort').value || 14444;
  if (!host) { toast('请输入底盘 IP', 'error'); return; }
  try {
    await API.chConn(host, port);
    $chStatus.textContent = '已连接';
    $chStatus.className = 'badge on';
    toast('底盘已连接', 'success');
    localStorage.setItem('chassisHost', host);
    localStorage.setItem('chassisPort', String(port));
    startChassisLoop();
    refreshMap();
  } catch (e) { toast('底盘连接失败: ' + e.message, 'error'); }
});
document.getElementById('btnChDisc').addEventListener('click', async () => {
  try { await API.chDisc(); } catch {}
  $chStatus.textContent = '未连接';
  $chStatus.className = 'badge off';
  $mapImg.src = '';
  stopChassisLoop();
});
document.getElementById('btnChCancel').addEventListener('click', async () => {
  try { await API.chCancel(); toast('已取消动作', 'success'); }
  catch (e) { toast('取消失败: ' + e.message, 'error'); }
});
document.getElementById('btnChRefreshMap').addEventListener('click', () => refreshMap(true));
document.getElementById('btnChMove').addEventListener('click', async () => {
  const x = parseFloat(document.getElementById('mvX').value);
  const y = parseFloat(document.getElementById('mvY').value);
  const yaw = document.getElementById('mvYaw').value;
  if (Number.isNaN(x) || Number.isNaN(y)) { toast('请输入 X/Y', 'error'); return; }
  try { await API.chMoveTo(x, y, yaw === '' ? null : parseFloat(yaw)); toast('已下发导航', 'success'); }
  catch (e) { toast('导航失败: ' + e.message, 'error'); }
});
document.getElementById('btnChRot').addEventListener('click', async () => {
  const a = parseFloat(document.getElementById('rotAng').value);
  if (Number.isNaN(a)) { toast('请输入角度', 'error'); return; }
  try { await API.chRotate(a); toast('已下发旋转', 'success'); }
  catch (e) { toast('旋转失败: ' + e.message, 'error'); }
});
document.getElementById('btnChHome').addEventListener('click', async () => {
  const dock = document.getElementById('dockCheck').checked;
  try { await API.chHome(dock); toast('已下发回桩', 'success'); }
  catch (e) { toast('回桩失败: ' + e.message, 'error'); }
});
document.querySelectorAll('.joystick button').forEach(btn => {
  btn.addEventListener('click', async () => {
    const dir = parseInt(btn.dataset.dir, 10);
    const dur = parseInt(document.getElementById('jsDur').value, 10) || 500;
    try { await API.chJoystick(dir, dur); }
    catch (e) { toast('遥控失败: ' + e.message, 'error'); }
  });
});

$mapImg.addEventListener('click', async (e) => {
  if (!$chStatus.classList.contains('on')) { toast('底盘未连接', 'error'); return; }
  const rect = $mapImg.getBoundingClientRect();
  const px = (e.clientX - rect.left) / rect.width;
  const py = (e.clientY - rect.top) / rect.height;
  const x = +(px - 0.5).toFixed(2);
  const y = +((1 - py) - 0.5).toFixed(2);
  if (!confirm(`导航到 (${x}, ${y})?`)) return;
  try { await API.chMoveTo(x, y, null); toast('已下发导航', 'success'); }
  catch (err) { toast('导航失败: ' + err.message, 'error'); }
});

async function refreshMap(force = false) {
  if (!$chStatus.classList.contains('on')) return;
  try {
    const blob = await API.chMap(force);
    if ($mapImg.src) URL.revokeObjectURL($mapImg.src);
    $mapImg.src = URL.createObjectURL(blob);
  } catch (e) { console.warn('地图加载失败:', e); }
}

let chTimer = null;
function startChassisLoop() {
  stopChassisLoop();
  chTimer = setInterval(async () => {
    try {
      const p = await API.chPose();
      const l = await API.chLaser();
      if (p && typeof p === 'object') {
        document.getElementById('poseLine').textContent =
          `x=${fmt(p.x, 2)} y=${fmt(p.y, 2)} yaw=${fmt(p.yaw, 3)}`;
      }
      document.getElementById('laserCount').textContent = (l.laser_points || []).length;
    } catch {}
  }, 1000);
}
function stopChassisLoop() { if (chTimer) { clearInterval(chTimer); chTimer = null; } }

const savedChHost = localStorage.getItem('chassisHost');
const savedChPort = localStorage.getItem('chassisPort');
if (savedChHost) document.getElementById('chassisHost').value = savedChHost;
if (savedChPort) document.getElementById('chassisPort').value = savedChPort;

// ===== 双臂流水线 =====
let flCfg = null;
let flPolling = false;
// PBVS 对准自动插在这些动作之后(列表里用绿色分隔行标出)
const FL_GATE_AFTER = {
  armleft:  { A1_transfer: 'PBVS 对准 · 中间件' },
  armright: { B1_approach: 'PBVS 对准 · 中间件',
              B3_transfer: 'PBVS 对准 · 料仓' },
};
// 左臂 A1 之前的视觉吸取序列(全自动,无需示教;渲染在动作列表最上方)
const FL_PICK_STEPS = [
  ['① 物料检测', '回扫描位 → 取帧 → YOLO+深度 → 基座系定位/分层,选最高层队首'],
  ['② 移到物料上方+下降', '路径沿用『物料检测』页设置: 一键直达=一段直线(不可达自动退分段走廊)/ 分段走廊=升越顶→转姿态→水平→降'],
  ['③ 下压', '沿吸嘴轴向(工具Z+)压入 press_mm(页面设置 0 则跳过)'],
  ['④ 吸取', '保持窗口内人工接真空(方式沿用『物料检测』页设置),吸完 A1 直接接管'],
];
// 运行徽章的 phase → 中文标签(左臂吸取子步 + 示教动作 + 等待)
const FL_PHASE_LABEL = {
  scan: '① 物料检测', pick_hover: '② 移到物料上方', pick_descend: '③ 下降',
  pick_press: '③ 下压', pick_grab: '④ 吸取·真空窗口',
  A1_transfer: 'A1 转移', A2_place: 'A2 放中间件',
  B0_pre: 'B0 预备', B1_approach: 'B1 到中间件', B2_pick: 'B2 取料',
  B3_transfer: 'B3 转移料仓', B4_place: 'B4 放料仓',
  gate_align: 'PBVS对准·中间件', gate_align2: 'PBVS对准·料仓',
  wait_empty: '等中间件空', wait_filled: '等中间件有料', wait_center: '等中心区空闲',
  idle: '空闲', aborted: '已软停', failed: '失败',
};

function flBadge(el, ok, text) {
  el.textContent = text;
  el.className = 'badge ' + (ok ? 'on' : 'off');
}

async function flRefresh() {
  try { flRender(await API.flConfigGet()); }
  catch (e) { toast('读取流水线配置失败: ' + e.message, 'error'); }
}

function flRender(r) {
  flCfg = r;
  const heL = r.handeye.armleft, heR = r.handeye.armright;
  flBadge(document.getElementById('flHeLeft'), heL.ok,
          '左臂手眼:' + (heL.ok ? '已标定' : '缺失'));
  flBadge(document.getElementById('flHeRight'), heR.ok,
          '右臂手眼:' + (heR.ok ? '已标定' : (heR.hint || '缺失')));
  flBadge(document.getElementById('flPickPose'), r.pick_scan_pose,
          '左臂扫描位:' + (r.pick_scan_pose ? '已示教' : '未示教'));
  const ul = document.getElementById('flReady');
  const miss = r.readiness.missing || [], warn = r.readiness.warnings || [];
  ul.innerHTML = (miss.length || warn.length)
    ? [...miss.map(m => `⛔ ${m}`), ...warn.map(m => `⚠ ${m}`)]
        .map(s => `<li>${s}</li>`).join('')
    : '<li>✅ 全部就绪,可启动</li>';
  const g = r.config.gate;
  document.getElementById('flGateMode').value = g.mode;
  document.getElementById('flTolXY').value = g.tol_xy_mm;
  document.getElementById('flTolZ').value = g.tol_z_mm;
  document.getElementById('flMaxCorr').value = g.max_corr_mm;
  document.getElementById('flIter').value = g.iterations;
  document.getElementById('flFrames').value = g.frames;
  const la = r.config.arms.armleft.align, ra = r.config.arms.armright;
  document.getElementById('flEnL').checked = la.enabled !== false;
  document.getElementById('flEnR1').checked = ra.align.enabled !== false;
  document.getElementById('flEnR2').checked = ra.align2.enabled !== false;
  document.getElementById('flLMkType').value = la.marker.type;
  document.getElementById('flLMkId').value = la.marker.id ?? 0;
  document.getElementById('flLMkData').value = la.marker.data ?? '';
  document.getElementById('flLMkSize').value = la.marker_size_mm;
  document.getElementById('flRMk1Type').value = ra.align.marker.type;
  document.getElementById('flRMk1Id').value = ra.align.marker.id ?? 0;
  document.getElementById('flRMk1Data').value = ra.align.marker.data ?? '';
  document.getElementById('flRMk1Size').value = ra.align.marker_size_mm;
  document.getElementById('flRMk2Type').value = ra.align2.marker.type;
  document.getElementById('flRMk2Id').value = ra.align2.marker.id ?? 0;
  document.getElementById('flRMk2Data').value = ra.align2.marker.data ?? '';
  document.getElementById('flRMk2Size').value = ra.align2.marker_size_mm;
  flRenderGrips(r);
  flRefLine('flRefInfoL', [['中间件', la.ref]]);
  flRefLine('flRefInfoR', [['中间件', ra.align.ref], ['料仓', ra.align2.ref]]);
  const snapL = la.ref && la.ref.snapshot;
  const imL = document.getElementById('flRefSnapL');
  imL.style.display = snapL ? '' : 'none';
  if (snapL) imL.src = snapL;
  const snapR = (ra.align2.ref && ra.align2.ref.snapshot) ||
                (ra.align.ref && ra.align.ref.snapshot);
  const imR = document.getElementById('flRefSnapR');
  imR.style.display = snapR ? '' : 'none';
  if (snapR) imR.src = snapR;
  flRenderActions('armleft', 'flActsLeft', r);
  flRenderActions('armright', 'flActsRight', r);
}

function flRefLine(id, items) {
  document.getElementById(id).textContent = items.map(([w, ref]) => {
    if (!ref) return `${w}参考:未记录`;
    const p = (ref.p_base || []).map(v => (v * 1000).toFixed(0)).join(',');
    return `${w}参考: ${ref.key || '?'} [${p}]mm z_cam=${ref.z_cam_mm}mm (${ref.created})`;
  }).join('  |  ');
}

// 从 UI 读一组码配置: ArUco → {type,id}; QR → {type,data}(留空=任意一张)
function flMarker(typeId, idId, dataId) {
  const t = document.getElementById(typeId).value;
  return t === 'qr'
    ? { type: 'qr', data: document.getElementById(dataId).value.trim() }
    : { type: 'aruco', id: +document.getElementById(idId).value };
}

function flRenderActions(arm, containerId, r) {
  const el = document.getElementById(containerId);
  el.innerHTML = '';
  // 左臂: 视觉吸取序列(①-⑤ 全自动)显示在 A1 之前,与 PBVS 分隔行同款风格
  if (arm === 'armleft') {
    const box = document.createElement('div');
    box.style.cssText =
      'border:1px dashed #6dd49a;border-radius:6px;padding:6px 10px;' +
      'margin:4px 0 8px;font-size:13px;line-height:1.8;';
    box.innerHTML =
      '<div style="color:#6dd49a;">👁 视觉吸取(自动 · 无需示教,' +
      '参数沿用『物料检测』页)</div>' +
      FL_PICK_STEPS.map(([t, d]) =>
        `<div><b>${t}</b> <span class="muted">— ${d}</span></div>`).join('') +
      '<div class="muted">↓ 吸住后进入示教动作 A1</div>';
    el.appendChild(box);
  }
  const acts = r.config.arms[arm].actions;
  for (const name of Object.keys(acts)) {
    const a = acts[name];
    const vac = a.vacuum;
    const div = document.createElement('div');
    div.className = 'row';
    div.style.cssText = 'margin:4px 0;align-items:center;flex-wrap:wrap;gap:6px;';
    div.innerHTML =
      `<b style="min-width:64px;">${name.split('_')[0]}</b>` +
      `<input data-op="title" data-arm="${arm}" data-act="${name}" value="${(a.title || '').replace(/"/g, '&quot;')}"` +
      ` title="步骤说明,可直接修改(回车或点别处保存)"` +
      ` style="flex:1;min-width:220px;color:#8aa7b0;background:transparent;border:1px dashed #456;border-radius:4px;padding:2px 6px;" />` +
      `<span class="badge ${a.waypoints.length ? 'on' : 'off'}">${a.waypoints.length}点</span>` +
      `<button class="primary" data-op="add" data-arm="${arm}" data-act="${name}">📌加路点</button>` +
      `<button data-op="del" data-arm="${arm}" data-act="${name}">🗑删末点</button>` +
      `<button data-op="clear" data-arm="${arm}" data-act="${name}">🧹清空</button>` +
      `<button data-op="play" data-arm="${arm}" data-act="${name}">▶试跑</button>` +
      `<label>真空
         <select data-op="vev" data-arm="${arm}" data-act="${name}">
           <option value="none"${!vac ? ' selected' : ''}>无</option>
           <option value="grab"${vac && vac.event === 'grab' ? ' selected' : ''}>吸取</option>
           <option value="release"${vac && vac.event === 'release' ? ' selected' : ''}>释放</option>
         </select></label>` +
      `<label>@
         <select data-op="vat" data-arm="${arm}" data-act="${name}">
           <option value="end"${!vac || vac.at === 'end' ? ' selected' : ''}>终点后</option>
           <option value="start"${vac && vac.at === 'start' ? ' selected' : ''}>起点前</option>
           <option value="k"${vac && !['start','end'].includes(String(vac.at)) ? ' selected' : ''}>第k点后</option>
         </select></label>` +
      `<label>k <input type="number" min="0" step="1" size="2" value="${vac && !['start','end'].includes(String(vac.at)) ? vac.at : 0}"
               data-op="vk" data-arm="${arm}" data-act="${name}"></label>`;
    el.appendChild(div);
    // ---- 逐点编辑: 覆盖(当前位姿)/删除/前后移动 ----
    if (a.waypoints.length) {
      const sub = document.createElement('div');
      sub.style.cssText = 'margin:0 0 6px 70px;display:flex;flex-direction:column;gap:2px;';
      a.waypoints.forEach((w, i) => {
        const p = (w.end_pose && w.end_pose.position) || [0, 0, 0];
        const line = document.createElement('div');
        line.className = 'row';
        line.style.cssText = 'align-items:center;gap:4px;flex-wrap:wrap;';
        line.innerHTML =
          `<span class="muted" style="min-width:36px;">点${i + 1}</span>` +
          `<span class="badge ${w.center ? 'on' : 'off'}" data-op="wpc" data-arm="${arm}" data-act="${name}" data-index="${i}"` +
          ` title="标记此点在双臂中心区(中间件上方重叠区)内: 回放走向它前抢中心锁,走到其后第一个非标记点(退出段走完)才交锁;都不标=整段互斥"` +
          ` style="cursor:pointer;">${w.center ? '●中点' : '○中点'}</span>` +
          `<span class="badge ${w.wait_s ? 'on' : 'off'}" data-op="wpw" data-arm="${arm}" data-act="${name}" data-index="${i}" data-wait="${w.wait_s || ''}"` +
          ` title="到点等待: 回放走到此点(到位验证之后)停留 N 秒再走剩余段 —— 真空触发点同点配了等待时顺序=到点→等→触发真空;点我输入秒数,0=取消"` +
          ` style="cursor:pointer;">${w.wait_s ? '⏱' + w.wait_s + 's' : '⏱等'}</span>` +
          `<span class="muted" style="min-width:180px;">[${p.map(v => Math.round(v * 1000)).join(', ')}]mm</span>` +
          `<span class="muted" style="flex:1;min-width:120px;">${(w.created || '').slice(5)}${w.updated ? ' ↩' + w.updated.slice(5) : ''}</span>` +
          `<button data-op="wpow" data-arm="${arm}" data-act="${name}" data-index="${i}" title="移臂到此点要改的姿态后点我,用当前位姿覆盖点${i + 1}">↩覆盖</button>` +
          `<button data-op="wpdel" data-arm="${arm}" data-act="${name}" data-index="${i}" title="删除点${i + 1}">✕删</button>` +
          `<button data-op="wpmv" data-arm="${arm}" data-act="${name}" data-index="${i}" data-offset="-1"${i === 0 ? ' disabled' : ''} title="点${i + 1} 前移一位">↑</button>` +
          `<button data-op="wpmv" data-arm="${arm}" data-act="${name}" data-index="${i}" data-offset="1"${i === a.waypoints.length - 1 ? ' disabled' : ''} title="点${i + 1} 后移一位">↓</button>`;
        sub.appendChild(line);
      });
      el.appendChild(sub);
    }
    const gate = (FL_GATE_AFTER[arm] || {})[name];
    if (gate) {
      const sep = document.createElement('div');
      sep.style.cssText = 'text-align:center;padding:1px 0;font-size:12px;color:#6dd49a;';
      sep.textContent = `⇣ ${gate}(自动,对准后才播下一个动作) ⇣`;
      el.appendChild(sep);
    }
  }
}

async function flHandleAction(e) {
  const btn = e.target.closest('[data-op]');
  if (!btn) return;
  const arm = btn.dataset.arm, act = btn.dataset.act, op = btn.dataset.op;
  const row = btn.closest('.row');
  const evSel = row.querySelector('[data-op="vev"]');
  const atSel = row.querySelector('[data-op="vat"]');
  const kIn = row.querySelector('[data-op="vk"]');
  try {
    if (op === 'add') {
      const r = await API.flWpAdd(arm, act);
      toast(`已加路点(共 ${r.count} 点)`, 'success');
      flRefresh();
    } else if (op === 'del') {
      const r = await API.flConfigGet();
      const n = r.config.arms[arm].actions[act].waypoints.length;
      if (!n) return toast('没有路点', '');
      await API.flWpDelLast(arm, act, n - 1);
      flRefresh();
    } else if (op === 'clear') {
      await API.flWpClear(arm, act);
      flRefresh();
    } else if (op === 'wpow') {
      const i = +btn.dataset.index;
      if (!confirm(`用当前臂位姿覆盖 ${act} 的点${i + 1}?(其余点不动)`)) return;
      const r = await API.flWpOverwrite(arm, act, i);
      toast(`点${i + 1} 已覆盖为当前位姿`, 'success');
      flRefresh();
    } else if (op === 'wpdel') {
      const i = +btn.dataset.index;
      if (!confirm(`删除 ${act} 的点${i + 1}?`)) return;
      await API.flWpDelLast(arm, act, i);
      flRefresh();
    } else if (op === 'wpmv') {
      await API.flWpMove(arm, act, +btn.dataset.index, +btn.dataset.offset);
      flRefresh();
    } else if (op === 'wpc') {
      const i = +btn.dataset.index;
      const on = btn.classList.contains('on');
      await API.flCenterMark(arm, act, i, !on);
      toast(`点${i + 1} ${on ? '取消中点' : '已标中点'}`, 'success');
      flRefresh();
    } else if (op === 'wpw') {
      const i = +btn.dataset.index;
      const v = prompt(`回放走到点${i + 1} 后等待几秒?(0 或留空=取消等待)`,
                       btn.dataset.wait || '1');
      if (v === null) return;                    // 关掉输入框=不改
      const s = parseFloat(v);
      if (Number.isNaN(s) || s < 0 || s > 60)
        return toast('秒数需在 0~60 之间', 'error');
      await API.flWpWait(arm, act, i, s);
      toast(`点${i + 1} ${s > 0 ? `到点等 ${s}s` : '取消等待'}`, 'success');
      flRefresh();
    } else if (op === 'title') {
      if (e.type !== 'change') return;          // 只在失焦/回车时存
      const t = btn.value.trim();
      if (!t) return toast('步骤说明不能为空', 'error');
      await API.flTitle(arm, act, t);
      toast(`${act.split('_')[0]} 说明已更新`, 'success');
    } else if (op === 'play') {
      await API.flPlay(arm, act);
      toast('试跑启动(段间可停)', 'success');
      flPoll(true);
    } else if (op === 'vev' || op === 'vat' || op === 'vk') {
      const ev = evSel.value;
      const at = atSel.value === 'k' ? String(Math.max(0, +kIn.value || 0))
                                     : atSel.value;
      await API.flVac({ arm, action: act, event: ev, at });
      toast('真空触发已存', 'success');
    }
  } catch (err) { toast(err.message, 'error'); }
}
document.getElementById('flActsLeft')
  .addEventListener('change', flHandleAction);
document.getElementById('flActsLeft')
  .addEventListener('click', flHandleAction);
document.getElementById('flActsRight')
  .addEventListener('change', flHandleAction);
document.getElementById('flActsRight')
  .addEventListener('click', flHandleAction);

document.getElementById('flRefresh').addEventListener('click', flRefresh);

document.getElementById('flSaveCfg').addEventListener('click', async () => {
  try {
    await API.flConfigSet({
      gate: { mode: document.getElementById('flGateMode').value,
              tol_xy_mm: +document.getElementById('flTolXY').value,
              tol_z_mm: +document.getElementById('flTolZ').value,
              max_corr_mm: +document.getElementById('flMaxCorr').value,
              iterations: +document.getElementById('flIter').value,
              frames: +document.getElementById('flFrames').value },
    });
    toast('闸门参数已存', 'success');
    flRefresh();
  } catch (e) { toast(e.message, 'error'); }
});

// 启用/停用 PBVS 对准(逐站,立即生效:只影响下一次启动的流水线)
const flEnToggle = (arm, which, el, label) =>
  el.addEventListener('change', async () => {
    try {
      await API.flConfigSet({ [arm]: { [which]: { enabled: el.checked } } });
      toast(`${label} PBVS 对准已${el.checked ? '启用' : '停用'}`
            + (el.checked ? '' : '(该站按示教回放,不测码不修正)'), 'success');
      flRefresh();
    } catch (e) { el.checked = !el.checked; toast(e.message, 'error'); }
  });
flEnToggle('armleft', 'align', document.getElementById('flEnL'), '左臂·中间件');
flEnToggle('armright', 'align', document.getElementById('flEnR1'), '右臂·中间件');
flEnToggle('armright', 'align2', document.getElementById('flEnR2'), '右臂·料仓');

document.getElementById('flSaveMkL').addEventListener('click', async () => {
  try {
    await API.flConfigSet({
      armleft: { align: {
        marker: flMarker('flLMkType', 'flLMkId', 'flLMkData'),
        marker_size_mm: +document.getElementById('flLMkSize').value } } });
    toast('左臂码参数已存', 'success');
  } catch (e) { toast(e.message, 'error'); }
});

document.getElementById('flSaveMkR').addEventListener('click', async () => {
  try {
    await API.flConfigSet({
      armright: {
        align: { marker: flMarker('flRMk1Type', 'flRMk1Id', 'flRMk1Data'),
                 marker_size_mm: +document.getElementById('flRMk1Size').value },
        align2: { marker: flMarker('flRMk2Type', 'flRMk2Id', 'flRMk2Data'),
                  marker_size_mm: +document.getElementById('flRMk2Size').value } } });
    toast('右臂码参数已存', 'success');
  } catch (e) { toast(e.message, 'error'); }
});

// 动作间夹爪整形: 渲染 + 保存
function flRenderGrips(r) {
  const el = document.getElementById('flGripList');
  if (!el) return;
  el.innerHTML = '';
  const names = { armleft: '左臂', armright: '右臂' };
  for (const [arm, slots] of Object.entries(r.post_grip_slots || {})) {
    const pg = (r.config.arms[arm].post_grippers) || {};
    const h = document.createElement('h4');
    h.style.cssText = 'margin:6px 0 2px;';
    h.textContent = names[arm] || arm;
    el.appendChild(h);
    for (const [slot, hint] of Object.entries(slots)) {
      const g = pg[slot] || { enabled: false, position: 300, velocity: 500, wait_s: 1 };
      const row = document.createElement('div');
      row.className = 'row';
      row.style.cssText = 'margin:2px 0;align-items:center;flex-wrap:wrap;gap:6px;';
      row.innerHTML =
        `<span class="muted" style="min-width:110px;">${hint}</span>` +
        `<label><input type="checkbox" data-g="${arm}|${slot}|on" ${g.enabled ? 'checked' : ''}/> 启用</label>` +
        `<label>开口 <input type="number" min="0" max="1000" step="10" value="${g.position}" size="4" data-g="${arm}|${slot}|pos"/></label>` +
        `<label>速度 <input type="number" min="100" max="1000" step="50" value="${g.velocity}" size="4" data-g="${arm}|${slot}|vel"/></label>` +
        `<label>等待s <input type="number" min="0" max="10" step="0.5" value="${g.wait_s}" size="3" data-g="${arm}|${slot}|wait"/></label>` +
        `<button class="primary" data-gsave="${arm}|${slot}">💾</button>`;
      el.appendChild(row);
    }
  }
}

document.getElementById('flGripList').addEventListener('click', async (e) => {
  const btn = e.target.closest('[data-gsave]');
  if (!btn) return;
  const [arm, slot] = btn.dataset.gsave.split('|');
  const q = s => document.querySelector(`[data-g="${arm}|${slot}|${s}"]`);
  try {
    await API.flConfigSet({ [arm]: { post_grippers: {
      [slot]: { enabled: q('on').checked,
                position: +q('pos').value || 0,
                velocity: +q('vel').value || 500,
                wait_s: +q('wait').value || 0 } } } });
    toast(`夹爪整形(${slot})已存`, 'success');
  } catch (err) { toast(err.message, 'error'); }
});

async function flDoRef(arm, which, infoId, snapId) {
  try {
    const r = await API.flRefRecord(arm, which);
    toast(`${which} 参考已记录(Δ0=${r.ref.p_base})`, 'success');
    flRefresh();
  } catch (e) { toast(e.message, 'error'); }
}

async function flDoTest(arm, which) {
  try {
    const r = await API.flAlignTest(arm, which);
    toast(`${which}: ${r.key} Δ=[${r.delta_mm}]mm z=${r.z_cam_mm}mm → ${r.verdict}`,
          r.verdict.startsWith('PASS') ? 'success' : 'error');
  } catch (e) { toast(e.message, 'error'); }
}

async function flDoGoto(arm, which) {
  const label = arm === 'armleft' ? '左臂' : '右臂';
  if (!confirm(`${label} 将按记录参考时的关节位回到${which === 'align' ? '中间件' : '料仓'}观察位(关节回放,速度=顶栏${label}速度),到位后自动测 Δ。确认臂周无障碍物后继续?`)) return;
  toast(`${label} 回观察位中…(移动+测量约需数秒,期间勿操作)`, 'success');
  try {
    const r = await API.flAlignGoto(arm, which);
    toast(`${which}: 已回观察位(${r.speed_pct}%) ${r.key} Δ=[${r.delta_mm}]mm → ${r.verdict}`,
          r.verdict.startsWith('PASS') ? 'success' : 'error');
  } catch (e) { toast(e.message, 'error'); }
}

async function flDoFix(arm, which) {
  const label = arm === 'armleft' ? '左臂' : '右臂';
  const where = which === 'align' ? '中间件' : '料仓';
  if (!confirm(`${label} 将回${where}观察位并真跑闸门:测 Δ → 超容差会实际修正(≤ max_corr)→ 复测收敛,修完停在修正后的位置(不回示教点)。单步验证 PBVS 用,确认臂周无障碍物后继续?`)) return;
  toast(`${label} 回观察位并修正中…(移动+测量+修正约 5~20s,期间勿操作)`, 'success');
  try {
    const r = await API.flAlignFix(arm, which);
    const g = r.gate || {};
    const detail = (g.tier === 'T0' || g.tier === 'off')
      ? g.tier : `${g.tier}${g.low_res ? '(低分辨率放行)' : ''} ` +
        `Δ=${g.dxy_mm}/${g.dz_mm}mm → 残余 ${g.resid_xy_mm}/${g.resid_z_mm}mm` +
        (g.reanchor_mm ? ` 重锚Δ=${g.reanchor_mm}mm` : '');
    toast(`${which} 修正完成: ${detail}`, 'success');
    document.getElementById('flFixOut').textContent =
      `—— ${label}·${where} ${new Date().toLocaleTimeString()} ——\n` +
      (r.log || []).join('\n');
  } catch (e) { toast(e.message, 'error'); }
}

document.getElementById('flRefL').addEventListener('click',
  () => flDoRef('armleft', 'align'));
document.getElementById('flTestL').addEventListener('click',
  () => flDoTest('armleft', 'align'));
document.getElementById('flGotoL').addEventListener('click',
  () => flDoGoto('armleft', 'align'));
document.getElementById('flFixL').addEventListener('click',
  () => flDoFix('armleft', 'align'));
document.getElementById('flRefR1').addEventListener('click',
  () => flDoRef('armright', 'align'));
document.getElementById('flTestR1').addEventListener('click',
  () => flDoTest('armright', 'align'));
document.getElementById('flGotoR1').addEventListener('click',
  () => flDoGoto('armright', 'align'));
document.getElementById('flFixR1').addEventListener('click',
  () => flDoFix('armright', 'align'));
document.getElementById('flRefR2').addEventListener('click',
  () => flDoRef('armright', 'align2'));
document.getElementById('flTestR2').addEventListener('click',
  () => flDoTest('armright', 'align2'));
document.getElementById('flGotoR2').addEventListener('click',
  () => flDoGoto('armright', 'align2'));
document.getElementById('flFixR2').addEventListener('click',
  () => flDoFix('armright', 'align2'));

document.getElementById('flStart').addEventListener('click', async () => {
  try {
    const rd = parseInt(document.getElementById('flRounds').value, 10);
    await API.flStart({
      step_mode: document.getElementById('flStepMode').checked,
      rounds: (Number.isFinite(rd) && rd > 0) ? rd : null });
    toast(`流水线已启动${(Number.isFinite(rd) && rd > 0)
      ? `(${rd} 轮后自动停)` : '(连续)'}`, 'success');
    flPoll(true);
  } catch (e) { toast(e.message, 'error'); }
});
for (const [id, arms, label] of [
  ['flStartL', 'left', '左臂测试已启动(1 循环自动停)'],
  ['flStartR', 'right', '右臂测试已启动(1 循环自动停)']]) {
  document.getElementById(id).addEventListener('click', async () => {
    try {
      await API.flStart({ step_mode:
                          document.getElementById('flStepMode').checked,
                          arms });
      toast(label, 'success');
      flPoll(true);
    } catch (e) { toast(e.message, 'error'); }
  });
}
document.getElementById('flStop').addEventListener('click', async () => {
  try { toast((await API.flStop()).note || '已请求停止', 'success'); }
  catch (e) { toast(e.message, 'error'); }
});
document.getElementById('flStepOk').addEventListener('click', async () => {
  try { await API.flStepOk(); } catch (e) { toast(e.message, 'error'); }
});

// ---- 运行报告(客户视图: 按时间段聚合每轮 A/B 大步,默认当天) ----
function flFmtDur(s) {
  if (s === null || s === undefined) return '-';
  return s >= 60 ? `${Math.floor(s / 60)}分${Math.round(s % 60)}秒` : `${s}秒`;
}
function flRptRender(rep) {
  const box = document.getElementById('flRptBox');
  const rs = rep.rounds || [];
  if (!rs.length) {
    box.className = 'muted';
    box.textContent = '该时间段无流水线记录。';
    return;
  }
  const s = rep.summary || {};
  const rows = [];
  for (const r of rs) {
    for (const st of (r.steps || [])) {
      rows.push(`<tr><td>第${r.session}次</td><td>第${r.round}轮</td>` +
      `<td>${st.arm_label}臂</td>` +
        `<td>${st.name}</td><td>${st.start || '-'}</td>` +
        `<td>${st.end || (st.ongoing ? '进行中' : '-')}</td>` +
        `<td>${st.ongoing ? '进行中' : flFmtDur(st.duration_s)}</td></tr>`);
    }
    rows.push(`<tr class="flrpt-sub"><td>第${r.session}次</td><td>第${r.round}轮</td>` +
      `<td></td>` +
      `<td><b>本轮小计${r.ongoing ? '(进行中)' : ''}</b></td>` +
      `<td>${r.start || '-'}</td>` +
      `<td>${r.end || (r.ongoing ? '进行中' : '-')}</td>` +
      `<td><b>${r.ongoing ? '-' : flFmtDur(r.duration_s)}</b></td></tr>`);
  }
  box.className = '';
  box.innerHTML =
    `<div class="muted" style="margin:2px 0 6px;">统计区间 ` +
    `${(s.range || {}).start || '-'} ~ ${(s.range || {}).end || '-'}` +
    ` | 共 ${s.sessions_total ?? 0} 次 / ${s.rounds_total ?? 0} 轮` +
    (s.ongoing_rounds ? `(另 ${s.ongoing_rounds} 轮进行中)` : '') +
    ` | 累计 ${flFmtDur(s.total_s)} | 平均每轮 ${flFmtDur(s.avg_s)}` +
    (s.first_start ? ` | 首轮开始 ${s.first_start}` : '') + '</div>' +
    '<table class="flrpt"><thead><tr><th>次</th><th>轮次</th><th>臂</th><th>大步</th>' +
    '<th>开始时间</th><th>结束时间</th><th>耗时</th></tr></thead><tbody>' +
    rows.join('') + '</tbody></table>';
}
{ // 默认当天(本地时区,避开 toISOString 的 UTC 偏移)
  const today = new Date(); today.setMinutes(today.getMinutes() - today.getTimezoneOffset());
  const ds = today.toISOString().slice(0, 10);
  document.getElementById('flRptStart').value = ds;
  document.getElementById('flRptEnd').value = ds;
}
document.getElementById('flRptView').addEventListener('click', async () => {
  const start = document.getElementById('flRptStart').value.trim();
  const end = document.getElementById('flRptEnd').value.trim();
  const box = document.getElementById('flRptBox');
  box.className = 'muted';
  box.textContent = '查询中…';
  try { flRptRender(await API.flRpt(start, end)); }
  catch (e) { box.textContent = '查询失败: ' + e.message; toast(e.message, 'error'); }
});
document.getElementById('flRptExport').addEventListener('click', () => {
  const a = document.createElement('a');
  a.href = '/api/flow/logs/export?start=' +
           encodeURIComponent(document.getElementById('flRptStart').value.trim()) +
           '&end=' +
           encodeURIComponent(document.getElementById('flRptEnd').value.trim());
  a.download = '';
  document.body.appendChild(a);
  a.click();
  a.remove();
});

function flLogBox(id, lines) {
  const el = document.getElementById(id);
  el.textContent = (lines && lines.length) ? lines.join('\n') : '-';
  el.scrollTop = el.scrollHeight;
}

async function flPoll(force) {
  if (flPolling && !force) return;
  flPolling = true;
  let idleRounds = 0;
  while (true) {
    let s = null, t = null;
    try {
      s = await API.flStatus();
      t = await API.flPlayStatus();
    } catch (e) { break; }
    flBadge(document.getElementById('flFixture'),
            s.fixture === 'FILLED', '中间件:' + (s.fixture || '-'));
    for (const [arm, ph, st, lg] of [
      ['armleft', 'flPhaseL', 'flStatL', 'flLogL'],
      ['armright', 'flPhaseR', 'flStatR', 'flLogR']]) {
      const a = (s.arms || {})[arm] || {};
      const waiting = s.waiting_step && s.waiting_step[arm];
      const label = FL_PHASE_LABEL[a.phase] || a.phase || '空闲';
      flBadge(document.getElementById(ph), false,
              waiting ? `⏸${label}` : label);
      document.getElementById(st).textContent =
        `循环 ${a.cycles || 0} | 上次 ${a.last_cycle_s ?? '-'}s | ` +
        `均值 ${a.avg_cycle_s ?? '-'}s` +
        (a.error ? ` | ✗ ${a.error}` : '');
      flLogBox(lg, a.log);
    }
    flLogBox('flLog', s.log);
    const busy = s.running || (t && t.running);
    if (!busy) { idleRounds++; if (idleRounds > 2) break; } else idleRounds = 0;
    await new Promise(res => setTimeout(res, 1000));
  }
  flPolling = false;
}

flRefresh();
flPoll();      // 页面加载即拉一次(可能上次流水线还在跑)

// ===== 启动 =====
API.health().then(h => {
  if (h.robot_connected) {
    $connStatus.textContent = '已连接';
    $connStatus.className = 'badge on';
    startStatusStream();
  }
  if (h.chassis_connected) {
    $chStatus.textContent = '已连接';
    $chStatus.className = 'badge on';
    startChassisLoop();
    refreshMap();
  }
  if (h.ik_helper_present) refreshIkBadge();
}).catch(() => {});

jtRefresh(); ctRefresh(); buildJogTable();
window.addEventListener('beforeunload', () => { stopAllCameras(); });
