'use strict';
const $ = id => document.getElementById(id);
const data = { values: {}, meta: {}, versions: {}, mode: '', connected: false, received: 0, logs: [], uptime: 0 };
const view = { kind: '2d', x: 0, y: 0, scale: 43, follow: false, yaw: -.8, pitch: .65, distance: 14,
  center: [0, 0, 0], image: null, mapVersion: 0, fitted: false, cloudFitted: '', width: 0, height: 0 };
let source, session = 0, lastLog = '', toastTimer;
const cameras = {};
const names = { map: 'SLAM 지도', scan: '2D LiDAR', cloud: 'MID-360 점군', object_cloud: 'Wrist 물체 점군',
  odom: '오도메트리', head_image: 'Head RGB', head_debug: 'Head 검출 영상', wrist_image: 'Wrist RGB',
  wrist_debug: 'Wrist 분할 영상', detections: '검출 결과', path: '계획 경로', battery: '배터리',
  joints: '관절 피드백', wrist_status: 'Wrist 상태' };
const fmt = (v, places = 1) => Number.isFinite(v) ? v.toFixed(places) : '—';
const text = (id, value) => { $(id).textContent = value; };
const elapsed = seconds => `${Math.floor(seconds / 60).toString().padStart(2, '0')}:${Math.floor(seconds % 60).toString().padStart(2, '0')}`;
function toast(message) { text('toast', message); $('toast').classList.add('visible'); clearTimeout(toastTimer); toastTimer = setTimeout(() => $('toast').classList.remove('visible'), 3500); }
function quality(key) {
  const m = data.meta[key];
  if (!m || m.age == null || data.values[key] == null) return 'waiting';
  if (!data.connected || performance.now() - data.received > 2500) return 'stale';
  if (m.error) return 'error';
  if (['map', 'path', 'object_cloud'].includes(key)) return data.mode === 'demo' || m.publishers > 0 ? 'cached' : 'stale';
  return m.age <= 2.5 && (m.source_age == null || (m.source_age >= -.5 && m.source_age <= 2.5)) ? 'live' : 'stale';
}
function fresh(key) { return ['live', 'cached'].includes(quality(key)); }
function badge(id, label, state) { text(id, label); $(id).className = 'tiny-pill' + (state === 'live' || state === 'cached' ? ' active' : state === 'stale' || state === 'error' ? ' warn' : ''); }
function connect() {
  if (source) source.close();
  data.connected = false;
  source = new EventSource('/api/events');
  source.onopen = () => {
    session += 1;
    data.values = {}; data.meta = {}; data.versions = {};
    view.image = null; view.mapVersion = 0; view.fitted = false; view.cloudFitted = '';
    for (const c of Object.values(cameras)) { c.key = ''; }
    data.connected = true;
  };
  source.onmessage = event => {
    try {
      const payload = JSON.parse(event.data);
      data.connected = true; data.received = performance.now();
      data.mode = payload.mode; data.meta = payload.channels; data.versions = payload.versions;
      data.logs = payload.logs; data.uptime = payload.uptime; data.fixedFrame = payload.fixed_frame;
      data.controlToken = payload.control_token;
      Object.assign(data.values, payload.changes);
      const map = data.values.map;
      if (map && payload.changes.map && view.mapVersion !== data.versions.map) {
        view.mapVersion = data.versions.map;
        const version = view.mapVersion, image = new Image();
        image.onload = () => { if (version !== view.mapVersion) return; view.image = image; if (!view.fitted) { fitMap(); view.fitted = true; } draw(); };
        image.src = map.image;
      } else if (!map) { view.image = null; }
      text('host-label', payload.hostname + ' · ROS 2 Humble');
      render();
    } catch (error) { toast('수신 데이터를 읽을 수 없습니다: ' + error.message); }
  };
  source.onerror = () => { data.connected = false; render(); };
}

function render() {
  const demo = data.mode === 'demo';
  const alive = data.connected && performance.now() - data.received < 2500;
  $('connection').className = 'connection ' + (alive ? 'connected' : 'disconnected');
  $('connection').replaceChildren(Object.assign(document.createElement('i'), {}), document.createTextNode(alive ? '서버 연결됨' : '서버 연결 끊김'));
  text('mode-badge', demo ? 'SAMPLE MODE' : data.mode === 'live' ? 'ROS LIVE' : '입력 대기');
  $('mode-badge').className = 'badge ' + (data.mode === 'live' && alive ? 'green' : 'amber');
  $('notice').classList.toggle('live', data.mode === 'live' && alive);
  text('notice-text', !alive ? '데이터 연결이 끊겼습니다. 마지막 수신 화면은 실시간 상태가 아닙니다.' : demo
    ? '샘플 모드 · 지도, 로봇 위치, 영상, 검출, 점군은 가상 데이터입니다. 실제 연결은 젯슨 서버와 맥 SSH 터널을 실행하세요.'
    : data.mode === 'live' ? '젯슨 실시간 연결 · 센서 제어와 SLAM 지도, 영상, 검출 결과를 표시합니다.'
    : '젯슨에서 관제센터를 시작하고 맥에서 SSH 터널을 연결하세요.');
  for (const el of document.querySelectorAll('.camera-sample')) el.classList.toggle('hidden', !demo);
  text('view-tag', demo ? 'SAMPLE DATA' : alive ? 'ROS OBSERVATION' : 'DISCONNECTED');
  const p = data.values.pose, odom = data.values.odom, map = data.values.map, detections = data.values.detections;
  text('position-value', p ? `${fmt(p.x, 2)}, ${fmt(p.y, 2)}` : '—');
  $('position-value').style.fontSize = p ? '22px' : '';
  text('position-foot', p ? `${p.frame} · yaw ${fmt(p.yaw * 180 / Math.PI, 1)}°${fresh('pose') ? '' : ' · 과거 수신값'}` : 'TF 수신 대기');
  text('speed-value', odom ? fmt(odom.linear, 2) : '—');
  text('speed-foot', odom ? `${fmt(odom.angular, 2)} rad/s${fresh('odom') ? '' : ' · 수신 지연'}` : '오도메트리 수신 대기');
  text('area-value', map ? fmt(map.known_area, 1) : '—');
  text('area-foot', map ? `${map.source_width} × ${map.source_height} cells · ${map.frame}${fresh('map') ? '' : ' · 과거 지도'}` : 'SLAM /map 수신 대기');
  text('detection-value', detections && fresh('detections') ? (detections.valid ? detections.objects.length : 0) : '—');
  text('detection-foot', detections ? `${detections.state} · ${fresh('detections') ? detections.valid ? '유효 관측' : '유효 목표 없음' : '수신 지연'}` : '검출 토픽 수신 대기');
  updateCamera('head'); updateCamera('wrist');
  renderDetections(); renderSensors(); renderSensorControls(); renderLogs();
  text('session-time', 'SESSION ' + elapsed(data.uptime));
  text('transport-label', demo ? 'SYNTHETIC SAMPLE · LOCALHOST' : 'ROS 2 → SSE / MJPEG → SSH TUNNEL');
  $('save-map-button').disabled = !map;
  if (view.follow && p && fresh('pose') && p.frame === currentFrame() && view.kind === '2d') { view.x = p.x; view.y = p.y; }
  draw();
}

function updateCamera(role) {
  const selection = $(role + '-source').value;
  let key = selection;
  if (selection === 'auto') {
    key = fresh(role + '_debug') ? role + '_debug' : data.values[role + '_image'] ? role + '_image' : role + '_debug';
  }
  const meta = data.meta[key], value = data.values[key], img = $(role + '-camera'), state = quality(key);
  cameras[role] ||= { key: '', version: 0 };
  const previous = cameras[role];
  const haveImage = Boolean(value && meta && meta.age != null);
  img.hidden = !haveImage;
  $(role + '-empty').classList.toggle('hidden', haveImage && state !== 'stale' && state !== 'error');
  img.closest('.camera-frame').classList.toggle('stale', state === 'stale' || state === 'error');
  const empty = $(role + '-empty');
  empty.querySelector('strong').textContent = haveImage ? '영상 수신 지연 · 마지막 프레임' : '카메라 입력 대기';
  empty.querySelector('small').textContent = meta?.topic || (role === 'head' ? 'Head RGB / YOLO debug' : 'Wrist RGB / SAM debug');
  if (haveImage) {
    const identity = `${key}:${session}:${data.mode}`;
    if (data.mode === 'demo') {
      if (previous.version !== data.versions[key] || previous.key !== identity) img.src = '/api/frame/' + key + '?v=' + data.versions[key];
    } else if (previous.key !== identity) {
      img.src = '/api/stream/' + key + '?session=' + session;
    }
    previous.key = identity; previous.version = data.versions[key];
  } else if (previous.key) { img.removeAttribute('src'); previous.key = ''; }
  text(role + '-hz', haveImage ? fmt(meta.hz, 1) + ' FPS' : '— FPS');
  text(role + '-resolution', haveImage ? `${value.width}×${value.height}` : '—');
  text(role + '-state', state === 'live' ? data.mode === 'demo' ? 'SAMPLE' : key.endsWith('_debug') ? 'DEBUG' : 'RGB LIVE' : state === 'stale' ? 'STALE' : state === 'error' ? 'ERROR' : 'WAITING');
  $(role + '-state').className = 'camera-state ' + (state === 'live' ? 'live' : state === 'stale' || state === 'error' ? 'stale' : '');
}

function renderDetections() {
  const d = data.values.detections, q = quality('detections');
  badge('target-status', d ? fresh('detections') ? d.state : 'STALE' : 'WAITING', q);
  text('target-summary', d ? `${d.valid ? '유효 관측' : '후보 / 무효 관측'} · ${d.measured ? 'YOLO 측정' : '추적 중 · 마지막 YOLO 신뢰도'}${d.processing_ms != null ? ' · ' + fmt(d.processing_ms) + ' ms' : ''}${q === 'stale' ? ' · 과거 결과' : ''}` : '모델이 발행한 클래스·신뢰도·추적 상태를 표시합니다.');
  const list = $('detection-list'); list.replaceChildren();
  if (!d || !d.objects.length) {
    const empty = document.createElement('div'); empty.className = 'empty-row';
    empty.textContent = d ? d.reason || '현재 검출된 목표가 없습니다.' : '검출 토픽 입력 대기'; list.append(empty);
  } else for (const item of d.objects) {
    const row = document.createElement('div'); row.className = 'detection-row';
    if (!d.valid || !fresh('detections')) row.style.opacity = '.5';
    const dot = document.createElement('span'); dot.className = 'object-dot';
    const label = document.createElement('strong'); label.textContent = item.label || 'object';
    label.title = `bbox (${fmt(item.x)}, ${fmt(item.y)}), ${fmt(item.width)}×${fmt(item.height)} px`;
    const bar = document.createElement('span'); bar.className = 'confidence-bar';
    const fill = document.createElement('span'); fill.style.width = Math.max(0, Math.min(100, item.confidence * 100)) + '%'; bar.append(fill);
    const score = document.createElement('small'); score.textContent = fmt(item.confidence * 100, 0) + '%';
    row.append(dot, label, bar, score); list.append(row);
  }
  text('wrist-summary', data.values.wrist_status ? data.values.wrist_status.text + (fresh('wrist_status') ? '' : ' · 마지막 수신 상태') : '분할 노드 상태 입력 대기');
}

function renderSensors() {
  const list = $('sensor-list'); list.replaceChildren();
  for (const key of ['map', 'scan', 'cloud', 'head_image', 'wrist_image']) {
    const m = data.meta[key], q = quality(key), row = document.createElement('div'); row.className = 'sensor-row';
    const light = document.createElement('span'); light.className = 'sensor-light ' + (q === 'cached' ? 'live' : q);
    const description = document.createElement('div'), label = document.createElement('strong'), topic = document.createElement('small');
    label.textContent = names[key]; topic.textContent = m?.topic || '발행 토픽 대기'; topic.title = m?.error || m?.topic || '';
    description.append(label, topic);
    const status = document.createElement('span'); status.textContent = q === 'live' ? fmt(m.hz) + ' Hz' : q === 'cached' ? '저장된 결과' : q === 'stale' ? '수신 지연' : q === 'error' ? '오류' : 'WAITING';
    row.append(light, description, status); list.append(row);
  }
  const h = data.values.health;
  text('health-cpu', `CPU ${h ? fmt(h.cpu, 0) : '—'}%`);
  text('health-memory', `RAM ${h ? fmt(h.memory_percent, 0) : '—'}%`);
  text('health-temperature', `온도 ${h ? fmt(h.temperature, 0) : '—'}°C`);
}

const sensorCards = {}, sensorPending = new Set();
for (const [key, label] of Object.entries({ head_camera: 'Head D435', wrist_camera: 'Wrist D435', g2_lidar: 'YDLIDAR G2', mid360_lidar: 'Livox MID-360' })) {
  const card = document.createElement('article'); card.className = 'sensor-control-card';
  const title = document.createElement('strong'); title.textContent = label;
  const status = document.createElement('span'); status.className = 'control-state'; status.setAttribute('role', 'status');
  const hint = document.createElement('small');
  const actions = document.createElement('div'); actions.className = 'sensor-control-actions';
  const start = document.createElement('button'), stop = document.createElement('button');
  start.textContent = '켜기'; stop.textContent = '끄기'; start.disabled = stop.disabled = true;
  start.setAttribute('aria-label', label + ' 드라이버 켜기'); stop.setAttribute('aria-label', label + ' 드라이버 끄기');
  start.onclick = () => controlSensor(key, 'start'); stop.onclick = () => controlSensor(key, 'stop');
  actions.append(start, stop); card.append(title, status, hint, actions); $('sensor-controls').append(card);
  sensorCards[key] = { card, status, hint, start, stop };
}
function renderSensorControls() {
  const alive = data.mode === 'live' && data.connected && performance.now() - data.received < 2500;
  const labels = { off: '중지됨', starting: '시작 요청 처리 중', running: '드라이버 실행 중', stopping: '중지 요청 처리 중', external: '외부 드라이버 실행 중', error: '실행 오류' };
  for (const [key, ui] of Object.entries(sensorCards)) {
    const sensor = data.values.sensor_control?.[key];
    ui.status.textContent = !alive ? '제어 연결 대기' : labels[sensor?.state] || '관리자 연결 대기';
    ui.card.classList.toggle('running', alive && sensor?.managed);
    ui.hint.textContent = !alive ? data.mode === 'demo' ? '샘플 모드 · 실제 제어 비활성' : '젯슨 서버에 연결하세요' : sensor?.error || (sensor?.state === 'external' ? '기존 실행에서 전환 필요 · README 참조' : sensor?.managed ? sensor.receiving ? '실시간 데이터 수신 중' : '프로세스 실행 · 데이터 수신 대기' : sensor?.topic || '센서 상태 대기');
    ui.hint.title = sensor?.log || '';
    ui.start.disabled = !alive || !sensor?.can_start || sensorPending.has(key);
    ui.stop.disabled = !alive || !sensor?.can_stop || sensorPending.has(key);
  }
  text('sensor-control-note', '켜기/끄기는 센서 드라이버를 실행·중지합니다. 장치의 전원 차단을 의미하지 않습니다.');
}
async function controlSensor(sensor, action) {
  sensorPending.add(sensor); renderSensorControls();
  try {
    const response = await fetch('/api/sensors', { method: 'POST', headers: { 'Content-Type': 'application/json', 'X-Control-Token': data.controlToken || '' }, body: JSON.stringify({ sensor, action }) });
    const result = await response.json(); if (!response.ok) throw new Error(result.error || response.status);
    toast('젯슨에 ' + (action === 'start' ? '시작' : '중지') + ' 요청을 전달했습니다. 센서 상태에서 결과를 볼 수 있습니다.');
  } catch (error) { toast(error.message); }
  finally { sensorPending.delete(sensor); renderSensorControls(); }
}

function renderLogs() {
  const signature = JSON.stringify(data.logs);
  if (signature === lastLog) return; lastLog = signature;
  const list = $('event-list'); list.replaceChildren();
  for (const item of data.logs) {
    const row = document.createElement('div'); row.className = 'event ' + item.level;
    const stamp = document.createElement('time'); stamp.textContent = new Date(item.time * 1000).toLocaleTimeString('ko-KR', { hour12: false });
    const dot = document.createElement('i'), body = document.createElement('p'), origin = document.createElement('small');
    body.textContent = item.message; origin.textContent = item.source; body.append(origin); row.append(stamp, dot, body); list.prepend(row);
  }
}

const canvas = $('map-canvas'), ctx = canvas.getContext('2d');
function currentFrame() { return data.values.map?.frame || data.values.pose?.frame || data.fixedFrame || 'map'; }
function mapBounds() {
  const m = data.values.map;
  if (!m) return [view.x - 5, view.y - 4, view.x + 5, view.y + 4];
  const [x, y, a] = m.origin, w = m.width * m.resolution, h = m.height * m.resolution;
  const corners = [[0, 0], [w, 0], [0, h], [w, h]].map(([u, v]) => [x + Math.cos(a) * u - Math.sin(a) * v, y + Math.sin(a) * u + Math.cos(a) * v]);
  return [Math.min(...corners.map(c => c[0])), Math.min(...corners.map(c => c[1])), Math.max(...corners.map(c => c[0])), Math.max(...corners.map(c => c[1]))];
}
function fitMap() {
  const b = mapBounds(); view.x = (b[0] + b[2]) / 2; view.y = (b[1] + b[3]) / 2;
  view.scale = Math.max(2, Math.min((view.width - 70) / Math.max(b[2] - b[0], .1), (view.height - 65) / Math.max(b[3] - b[1], .1)));
}
function fitCloud() {
  const cloud = data.values[$('cloud-source').value];
  if (!cloud?.points.length) return;
  let min = [Infinity, Infinity, Infinity], max = [-Infinity, -Infinity, -Infinity];
  for (const p of cloud.points) for (let i = 0; i < 3; i++) { min[i] = Math.min(min[i], p[i]); max[i] = Math.max(max[i], p[i]); }
  view.center = min.map((v, i) => (v + max[i]) / 2);
  view.distance = Math.max(.4, Math.hypot(...min.map((v, i) => max[i] - v)) * .95);
}
function fit() { if (view.kind === '2d') fitMap(); else fitCloud(); draw(); }
function spacing(value) { const power = 10 ** Math.floor(Math.log10(Math.max(value, .001))); return ([1, 2, 5, 10].find(n => n * power >= value) || 10) * power; }
function world(p) { return [view.width / 2 + (p[0] - view.x) * view.scale, view.height / 2 - (p[1] - view.y) * view.scale]; }
function worldLine(points, color, width = 1, dashed = false) {
  if (!points?.length) return; ctx.beginPath();
  points.forEach((p, i) => { const [x, y] = world(p); if (i) ctx.lineTo(x, y); else ctx.moveTo(x, y); });
  ctx.strokeStyle = color; ctx.lineWidth = width; ctx.setLineDash(dashed ? [5, 5] : []); ctx.stroke(); ctx.setLineDash([]);
}
function draw() {
  const rect = canvas.getBoundingClientRect();
  if (rect.width < 1 || rect.height < 1) return;
  const dpr = window.devicePixelRatio || 1;
  if (canvas.width !== Math.round(rect.width * dpr) || canvas.height !== Math.round(rect.height * dpr)) {
    canvas.width = Math.round(rect.width * dpr); canvas.height = Math.round(rect.height * dpr);
  }
  view.width = rect.width; view.height = rect.height;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0); ctx.clearRect(0, 0, view.width, view.height);
  ctx.fillStyle = '#111b25'; ctx.fillRect(0, 0, view.width, view.height);
  if (view.kind === '2d') drawMap(); else drawCloud();
}
function drawMap() {
  const m = data.values.map, p = data.values.pose, scan = data.values.scan, route = data.values.path, frame = currentFrame();
  const grid = spacing(38 / view.scale), xMin = view.x - view.width / view.scale / 2, xMax = view.x + view.width / view.scale / 2;
  const yMin = view.y - view.height / view.scale / 2, yMax = view.y + view.height / view.scale / 2;
  for (let x = Math.ceil(xMin / grid) * grid; x <= xMax; x += grid) worldLine([[x, yMin], [x, yMax]], '#233442');
  for (let y = Math.ceil(yMin / grid) * grid; y <= yMax; y += grid) worldLine([[xMin, y], [xMax, y]], '#233442');
  if (m && view.image) {
    ctx.save(); ctx.translate(view.width / 2, view.height / 2); ctx.scale(view.scale, -view.scale); ctx.translate(-view.x, -view.y);
    ctx.translate(m.origin[0], m.origin[1]); ctx.rotate(m.origin[2]); ctx.imageSmoothingEnabled = false;
    ctx.globalAlpha = fresh('map') ? .92 : .48; ctx.drawImage(view.image, 0, 0, m.width * m.resolution, m.height * m.resolution); ctx.restore();
  }
  if (p?.frame === frame && $('layer-trail').checked) worldLine(p.trail, '#e5bb7166', 2);
  if (route?.frame === frame && $('layer-path').checked) worldLine(route.points, fresh('path') ? '#c18bdf' : '#c18bdf66', 2, true);
  if (scan?.frame === frame && $('layer-scan').checked && fresh('scan')) {
    ctx.fillStyle = '#3ec7cf'; for (const point of scan.points) { const [x, y] = world(point); ctx.fillRect(x - 1.3, y - 1.3, 2.6, 2.6); }
  }
  if (p?.frame === frame) {
    const [x, y] = world([p.x, p.y]), live = fresh('pose');
    ctx.save(); ctx.translate(x, y); ctx.rotate(-p.yaw);
    const size = Math.max(12, Math.min(27, .48 * view.scale));
    ctx.strokeStyle = live ? '#b8ed7a44' : '#f0b86955'; ctx.lineWidth = 1; ctx.beginPath(); ctx.arc(0, 0, size + 8, 0, Math.PI * 2); ctx.stroke();
    ctx.fillStyle = live ? '#b8ed7a' : '#f0b869'; ctx.strokeStyle = '#173225'; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(size * .7, 0); ctx.lineTo(-size * .55, -size * .45); ctx.lineTo(-size * .25, 0); ctx.lineTo(-size * .55, size * .45); ctx.closePath(); ctx.fill(); ctx.stroke(); ctx.restore();
  }
  const useful = Boolean(m || scan?.frame === frame || p?.frame === frame);
  $('canvas-empty').classList.toggle('hidden', useful);
  text('frame-tag', 'FRAME / ' + frame);
  const q = quality('map'); badge('map-status', m ? q === 'cached' ? 'MAP LOADED' : q.toUpperCase() : 'WAITING SLAM', q);
  text('map-detail', m ? `${fmt(m.resolution, 3)} m/cell · ${fmt(m.known_area)} m²` : `${frame} · /map 입력 대기`);
  const unit = spacing(55 / view.scale); $('scale-bar').style.width = unit * view.scale + 'px'; text('scale-label', fmt(unit, unit < 1 ? 2 : 0) + ' m');
}
function project(point) {
  const dx = point[0] - view.center[0], dy = point[1] - view.center[1], dz = point[2] - view.center[2];
  const rx = Math.cos(view.yaw) * dx - Math.sin(view.yaw) * dy, ry = Math.sin(view.yaw) * dx + Math.cos(view.yaw) * dy;
  const vertical = -Math.sin(view.pitch) * ry + Math.cos(view.pitch) * dz;
  const depth = view.distance - Math.cos(view.pitch) * ry - Math.sin(view.pitch) * dz;
  if (depth < .02) return null;
  const focal = Math.min(view.width, view.height) * .95;
  return [view.width / 2 + rx * focal / depth, view.height / 2 - vertical * focal / depth, depth];
}
function line3d(a, b, color) {
  const p = project(a), q = project(b); if (!p || !q) return;
  ctx.strokeStyle = color; ctx.lineWidth = 1; ctx.beginPath(); ctx.moveTo(p[0], p[1]); ctx.lineTo(q[0], q[1]); ctx.stroke();
}
function drawCloud() {
  const key = $('cloud-source').value, cloud = data.values[key];
  if (cloud?.points.length && view.cloudFitted !== key + cloud.frame) { fitCloud(); view.cloudFitted = key + cloud.frame; }
  const grid = spacing(view.distance / 14), span = grid * 6;
  for (let i = -6; i <= 6; i++) {
    line3d([view.center[0] + i * grid, view.center[1] - span, 0], [view.center[0] + i * grid, view.center[1] + span, 0], '#253b4a');
    line3d([view.center[0] - span, view.center[1] + i * grid, 0], [view.center[0] + span, view.center[1] + i * grid, 0], '#253b4a');
  }
  const axis = Math.max(.1, view.distance / 9);
  line3d([0, 0, 0], [axis, 0, 0], '#df847f'); line3d([0, 0, 0], [0, axis, 0], '#acd477'); line3d([0, 0, 0], [0, 0, axis], '#7c9fe4');
  if (cloud) {
    ctx.globalAlpha = fresh(key) ? .9 : .3;
    for (const point of cloud.points) {
      const p = project(point); if (!p || p[0] < 0 || p[0] > view.width || p[1] < 0 || p[1] > view.height) continue;
      ctx.fillStyle = `hsl(${190 - Math.max(0, Math.min(1, (point[2] - view.center[2] + 1) / 3)) * 95},65%,66%)`;
      const size = Math.max(1, Math.min(3, 15 / p[2])); ctx.fillRect(p[0], p[1], size, size);
    }
    ctx.globalAlpha = 1;
  }
  $('canvas-empty').classList.toggle('hidden', Boolean(cloud?.points.length));
  $('canvas-empty').querySelector('strong').textContent = '3D 점군 입력을 기다리고 있습니다';
  $('canvas-empty').querySelector('p').textContent = 'MID-360 PointCloud2 또는 Wrist 물체 점군을 선택하세요.';
  text('frame-tag', 'FRAME / ' + (cloud?.frame || '—'));
  badge('map-status', cloud ? fresh(key) ? key === 'object_cloud' ? 'CACHED CLOUD' : 'POINT CLOUD' : 'STALE' : 'WAITING', quality(key));
  text('map-detail', cloud ? `${cloud.points.length.toLocaleString()} / ${cloud.total_points.toLocaleString()} points · ${cloud.frame}` : 'NO POINT CLOUD');
  $('scale-bar').style.width = '55px'; text('scale-label', 'XYZ / m');
}

function setView(kind) {
  view.kind = kind;
  $('view-2d').classList.toggle('selected', kind === '2d'); $('view-3d').classList.toggle('selected', kind === '3d');
  $('nav-map').classList.toggle('active', kind === '2d'); $('nav-cloud').classList.toggle('active', kind === '3d');
  $('map-layers').classList.toggle('hidden', kind === '3d'); $('cloud-source').classList.toggle('hidden', kind === '2d');
  $('follow-button').disabled = kind === '3d';
  text('map-coordinates', kind === '2d' ? '드래그로 이동 · 스크롤로 확대' : '드래그로 회전 · 스크롤로 확대');
  if (kind === '2d') {
    $('canvas-empty').querySelector('strong').textContent = '지도 입력을 기다리고 있습니다';
    $('canvas-empty').querySelector('p').textContent = '/map과 map → base_link TF를 기다립니다.';
  }
  draw();
}
let dragging = null;
canvas.addEventListener('pointerdown', e => { dragging = [e.clientX, e.clientY]; canvas.setPointerCapture(e.pointerId); });
canvas.addEventListener('pointermove', e => {
  if (!dragging) return;
  const dx = e.clientX - dragging[0], dy = e.clientY - dragging[1]; dragging = [e.clientX, e.clientY];
  if (view.kind === '2d') { view.x -= dx / view.scale; view.y += dy / view.scale; view.follow = false; $('follow-button').setAttribute('aria-pressed', 'false'); }
  else { view.yaw += dx * .007; view.pitch = Math.max(-.9, Math.min(1.45, view.pitch + dy * .006)); }
  draw();
});
canvas.addEventListener('pointerup', () => dragging = null); canvas.addEventListener('pointercancel', () => dragging = null);
function zoom(factor, x = view.width / 2, y = view.height / 2) {
  if (view.kind === '3d') view.distance = Math.max(.1, Math.min(1000, view.distance / factor));
  else {
    const wx = view.x + (x - view.width / 2) / view.scale, wy = view.y - (y - view.height / 2) / view.scale;
    view.scale = Math.max(2, Math.min(1000, view.scale * factor));
    view.x = wx - (x - view.width / 2) / view.scale; view.y = wy + (y - view.height / 2) / view.scale;
  }
  draw();
}
canvas.addEventListener('wheel', e => { e.preventDefault(); const rect = canvas.getBoundingClientRect(); zoom(Math.exp(-e.deltaY * .0015), e.clientX - rect.left, e.clientY - rect.top); }, { passive: false });
$('zoom-in').onclick = () => zoom(1.25); $('zoom-out').onclick = () => zoom(.8);
$('fit-button').onclick = fit;
$('follow-button').onclick = () => { view.follow = !view.follow; $('follow-button').setAttribute('aria-pressed', String(view.follow)); render(); };
$('view-2d').onclick = $('nav-map').onclick = () => setView('2d');
$('view-3d').onclick = $('nav-cloud').onclick = () => setView('3d');
$('cloud-source').onchange = () => { view.cloudFitted = ''; draw(); };
for (const id of ['layer-scan', 'layer-path', 'layer-trail']) $(id).onchange = draw;
for (const role of ['head', 'wrist']) $(role + '-source').onchange = () => updateCamera(role);
$('reconnect-button').onclick = connect;
function download(url, name) { const link = document.createElement('a'); link.href = url; link.download = name; document.body.append(link); link.click(); link.remove(); }
$('save-map-button').onclick = () => download('/api/map.png', 'slam-map.png');
$('export-button').onclick = async () => {
  try { const response = await fetch('/api/snapshot'); if (!response.ok) throw new Error(response.status); const snapshot = await response.json(); const url = URL.createObjectURL(new Blob([JSON.stringify(snapshot, null, 2)], { type: 'application/json' })); download(url, 'inha-control-center-state.json'); setTimeout(() => URL.revokeObjectURL(url), 1000); }
  catch (error) { toast('상태 저장 실패: ' + error.message); }
};

const allowedTypes = { map: ['nav_msgs/msg/OccupancyGrid'], scan: ['sensor_msgs/msg/LaserScan'],
  odom: ['nav_msgs/msg/Odometry'], path: ['nav_msgs/msg/Path'], cloud: ['sensor_msgs/msg/PointCloud2'],
  object_cloud: ['sensor_msgs/msg/PointCloud2'], detections: ['robocup_detection_msgs/msg/HeadTarget', 'yolo_msgs/msg/DetectionArray', 'vision_msgs/msg/Detection2DArray'] };
for (const key of ['head_image', 'head_debug', 'wrist_image', 'wrist_debug']) allowedTypes[key] = ['sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage'];
function openSettings() {
  const fields = $('topic-fields'); fields.replaceChildren();
  for (const [key, types] of Object.entries(allowedTypes)) {
    const row = document.createElement('label'); row.className = 'topic-field';
    const label = document.createElement('span'); label.textContent = names[key];
    const select = document.createElement('select'); select.name = key; select.append(new Option('자동 탐색', ''));
    for (const topic of data.values.topics || []) if (topic.types.some(type => types.includes(type))) select.append(new Option(topic.name, topic.name));
    const m = data.meta[key];
    if (m?.manual && m.topic) {
      if (![...select.options].some(option => option.value === m.topic)) select.append(new Option(m.topic + ' · 발행 대기', m.topic));
      select.value = m.topic;
    }
    row.append(label, select); fields.append(row);
  }
  $('apply-topics').disabled = data.mode !== 'live' || !data.connected;
  text('topic-result', '');
  text('dialog-tip', data.mode === 'live' ? '토픽을 바꾸면 선택한 수신 채널만 재연결합니다. 설정은 서버를 종료할 때 초기화됩니다.' : '샘플 / 대기 모드입니다. 젯슨에서 관제센터를 시작하고 맥에서 SSH 터널을 연결하세요.');
  $('settings-dialog').showModal();
}
$('settings-button').onclick = $('topic-button').onclick = $('nav-topics').onclick = openSettings;
$('close-settings').onclick = () => $('settings-dialog').close();
$('settings-dialog').addEventListener('click', e => { if (e.target === $('settings-dialog')) { const r = e.target.getBoundingClientRect(); if (e.clientX < r.left || e.clientX > r.right || e.clientY < r.top || e.clientY > r.bottom) e.target.close(); } });
$('topic-form').onsubmit = async e => {
  e.preventDefault();
  try {
    const topics = Object.fromEntries(new FormData(e.currentTarget));
    const response = await fetch('/api/select', { method: 'POST', headers: { 'Content-Type': 'application/json', 'X-Control-Token': data.controlToken || '' }, body: JSON.stringify(topics) });
    const result = await response.json(); if (!response.ok) throw new Error(result.error || response.status);
    text('topic-result', '선택을 전달했습니다. ROS 탐색 주기에서 재연결합니다.');
  } catch (error) { text('topic-result', error.message); }
};
new ResizeObserver(draw).observe($('map-stage'));
setInterval(() => { text('clock', new Date().toLocaleTimeString('ko-KR', { hour12: false, timeZone: 'Asia/Seoul' })); if (data.received && performance.now() - data.received > 2500) { data.connected = false; render(); } }, 1000);
connect();
