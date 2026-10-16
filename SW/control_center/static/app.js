'use strict';
const $ = id => document.getElementById(id);
const data = { values: {}, meta: {}, versions: {}, mode: '', connected: false, received: 0, logs: [], uptime: 0 };
const view = { kind: '2d', x: 0, y: 0, scale: 43, follow: false, yaw: 0, pitch: .65, distance: 14,
  center: [0, 0, 0], image: null, mapVersion: 0, fitted: false, cloudFitted: '', width: 0, height: 0 };
let source, session = 0, lastLog = '', toastTimer;
const cameras = {};
const pick = { mode: null, start: null, end: null };  // map click tool: 'goal' | 'initial'
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
  if (m?.error) return 'error';
  if (!m || m.age == null || data.values[key] == null) return 'waiting';
  if (!data.connected || performance.now() - data.received > 2500) return 'stale';
  if (['map', 'path', 'object_cloud'].includes(key)) return data.mode === 'demo' || m.publishers > 0 ? 'cached' : 'stale';
  return m.age <= 2.5 && (m.source_age == null || (m.source_age >= -.5 && m.source_age <= 2.5)) ? 'live' : 'stale';
}
function fresh(key) { return ['live', 'cached'].includes(quality(key)); }
function badge(id, label, state) { text(id, label); $(id).className = 'tiny-pill' + (state === 'live' || state === 'cached' ? ' active' : state === 'stale' || state === 'error' ? ' warn' : ''); }
function connect() {
  if (source) source.close();
  data.shutdownRequested = false;
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
      if (payload.shutting_down) {
        data.shutdownRequested = true;
        source.close();
        data.connected = false;
      }
      Object.assign(data.values, payload.changes);
      const map = data.values.map;
      if (map && payload.changes.map && view.mapVersion !== data.versions.map) {
        view.mapVersion = data.versions.map;
        const version = view.mapVersion, image = new Image();
        image.onload = () => { if (version !== view.mapVersion) return; view.image = image; view.known = knownBounds(image, map); if (!view.fitted || !view.touched) { fitMap(); view.fitted = true; } draw(); };
        image.src = map.image;
      } else if (!map) { view.image = null; view.known = null; }
      render();
    } catch (error) { toast('수신 데이터를 읽을 수 없습니다: ' + error.message); }
  };
  source.onerror = () => { data.connected = false; render(); };
}

function render() {
  const demo = data.mode === 'demo';
  const alive = !data.shutdownRequested && data.connected && performance.now() - data.received < 2500;
  $('shutdown-button').disabled = data.mode !== 'live' || !alive || Boolean(data.shutdownPending);
  text('shutdown-button', data.shutdownRequested ? '종료 요청됨' : data.shutdownPending ? '종료 요청 중…' : '서버 종료');
  $('connection').className = 'connection ' + (alive ? 'connected' : 'disconnected');
  $('connection').replaceChildren(Object.assign(document.createElement('i'), {}), document.createTextNode(alive ? '서버 연결됨' : '서버 연결 끊김'));
  text('mode-badge', demo ? 'SAMPLE MODE' : data.mode === 'live' ? 'ROS LIVE' : '입력 대기');
  $('mode-badge').className = 'badge ' + (data.mode === 'live' && alive ? 'green' : 'amber');
  $('notice').classList.toggle('live', data.mode === 'live' && alive);
  text('notice-text', data.shutdownRequested ? '서버 종료를 요청했습니다. 센서·SLAM·검출 프로세스를 정리합니다. 다시 사용하려면 젯슨에서 서버를 시작하세요.' : !alive ? '데이터 연결이 끊겼습니다. 마지막 수신 화면은 실시간 상태가 아닙니다.' : demo
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
  renderDetections(); renderSensors(); renderSensorControls(); renderCanOdometry(); renderServices(); renderManipulation(); renderLogs();
  text('session-time', 'SESSION ' + elapsed(data.uptime));
  $('save-map-button').disabled = !map;
  if (view.follow && p && fresh('pose') && p.frame === currentFrame() && view.kind === '2d') { view.x = p.x; view.y = p.y; }
  draw();
}

function renderCanOdometry() {
  const can = data.values.can_status, odometry = data.values.odometry_health;
  const live = data.mode === 'live';
  const canFresh = live && quality('can_status') === 'live';
  const odomFresh = live && quality('odometry_health') === 'live';
  const healthy = canFresh && odomFresh && odometry?.state === 'ok';
  const status = !live ? '실물 입력 대기' : !canFresh || !odomFresh ? '연결 대기 / 지연' : healthy ? '정상 수신' : '입력 대기 / 이상';
  badge('odom-health-status', status, healthy ? 'live' : live ? 'stale' : 'waiting');
  text('can-mode', canFresh ? can.automatic ? '자동 연결 사용 중' : '관측 전용' : 'CAN 상태 수신 대기');
  const container = $('can-readings'); container.replaceChildren();
  function row(title, summary, detail, good) {
    const card = document.createElement('article'); card.className = 'can-reading' + (good ? ' receiving' : '');
    const label = document.createElement('strong'); label.textContent = title;
    const value = document.createElement('span'); value.textContent = summary;
    const description = document.createElement('small'); description.textContent = detail;
    card.title = detail; card.append(label, value, description); container.append(card);
  }
  for (const [key, title] of [['tracer', 'Tracer CAN'], ['piper', 'PiPER CAN']]) {
    const c = can?.interfaces?.[key];
    const labels = { missing: '장치 없음', down: '연결 준비', bus_off: 'BUS-OFF', mismatch: 'bitrate 불일치', error: '연결 오류' };
    const summary = !canFresh ? '상태 수신 대기 / 지연' : c?.ready ? c.receiving ? 'CAN 데이터 수신 중' : '인터페이스 연결 · 수신 대기' : labels[c?.state] || '입력 대기';
    row(title + ' (설정)', summary, c ? `${c.interface}${c.hardware ? ' · ' + c.hardware : ''} · ${fmt((c.bitrate ?? c.expected_bitrate) / 1000, 0)} kbps · RX ${c.rx_packets ?? '—'}${c.error ? ' · ' + c.error : ''}${c.bus_state ? ' · ' + c.bus_state : ''}` : 'CAN 입력 대기', canFresh && c?.ready && c.receiving);
  }
  const arm = data.values.piper_feedback, armFresh = live && quality('piper_feedback') === 'live';
  const armGood = armFresh && arm?.state === 'live';
  const armReasons = { waiting_for_joint_frames: '관절 응답 없음', missing_joint_frames: '관절 응답 누락 / 지연',
    incomplete_joint_sample: '관절 데이터 시각 불일치', invalid_joint_angles: '관절 각도 형식 오류', invalid_status: '상태 형식 오류' };
  row('PiPER 관절 피드백', armGood ? '6개 관절 실측 수신 중' : !armFresh ? '피드백 노드 입력 대기 / 지연' : armReasons[arm.reason] || '관절 수신 대기 / 이상',
    `/piper/joint_states_feedback · 누락 ${(arm?.missing_frames || []).join(', ') || '없음'} · ${arm?.reason || '노드 대기'}`, armGood);
  for (const [key, title, topic] of [['wheel', '바퀴 오도메트리', '/odom'], ['fused', '융합 오도메트리', '/odom_ekf']]) {
    const o = odometry?.[key];
    const good = odomFresh && o?.state === 'live';
    const labels = { waiting: '입력 대기', invalid: '좌표계 / 수치 오류', stale: '수신 / timestamp 지연' };
    row(title, good ? `${fmt(o.hz)} Hz · ${fmt(o.linear, 2)} m/s` : !odomFresh ? '상태 수신 대기 / 지연' : labels[o?.state] || '입력 대기',
      `${topic} · 원본 ${fmt(o?.source_hz)}Hz / 수신 ${fmt(o?.receive_hz ?? o?.hz)}Hz · 최대 원본 간격 ${fmt(o?.source_gap_max_ms)}ms · yaw ${fmt(o?.yaw * 180 / Math.PI)}° · 회전 ${fmt(o?.angular, 3)} rad/s · 지연 ${fmt(o?.source_age)}s`, good);
  }
  const rotation = odometry?.rotation, gyro = rotation?.imu;
  const gyroGood = odomFresh && gyro?.state === 'live';
  row('회전 입력', gyroGood ? 'MID-360 IMU 사용' : odomFresh && rotation?.source === 'wheel_fallback' ? '바퀴 각속도 대체 · IMU 대기' : '회전 입력 대기 / 지연',
    `/imu/base · ${fmt(gyro?.angular, 3)} rad/s · 지연 ${fmt(gyro?.source_age)}s`, gyroGood);
  const tf = odometry?.tf, tfGood = odomFresh && tf?.state === 'live';
  row('오도메트리 TF', tfGood ? '정상 수신' : 'TF 입력 대기 / 지연', `odom → ${data.values.odometry_health?.wheel?.child_frame || 'base_link'} · 지연 ${fmt(tf?.age)}s${tf?.error ? ' · ' + tf.error : ''}`, tfGood);
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
  text('target-summary', d?.mode === 'all_objects'
    ? `6종 전체 객체 검출 · Camera ${fmt(d.camera_fps)} FPS · YOLO ${fmt(d.detection_fps)} FPS${d.processing_ms != null ? ' · ' + fmt(d.processing_ms) + ' ms' : ''}${d.reason ? ' · ' + d.reason : ''}${q === 'stale' ? ' · 과거 결과' : ''}`
    : d ? `${d.valid ? '유효 관측' : '후보 / 무효 관측'} · ${d.measured ? 'YOLO 측정' : '추적 중 · 마지막 YOLO 신뢰도'}${d.processing_ms != null ? ' · ' + fmt(d.processing_ms) + ' ms' : ''}${q === 'stale' ? ' · 과거 결과' : ''}` : 'green_apple · peach · tomato · fanta_can · cup · banana 전체 검출 대기');
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
    label.textContent = names[key]; topic.textContent = m?.error || m?.topic || '발행 토픽 대기'; topic.title = m?.error || m?.topic || '';
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
    ui.hint.title = sensor?.log || ''; ui.card.title = ui.hint.textContent + (sensor?.log ? ' · ' + sensor.log : '');
    ui.start.disabled = !alive || !sensor?.can_start || sensorPending.has(key);
    ui.stop.disabled = !alive || !sensor?.can_stop || sensorPending.has(key);
  }
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
    const stamp = document.createElement('time'); stamp.textContent = new Date(item.time * 1000).toLocaleTimeString('en-GB', { hour12: false });
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
function knownBounds(image, m) {
  // Bounding box of observed cells (anything but the unknown gray 95) in the map frame.
  try {
    const c = document.createElement('canvas'); c.width = image.width; c.height = image.height;
    const g = c.getContext('2d'); g.drawImage(image, 0, 0); const px = g.getImageData(0, 0, c.width, c.height).data;
    let u0 = Infinity, v0 = Infinity, u1 = -1, v1 = -1;
    for (let v = 0; v < c.height; v++) for (let u = 0; u < c.width; u++) if (px[(v * c.width + u) * 4] !== 95) { if (u < u0) u0 = u; if (u > u1) u1 = u; if (v < v0) v0 = v; if (v > v1) v1 = v; }
    if (u1 < 0) return null;
    const [x, y, a] = m.origin, r = m.resolution;
    const corners = [[u0, v0], [u1 + 1, v0], [u0, v1 + 1], [u1 + 1, v1 + 1]].map(([u, v]) => [x + Math.cos(a) * u * r - Math.sin(a) * v * r, y + Math.sin(a) * u * r + Math.cos(a) * v * r]);
    return [Math.min(...corners.map(k => k[0])), Math.min(...corners.map(k => k[1])), Math.max(...corners.map(k => k[0])), Math.max(...corners.map(k => k[1]))];
  } catch { return null; }
}
function fitMap() {
  const b = view.known || mapBounds(); view.x = (b[0] + b[2]) / 2; view.y = (b[1] + b[3]) / 2;
  view.scale = Math.max(2, Math.min((view.width - 40) / Math.max(b[2] - b[0], .1), (view.height - 40) / Math.max(b[3] - b[1], .1)));
}
function fitCloud(resetOrientation = false) {
  const cloud = data.values[$('cloud-source').value];
  if (!cloud?.points.length) return;
  if (resetOrientation) { view.yaw = 0; view.pitch = .65; }
  let min = [Infinity, Infinity, Infinity], max = [-Infinity, -Infinity, -Infinity];
  for (const p of cloud.points) for (let i = 0; i < 3; i++) { min[i] = Math.min(min[i], p[i]); max[i] = Math.max(max[i], p[i]); }
  view.center = min.map((v, i) => (v + max[i]) / 2);
  view.distance = Math.max(.4, Math.hypot(...min.map((v, i) => max[i] - v)) * .95);
}
function fit() { view.touched = false; if (view.kind === '2d') fitMap(); else fitCloud(true); draw(); }
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
function collisionMarker(frame) {
  const safety = data.values.collision_safety, robot = data.values.pose;
  if (!safety || !fresh('collision_safety')) return null;
  const blocking = Boolean(safety.blocking_point);
  const source = blocking ? safety.blocking_source_point : safety.nearest_source_point;
  const point = source || (blocking ? safety.blocking_point : safety.nearest_point);
  if (!point) return null;
  let placed;
  if (frame === safety.frame) placed = point;
  else if (robot?.frame === frame && safety.robot_pose_odom?.length === 3) {
    const origin = safety.robot_pose_odom, yaw = robot.yaw-origin[2], c = Math.cos(yaw), s = Math.sin(yaw);
    const dx = point[0]-origin[0], dy = point[1]-origin[1];
    placed = [robot.x+c*dx-s*dy, robot.y+s*dx+c*dy, point[2]];
  } else if (!blocking && safety.display_frame === frame) placed = safety.display_point;
  if (!placed) return null;
  const age = blocking ? safety.blocking_source_age : safety.nearest_source_age;
  const height = blocking ? safety.blocking_height_above_floor : safety.nearest_height_above_floor;
  const remembered = Number.isFinite(age) && age > .4;
  const label = `${blocking ? '경로 장애물' : '최근접'}${Number.isFinite(height) ? ' · 바닥 +' + fmt(height*100, 0) + 'cm' : ''}${remembered ? ' · 기억 ' + fmt(age, 1) + '초' : ''}`;
  return {point: placed, height, label, color: blocking ? '#ff8c73' : remembered ? '#a3adb7' : '#3ec7cf', blocking};
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
  const safety = data.values.collision_safety;
  if (p?.frame === frame && safety && fresh('collision_safety')) {
    const c = Math.cos(p.yaw), s = Math.sin(p.yaw);
    const polygon = points => points?.map(([x, y]) => [p.x+c*x-s*y, p.y+s*x+c*y]);
    const outline = polygon((pts => pts?.map(([x, y]) => [x - Math.sign(x)*Math.min(.10, Math.abs(x)), y - Math.sign(y)*Math.min(.10, Math.abs(y))]))(safety.body_footprint)), clearance = polygon(safety.clearance_footprint);
    if (outline?.length) worldLine([...outline, outline[0]], '#b8ed7a', 1.5);
    if (clearance?.length && safety.clearance > 0) worldLine([...clearance, clearance[0]], safety.state === 'blocked' ? '#ff8c73' : '#e5bb71', 1.5, true);
    const marker = collisionMarker(frame);
    if (marker) {
      const [x, y] = world(marker.point);
      ctx.fillStyle = marker.color; ctx.beginPath(); ctx.arc(x, y, marker.blocking ? 4 : 3, 0, Math.PI*2); ctx.fill();
      ctx.font = '11px sans-serif'; ctx.fillText(marker.label, x+8, y-8);
    }
  }
  if (p?.frame === frame) {
    const [x, y] = world([p.x, p.y]), live = fresh('pose');
    ctx.save(); ctx.translate(x, y); ctx.rotate(-p.yaw);
    const size = Math.max(12, Math.min(27, .48 * view.scale));
    ctx.strokeStyle = live ? '#b8ed7a44' : '#f0b86955'; ctx.lineWidth = 1; ctx.beginPath(); ctx.arc(0, 0, size + 8, 0, Math.PI * 2); ctx.stroke();
    ctx.fillStyle = live ? '#b8ed7a' : '#f0b869'; ctx.strokeStyle = '#173225'; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(size * .7, 0); ctx.lineTo(-size * .55, -size * .45); ctx.lineTo(-size * .25, 0); ctx.lineTo(-size * .55, size * .45); ctx.closePath(); ctx.fill(); ctx.stroke(); ctx.restore();
  }
  const goal = data.values.navigation?.goal;
  const graspPlan = data.values.detection_control?.plan;
  if (fresh('detection_control') && graspPlan?.frame === frame) {
    const corners = graspPlan.table_corners_map;
    if (corners?.length) worldLine([...corners, corners[0]], '#62d9ce88', 1.5);
    if (graspPlan.dock_map) drawArrow(graspPlan.dock_map, graspPlan.dock_map[2], '#62d9ce', graspPlan.boundary_verified ? '파지 위치' : '파지 후보 · 경계 관측 대기');
  }
  const objectTarget = data.values.detection_control?.target;
  if (fresh('detection_control') && objectTarget?.frame === frame && Array.isArray(objectTarget.point_map)) {
    ctx.fillStyle = objectTarget.visible ? '#62d9ce88' : '#e5bb7188';
    for (const point of objectTarget.lidar_points_map || []) {
      const [x, y] = world(point); ctx.fillRect(x-1.5, y-1.5, 3, 3);
    }
    const [x, y] = world(objectTarget.point_map);
    ctx.strokeStyle = objectTarget.visible ? '#62d9ce' : '#e5bb71'; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.arc(x, y, 7, 0, Math.PI*2); ctx.stroke();
    ctx.fillStyle = ctx.strokeStyle; ctx.font = '12px sans-serif';
    ctx.fillText(`${objectTarget.label} · ${objectTarget.visible ? 'YOLO + LiDAR' : '저장 LiDAR'}`, x+11, y-11);
  }
  if (goal?.frame === frame) drawArrow([goal.x, goal.y], goal.yaw, ['active', 'sending', 'canceling'].includes(data.values.navigation.state) ? '#c18bdf' : '#c18bdf77', '목표');
  if (pick.start) {
    const [dx, dy] = [pick.end[0] - pick.start[0], pick.end[1] - pick.start[1]];
    drawArrow(pick.start, Math.hypot(dx, dy) * view.scale > 6 ? Math.atan2(dy, dx) : pickDefaultYaw(pick.start), '#c18bdf', '목표');
  }
  const useful = Boolean(m || scan?.frame === frame || p?.frame === frame);
  $('canvas-empty').classList.toggle('hidden', useful);
  text('frame-tag', 'FRAME / ' + frame);
  const q = quality('map'); badge('map-status', m ? q === 'cached' ? 'MAP LOADED' : q.toUpperCase() : 'WAITING SLAM', q);
  text('map-detail', m ? `${fmt(m.resolution, 3)} m/cell · ${fmt(m.known_area)} m²` : `${frame} · /map 입력 대기`);
  const unit = spacing(55 / view.scale); $('scale-bar').style.width = unit * view.scale + 'px'; text('scale-label', fmt(unit, unit < 1 ? 2 : 0) + ' m');
}
function drawArrow(point, angle, color, label) {
  const [x, y] = world(point), size = Math.max(14, Math.min(30, .5 * view.scale));
  ctx.save(); ctx.translate(x, y);
  ctx.strokeStyle = color; ctx.fillStyle = color; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.arc(0, 0, 4, 0, Math.PI * 2); ctx.fill();
  ctx.rotate(-angle); ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(size, 0); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(size + 6, 0); ctx.lineTo(size - 3, -5); ctx.lineTo(size - 3, 5); ctx.closePath(); ctx.fill();
  ctx.restore(); ctx.fillStyle = color; ctx.font = '10px ui-monospace, monospace'; ctx.fillText(label, x + 8, y - 8);
}
function project(point) {
  const dx = point[0] - view.center[0], dy = point[1] - view.center[1], dz = point[2] - view.center[2];
  const rx = Math.cos(view.yaw) * dx - Math.sin(view.yaw) * dy, ry = Math.sin(view.yaw) * dx + Math.cos(view.yaw) * dy;
  // Right-handed camera on the -Y/+Z side of the scene. At yaw=0 the
  // ground's +X goes right and +Y goes up, matching the 2D map convention.
  const vertical = Math.sin(view.pitch) * ry + Math.cos(view.pitch) * dz;
  const depth = view.distance + Math.cos(view.pitch) * ry - Math.sin(view.pitch) * dz;
  if (depth < .02) return null;
  const focal = Math.min(view.width, view.height) * .95;
  return [view.width / 2 + rx * focal / depth, view.height / 2 - vertical * focal / depth, depth];
}
function line3d(a, b, color, width = 1) {
  const p = project(a), q = project(b); if (!p || !q) return;
  ctx.strokeStyle = color; ctx.lineWidth = width; ctx.beginPath(); ctx.moveTo(p[0], p[1]); ctx.lineTo(q[0], q[1]); ctx.stroke();
}
function drawCloud() {
  const key = $('cloud-source').value, cloud = data.values[key];
  if (cloud?.points.length && view.cloudFitted !== key + cloud.frame) { fitCloud(); view.cloudFitted = key + cloud.frame; }
  const grid = spacing(view.distance / 14), span = grid * 6;
  const floor = key === 'cloud' && cloud?.frame !== cloud?.source_frame
    ? data.values.collision_safety?.cloud?.floor_z ?? -.1425 : 0;
  for (let i = -6; i <= 6; i++) {
    line3d([view.center[0] + i * grid, view.center[1] - span, floor], [view.center[0] + i * grid, view.center[1] + span, floor], '#253b4a');
    line3d([view.center[0] - span, view.center[1] + i * grid, floor], [view.center[0] + span, view.center[1] + i * grid, floor], '#253b4a');
  }
  const axis = Math.max(.1, view.distance / 9);
  line3d([0, 0, 0], [axis, 0, 0], '#df847f'); line3d([0, 0, 0], [0, axis, 0], '#acd477'); line3d([0, 0, 0], [0, 0, axis], '#7c9fe4');
  for (const [tip, label, color] of [[[axis, 0, 0], '+X', '#df847f'], [[0, axis, 0], '+Y', '#acd477'], [[0, 0, axis], '+Z', '#7c9fe4']]) {
    const point = project(tip);
    if (point) { ctx.fillStyle = color; ctx.font = '11px sans-serif'; ctx.fillText(label, point[0]+5, point[1]); }
  }
  if (cloud) {
    ctx.globalAlpha = fresh(key) ? .9 : .3;
    for (const point of cloud.points) {
      const p = project(point); if (!p || p[0] < 0 || p[0] > view.width || p[1] < 0 || p[1] > view.height) continue;
      ctx.fillStyle = `hsl(${190 - Math.max(0, Math.min(1, (point[2] - view.center[2] + 1) / 3)) * 95},65%,66%)`;
      const size = cloud.point_size || 3; ctx.fillRect(p[0] - size / 2, p[1] - size / 2, size, size);
    }
    ctx.globalAlpha = 1;
  }
  const robot = data.values.pose, body = (pts => pts?.map(([x, y]) => [x - Math.sign(x)*Math.min(.10, Math.abs(x)), y - Math.sign(y)*Math.min(.10, Math.abs(y))]))(data.values.collision_safety?.body_footprint);  // 표시용 몸체 각 변 10 cm 안쪽 (Nav2 와 같게)
  const collision = collisionMarker(cloud?.frame);
  if (collision) {
    const tip = project(collision.point);
    if (tip) {
      const bottom = [...collision.point]; bottom[2] -= Number.isFinite(collision.height) ? collision.height : collision.point[2]-floor;
      line3d(bottom, collision.point, collision.color);
      ctx.fillStyle = collision.color; ctx.beginPath(); ctx.arc(tip[0], tip[1], 5, 0, Math.PI*2); ctx.fill();
      ctx.font = '11px sans-serif'; ctx.fillText(collision.label, tip[0]+8, tip[1]-8);
    }
  }
  if (robot && robot.frame === cloud?.frame) {
    const c = Math.cos(robot.yaw), s = Math.sin(robot.yaw), z = floor+.02;
    const place = ([x, y]) => [robot.x+c*x-s*y, robot.y+s*x+c*y, z];
    if (body?.length) for (let i = 0; i < body.length; i++) line3d(place(body[i]), place(body[(i+1)%body.length]), '#b8ed7a', 2);
    const origin = [robot.x, robot.y, z], tip = place([.55, 0]);
    line3d(origin, tip, '#b8ed7a', 2);
    line3d(tip, place([.42, .08]), '#b8ed7a', 2); line3d(tip, place([.42, -.08]), '#b8ed7a', 2);
    const marker = project(origin);
    if (marker) { ctx.fillStyle = '#b8ed7a'; ctx.beginPath(); ctx.arc(marker[0], marker[1], 4, 0, Math.PI*2); ctx.fill(); ctx.font = '11px sans-serif'; ctx.fillText('로봇', marker[0]+8, marker[1]-8); }
  }
  $('canvas-empty').classList.toggle('hidden', Boolean(cloud?.points.length));
  $('canvas-empty').querySelector('strong').textContent = '3D 점군 입력을 기다리고 있습니다';
  $('canvas-empty').querySelector('p').textContent = data.meta[key]?.error || (key === 'cloud'
    ? data.values.sensor_control?.mid360_lidar?.error || 'MID-360 PointCloud2 수신 대기 · 상단 LiDAR 실행 상태를 확인하세요.'
    : 'Wrist 물체 점군 수신 대기');
  text('frame-tag', 'FRAME / ' + (cloud?.frame || '—'));
  badge('map-status', cloud ? fresh(key) ? key === 'object_cloud' ? 'CACHED CLOUD' : 'POINT CLOUD' : 'STALE' : 'WAITING', quality(key));
  text('map-detail', cloud ? `${cloud.points.length.toLocaleString()} points · ${cloud.decay_sec ? fmt(cloud.decay_sec) + 's 누적 · ' + cloud.accumulated_frames + '프레임' : '현재 프레임'} · ${cloud.frame}${cloud.accumulation_warning ? ' · ' + cloud.accumulation_warning : ''}` : 'NO POINT CLOUD');
  $('scale-bar').style.width = '55px'; text('scale-label', 'XYZ / m');
}

function setView(kind) {
  view.kind = kind;
  if (kind === '3d') view.pitch = Math.max(.12, Math.min(1.45, view.pitch));
  $('view-2d').classList.toggle('selected', kind === '2d'); $('view-3d').classList.toggle('selected', kind === '3d');
  $('map-layers').classList.toggle('hidden', kind === '3d'); $('cloud-source').classList.toggle('hidden', kind === '2d');
  $('follow-button').disabled = kind === '3d';
  text('map-coordinates', kind === '2d' ? pick.mode ? pickHint() : '드래그로 이동 · 스크롤로 확대' : '드래그로 회전 · 스크롤로 확대');
  if (kind === '2d') {
    $('canvas-empty').querySelector('strong').textContent = '지도 입력을 기다리고 있습니다';
    $('canvas-empty').querySelector('p').textContent = '/map과 map → base_link TF를 기다립니다.';
  }
  draw();
}
let dragging = null;
function screenToWorld(e) { const r = canvas.getBoundingClientRect(); return [view.x + (e.clientX - r.left - view.width / 2) / view.scale, view.y - (e.clientY - r.top - view.height / 2) / view.scale]; }
canvas.addEventListener('pointerdown', e => {
  canvas.setPointerCapture(e.pointerId);
  if (pick.mode && view.kind === '2d') { pick.start = pick.end = screenToWorld(e); draw(); return; }
  dragging = [e.clientX, e.clientY];
});
canvas.addEventListener('pointermove', e => {
  if (pick.start) { pick.end = screenToWorld(e); draw(); return; }
  if (!dragging) return;
  const dx = e.clientX - dragging[0], dy = e.clientY - dragging[1]; dragging = [e.clientX, e.clientY];
  if (view.kind === '2d') { view.touched = true; view.x -= dx / view.scale; view.y += dy / view.scale; view.follow = false; $('follow-button').setAttribute('aria-pressed', 'false'); }
  else { view.yaw += dx * .007; view.pitch = Math.max(.12, Math.min(1.45, view.pitch + dy * .006)); }
  draw();
});
canvas.addEventListener('pointerup', e => { dragging = null; if (pick.start) { pick.end = screenToWorld(e); finishPick(); } });
canvas.addEventListener('pointercancel', () => { dragging = null; pick.start = pick.end = null; draw(); });
function zoom(factor, x = view.width / 2, y = view.height / 2) {
  if (view.kind === '3d') view.distance = Math.max(.1, Math.min(1000, view.distance / factor));
  else {
    view.touched = true;
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
$('view-2d').onclick = () => setView('2d');
$('view-3d').onclick = () => { setPick(null); setView('3d'); };
$('cloud-source').onchange = () => { view.cloudFitted = ''; draw(); };
for (const id of ['layer-scan', 'layer-path', 'layer-trail']) $(id).onchange = draw;
for (const role of ['head', 'wrist']) $(role + '-source').onchange = () => updateCamera(role);
$('reconnect-button').onclick = connect;
$('shutdown-button').onclick = async () => {
  if (data.shutdownPending || data.shutdownRequested) return;
  data.shutdownPending = true;
  render();
  try {
    const response = await fetch('/api/shutdown', {
      method: 'POST', headers: { 'Content-Type': 'application/json', 'X-Control-Token': data.controlToken || '' },
      body: JSON.stringify({ action: 'shutdown' })
    });
    const result = await response.json();
    if (!response.ok) throw new Error(result.error || response.status);
    data.shutdownRequested = true;
    data.connected = false;
    source?.close();
    for (const role of ['head', 'wrist']) $(role + '-camera').removeAttribute('src');
    toast('서버 종료를 요청했습니다.');
  } catch (error) {
    toast('서버 종료 요청 실패: ' + error.message);
  } finally {
    data.shutdownPending = false;
    render();
  }
};
function download(url, name) { const link = document.createElement('a'); link.href = url; link.download = name; document.body.append(link); link.click(); link.remove(); }
$('save-map-button').onclick = () => download('/api/map.png', 'slam-map.png');
$('export-button').onclick = async () => {
  try { const response = await fetch('/api/snapshot'); if (!response.ok) throw new Error(response.status); const snapshot = await response.json(); const url = URL.createObjectURL(new Blob([JSON.stringify(snapshot, null, 2)], { type: 'application/json' })); download(url, 'inha-control-center-state.json'); setTimeout(() => URL.revokeObjectURL(url), 1000); }
  catch (error) { toast('상태 저장 실패: ' + error.message); }
};

const allowedTypes = { map: ['nav_msgs/msg/OccupancyGrid'], scan: ['sensor_msgs/msg/LaserScan'],
  odom: ['nav_msgs/msg/Odometry'], path: ['nav_msgs/msg/Path'], cloud: ['sensor_msgs/msg/PointCloud2'],
  object_cloud: ['sensor_msgs/msg/PointCloud2'], detections: ['std_msgs/msg/String', 'robocup_detection_msgs/msg/HeadTarget', 'yolo_msgs/msg/DetectionArray', 'vision_msgs/msg/Detection2DArray'] };
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
$('settings-button').onclick = $('topic-button').onclick = openSettings;
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
// Panel fullscreen: browser Fullscreen API on the whole panel (toolbar and selectors stay usable); a fixed full-window
// layout where the API is missing or refused (e.g. iPhone Safari). Esc or the same button leaves it.
const nativeFullscreen = () => document.fullscreenElement || document.webkitFullscreenElement || null;
const fullscreenPanel = () => nativeFullscreen() || document.querySelector('.panel.pseudo-fullscreen');
function syncFullscreen() {
  const current = fullscreenPanel();
  for (const panel of document.querySelectorAll('.panel')) {
    panel.classList.toggle('is-fullscreen', panel === current);
    if (panel !== current) panel.classList.remove('pseudo-fullscreen');
  }
  document.body.classList.toggle('has-pseudo-fullscreen', Boolean(document.querySelector('.panel.pseudo-fullscreen')));
  for (const button of document.querySelectorAll('.fullscreen-button')) {
    const active = button.closest('.panel') === current;
    button.setAttribute('aria-pressed', String(active)); button.title = active ? '전체화면 종료 (Esc)' : '전체화면';
  }
  requestAnimationFrame(draw);
}
async function leaveFullscreen() {
  if (nativeFullscreen()) { try { await (document.exitFullscreen || document.webkitExitFullscreen).call(document); } catch { /* already left */ } }
  document.querySelector('.panel.pseudo-fullscreen')?.classList.remove('pseudo-fullscreen');
  syncFullscreen();
}
async function toggleFullscreen(panel) {
  const current = fullscreenPanel();
  if (current) { await leaveFullscreen(); if (current === panel) return; }
  const request = panel.requestFullscreen || panel.webkitRequestFullscreen;
  if (request) {
    // Some embedded browsers neither grant nor reject the request; fall back if it has not taken effect shortly.
    try { await Promise.race([request.call(panel), new Promise(resolve => setTimeout(resolve, 700))]); } catch { /* refused */ }
    if (nativeFullscreen() === panel) return;
  }
  panel.classList.add('pseudo-fullscreen'); syncFullscreen();
}
for (const button of document.querySelectorAll('.fullscreen-button')) button.onclick = () => toggleFullscreen(button.closest('.panel'));
for (const frame of document.querySelectorAll('.camera-frame')) frame.addEventListener('dblclick', () => toggleFullscreen(frame.closest('.panel')));
document.addEventListener('fullscreenchange', syncFullscreen); document.addEventListener('webkitfullscreenchange', syncFullscreen);
document.addEventListener('keydown', e => { if (e.key === 'Escape' && document.querySelector('.panel.pseudo-fullscreen') && !$('settings-dialog').open) leaveFullscreen(); });
new ResizeObserver(() => { const r = $('map-stage').getBoundingClientRect(); view.width = r.width; view.height = r.height; if (view.kind === '2d' && !view.touched && data.values.map) fitMap(); draw(); }).observe($('map-stage'));
setInterval(() => { text('clock', new Date().toLocaleTimeString('ko-KR', { hour12: false, timeZone: 'Asia/Seoul' })); if (data.received && performance.now() - data.received > 2500) { data.connected = false; render(); } }, 1000);

// ---- Modes (관측 / SLAM / NAV) ----
const modes = { observe: 1, slam: 1, nav: 1, detection: 1, manipulation: 1 };
let mode = 'observe';
function setMode(next) {
  mode = modes[next] ? next : 'observe';
  for (const el of document.querySelectorAll('[data-mode-button]')) {
    const on = el.dataset.modeButton === mode;
    el.classList.toggle('active', on); if (el.getAttribute('role') === 'tab') el.setAttribute('aria-selected', String(on));
  }
  for (const el of document.querySelectorAll('.mode-section')) el.classList.toggle('hidden', el.dataset.mode !== mode);
  if (mode !== 'observe' && view.kind !== '2d') setView('2d');
  if (mode !== 'nav') setPick(null);
  try { localStorage.setItem('inha-cc-mode', mode); } catch { /* storage unavailable */ }
  renderServices(); requestAnimationFrame(draw);
}
for (const el of document.querySelectorAll('[data-mode-button]')) el.onclick = () => setMode(el.dataset.modeButton);

const pickHint = () => '목표 지정 · 클릭 후 드래그로 방향 · Esc 취소';
function pickDefaultYaw(point) {
  const p = data.values.pose;
  return p ? Math.atan2(point[1] - p.y, point[0] - p.x) : 0;
}
function setPick(next) {
  pick.mode = next; pick.start = pick.end = null;
  for (const el of document.querySelectorAll('[data-pick]')) el.setAttribute('aria-pressed', String(el.dataset.pick === next));
  $('map-stage').classList.toggle('picking', Boolean(next));
  if (next && view.kind !== '2d') setView('2d');
  text('map-coordinates', view.kind === '2d' ? next ? pickHint() : '드래그로 이동 · 스크롤로 확대' : '드래그로 회전 · 스크롤로 확대');
  draw();
}
async function post(url, body) {
  const response = await fetch(url, { method: 'POST', headers: { 'Content-Type': 'application/json', 'X-Control-Token': data.controlToken || '' }, body: JSON.stringify(body) });
  const result = await response.json(); if (!response.ok) throw new Error(result.error || response.status);
  return result;
}
const servicePending = new Set();
async function serviceAction(url, body, message, key) {
  servicePending.add(key); renderServices();
  try { await post(url, body); if (message) toast(message); return true; }
  catch (error) { toast(error.message); return false; }
  finally { servicePending.delete(key); renderServices(); }
}
async function finishPick() {
  const start = pick.start, end = pick.end;
  const [dx, dy] = [end[0] - start[0], end[1] - start[1]];
  const yaw = Math.hypot(dx, dy) * view.scale > 6 ? Math.atan2(dy, dx) : pickDefaultYaw(start);
  setPick(null);
  if (currentFrame() !== 'map') { toast('map 좌표의 지도가 있어야 지정할 수 있습니다.'); return; }
  await serviceAction('/api/navigation', { action: 'goal', x: start[0], y: start[1], yaw },
    `목표 전송 (${fmt(start[0], 2)}, ${fmt(start[1], 2)}) ${fmt(yaw * 180 / Math.PI, 0)}°`, 'pick');
}
document.addEventListener('keydown', e => { if (e.key === 'Escape' && pick.mode) setPick(null); });
for (const el of document.querySelectorAll('[data-pick]')) el.onclick = () => setPick(pick.mode === el.dataset.pick ? null : el.dataset.pick);

const defaultMapName = () => { const d = new Date(), z = n => String(n).padStart(2, '0'); return `map_${d.getFullYear()}${z(d.getMonth() + 1)}${z(d.getDate())}_${z(d.getHours())}${z(d.getMinutes())}`; };
$('map-name').value = defaultMapName();
$('slam-start').onclick = () => serviceAction('/api/mapping', { action: 'start' }, 'SLAM 시작을 요청했습니다.', 'mapping');
$('slam-stop').onclick = () => { if (confirm('SLAM을 중지할까요? 저장하지 않은 지도는 사라집니다.')) serviceAction('/api/mapping', { action: 'stop' }, 'SLAM 중지를 요청했습니다.', 'mapping'); };
$('slam-reset').onclick = () => { if (confirm('지도를 초기화할까요? 저장하지 않은 지도는 사라지고 현재 위치에서 새로 시작합니다.')) serviceAction('/api/mapping', { action: 'reset' }, '지도를 초기화합니다.', 'mapping'); };
$('map-save').onclick = () => {
  const name = $('map-name').value.trim();
  if (!/^[A-Za-z0-9_-]{1,48}$/.test(name)) { toast('지도 이름은 영문·숫자·_·- 1~48자입니다.'); return; }
  if ((data.values.service_control?.maps?.items || []).some(m => m.name === name) && !confirm(name + ' 지도를 덮어쓸까요?')) return;
  serviceAction('/api/mapping', { action: 'save', name }, '지도 저장을 요청했습니다.', 'save');
};
async function localize(name) {
  if (!name) { toast('위치추정에 쓸 저장 지도를 고르세요.'); return; }
  if (data.values.service_control?.mapping?.managed && data.values.service_control.mapping.mode === 'mapping' && !confirm('작성 중인 지도를 닫고 ' + name + ' 지도로 위치추정을 시작할까요? 저장하지 않은 지도는 사라집니다.')) return;
  const ok = await serviceAction('/api/mapping', { action: 'localize', map: name }, name + ' 지도 전체에서 위치와 방향을 자동으로 찾습니다.', 'mapping');
  if (ok) setMode('nav');
}
$('nav-localize').onclick = () => localize($('nav-map-select').value);
$('nav-start').onclick = () => serviceAction('/api/navigation', { action: 'start' }, 'Nav2 시작을 요청했습니다. 준비까지 10~20초 걸립니다.', 'navigation');
$('nav-stop').onclick = () => serviceAction('/api/navigation', { action: 'stop' }, 'Nav2 중지를 요청했습니다.', 'navigation');
$('nav-cancel').onclick = () => serviceAction('/api/navigation', { action: 'cancel' }, '이동 취소를 요청했습니다.', 'cancel');
$('det-start').onclick = () => serviceAction('/api/detection', { action: 'start' }, 'YOLO 검출 시작을 요청했습니다.', 'detection');
$('det-stop').onclick = () => serviceAction('/api/detection', { action: 'stop' }, '검출·접근 중지를 요청했습니다.', 'detection');
$('det-target-form').onsubmit = event => {
  event.preventDefault();
  moveToNamedObject();
};
$('det-clear').onclick = () => serviceAction('/api/detection', { action: 'clear' }, '저장된 물체 좌표를 지웁니다.', 'detection');
$('det-nav-start').onclick = () => serviceAction('/api/navigation', { action: 'start' }, 'Nav2 시작을 요청했습니다.', 'navigation');
function moveToNamedObject() {
  const label = $('det-target-name').value.trim() || data.values.detection_control?.label || '';
  if (!label) { toast('찾을 물체 이름을 입력하세요.'); return; }
  const edge_clearance = Number($('det-distance').value);
  if (!Number.isFinite(edge_clearance) || edge_clearance < 0 || edge_clearance > .1) { toast('책상 추가 이격은 0~0.1m입니다.'); return; }
  serviceAction('/api/detection', { action: 'find_and_approach', label, edge_clearance }, 'YOLO 물체 방향으로 접근합니다. LiDAR 좌표는 이동 중 확보합니다.', 'detection');
}
$('det-approach').onclick = moveToNamedObject;
$('det-cancel').onclick = () => serviceAction('/api/detection', { action: 'cancel' }, '물체 접근 취소를 요청했습니다.', 'detection');

function renderDetectionControl() {
  const alive = data.mode === 'live' && data.connected && performance.now()-data.received < 2500;
  const sc = data.values.service_control || {}, service = sc.detection, control = data.values.detection_control;
  const connected = alive && fresh('detection_control');
  const running = Boolean(service?.managed || service?.state === 'external');
  const active = Boolean(control?.active), target = control?.target, visual = control?.visual_target;
  const busy = !connected || servicePending.has('detection') || service?.busy;
  const safetyReady = Boolean(alive && data.meta.collision_safety?.age <= .8 && data.meta.collision_actuator?.age <= .8
    && data.values.collision_safety?.ready && data.values.collision_actuator?.ready);
  const nav = sc.navigation, navRunning = Boolean(nav?.managed || nav?.state === 'external');
  const slamRunning = Boolean(sc.mapping?.managed || sc.mapping?.state === 'external');
  const navBusy = ['sending', 'active', 'canceling'].includes(data.values.navigation?.state) || Boolean(data.values.navigation?.front_retreat?.active);
  const phases = { idle: '대상 입력 대기', searching: '관측 대기', confirming: '위치 확인 중', ready: '접근 준비',
    waiting_target: 'YOLO 물체 대기 · 발견 시 접근', visual_waiting: 'YOLO 접근 입력 대기',
    visual_aligning: 'YOLO 물체 방향 정렬', visual_approaching: 'YOLO 영상으로 접근 중',
    visual_handoff: 'LiDAR 좌표 저장·근접 접근 전환', waiting_nav: 'Nav2 연결 중', navigating_target: '3D 물체 좌표로 이동 중',
    target_settling: '3D 좌표 접근 전환', point_handoff: 'Nav2 정지·LiDAR 근접 접근 전환', point_aligning: '3D 물체 방향 정렬', point_approaching: '3D 물체로 계속 접근', at_target: '3D 목표 앞 도착',
    planning: '책상 파지 위치 계획', planning_table: '도달 가능한 책상 면 계획', rerouting_table: '책상 우회 경로 전환',
    front_retreat: '전방 장애물 · 짧은 후진 우선', rerouting_obstacle: '후진 효과 없음 · 다른 경로 선택', checking_reverse: '실패 전 후진 가능 여부 확인', final_reverse: '최종 후진 판단·재접근',
    navigating: 'Nav2 책상 접근 면으로 이동', docking_handoff: '2D 주행 종료·3D 도킹 전환', observing_table: '우회한 면의 가장자리 확인', settling: '정지·전환 중',
    recovering: '회전 우선 · 필요 시 후진', realigning: '회전 공간 확보 후 선정렬', aligning: '물체 방향 정렬', approaching: '책상 최대 접근 중', final_align: '최종 방향·정지 확인', succeeded: '파지 위치·방향 도착', canceled: '접근 취소', error: '접근 중지' };
  badge('det-pill', !connected ? '대기' : active ? 'APPROACH' : running ? 'YOLO' : 'OFF', active || running ? 'live' : 'waiting');
  text('det-state', !alive ? '연결 대기' : service?.state === 'starting' ? 'YOLO 시작 중…' : service?.state === 'stopping' ? 'YOLO 중지 중…' : running ? 'YOLO 검출 중' : service?.state === 'error' ? 'YOLO 실행 오류' : '검출 중지됨');
  text('det-hint', service?.error || (running ? `Head RGB · YOLO ${fmt(data.values.detections?.detection_fps)} FPS · MID-360 위치 연결` : 'Head 카메라와 MID-360 센서를 켠 뒤 검출을 시작하세요.'));
  $('det-start').disabled = busy || running;
  $('det-stop').disabled = busy || !service?.managed;
  $('det-select').disabled = busy || active || navBusy;
  $('det-target-name').disabled = !connected || active;
  if (!$('det-target-name').value && control?.label && document.activeElement !== $('det-target-name')) $('det-target-name').value = control.label;
  text('det-target-state', !connected ? '대상 연결 대기' : `${control?.label || '대상 미선택'} · ${target ? target.visible ? 'YOLO + LiDAR 관측 중' : '저장 LiDAR 좌표 사용' : visual ? 'YOLO 영상으로 접근 · LiDAR 좌표 수집 중' : phases[control?.phase] || '위치 관측 대기'}${!active && ['idle', 'searching', 'confirming', 'ready'].includes(control?.phase) ? ' · 이동 시작 대기' : ''}`);
  const inputs = control?.inputs;
  text('det-inputs', inputs ? `${active ? '이동 요청됨' : '관측 상태'} · YOLO ${fmt(inputs.yolo_source_age, 2)}초 전 · LiDAR ${fmt(inputs.cloud_source_age, 2)}초 전 · ${inputs.obstacle_frames}개 점군${control?.position_note ? ' · ' + control.position_note : ''}${inputs.input_error ? ' · 입력 오류: ' + inputs.input_error : ''}` : '3D 위치 입력 대기');
  const base = target?.point_base, map = target?.point_map;
  text('det-position', map ? `${target.frame} (${map.map(v => fmt(v, 2)).join(', ')}) m` : target ? `odom (${target.point_odom.map(v => fmt(v, 2)).join(', ')}) m · map TF 대기` : visual ? `YOLO 방향 ${fmt(visual.bearing*180/Math.PI, 1)}° · 영상 접근 중 LiDAR 좌표 수집` : 'YOLO 발견 시 먼저 접근하고, 이동 중 물체의 LiDAR 좌표를 저장합니다.');
  text('det-source', target ? `Head YOLO로 선택 · MID-360 ${target.lidar_count}점 좌표 · ${target.table ? '책상 형상 확인됨' : '책상 형상 별도 관측'}${base ? ` · ${base[0] >= 0 ? '앞' : '뒤'} ${fmt(Math.abs(base[0]), 2)}m / ${base[1] >= 0 ? '왼쪽' : '오른쪽'} ${fmt(Math.abs(base[1]), 2)}m` : ''}` : '');
  text('det-capture', target ? `${new Date(target.capture_stamp*1000).toLocaleTimeString('ko-KR', {hour12: false})} 촬영 · ${fmt(target.capture_age, 1)}초 전 · LiDAR ${new Date(target.lidar_stamp*1000).toLocaleTimeString('ko-KR', {hour12: false})} 측정${target.range != null ? ' · 수평 거리 ' + fmt(target.range, 2) + 'm' : ''}` : '');
  $('det-clear').disabled = busy || active || !target;
  $('det-distance').disabled = !connected || active;
  text('det-approach-state', !connected ? '접근 연결 대기' : phases[control?.phase] || '접근 대기');
  text('det-approach-hint', (control?.message || '물체 이름을 적용하고 물체로 이동을 누르세요. 3D 관측부터 이동을 연결합니다.')+(control?.table_gap != null ? ` · 책상 높이 간격 ${fmt(Math.max(0, control.table_gap)*100, 1)}cm / 완료 상한 ${fmt(control.table_gap_limit*100, 1)}cm${control.grasp_distance_satisfied ? ' · 거리 충족' : ' · 더 접근 필요'}` : '')+(control?.reverse_assessment ? ' · '+control.reverse_assessment.reason : ''));
  $('det-nav-start').disabled = !connected || navRunning || nav?.busy || servicePending.has('navigation') || !(sc.mapping?.managed || sc.mapping?.state === 'external');
  $('det-nav-start').classList.toggle('hidden', navRunning);
  $('det-approach').disabled = busy || active || navBusy;
  $('det-cancel').disabled = !alive || !active;
  text('det-motion-hint', !safetyReady ? '현재 3D 충돌 보호 입력 대기 · 이동 요청은 예약할 수 있습니다.' : control?.plan ? `책상 ${control.plan.side+1}번 면 · ${control.plan.boundary_verified ? '실제 가장자리 확인됨' : '우회 후 가장자리 확인 필요'} · 추가 이격 ${fmt(control.edge_clearance*100, 1)}cm · 물체 거리 ${fmt(control.distance, 2)}m${control.plan.position_error != null ? ' · 위치 오차 ' + fmt(control.plan.position_error*100, 1) + 'cm' : ''}${base ? ' · 물체 방향 오차 ' + fmt(Math.atan2(base[1], base[0])*180/Math.PI, 1) + '°' : ''}${control.surface_note ? ' · ' + control.surface_note : ''}` : active && !target ? 'YOLO 영상 방향으로 최대 0.04m/s 접근 · 이동 중 LiDAR 좌표 확보' : active ? `저장한 3D 목표로 접근 중${target?.range != null ? ' · 현재 물체 거리 ' + fmt(target.range, 2) + 'm' : ''}${control?.surface_note ? ' · ' + control.surface_note : ''}` : 'YOLO 발견 시 먼저 접근합니다. LiDAR 좌표 확보 후 저장 좌표로 근접 접근하며, 책상 우회에는 Nav2를 사용합니다.');
}

function renderServices() {
  const alive = data.mode === 'live' && data.connected && performance.now() - data.received < 2500;
  const sc = data.values.service_control || {}, slam = sc.mapping, nav = sc.navigation, maps = sc.maps || {}, goal = data.values.navigation;
  const labels = { off: '중지됨', starting: '시작 중…', running: '실행 중', stopping: '중지 중…', external: '외부 실행 중', error: '오류' };
  const slamRunning = Boolean(slam && (slam.managed || slam.state === 'external'));
  const localizing = slam?.mode === 'localization';
  const navRunning = Boolean(nav && (nav.managed || nav.state === 'external'));
  // SLAM card
  const slamLabel = !alive ? '연결 대기' : slam ? (slam.managed && slam.state === 'running' ? localizing ? '위치추정 중 · ' + slam.map : '지도 작성 중' : labels[slam.state] || slam.state) : '상태 수신 대기';
  text('slam-state', slamLabel);
  const m = data.values.map;
  text('slam-hint', !alive ? '' : slam?.error || (slam?.state === 'external' ? '외부 실행' : slamRunning ? m ? `${fmt(m.known_area, 1)} m²` : '/map 대기' : ''));
  $('slam-hint').title = slam?.log || '';
  badge('slam-pill', !alive ? '대기' : slamRunning ? localizing ? 'LOCALIZING' : 'MAPPING' : slam?.state === 'error' ? 'ERROR' : 'OFF', slamRunning ? 'live' : slam?.state === 'error' ? 'error' : 'waiting');
  const busy = !alive || servicePending.has('mapping') || slam?.busy;
  $('slam-start').disabled = busy || slamRunning || navRunning;
  $('slam-stop').disabled = busy || !slam?.managed || navRunning;
  $('slam-reset').disabled = busy || !slam?.managed || navRunning;
  $('slam-reset').textContent = localizing ? '다시 불러오기' : '초기화';
  text('slam-caption', navRunning ? 'Nav2 실행 중 · 잠금' : '');
  // Save card
  $('map-save').disabled = !alive || !slamRunning || Boolean(maps.saving) || servicePending.has('save');
  text('save-state', maps.saving ? maps.saving + ' 저장 중…' : maps.last ? (maps.last.ok ? '저장 완료 · ' : '저장 실패 · ') + maps.last.name : '—');
  text('save-hint', maps.last && !maps.saving && !maps.last.ok ? maps.last.message : '');
  // Saved maps list + NAV map selector
  const items = maps.items || [], signature = JSON.stringify(items.map(i => [i.name, i.posegraph, i.image, i.time])) + alive + localizing + navRunning;
  if (signature !== renderServices.signature) {
    renderServices.signature = signature;
    const list = $('map-list'); list.replaceChildren();
    if (!items.length) { const e = document.createElement('div'); e.className = 'empty-row'; e.textContent = '저장된 지도가 없습니다'; list.append(e); }
    for (const item of items) {
      const row = document.createElement('div'); row.className = 'map-row';
      const name = document.createElement('strong'); name.textContent = item.name;
      const meta = document.createElement('small'); meta.textContent = new Date(item.time * 1000).toLocaleString('ko-KR', { hour12: false }) + (item.posegraph ? '' : ' · pose graph 없음');
      const use = document.createElement('button'); use.textContent = '위치추정'; use.disabled = !alive || !item.posegraph || navRunning;
      use.onclick = () => localize(item.name);
      const info = document.createElement('div'); info.append(name, meta); row.append(info, use); list.append(row);
    }
    const select = $('nav-map-select'), previous = select.value; select.replaceChildren(new Option('저장된 지도 선택', ''));
    for (const item of items) if (item.posegraph) select.append(new Option(item.name, item.name));
    select.value = [...select.options].some(o => o.value === previous) ? previous : localizing && slam?.map ? slam.map : '';
  }
  // NAV cards
  text('nav-localization', !alive ? '연결 대기' : !slamRunning ? 'SLAM / 위치추정 없음' : localizing ? '저장 지도 위치추정 · ' + slam.map : slam.state === 'external' ? '외부 SLAM 사용' : '작성 중인 지도 사용 (SLAM)');
  const localization = data.values.localization;
  const located = Boolean(localization?.state === 'localized' && localization.map === slam?.map);
  text('nav-localization-hint', localizing ? localization?.reason || '자동 위치 탐색 준비 중' : '');
  $('nav-localize').disabled = !alive || navRunning || servicePending.has('mapping') || !$('nav-map-select').value;
  const ready = Boolean(goal?.available);
  const safety = data.values.collision_safety, actuator = data.values.collision_actuator;
  const safetyFresh = alive && data.meta.collision_safety?.age <= .8 && data.meta.collision_actuator?.age <= .8;
  const safetyReady = Boolean(safetyFresh && safety?.ready && actuator?.ready);
  const safetyLabels = { clear: '보호 중', slowed: '감속 중', blocked: '정지 중', idle: '보호 대기 · 정지' };
  const safetyReasons = {
    waiting_for_mid360: 'MID-360 입력 대기', stale_mid360: 'MID-360 유효 관측 지연 · 정지',
    stale_collision_scene: '충돌 점군 전달 지연 · 정지',
    missing_cloud_health: '3D 전처리 상태 수신 대기', insufficient_raw_points: 'MID-360 유효 점 부족',
    stale_odometry: '오도메트리 수신 지연', missing_odometry_tf: '차체 TF 대기',
    stale_or_inconsistent_tf: '차체 TF 시간·자세 불일치', odometry_discontinuity: '오도메트리 불연속',
    stale_arm_feedback: '팔 CAN 피드백 없음 · 팔 연결 복구 후 주행 가능', arm_is_moving: '팔 이동 중 · 차체 정지',
    clearance_boundary: '외곽 여유 거리 부족', predicted_clearance_violation: '이동·제동 중 여유 거리 부족',
    braking_clearance: '제동 여유 확보를 위해 감속', clear_path: '주행 경로 여유 확보',
    command_timeout: '이동 명령 없음', zero_command: '정지 명령',
    command_changed: '이동 명령 변경 · 다음 계산 대기',
    computation_timeout: '충돌 계산 지연 · 차체 정지', guard_error: '충돌 보호 처리 오류',
    tracer_requires_protected_remap: '차체 보호 입력 적용 필요 · 관제센터 재시작',
    waiting_for_protected_tracer: '차체 보호 입력 대기', duplicate_velocity_guard: '충돌 보호 노드 중복',
    missing_actuator_watchdog: '차체 watchdog 대기', velocity_route_unavailable: '차체 명령 연결 대기',
    missing_per_point_time: 'MID-360 점별 시간 정보 없음', invalid_per_point_time: 'MID-360 시간 정보 오류',
    odometry_gap: '전처리 자세 이력 공백', missing_odometry_interval: '점군 시각의 자세 이력 대기',
    processing_deadline_exceeded: '전처리 완료 시 관측 시간 초과', obstacle_capacity_exceeded: '3D 장애물 저장 한도 초과',
    unreliable_floor_plane: '바닥 평면 추정 대기 · 차체 정지',
    missing_sensor_tf: 'MID-360 장착 TF 대기', invalid_cloud_health: '3D 전처리 상태 형식 오류',
    cloud_clock_reset: 'MID-360 관측 시각 초기화'
  };
  text('collision-state', !safetyFresh ? '보호 상태 수신 대기 / 지연' : !actuator?.ready ? '차체 출력 정지' : safetyLabels[safety?.state] || '보호 입력 대기');
  const clearanceCm = Number.isFinite(safety?.minimum_distance) ? safety.minimum_distance*100 : NaN;
  text('collision-distance', `로봇 외곽 거리 ${fmt(clearanceCm, 1)} cm · ${safety?.body_only ? '몸체 형상만 적용 · 추가 여유 0cm' : '기준 ' + fmt((safety?.clearance ?? 0)*100, 0) + ' cm'}`);
  const pointKind = Array.isArray(safety?.blocking_source_point_base) ? 'blocking' : 'nearest';
  const sensed = safety?.[pointKind + '_source_point_base'];
  const sensedHeight = safety?.[pointKind + '_height_above_floor'];
  const sensedAge = safety?.[pointKind + '_source_age'];
  const sensedStamp = safety?.[pointKind + '_last_seen_stamp'];
  const sensedWhen = Number.isFinite(sensedStamp) ? new Date(sensedStamp*1000).toLocaleTimeString('ko-KR', {hour12: false}) + ' 관측' : '';
  text('collision-point', safetyFresh && Array.isArray(sensed)
    ? `${pointKind === 'blocking' ? '경로 장애물' : '최근접 점'} · MID-360: ${sensed[0] >= 0 ? '앞' : '뒤'} ${fmt(Math.abs(sensed[0])*100, 0)}cm · ${sensed[1] >= 0 ? '왼쪽' : '오른쪽'} ${fmt(Math.abs(sensed[1])*100, 0)}cm${Number.isFinite(sensedHeight) ? ' · 바닥 +' + fmt(sensedHeight*100, 1) + 'cm' : ''}${Number.isFinite(sensedAge) ? ' · ' + fmt(sensedAge, 1) + '초 전' : ''}${sensedWhen ? ' (' + sensedWhen + ')' : ''}`
    : '장애물 측정 위치 대기');
  const cloudTiming = safety?.cloud;
  const timingText = cloudTiming?.raw_points > 0 && Number.isFinite(cloudTiming.processing_ms)
    ? ` · 계산 ${fmt(cloudTiming.processing_ms, 0)} ms · 대기 ${fmt(cloudTiming.queue_wait_ms, 0)} ms · 최신 관측 ${fmt(cloudTiming.source_age*1000, 0)} ms${Number.isFinite(safety?.scene_source_age) ? ' · 검사 입력 ' + fmt(safety.scene_source_age*1000, 0) + ' ms' : ''}` : '';
  text('collision-hint', !safetyFresh ? 'MID-360·차체 보호 상태를 기다립니다.' : (safetyReasons[safety?.reason] || safety?.reason || '보호 입력 대기') + timingText);
  text('nav-state', !alive ? '연결 대기' : nav ? nav.managed && nav.state === 'running' ? ready ? safetyReady ? '준비 완료' : '실행 중 · 주행 보호 대기' : '서버 활성화 대기…' : labels[nav.state] || nav.state : '상태 수신 대기');
  text('nav-hint', !alive ? '' : nav?.error || (maps.nav2_installed === false ? 'Nav2 미설치' : !slamRunning ? 'SLAM 먼저' : !safetyReady ? safetyReasons[safety?.reason] || '3D 보호 입력 대기 · 서버 시작 가능, 주행은 정지' : ''));
  $('nav-hint').title = nav?.log || '';
  badge('nav-pill', !alive ? '대기' : navRunning ? ready ? goal?.state === 'active' ? 'DRIVING' : 'READY' : 'STARTING' : nav?.state === 'error' ? 'ERROR' : 'OFF', navRunning && ready ? 'live' : nav?.state === 'error' ? 'error' : 'waiting');
  const navBusy = !alive || servicePending.has('navigation') || nav?.busy;
  $('nav-start').disabled = navBusy || navRunning || !slamRunning || (localizing && !located) || maps.nav2_installed === false;
  $('nav-stop').disabled = navBusy || !nav?.managed;
  const goalLabels = { idle: '목표 없음', sending: '목표 전송 중', active: '이동 중', canceling: '취소 중', succeeded: '도착', aborted: '실패', canceled: '취소됨', rejected: '거부됨', error: '오류' };
  text('goal-state', goal ? (goalLabels[goal.state] || goal.state) + (goal.goal ? ` · (${fmt(goal.goal.x, 2)}, ${fmt(goal.goal.y, 2)})` : '') : '목표 없음');
  text('goal-hint', goal?.state === 'active' ? `남은 거리 ${fmt(goal.distance, 2)} m · 예상 ${fmt(goal.eta, 0)} s · 복구 ${goal.recoveries ?? 0}회${goal.reverse_assessment ? ' · '+goal.reverse_assessment.reason : ''}` : goal && goal.state !== 'idle' ? goal.message : '');
  if (goal?.detection_active) {
    const stages = { waiting_target: 'YOLO 3D 관측 후 이동', waiting_nav: 'Nav2 연결', navigating_target: '표시된 3D 물체로 이동',
      target_settling: '3D 목표 접근 전환', point_aligning: '물체 방향 정렬', point_approaching: '3D 물체로 접근',
      planning: '파지 위치 계획', planning_table: '책상 접근 면 계획', rerouting_table: '책상 우회로 전환',
      front_retreat: '전방 장애물 · 짧은 후진 우선', rerouting_obstacle: '후진 효과 없음 · 다른 경로 선택', checking_reverse: '실패 전 후진 가능 여부 확인', final_reverse: '최종 후진 판단·재접근',
      navigating: 'Nav2 책상 접근 면으로 이동', docking_handoff: '2D 주행 종료·3D 도킹 전환', observing_table: '우회한 면의 가장자리 확인',
      recovering: '회전 우선 · 필요 시 후진', realigning: '공간 확보 후 물체 방향 선정렬', settling: '정지·제어 전환', aligning: '물체 방향 정렬', approaching: '책상 최대 접근', final_align: '최종 방향·정지 확인' };
    text('goal-state', 'Detection · '+(stages[goal.detection_phase] || '물체 접근'));
    text('goal-hint', (data.values.detection_control?.message || '')+(['navigating', 'navigating_target'].includes(goal.detection_phase) ? ` · 남은 거리 ${fmt(goal.distance, 2)}m · 복구 ${goal.recoveries ?? 0}회` : '')+(data.values.detection_control?.reverse_assessment ? ' · '+data.values.detection_control.reverse_assessment.reason : goal.reverse_assessment ? ' · '+goal.reverse_assessment.reason : ''));
  }
  if (goal?.front_retreat?.active && !goal?.detection_active) {
    text('goal-state', '전방 장애물 · 짧은 후진 우선');
    text('goal-hint', goal.front_retreat.reason);
  }
  $('nav-goal').disabled = !alive || !navRunning || !ready || !safetyReady || Boolean(data.values.detection_control?.active) || Boolean(goal?.front_retreat?.active);
  $('nav-cancel').disabled = !alive || (!data.values.detection_control?.active && !goal?.front_retreat?.active && (!goal || !['active', 'sending', 'canceling'].includes(goal.state)));
  if (pick.mode === 'goal' && $('nav-goal').disabled) setPick(null);
  renderDetectionControl();
}
let savedMode = 'observe';
try { savedMode = localStorage.getItem('inha-cc-mode') || 'observe'; } catch { /* storage unavailable */ }
setMode(savedMode);
connect();

// Same snapshot/event stream as navigation; no extra polling or inference.
function renderManipulation() {
  const m = data.values.manipulation || {}, status = m.status || {}, readiness = m.readiness || {};
  const ages = m.ages || {}, blockers = m.execution?.blockers || ['ROS 상태 수신 대기'];
  const fresh = key => Number.isFinite(ages[key]) && ages[key] <= 3;
  const online = data.connected && m.available;
  const prepared = online && fresh('readiness') && readiness.ready === true;
  const handoff = m.execution?.handoff || {}, handedOff = online && handoff.ready === true;
  const currentStatus = !handoff.task_id || status.task_id === handoff.task_id;
  const taskHasRun = currentStatus && status.stage && (m.active || ['SUCCESS', 'FAILED', 'CANCELED', 'REJECTED'].includes(status.outcome));
  text('manip-pill', !online ? '미연결' : m.active ? '진행 중' : handedOff ? '인계 완료' : m.execution?.armed ? '인계 대기' : prepared ? '준비' : '준비 필요');
  text('manip-state', !online ? (m.reason || '매니퓰 ROS 연결 대기') : m.active ? '매니퓰레이션 진행 중' : handedOff ? (blockers.length ? '도킹 인계 완료 · 팔 실행 준비 필요' : '도킹 인계 완료 · 픽앤플레이스 실행 가능') : prepared ? '실행기 준비 완료' : '실행 조건 확인 필요');
  text('manip-readiness', fresh('readiness') ? (readiness.errors?.join(' · ') || '실행기 준비 정상') : '실행기 상태 수신 없음 / 지연');
  text('manip-blockers', blockers.join(' · '));
  text('manip-stage', taskHasRun ? status.stage : handoff.message || status.stage || '새 물체 도킹 완료 대기');
  text('manip-task', '작업 ID · '+(handoff.task_id || status.task_id || '—'));
  text('manip-result', currentStatus && status.outcome ? [status.outcome, status.reason, '물체 '+(status.holding || 'UNKNOWN')].filter(Boolean).join(' · ') : handedOff ? '물체·좌표·도킹 완료·실측 정지 인계됨 · 실행 요청 시 베이스 잠금 후 팔 동작' : '손목 재관측 → 정밀 위치·마스크 → 규칙 기반 파지 → 좌/우 배치 → 팔 복귀');
  const p = currentStatus ? status.precise_center : null, coarse = handoff.coarse_position;
  text('manip-position', p?.frame && p.xyz?.every(Number.isFinite) ? p.frame+' ('+p.xyz.map(v=>v.toFixed(3)).join(', ')+') m' : coarse?.frame && coarse.xyz?.every(Number.isFinite) ? '인계 좌표 · '+coarse.frame+' ('+coarse.xyz.map(v=>v.toFixed(3)).join(', ')+') m · 손목 정밀 재검출 대기' : '정밀 좌표 —');
  text('manip-base', !fresh('base') ? '베이스 게이트 상태 수신 없음 / 지연' : '베이스 '+(m.base.locked ? '잠금 유지' : '잠금 해제')+' · '+(m.base.stopped ? '정지 확인' : '정지 미확인')+' · '+(m.base.reason || ''));
  text('manip-inputs', [['adapter','디텍션 인계'],['wrist','손목'],['readiness','실행기'],['base','베이스']].map(([k,n])=>n+' '+(fresh(k)?'연결':'미수신')).join(' · '));
  $('manip-arm').disabled = !online || (m.execution?.arm_blockers || ['대기']).length > 0 || m.execution?.armed || servicePending.has('manipulation');
  $('manip-run').disabled = !online || blockers.length > 0 || servicePending.has('manipulation');
  $('manip-cancel').disabled = !online || (!m.active && !m.execution?.armed) || servicePending.has('manipulation');
}
$('manip-run').onclick = () => serviceAction('/api/manipulation', {action:'run'}, '매니퓰 실행 요청을 보냈습니다. 접수 결과는 이벤트에서 확인하세요.', 'manipulation');
$('manip-cancel').onclick = () => serviceAction('/api/manipulation', {action:'cancel'}, '매니퓰 작업 취소를 요청했습니다.', 'manipulation');

$('manip-arm').onclick = () => serviceAction('/api/manipulation', {action:'arm'}, '새 Detection 접근 완료 후 자동 실행을 예약했습니다. 접수 결과는 이벤트에서 확인하세요.', 'manipulation');
