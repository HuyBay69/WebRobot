'use strict';

// ── DOM ───────────────────────────────────────────────────────────────────────
const wsHostInput    = document.getElementById('wsHost');
const btnConnect     = document.getElementById('btnConnect');
const ROLE_NAME      = 'hero';
const connDot        = document.getElementById('connDot');
const connStatus     = document.getElementById('connStatus');

const mapTitle       = document.getElementById('mapTitle');
const btnUpload      = document.getElementById('btnUpload');
const mapFileInput   = document.getElementById('mapFileInput');
const mapPlaceholder = document.getElementById('mapPlaceholder');
const mapImg         = document.getElementById('mapImg');

const uploadModal    = document.getElementById('uploadModal');
const dropZone       = document.getElementById('dropZone');
const dropHint       = document.getElementById('dropHint');
const mapNameInput   = document.getElementById('mapNameInput');
const btnModalCancel = document.getElementById('btnModalCancel');
const btnModalUpload = document.getElementById('btnModalUpload');
const uploadFeedback = document.getElementById('uploadFeedback');

const goalX          = document.getElementById('goalX');
const goalY          = document.getElementById('goalY');
const btnSendGoal    = document.getElementById('btnSendGoal');
const goalFeedback   = document.getElementById('goalFeedback');
const lastGoalEl     = document.getElementById('lastGoal');

const speedKmh       = document.getElementById('speedKmh');
const speedMpsEl     = document.getElementById('speedMps');
const btnSendSpeed   = document.getElementById('btnSendSpeed');
const btnStop        = document.getElementById('btnStop');
const speedFeedback  = document.getElementById('speedFeedback');
const lastSpeedEl    = document.getElementById('lastSpeed');

const logBody        = document.getElementById('logBody');

// ── State ─────────────────────────────────────────────────────────────────────
let ros       = null;
let goalPub   = null;
let speedPub  = null;
let connected = false;
let pendingFile = null;

// ── Logging ───────────────────────────────────────────────────────────────────
function addLog(level, msg) {
  const empty = logBody.querySelector('.log-empty');
  if (empty) empty.remove();

  const now = new Date().toLocaleTimeString('vi-VN', { hour12: false });
  const row = document.createElement('div');
  row.className = `log-row ${level}`;
  row.innerHTML = `
    <span class="log-time">${now}</span>
    <span class="log-pip"></span>
    <span class="log-text">${escHtml(msg)}</span>
  `;
  logBody.prepend(row);
  while (logBody.children.length > 80) logBody.lastChild.remove();
}

function escHtml(s) {
  return s.replace(/[&<>"']/g, c =>
    ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
}

// ── Feedback ──────────────────────────────────────────────────────────────────
function setFeedback(el, msg, ok) {
  el.textContent = msg;
  el.className = 'feedback ' + (ok ? 'ok' : 'err');
  if (ok) setTimeout(() => { el.textContent = ''; el.className = 'feedback'; }, 3000);
}

// ── ROS Connection ────────────────────────────────────────────────────────────
function setConnectedUI(state) {
  connected = state === 'connected';

  connDot.className = 'conn-dot ' +
    (state === 'connected' ? 'connected' : state === 'connecting' ? 'connecting' : '');

  if (state === 'connected') {
    connStatus.textContent = 'Connected';
    btnConnect.textContent = 'Disconnect';
    btnConnect.classList.add('connected');
    btnConnect.disabled = false;
  } else if (state === 'connecting') {
    connStatus.textContent = 'Connecting…';
    btnConnect.disabled = true;
  } else {
    connStatus.textContent = 'Disconnected';
    btnConnect.textContent = 'Connect ROS';
    btnConnect.classList.remove('connected');
    btnConnect.disabled = false;
  }
  // Buttons always enabled — gửi qua API khi không có ROS WebSocket
}

function connectROS() {
  const host = wsHostInput.value.trim() || 'localhost:9090';
  setConnectedUI('connecting');
  addLog('info', `Connecting to ws://${host} …`);

  ros = new ROSLIB.Ros({ url: `ws://${host}` });

  ros.on('connection', () => {
    addLog('info', `Connected to ROS — role: ${ROLE_NAME}`);
    setConnectedUI('connected');

    goalPub = new ROSLIB.Topic({
      ros,
      name: '/goal_pose',
      messageType: 'geometry_msgs/PoseStamped',
    });

    speedPub = new ROSLIB.Topic({
      ros,
      name: `/carla/${ROLE_NAME}/target_speed`,
      messageType: 'std_msgs/Float64',
    });
  });

  ros.on('error', err => {
    addLog('error', `Connection error: ${err}`);
    setConnectedUI('disconnected');
  });

  ros.on('close', () => {
    addLog('warn', 'Disconnected from ROS — chuyển sang chế độ API');
    goalPub  = null;
    speedPub = null;
    setConnectedUI('disconnected');
  });
}

function disconnectROS() {
  if (ros) { ros.close(); ros = null; }
}

btnConnect.addEventListener('click', () => {
  if (connected) disconnectROS();
  else connectROS();
});

// ── Upload bản đồ ─────────────────────────────────────────────────────────────
btnUpload.addEventListener('click', () => {
  pendingFile = null;
  dropHint.textContent = '';
  mapNameInput.value = '';
  uploadFeedback.textContent = '';
  uploadFeedback.className = 'feedback';
  btnModalUpload.disabled = true;
  uploadModal.style.display = 'flex';
});

btnModalCancel.addEventListener('click', closeModal);
uploadModal.addEventListener('click', e => { if (e.target === uploadModal) closeModal(); });

function closeModal() {
  uploadModal.style.display = 'none';
  pendingFile = null;
}

function acceptFile(file) {
  if (!file || !file.type.startsWith('image/')) {
    dropHint.textContent = 'Chỉ hỗ trợ file ảnh.';
    return;
  }
  pendingFile = file;
  dropHint.textContent = `✓ ${file.name}`;
  if (!mapNameInput.value.trim()) mapNameInput.value = file.name.replace(/\.[^.]+$/, '');
  btnModalUpload.disabled = false;
}

mapFileInput.addEventListener('change', () => { if (mapFileInput.files[0]) acceptFile(mapFileInput.files[0]); });

dropZone.addEventListener('click', e => { if (!e.target.classList.contains('link')) mapFileInput.click(); });
dropZone.addEventListener('dragover',  e => { e.preventDefault(); dropZone.classList.add('dragover'); });
dropZone.addEventListener('dragleave', () => dropZone.classList.remove('dragover'));
dropZone.addEventListener('drop', e => {
  e.preventDefault();
  dropZone.classList.remove('dragover');
  if (e.dataTransfer.files[0]) acceptFile(e.dataTransfer.files[0]);
});

btnModalUpload.addEventListener('click', async () => {
  if (!pendingFile) return;
  const name = mapNameInput.value.trim() || pendingFile.name;
  btnModalUpload.disabled = true;
  setFeedback(uploadFeedback, 'Đang tải lên…', true);

  const fd = new FormData();
  fd.append('file', pendingFile);
  fd.append('map_name', name);

  try {
    const res  = await fetch('/api/upload-map', { method: 'POST', body: fd });
    const data = await res.json();
    if (data.ok) {
      mapTitle.textContent = data.map_name;
      mapImg.src = data.url + '?t=' + Date.now();
      mapImg.style.display = 'block';
      mapPlaceholder.style.display = 'none';
      addLog('info', `Map uploaded: ${data.map_name}`);
      closeModal();
    } else {
      setFeedback(uploadFeedback, data.error || 'Lỗi tải lên', false);
      btnModalUpload.disabled = false;
    }
  } catch {
    setFeedback(uploadFeedback, 'Không thể kết nối server', false);
    btnModalUpload.disabled = false;
  }
});

// ── Tốc độ — sync m/s badge ───────────────────────────────────────────────────
speedKmh.addEventListener('input', () => {
  const v = parseFloat(speedKmh.value);
  speedMpsEl.textContent = isNaN(v) ? '— m/s' : `${(v / 3.6).toFixed(3)} m/s`;
});

// ── Gửi tọa độ ────────────────────────────────────────────────────────────────
async function sendGoal() {
  const x = parseFloat(goalX.value);
  const y = parseFloat(goalY.value);

  if (isNaN(x) || isNaN(y)) {
    setFeedback(goalFeedback, 'X và Y phải là số hợp lệ.', false);
    return;
  }

  const ts = new Date().toLocaleTimeString('en-GB', { hour12: false });

  // Ưu tiên ROS WebSocket nếu đang kết nối
  if (connected && goalPub) {
    goalPub.publish(new ROSLIB.Message({
      header: { frame_id: 'map' },
      pose: { position: { x, y, z: 0.0 }, orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 } },
    }));
    lastGoalEl.innerHTML = `Sent at ${ts}: <b>x=${x.toFixed(3)}, y=${y.toFixed(3)}</b>`;
    setFeedback(goalFeedback, '✓ Published to /goal_pose (ROS)', true);
    addLog('info', `Goal [ROS] → x=${x.toFixed(3)}, y=${y.toFixed(3)}`);
    return;
  }

  // Fallback: gửi qua Flask API
  btnSendGoal.disabled = true;
  try {
    const res  = await fetch('/api/send-goal', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ x, y }),
    });
    const data = await res.json();
    if (data.ok) {
      let detail = `x=${x.toFixed(3)}, y=${y.toFixed(3)}`;
      if (data.snapped) detail += ` → waypoint (${data.wx.toFixed(3)}, ${data.wy.toFixed(3)}), dist=${data.dist} m`;
      const via = data.ros ? ' → ROS' : ' (no ROS)';
      lastGoalEl.innerHTML = `Sent at ${ts}: <b>x=${x.toFixed(3)}, y=${y.toFixed(3)}</b>`;
      setFeedback(goalFeedback, `✓ Sent via server${via}`, true);
      addLog('info', `Goal [API${via}] → ${detail}`);
    } else {
      setFeedback(goalFeedback, data.error || 'Server error', false);
      addLog('error', `Goal failed: ${data.error}`);
    }
  } catch {
    setFeedback(goalFeedback, 'Không thể kết nối server', false);
    addLog('error', 'Goal: không thể kết nối server');
  } finally {
    btnSendGoal.disabled = false;
  }
}

btnSendGoal.addEventListener('click', sendGoal);
[goalX, goalY].forEach(el => el.addEventListener('keydown', e => { if (e.key === 'Enter') sendGoal(); }));

// ── Gửi tốc độ ────────────────────────────────────────────────────────────────
async function sendSpeed(kmh) {
  const ts = new Date().toLocaleTimeString('en-GB', { hour12: false });

  // Ưu tiên ROS WebSocket nếu đang kết nối
  if (connected && speedPub) {
    const mps = kmh / 3.6;
    speedPub.publish(new ROSLIB.Message({ data: mps }));
    lastSpeedEl.innerHTML = `Sent at ${ts}: <b>${kmh.toFixed(2)} km/h (${mps.toFixed(3)} m/s)</b>`;
    setFeedback(speedFeedback, `✓ Published to /carla/${ROLE_NAME}/target_speed (ROS)`, true);
    addLog('info', `Speed [ROS] → ${kmh.toFixed(2)} km/h = ${mps.toFixed(3)} m/s`);
    return;
  }

  // Fallback: gửi qua Flask API
  btnSendSpeed.disabled = true;
  btnStop.disabled      = true;
  try {
    const res  = await fetch('/api/send-speed', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ speed_kmh: kmh }),
    });
    const data = await res.json();
    if (data.ok) {
      const via = data.ros ? ' → ROS' : ' (no ROS)';
      lastSpeedEl.innerHTML = `Sent at ${ts}: <b>${kmh.toFixed(2)} km/h (${data.speed_mps} m/s)</b>`;
      setFeedback(speedFeedback, `✓ Sent via server${via}`, true);
      addLog('info', `Speed [API${via}] → ${kmh.toFixed(2)} km/h = ${data.speed_mps} m/s`);
    } else {
      setFeedback(speedFeedback, data.error || 'Server error', false);
      addLog('error', `Speed failed: ${data.error}`);
    }
  } catch {
    setFeedback(speedFeedback, 'Không thể kết nối server', false);
    addLog('error', 'Speed: không thể kết nối server');
  } finally {
    btnSendSpeed.disabled = false;
    btnStop.disabled      = false;
  }
}

btnSendSpeed.addEventListener('click', () => {
  const v = parseFloat(speedKmh.value);
  if (isNaN(v) || v < 0) { setFeedback(speedFeedback, 'Speed phải là số >= 0.', false); return; }
  sendSpeed(v);
});

speedKmh.addEventListener('keydown', e => { if (e.key === 'Enter') btnSendSpeed.click(); });

btnStop.addEventListener('click', async () => {
  speedKmh.value = '0';
  speedMpsEl.textContent = '0.000 m/s';
  await sendSpeed(0);
  if (connected) setFeedback(speedFeedback, '⚠ Emergency stop sent!', true);
  addLog('warn', 'EMERGENCY STOP — speed = 0');
});
