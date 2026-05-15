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
let btnUploadWaypoint = document.getElementById('btnUploadWaypoint');
let waypointFileInput = document.getElementById('waypointFileInput');
let mapCanvasWrap = document.getElementById('mapCanvasWrap');
let overlayCanvas = document.getElementById('overlayCanvas');
let mapHud = document.getElementById('mapHud');
let mapHint = document.getElementById('mapHint');
let mapScaleLabel = document.getElementById('mapScaleLabel');
let btnSaveAlignment = document.getElementById('btnSaveAlignment');

let waypoints = [];
let waypointMetadata = null;
let waypointFileName = '';
let mapHasWaypoint = false;
let mapHasImage = false;
let mapImageNatural = { width: 0, height: 0 };
let imageTransform = { x: 0, y: 0, scale: 1 };
let overlayTransform = { x: 0, y: 0, scale: 1 };
let alignmentLocked = false;
let mapDrag = { active: false, startX: 0, startY: 0, origX: 0, origY: 0 };
let mapBaseBounds = { minX: 0, minY: 0, maxX: 0, maxY: 0 };
let mapBaseFit = { width: 0, height: 0 };

const MIN_ZOOM_STATIC = 0.1;  // 10%
const MAX_ZOOM_STATIC = 2.0;  // 200%
let MIN_ZOOM = 0.1;
let MAX_ZOOM = 2.0;

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

function updateMapHud() {
  mapHud.style.display = (mapHasWaypoint || mapHasImage) ? 'flex' : 'none';
  const hintParts = [];
  if (!mapHasWaypoint) hintParts.push('Upload waypoint JSON first');
  if (!mapHasImage) hintParts.push('Upload map image after');
  if (hintParts.length === 0) {
    if (alignmentLocked && hasValidWaypointMetadata(waypointMetadata)) {
      hintParts.push('Auto-aligned from metadata. Scroll to zoom, drag to pan.');
    } else {
      hintParts.push('Click a waypoint to fill the coordinate fields.');
    }
  }
  mapHint.textContent = hintParts.join(' · ');
  const currentScale = imageTransform.scale;
  const rect = mapCanvasWrap.getBoundingClientRect();
  mapScaleLabel.textContent = `zoom ${(currentScale * 100).toFixed(0)}% | viewport ${Math.round(rect.width)}×${Math.round(rect.height)}px`;
}

function setMapTransform() {
  if (mapImg.style.display !== 'none') {
    mapImg.style.transform = `translate(${imageTransform.x}px, ${imageTransform.y}px) scale(${imageTransform.scale})`;
  }
}

function calculateDynamicMinZoom() {
  const rect = mapCanvasWrap.getBoundingClientRect();
  if (!rect.width || !rect.height || !mapImageNatural.width || !mapImageNatural.height) {
    MIN_ZOOM = MIN_ZOOM_STATIC;
    return;
  }
  const minScaleX = rect.width / mapImageNatural.width;
  const minScaleY = rect.height / mapImageNatural.height;
  MIN_ZOOM = Math.max(MIN_ZOOM_STATIC, Math.max(minScaleX, minScaleY));
}

function constrainImageTransform() {
  const rect = mapCanvasWrap.getBoundingClientRect();
  if (!rect.width || !rect.height || !mapImageNatural.width || !mapImageNatural.height) return;
  
  const scaledWidth = mapImageNatural.width * imageTransform.scale;
  const scaledHeight = mapImageNatural.height * imageTransform.scale;
  
  // Giới hạn để không lộ vùng đen
  if (scaledWidth <= rect.width) {
    // Ảnh nhỏ hơn viewport ngang, center nó
    imageTransform.x = (rect.width - scaledWidth) / 2;
  } else {
    // Ảnh lớn hơn, clamp để không kéo ra ngoài
    imageTransform.x = clamp(imageTransform.x, rect.width - scaledWidth, 0);
  }
  
  if (scaledHeight <= rect.height) {
    // Ảnh nhỏ hơn viewport dọc, center nó
    imageTransform.y = (rect.height - scaledHeight) / 2;
  } else {
    // Ảnh lớn hơn, clamp để không kéo ra ngoài
    imageTransform.y = clamp(imageTransform.y, rect.height - scaledHeight, 0);
  }
  
  if (alignmentLocked) {
    overlayTransform.x = imageTransform.x;
    overlayTransform.y = imageTransform.y;
  }
}

function clamp(value, min, max) {
  return Math.min(Math.max(value, min), max);
}

function hasValidWaypointMetadata(metadata) {
  if (!metadata) return false;
  const required = ['min_x', 'min_y', 'scale', 'image_width', 'image_height', 'pixel_mode'];
  if (!required.every(key => metadata[key] !== undefined && metadata[key] !== null)) return false;
  return metadata.pixel_mode === 'xy_y_down';
}

function applyMetadataAlignment() {
  if (!hasValidWaypointMetadata(waypointMetadata)) return false;
  if (!mapImg.naturalWidth || !mapImg.naturalHeight) return false;
  if (mapImg.naturalWidth !== waypointMetadata.image_width || mapImg.naturalHeight !== waypointMetadata.image_height) {
    console.warn('Waypoint metadata image size does not match actual image size');
    return false;
  }

  const rect = mapCanvasWrap.getBoundingClientRect();
  const imageWidth = mapImg.naturalWidth;
  const imageHeight = mapImg.naturalHeight;
  const centerX = (rect.width - imageWidth) / 2;
  const centerY = (rect.height - imageHeight) / 2;

  imageTransform = { x: centerX, y: centerY, scale: 1 };
  overlayTransform = { x: centerX, y: centerY, scale: 1 };
  setMapTransform();
  drawOverlay();
  mapHasWaypoint = true;
  mapHasImage = true;
  alignmentLocked = true;
  btnSaveAlignment.textContent = 'Alignment Locked';
  btnSaveAlignment.disabled = true;
  addLog('info', 'Auto-aligned image and waypoint bằng metadata (1:1 pixel).');
  return true;
}

function calculateWaypointBounds() {
  if (!waypoints.length) return;
  let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
  waypoints.forEach(wp => {
    if (typeof wp.pixel_x !== 'number' || typeof wp.pixel_y !== 'number') return;
    minX = Math.min(minX, wp.pixel_x);
    minY = Math.min(minY, wp.pixel_y);
    maxX = Math.max(maxX, wp.pixel_x);
    maxY = Math.max(maxY, wp.pixel_y);
  });
  mapBaseBounds = { minX, minY, maxX, maxY };
  mapBaseFit = { width: maxX - minX, height: maxY - minY };
}

function updateCanvasSize() {
  const rect = mapCanvasWrap.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  overlayCanvas.width = Math.max(1, rect.width * dpr);
  overlayCanvas.height = Math.max(1, rect.height * dpr);
  overlayCanvas.style.width = `${rect.width}px`;
  overlayCanvas.style.height = `${rect.height}px`;
  const ctx = overlayCanvas.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  drawOverlay();
}

function drawOverlay() {
  const ctx = overlayCanvas.getContext('2d');
  const rect = mapCanvasWrap.getBoundingClientRect();
  ctx.clearRect(0, 0, rect.width, rect.height);
  if (!waypoints.length) return;
  ctx.save();
  ctx.translate(overlayTransform.x, overlayTransform.y);
  ctx.scale(overlayTransform.scale, overlayTransform.scale);
  ctx.strokeStyle = 'rgba(0, 200, 150, 0.95)';
  ctx.fillStyle = 'rgba(0, 200, 150, 0.95)';
  ctx.lineWidth = 2 / Math.max(overlayTransform.scale, 0.2);
  waypoints.forEach(wp => {
    if (typeof wp.pixel_x !== 'number' || typeof wp.pixel_y !== 'number') return;
    ctx.beginPath();
    ctx.arc(wp.pixel_x, wp.pixel_y, 3 / Math.max(overlayTransform.scale, 0.2), 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
  });
  ctx.restore();
}

function centerView() {
  const rect = mapCanvasWrap.getBoundingClientRect();
  if (!rect.width || !rect.height) return;

  calculateDynamicMinZoom();

  if (mapHasWaypoint && mapHasImage && hasValidWaypointMetadata(waypointMetadata)) {
    if (applyMetadataAlignment()) {
      updateMapHud();
      return;
    }
  }

  let centerX = rect.width / 2;
  let centerY = rect.height / 2;

  const overlayIsDefault = overlayTransform.x === 0 && overlayTransform.y === 0 && overlayTransform.scale === 1;
  const imageIsDefault = imageTransform.x === 0 && imageTransform.y === 0 && imageTransform.scale === 1;

  if (mapHasWaypoint && mapBaseFit.width > 0 && mapBaseFit.height > 0 && (!alignmentLocked || overlayIsDefault)) {
    const scale = Math.min(1, rect.width / (mapBaseFit.width + 40), rect.height / (mapBaseFit.height + 40));
    overlayTransform.scale = scale;
    overlayTransform.x = centerX - ((mapBaseBounds.minX + mapBaseBounds.maxX) / 2) * scale;
    overlayTransform.y = centerY - ((mapBaseBounds.minY + mapBaseBounds.maxY) / 2) * scale;
    if (!alignmentLocked || imageIsDefault) {
      imageTransform = { ...overlayTransform };
    }
  }

  if (mapHasImage && mapImageNatural.width && mapImageNatural.height) {
    if (!mapHasWaypoint || (!alignmentLocked && imageIsDefault)) {
      const scale = Math.min(1, rect.width / mapImageNatural.width, rect.height / mapImageNatural.height);
      imageTransform.scale = scale;
      imageTransform.x = (rect.width - mapImageNatural.width * scale) / 2;
      imageTransform.y = (rect.height - mapImageNatural.height * scale) / 2;
      if (!alignmentLocked) {
        overlayTransform = { ...imageTransform };
      }
    }
  }

  setMapTransform();
  updateMapHud();
  drawOverlay();
}

function handleMapWheel(event) {
  if (!mapHasWaypoint && !mapHasImage) return;
  event.preventDefault();
  calculateDynamicMinZoom();
  const delta = event.deltaY < 0 ? 1.04 : 0.96;
  const rect = mapCanvasWrap.getBoundingClientRect();
  const mouseX = event.clientX - rect.left;
  const mouseY = event.clientY - rect.top;
  const oldScale = imageTransform.scale;
  imageTransform.scale = clamp(imageTransform.scale * delta, MIN_ZOOM, MAX_ZOOM);
  const scaleRatio = imageTransform.scale / oldScale;
  imageTransform.x = mouseX - (mouseX - imageTransform.x) * scaleRatio;
  imageTransform.y = mouseY - (mouseY - imageTransform.y) * scaleRatio;
  if (alignmentLocked) {
    overlayTransform.scale *= scaleRatio;
    overlayTransform.x = mouseX - (mouseX - overlayTransform.x) * scaleRatio;
    overlayTransform.y = mouseY - (mouseY - overlayTransform.y) * scaleRatio;
  }
  constrainImageTransform();
  setMapTransform();
  updateMapHud();
  drawOverlay();
}

function handleMapPointerDown(event) {
  if (!mapHasWaypoint && !mapHasImage) return;
  event.preventDefault();
  mapDrag.active = true;
  mapCanvasWrap.classList.add('dragging');
  mapDrag.startX = event.clientX;
  mapDrag.startY = event.clientY;
  mapDrag.origX = imageTransform.x;
  mapDrag.origY = imageTransform.y;
}

function handleMapPointerMove(event) {
  if (!mapDrag.active) return;
  event.preventDefault();
  const dx = event.clientX - mapDrag.startX;
  const dy = event.clientY - mapDrag.startY;
  imageTransform.x = mapDrag.origX + dx;
  imageTransform.y = mapDrag.origY + dy;
  if (alignmentLocked) {
    overlayTransform.x = overlayTransform.x + dx;
    overlayTransform.y = overlayTransform.y + dy;
    mapDrag.startX = event.clientX;
    mapDrag.startY = event.clientY;
    mapDrag.origX = imageTransform.x;
    mapDrag.origY = imageTransform.y;
  }
  constrainImageTransform();
  setMapTransform();
  drawOverlay();
  updateMapHud();
}

function handleMapPointerUp() {
  mapDrag.active = false;
  mapCanvasWrap.classList.remove('dragging');
}

function findWaypointAt(x, y) {
  if (!waypoints.length) return null;
  let best = null;
  let bestDist = 20;
  waypoints.forEach(wp => {
    if (typeof wp.pixel_x !== 'number' || typeof wp.pixel_y !== 'number') return;
    const dx = wp.pixel_x * overlayTransform.scale + overlayTransform.x - x;
    const dy = wp.pixel_y * overlayTransform.scale + overlayTransform.y - y;
    const dist = Math.hypot(dx, dy);
    if (dist < bestDist) {
      bestDist = dist;
      best = wp;
    }
  });
  return best;
}

function saveAlignment() {
  alignmentLocked = true;
  btnSaveAlignment.textContent = 'Alignment Locked';
  btnSaveAlignment.disabled = true;
  setMapTransform();
  drawOverlay();
  addLog('info', 'Alignment locked. Waypoint và ảnh giờ đồng bộ.');
}

function loadWaypointFile(file) {
  const reader = new FileReader();
  reader.onload = () => {
    try {
      const data = JSON.parse(reader.result);
      waypointFileName = file.name;
      waypointMetadata = data.metadata || null;
      waypoints = Array.isArray(data.waypoints) ? data.waypoints : [];
      if (!waypoints.length) {
        setFeedback(uploadFeedback, 'Waypoint JSON không có trường waypoints.', false);
        return;
      }
      mapHasWaypoint = true;
      mapHasImage = mapImg.src !== '' && mapImg.style.display !== 'none';
      mapTitle.textContent = waypointFileName;
      calculateWaypointBounds();
      updateMapHud();
      overlayCanvas.style.display = 'block';
      mapPlaceholder.style.display = 'none';
      btnSaveAlignment.disabled = !mapHasImage;
      addLog('info', `Waypoints loaded: ${waypoints.length} points.`);
      if (mapHasImage && hasValidWaypointMetadata(waypointMetadata)) {
        centerView();
      } else if (!mapHasImage) {
        centerView();
      }
    } catch (err) {
      setFeedback(uploadFeedback, 'Không thể đọc file waypoint JSON.', false);
      addLog('error', `Waypoint load error: ${err}`);
    }
  };
  reader.readAsText(file);
}

function handleWaypointClick(event) {
  const rect = mapCanvasWrap.getBoundingClientRect();
  const clickX = event.clientX - rect.left;
  const clickY = event.clientY - rect.top;
  const hit = findWaypointAt(clickX, clickY);
  if (!hit) return;
  if (typeof hit.x === 'number' && typeof hit.y === 'number') {
    goalX.value = hit.x.toFixed(3);
    goalY.value = hit.y.toFixed(3);
    setFeedback(goalFeedback, 'Waypoint selected. Nhấn Send Goal để gửi.', true);
    addLog('info', `Waypoint selected → x=${hit.x.toFixed(3)}, y=${hit.y.toFixed(3)}`);
  }
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

mapImg.addEventListener('load', () => {
  mapImageNatural = { width: mapImg.naturalWidth, height: mapImg.naturalHeight };
  mapHasImage = true;
  mapPlaceholder.style.display = 'none';
  mapImg.style.display = 'block';
  if (mapHasWaypoint && hasValidWaypointMetadata(waypointMetadata)) {
    centerView();
  } else if (!alignmentLocked) {
    centerView();
  }
  btnSaveAlignment.disabled = !mapHasWaypoint;
  updateMapHud();
});

btnUploadWaypoint.addEventListener('click', () => waypointFileInput.click());
waypointFileInput.addEventListener('change', () => {
  if (waypointFileInput.files[0]) loadWaypointFile(waypointFileInput.files[0]);
});

mapCanvasWrap.addEventListener('mousedown', handleMapPointerDown);
mapCanvasWrap.addEventListener('mousemove', handleMapPointerMove);
mapCanvasWrap.addEventListener('mouseup', handleMapPointerUp);
mapCanvasWrap.addEventListener('mouseleave', handleMapPointerUp);
mapCanvasWrap.addEventListener('wheel', handleMapWheel, { passive: false });
mapCanvasWrap.addEventListener('click', handleWaypointClick);
window.addEventListener('mousemove', handleMapPointerMove);
window.addEventListener('mouseup', handleMapPointerUp);
window.addEventListener('resize', updateCanvasSize);
btnSaveAlignment.addEventListener('click', saveAlignment);
updateCanvasSize();

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
      mapHasImage = true;
      btnSaveAlignment.disabled = !mapHasWaypoint;
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
