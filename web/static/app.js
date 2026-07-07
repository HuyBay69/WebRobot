'use strict';

// ── DOM ───────────────────────────────────────────────────────────────────────
const btnConnect     = document.getElementById('btnConnect');
const ROLE_NAME      = 'hero';
const connDot        = document.getElementById('connDot');
const connStatus     = document.getElementById('connStatus');

const mapTitle       = document.getElementById('mapTitle');
const mapPlaceholder = document.getElementById('mapPlaceholder');
const mapImg         = document.getElementById('mapImg');
const selectedWaypointPin = document.getElementById('selectedWaypointPin');

const goalX          = document.getElementById('goalX');
const goalY          = document.getElementById('goalY');
const btnSendGoal    = document.getElementById('btnSendGoal');
const speedKmh       = document.getElementById('speedKmh');
const speedKmhValue  = document.getElementById('speedKmhValue');
const btnStop        = document.getElementById('btnStop');

const logBody        = document.getElementById('logBody');
const locationNameError = document.getElementById('locationNameError');

const btnSpawnCar    = document.getElementById('btnSpawnCar');
const spawnFeedback  = document.getElementById('spawnFeedback');

let mapCanvasWrap = document.getElementById('mapCanvasWrap');
let overlayCanvas = document.getElementById('overlayCanvas');
let mapHud        = document.getElementById('mapHud');
let mapHint       = document.getElementById('mapHint');
let mapScaleLabel = document.getElementById('mapScaleLabel');

const carIcon      = document.getElementById('carIcon');
const actualSpeedEl = document.getElementById('actualSpeed');

// ── State ─────────────────────────────────────────────────────────────────────
let ros       = null;   // giữ lại để không break các chỗ check `connected && goalPub`
let goalPub   = null;
let speedPub  = null;
let connected = false;  // alias rosRunning, giữ để sendGoal/sendSpeed fallback đúng

let waypoints        = [];
let waypointMetadata = null;
let mapHasWaypoint   = false;
let mapHasImage      = false;
let mapImageNatural  = { width: 0, height: 0 };
let imageTransform   = { x: 0, y: 0, scale: 1 };
let overlayTransform = { x: 0, y: 0, scale: 1 };
let alignmentLocked  = false;
let mapDrag = { active: false, startX: 0, startY: 0, origX: 0, origY: 0 };
let mapBaseBounds = { minX: 0, minY: 0, maxX: 0, maxY: 0 };
let mapBaseFit    = { width: 0, height: 0 };
let selectedWaypoint = null;

// ── Spawn Car state ────────────────────────────────────────────────────────────
// spawnPoints: danh sách điểm spawn lấy từ CARLA — [{ id, x, y, z, yaw }, ...]
// Ban đầu ẩn trên bản đồ; bấm nút "Spawn Car" để hiện/ẩn, chọn 1 điểm để spawn xe.
let spawnPoints        = [];
let spawnPointsVisible = false;

// ── Live Tracking state ───────────────────────────────────────────────────────
// (sẽ được khởi tạo lại khi viết odom_node.py)

// ── Odom WebSocket ────────────────────────────────────────────────────────────
// Nhận dữ liệu position/yaw từ odom_node.py qua /ws/odom_stream.
// Mỗi frame: { x, y, yaw, ros_t, delta_ms } — tọa độ ROS frame (meters).
// Log ra F12 console để debug; vẽ chấm đỏ lên canvas.

let odomWs        = null;
let latestOdom    = null;   // frame cuối nhận được — { x, y, yaw, ros_t, delta_ms }

function startOdomStream() {
  if (odomWs && odomWs.readyState === WebSocket.OPEN) return;

  const url = `ws://${location.host}/ws/odom_stream`;
  odomWs = new WebSocket(url);

  odomWs.onopen = () => {
    console.log('[OdomStream] Kết nối WebSocket thành công:', url);
  };

  odomWs.onmessage = (e) => {
    let frame;
    try {
      frame = JSON.parse(e.data);
    } catch {
      console.warn('[OdomStream] Frame lỗi JSON:', e.data);
      return;
    }

    latestOdom = frame;

    // ── Log ra F12 console ─────────────────────────────────────────────────
    console.debug(
      `[OdomStream] x=${frame.x.toFixed(3)} y=${frame.y.toFixed(3)} ` +
      `yaw=${frame.yaw.toFixed(4)} ros_t=${frame.ros_t.toFixed(3)} ` +
      `Δt=${frame.delta_ms.toFixed(2)}ms`
    );

    // ── Vẽ xe lên map ─────────────────────────────────────────────────────
    drawVehicleDot(frame.x, frame.y, frame.yaw);
  };

  odomWs.onerror = (e) => {
    console.warn('[OdomStream] WebSocket lỗi:', e);
  };

  odomWs.onclose = () => {
    console.log('[OdomStream] WebSocket đóng — thử lại sau 3s...');
    odomWs = null;
    setTimeout(startOdomStream, 3000);   // tự reconnect
  };
}

// ── Speed slider (thanh kéo tốc độ) ──────────────────────────────────────────
// Kéo tới đâu thì số hiển thị cạnh thanh kéo cập nhật theo tới đó (live).
speedKmhValue.textContent = speedKmh.value;   // đồng bộ với giá trị mặc định trong HTML
speedKmh.addEventListener('input', () => {
  speedKmhValue.textContent = speedKmh.value;
});

// ── Speedometer WebSocket (tốc độ thực) ──────────────────────────────────────
// Nhận tốc độ thực (km/h, đã quy đổi từ m/s) từ speedometer_node.py qua
// /ws/speedometer_stream, hiển thị vào ô "Actual" cạnh thanh kéo Speed.
// Node này khởi động sau odom_node (ưu tiên thấp hơn) nên có thể tới muộn hơn
// vài giây — trước khi có frame đầu tiên, ô Actual giữ trạng thái "no-data".

let speedometerWs      = null;
let lastSpeedUpdateT   = 0;      // timestamp (ms) — 0 = chưa từng nhận frame nào
const SPEED_STALE_MS   = 2000;   // quá 2s không có frame mới → coi là "stale"

function setActualSpeedDisplay(kmh) {
  const rounded = Math.round(Math.max(0, kmh));   // số nguyên, luôn dương
  actualSpeedEl.textContent = `${rounded} km/h`;
  actualSpeedEl.classList.remove('no-data', 'stale');
}

function startSpeedometerStream() {
  if (speedometerWs && speedometerWs.readyState === WebSocket.OPEN) return;

  const url = `ws://${location.host}/ws/speedometer_stream`;
  speedometerWs = new WebSocket(url);

  speedometerWs.onopen = () => {
    console.log('[SpeedometerStream] Kết nối WebSocket thành công:', url);
  };

  speedometerWs.onmessage = (e) => {
    let frame;
    try {
      frame = JSON.parse(e.data);
    } catch {
      console.warn('[SpeedometerStream] Frame lỗi JSON:', e.data);
      return;
    }
    lastSpeedUpdateT = Date.now();
    setActualSpeedDisplay(frame.kmh);
  };

  speedometerWs.onerror = (e) => {
    console.warn('[SpeedometerStream] WebSocket lỗi:', e);
  };

  speedometerWs.onclose = () => {
    console.log('[SpeedometerStream] WebSocket đóng — thử lại sau 3s...');
    speedometerWs = null;
    setTimeout(startSpeedometerStream, 3000);   // tự reconnect
  };
}

// Kiểm tra định kỳ — quá lâu không có frame mới thì đánh dấu "stale"
// (chưa từng nhận frame nào thì giữ nguyên trạng thái "no-data" ban đầu).
setInterval(() => {
  if (lastSpeedUpdateT === 0) return;
  if (Date.now() - lastSpeedUpdateT > SPEED_STALE_MS) {
    actualSpeedEl.classList.add('stale');
  }
}, 1000);

/** Vẽ chấm đỏ viền trắng tại vị trí xe (ROS frame → pixel → screen). */
// ── Vehicle icon (car.svg) ────────────────────────────────────────────────────
// Dùng img#carIcon đã có trong HTML, đặt position absolute trên map-canvas-wrap.
// SVG gốc: mũi xe trỏ lên (12h) — khớp với yaw=0 = hướng Bắc (ROS +X).
// Công thức rotate: css_yaw = -(yaw_rad * 180/PI) + 90
//   vì ROS yaw=0 → xe hướng +X (3h) nhưng SVG mũi trỏ 12h → cần offset +90°
//   dấu âm vì CSS rotate() thuận chiều kim đồng hồ, ROS ngược chiều.

const CAR_SIZE = 44;   // px — tăng từ 32 lên 44

carIcon.style.width    = `${CAR_SIZE}px`;
carIcon.style.height   = `${CAR_SIZE}px`;
carIcon.style.position = 'absolute';
carIcon.style.display  = 'none';
carIcon.style.transformOrigin = '50% 50%';
carIcon.style.pointerEvents   = 'none';
carIcon.src = '/static/icons/car.svg';

let _lastCarPixel  = null;
let _cssYawAccum   = 0;     // yaw tích lũy (unwrapped) — tránh CSS rotate giật tại ±π

function updateCarIcon() {
  if (!_lastCarPixel || !mapHasImage || !hasValidWaypointMetadata(waypointMetadata)) return;
  const { pixel_x, pixel_y } = _lastCarPixel;

  // Kích thước hiển thị = CAR_SIZE * s * currentScale  (carLayer đang scale currentScale)
  // Dưới 50%: cố định 44px  → s = 1/currentScale
  // Từ  50%:  to theo bản đồ → s = 1/SCALE_THRESHOLD (hằng số)
  //   tại 50%  → 44 * 2.0 * 0.5 = 44px  (liên tục, không giật)
  //   tại 100% → 44 * 2.0 * 1.0 = 88px
  //   tại 200% → 44 * 2.0 * 2.0 = 176px
  const SCALE_THRESHOLD = 0.5;
  const s = imageTransform.scale < SCALE_THRESHOLD
    ? 1 / imageTransform.scale          // chế độ cố định
    : 1 / SCALE_THRESHOLD;              // chế độ theo bản đồ (s = 2.0)

  carIcon.style.transformOrigin = '0 0';
  carIcon.style.transform =
    `translate(${pixel_x}px, ${pixel_y}px)` +
    ` rotate(${_cssYawAccum}deg)` +
    ` scale(${s})` +
    ` translate(${-CAR_SIZE / 2}px, ${-CAR_SIZE / 2}px)`;
  carIcon.style.display = 'block';
}

function drawVehicleDot(rosX, rosY, yawRad) {
  if (!mapHasImage || !hasValidWaypointMetadata(waypointMetadata)) return;

  const pixel = rosToPixel(rosX, rosY);
  if (!pixel) return;

  // Unwrap yaw — tính delta ngắn nhất từ góc tích lũy hiện tại
  const rawDeg  = -(yawRad * 180 / Math.PI) + 90;
  let diff      = rawDeg - (_cssYawAccum % 360);
  if (diff >  180) diff -= 360;
  if (diff < -180) diff += 360;
  _cssYawAccum += diff;

  _lastCarPixel = { pixel_x: pixel.pixel_x, pixel_y: pixel.pixel_y };

  drawOverlay();
  updateCarIcon();
}

const MIN_ZOOM_STATIC = 0.1;
const MAX_ZOOM_STATIC = 2.0;
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
  return String(s).replace(/[&<>"']/g, c =>
    ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
}

// ── Feedback ──────────────────────────────────────────────────────────────────
function setFeedback(el, msg, ok) {
  el.textContent = msg;
  el.className = 'feedback ' + (ok ? 'ok' : 'err');
  if (ok) setTimeout(() => { el.textContent = ''; el.className = 'feedback'; }, 5000);
}

// ── Map Canvas & Overlay ─────────────────────────────────────────────────────
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

// ── carLayer — div cùng transform với mapImg, carIcon là con của nó ──────────
// Tạo và inject vào mapCanvasWrap ngay khi script load
const carLayer = document.createElement('div');
carLayer.id = 'carLayer';
carLayer.style.cssText = `
  position: absolute;
  top: 0; left: 0;
  width: 0; height: 0;
  overflow: visible;
  pointer-events: none;
  transform-origin: 0 0;
`;
// Đặt carIcon vào carLayer thay vì mapCanvasWrap
carLayer.appendChild(carIcon);
// Inject vào mapCanvasWrap (sẽ có sau DOMContentLoaded)
document.addEventListener('DOMContentLoaded', () => {
  mapCanvasWrap.appendChild(carLayer);
}, { once: true });
// Fallback nếu DOM đã load
if (document.readyState !== 'loading') mapCanvasWrap.appendChild(carLayer);

function setMapTransform() {
  const t = `translate(${imageTransform.x}px, ${imageTransform.y}px) scale(${imageTransform.scale})`;
  if (mapImg.style.display !== 'none') {
    mapImg.style.transform = t;
  }
  carLayer.style.transform = t;
  updateCarIcon();   // counter-scale cập nhật cùng tick, không delay
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

  const scaledWidth  = mapImageNatural.width  * imageTransform.scale;
  const scaledHeight = mapImageNatural.height * imageTransform.scale;

  if (scaledWidth <= rect.width) {
    imageTransform.x = (rect.width - scaledWidth) / 2;
  } else {
    imageTransform.x = clamp(imageTransform.x, rect.width - scaledWidth, 0);
  }

  if (scaledHeight <= rect.height) {
    imageTransform.y = (rect.height - scaledHeight) / 2;
  } else {
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

  const rect        = mapCanvasWrap.getBoundingClientRect();
  const imageWidth  = mapImg.naturalWidth;
  const imageHeight = mapImg.naturalHeight;
  const centerX     = (rect.width  - imageWidth)  / 2;
  const centerY     = (rect.height - imageHeight) / 2;

  imageTransform   = { x: centerX, y: centerY, scale: 1 };
  overlayTransform = { x: centerX, y: centerY, scale: 1 };
  setMapTransform();
  drawOverlay();

  mapHasWaypoint  = true;
  mapHasImage     = true;
  alignmentLocked = true;
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
  mapBaseFit    = { width: maxX - minX, height: maxY - minY };
}

function updateCanvasSize() {
  const rect = mapCanvasWrap.getBoundingClientRect();
  const dpr  = window.devicePixelRatio || 1;
  overlayCanvas.width  = Math.max(1, rect.width  * dpr);
  overlayCanvas.height = Math.max(1, rect.height * dpr);
  overlayCanvas.style.width  = `${rect.width}px`;
  overlayCanvas.style.height = `${rect.height}px`;
  const ctx = overlayCanvas.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  drawOverlay();
  updateCarIcon();
  updateSelectedPin();

}

function updateSelectedPin() {
  if (!selectedWaypoint || !mapHasImage) {
    selectedWaypointPin.style.display = 'none';
    return;
  }
  const x = selectedWaypoint.pixel_x * overlayTransform.scale + overlayTransform.x;
  const y = selectedWaypoint.pixel_y * overlayTransform.scale + overlayTransform.y;
  selectedWaypointPin.style.left    = `${x}px`;
  selectedWaypointPin.style.top     = `${y}px`;
  selectedWaypointPin.style.display = 'block';
}

// ── Live Tracking helpers ─────────────────────────────────────────────────────

/**
 * Chuyển Quaternion → Yaw (radians).
 * Công thức: yaw = atan2(2(wz + xy), 1 − 2(y² + z²))
 */
function quatToYawRad(q) {
  const { x, y, z, w } = q;
  return Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
}

/**
 * Chiếu tọa độ ROS (meters) → pixel trong ảnh gốc.
 * Sau khi đảo Y trong waypoints.json (y_new = -y_ros):
 *   pixel_x = (ros_x - min_x) * scale
 *   pixel_y = (max_y - y_flipped) * scale  với y_flipped = -ros_y
 *           = (max_y + ros_y) * scale
 * Đã xác minh khớp pixel_y thực tế trong waypoints.json.
 */
function rosToPixel(rosX, rosY) {
  if (!hasValidWaypointMetadata(waypointMetadata)) return null;
  const m = waypointMetadata;
  return {
    pixel_x: (rosX - m.min_x) * m.scale,
    pixel_y: (m.max_y - rosY) * m.scale,
  };
}

/**
 * Chuyển pixel gốc trong ảnh → vị trí tuyệt đối (px) trên màn hình,
 * có tính pan/zoom hiện tại qua overlayTransform.
 */
function imagePixelToScreen(pixel_x, pixel_y) {
  return {
    screenX: pixel_x * overlayTransform.scale + overlayTransform.x,
    screenY: pixel_y * overlayTransform.scale + overlayTransform.y,
  };
}

function drawOverlay() {
  const ctx  = overlayCanvas.getContext('2d');
  const rect = mapCanvasWrap.getBoundingClientRect();
  ctx.clearRect(0, 0, rect.width, rect.height);

  if (waypoints.length) {
    ctx.save();
    ctx.strokeStyle = 'rgba(0, 200, 150, 0.95)';
    ctx.fillStyle   = 'rgba(0, 200, 150, 0.95)';
    ctx.lineWidth   = 2;
    waypoints.forEach(wp => {
      if (typeof wp.pixel_x !== 'number' || typeof wp.pixel_y !== 'number') return;
      const x      = wp.pixel_x * overlayTransform.scale + overlayTransform.x;
      const y      = wp.pixel_y * overlayTransform.scale + overlayTransform.y;
      const radius = Math.max(1, 3 * overlayTransform.scale);
      ctx.beginPath();
      ctx.arc(x, y, radius, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();
    });
    ctx.restore();
  }

  // ── Spawn points (CARLA) — chỉ vẽ khi người dùng bấm "Spawn Car" ──────────
  if (spawnPointsVisible && spawnPoints.length) {
    ctx.save();
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.95)';
    ctx.fillStyle   = 'rgba(59, 130, 246, 0.95)';   // var(--info)
    ctx.lineWidth   = 2.5;
    spawnPoints.forEach(sp => {
      const pixel = rosToPixel(sp.x, sp.y);
      if (!pixel) return;
      const x      = pixel.pixel_x * overlayTransform.scale + overlayTransform.x;
      const y      = pixel.pixel_y * overlayTransform.scale + overlayTransform.y;
      const radius = Math.max(6, 11 * overlayTransform.scale);
      ctx.beginPath();
      ctx.arc(x, y, radius, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();
    });
    ctx.restore();
  }
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

  const centerX = rect.width  / 2;
  const centerY = rect.height / 2;

  const overlayIsDefault = overlayTransform.x === 0 && overlayTransform.y === 0 && overlayTransform.scale === 1;
  const imageIsDefault   = imageTransform.x   === 0 && imageTransform.y   === 0 && imageTransform.scale   === 1;

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
      imageTransform.x = (rect.width  - mapImageNatural.width  * scale) / 2;
      imageTransform.y = (rect.height - mapImageNatural.height * scale) / 2;
      if (!alignmentLocked) {
        overlayTransform = { ...imageTransform };
      }
    }
  }

  setMapTransform();
  updateMapHud();
  drawOverlay();
  updateCarIcon();
}

// ── Map interaction events ────────────────────────────────────────────────────
function handleMapWheel(event) {
  if (!mapHasWaypoint && !mapHasImage) return;
  event.preventDefault();
  calculateDynamicMinZoom();
  const delta  = event.deltaY < 0 ? 1.04 : 0.96;
  const rect   = mapCanvasWrap.getBoundingClientRect();
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
  updateSelectedPin();
  requestAnimationFrame(() => { drawOverlay(); });
}

function handleMapPointerDown(event) {
  if (!mapHasWaypoint && !mapHasImage) return;
  event.preventDefault();
  mapDrag.active = true;
  mapCanvasWrap.classList.add('dragging');
  mapDrag.startX = event.clientX;
  mapDrag.startY = event.clientY;
  mapDrag.origX  = imageTransform.x;
  mapDrag.origY  = imageTransform.y;
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
    mapDrag.origX  = imageTransform.x;
    mapDrag.origY  = imageTransform.y;
  }
  constrainImageTransform();
  setMapTransform();
  updateSelectedPin();
  requestAnimationFrame(() => { drawOverlay(); });
  updateMapHud();
}

function handleMapPointerUp() {
  mapDrag.active = false;
  mapCanvasWrap.classList.remove('dragging');
}

function findWaypointAt(x, y) {
  if (!waypoints.length) return null;
  let best = null, bestDist = 20;
  waypoints.forEach(wp => {
    if (typeof wp.pixel_x !== 'number' || typeof wp.pixel_y !== 'number') return;
    const dx   = wp.pixel_x * overlayTransform.scale + overlayTransform.x - x;
    const dy   = wp.pixel_y * overlayTransform.scale + overlayTransform.y - y;
    const dist = Math.hypot(dx, dy);
    if (dist < bestDist) { bestDist = dist; best = wp; }
  });
  return best;
}

function findSpawnPointAt(x, y) {
  if (!spawnPoints.length) return null;
  let best = null, bestDist = 26;
  spawnPoints.forEach(sp => {
    const pixel = rosToPixel(sp.x, sp.y);
    if (!pixel) return;
    const dx   = pixel.pixel_x * overlayTransform.scale + overlayTransform.x - x;
    const dy   = pixel.pixel_y * overlayTransform.scale + overlayTransform.y - y;
    const dist = Math.hypot(dx, dy);
    if (dist < bestDist) { bestDist = dist; best = sp; }
  });
  return best;
}

function handleWaypointClick(event) {
  const rect   = mapCanvasWrap.getBoundingClientRect();
  const clickX = event.clientX - rect.left;
  const clickY = event.clientY - rect.top;

  // ── Đang ở chế độ chọn spawn point → ưu tiên xử lý riêng, bỏ qua waypoint thường
  if (spawnPointsVisible) {
    const spHit = findSpawnPointAt(clickX, clickY);
    if (spHit) selectSpawnPoint(spHit);
    return;
  }

  const hit    = findWaypointAt(clickX, clickY);
  if (!hit) return;
  selectedWaypoint = hit;
  updateSelectedPin();
  if (typeof hit.x === 'number' && typeof hit.y === 'number') {
    goalX.value = hit.x.toFixed(3);
    goalY.value = hit.y.toFixed(3);
    document.getElementById('navFeedback').textContent = 'Chọn vị trí thành công. Nhấn Send Goal để gửi.';
    document.getElementById('navFeedback').className   = 'feedback ok';
    addLog('info', `Waypoint selected → x=${hit.x.toFixed(3)}, y=${hit.y.toFixed(3)}`);
  }
}

mapCanvasWrap.addEventListener('mousedown',  handleMapPointerDown);
mapCanvasWrap.addEventListener('mousemove',  handleMapPointerMove);
mapCanvasWrap.addEventListener('mouseup',    handleMapPointerUp);
mapCanvasWrap.addEventListener('mouseleave', handleMapPointerUp);
mapCanvasWrap.addEventListener('wheel',      handleMapWheel, { passive: false });
mapCanvasWrap.addEventListener('click',      handleWaypointClick);
window.addEventListener('mousemove', handleMapPointerMove);
window.addEventListener('mouseup',   handleMapPointerUp);
window.addEventListener('resize',    updateCanvasSize);
window.addEventListener('load',      updateCanvasSize);
updateCanvasSize();

mapImg.addEventListener('load', () => {
  mapImageNatural = { width: mapImg.naturalWidth, height: mapImg.naturalHeight };
  mapHasImage     = true;
  mapPlaceholder.style.display = 'none';
  mapImg.style.display = 'block';
  updateCanvasSize();
  if (mapHasWaypoint && hasValidWaypointMetadata(waypointMetadata)) {
    centerView();
  } else if (!alignmentLocked) {
    centerView();
  }
  updateMapHud();
  updateSelectedPin();
});

// ── Load waypoint từ JSON text (dùng nội bộ sau khi fetch) ───────────────────
function applyWaypointData(jsonText, sourceName) {
  try {
    const data = JSON.parse(jsonText);
    waypointMetadata = data.metadata || null;
    waypoints = Array.isArray(data.waypoints) ? data.waypoints : [];
    if (!waypoints.length) {
      addLog('warn', `${sourceName}: Không có trường waypoints.`);
      return false;
    }
    mapHasWaypoint  = true;
    alignmentLocked = false;
    calculateWaypointBounds();
    overlayCanvas.style.display = 'block';
    selectedWaypoint = null;
    addLog('info', `Waypoints loaded (${sourceName}): ${waypoints.length} điểm.`);
    return true;
  } catch (err) {
    addLog('error', `Waypoint parse error (${sourceName}): ${err}`);
    return false;
  }
}

// ── Hàm cốt lõi: load bản đồ theo map object trả về từ API ──────────────────
async function loadMapById(mapObj) {
  addLog('info', `Đang load bản đồ: ${mapObj.name}…`);

  // Báo cho backend biết bản đồ này vừa được chọn/load — server sẽ tự động lấy
  // spawn point mới từ CARLA (nếu bridge đang kết nối). Không chặn UI nếu lỗi.
  fetch(`/api/maps/${mapObj.id}/load`)
    .then(res => res.json())
    .then(data => {
      if (!data.ok) addLog('warn', `Báo load map tới server thất bại: ${data.error || ''}`);
    })
    .catch(err => addLog('warn', `Không báo được map load tới server: ${err}`));

  // Reset state
  waypoints        = [];
  waypointMetadata = null;
  mapHasWaypoint   = false;
  mapHasImage      = false;
  alignmentLocked  = false;
  selectedWaypoint = null;
  imageTransform   = { x: 0, y: 0, scale: 1 };
  overlayTransform = { x: 0, y: 0, scale: 1 };

  mapImg.style.display = 'none';
  mapImg.src = '';
  overlayCanvas.style.display = 'none';
  mapPlaceholder.style.display = 'flex';
  mapTitle.textContent = mapObj.name;

  // Load waypoint
  if (mapObj.waypoint_url) {
    try {
      const res = await fetch(mapObj.waypoint_url);
      if (!res.ok) throw new Error(`HTTP ${res.status}`);
      const text = await res.text();
      applyWaypointData(text, `#${mapObj.id}`);
    } catch (err) {
      addLog('error', `Không load được waypoint: ${err}`);
    }
  } else {
    addLog('warn', `Bản đồ #${mapObj.id} chưa có file waypoint.`);
  }

  // Load ảnh
  if (mapObj.image_url) {
    mapImg.src = mapObj.image_url + '?t=' + Date.now();
    // mapImg onload sẽ tự gọi centerView/updateMapHud
  } else {
    addLog('warn', `Bản đồ #${mapObj.id} chưa có file ảnh.`);
    // Nếu có waypoint nhưng không có ảnh → vẫn render overlay
    if (mapHasWaypoint) {
      overlayCanvas.style.display = 'block';
      mapPlaceholder.style.display = 'none';
      centerView();
      updateMapHud();
    }
  }
}

// ══════════════════════════════════════════════════════════════════════════════
//  MAP MANAGER MODAL
// ══════════════════════════════════════════════════════════════════════════════
const btnSelectMap     = document.getElementById('btnSelectMap');
const mapManagerModal  = document.getElementById('mapManagerModal');
const btnManagerClose  = document.getElementById('btnManagerClose');
const mapListWrap      = document.getElementById('mapListWrap');
const mapListEl        = document.getElementById('mapList');
const mapListEmpty     = document.getElementById('mapListEmpty');
const btnLoadMap       = document.getElementById('btnLoadMap');
const btnOpenUpload    = document.getElementById('btnOpenUpload');
const managerFeedback  = document.getElementById('managerFeedback');

let selectedMapObj = null;  // bản đồ đang được chọn trong danh sách

function openMapManager() {
  selectedMapObj = null;
  btnLoadMap.disabled = true;
  managerFeedback.textContent = '';
  mapManagerModal.style.display = 'flex';
  loadMapList();
}

function closeMapManager() {
  mapManagerModal.style.display = 'none';
}

async function loadMapList() {
  // Hiện loading
  mapListEl.innerHTML = '';
  mapListEmpty.style.display = 'none';
  mapListWrap.innerHTML = `
    <div class="map-list-loading">
      <div class="spinner"></div>
      Đang tải danh sách…
    </div>`;

  try {
    const res  = await fetch('/api/maps');
    const data = await res.json();

    mapListWrap.innerHTML = '';
    mapListWrap.appendChild(mapListEmpty);
    mapListWrap.appendChild(mapListEl);

    if (!data.ok || !data.maps.length) {
      mapListEmpty.style.display = 'block';
      mapListEl.innerHTML = '';
      return;
    }

    mapListEmpty.style.display = 'none';
    mapListEl.innerHTML = '';

    data.maps.forEach(m => {
      const li = document.createElement('li');
      li.className = 'map-list-item';
      li.dataset.mapId = m.id;

      const imgTag  = m.has_image    ? '<span class="map-item-tag">Ảnh ✓</span>'     : '<span class="map-item-tag missing">Thiếu ảnh</span>';
      const wpTag   = m.has_waypoint ? '<span class="map-item-tag">Waypoint ✓</span>' : '<span class="map-item-tag missing">Thiếu waypoint</span>';

      li.innerHTML = `
        <div class="map-item-idx">${m.id}</div>
        <div class="map-item-info">
          <div class="map-item-name">${escHtml(m.name)}</div>
          <div class="map-item-meta">${imgTag}${wpTag}</div>
        </div>`;

      li.addEventListener('click', () => {
        // Bỏ chọn item cũ
        mapListEl.querySelectorAll('.map-list-item').forEach(el => el.classList.remove('selected'));
        li.classList.add('selected');
        selectedMapObj = m;
        btnLoadMap.disabled = false;
      });

      // Double-click = load ngay
      li.addEventListener('dblclick', () => {
        selectedMapObj = m;
        doLoadMap();
      });

      mapListEl.appendChild(li);
    });
  } catch (err) {
    mapListWrap.innerHTML = '';
    mapListWrap.appendChild(mapListEmpty);
    mapListWrap.appendChild(mapListEl);
    mapListEmpty.style.display = 'block';
    mapListEmpty.textContent = `Lỗi tải danh sách: ${err}`;
    addLog('error', `Map list error: ${err}`);
  }
}

async function doLoadMap() {
  if (!selectedMapObj) return;
  closeMapManager();
  await loadMapById(selectedMapObj);
}

btnSelectMap.addEventListener('click', openMapManager);
btnManagerClose.addEventListener('click', closeMapManager);
mapManagerModal.addEventListener('click', e => { if (e.target === mapManagerModal) closeMapManager(); });
btnLoadMap.addEventListener('click', doLoadMap);
btnOpenUpload.addEventListener('click', () => {
  closeMapManager();
  openUploadModal();
});

// ══════════════════════════════════════════════════════════════════════════════
//  UPLOAD MAP MỚI MODAL
// ══════════════════════════════════════════════════════════════════════════════
const uploadMapModal    = document.getElementById('uploadMapModal');
const btnUploadClose    = document.getElementById('btnUploadClose');
const btnUploadCancel   = document.getElementById('btnUploadCancel');
const btnUploadSubmit   = document.getElementById('btnUploadSubmit');
const newMapName        = document.getElementById('newMapName');
const newWaypointInput  = document.getElementById('newWaypointInput');
const newImageInput     = document.getElementById('newImageInput');
const btnPickWaypoint   = document.getElementById('btnPickWaypoint');
const btnPickImage      = document.getElementById('btnPickImage');
const wpFileName        = document.getElementById('wpFileName');
const imgFileName       = document.getElementById('imgFileName');
const errMapName        = document.getElementById('errMapName');
const errWaypoint       = document.getElementById('errWaypoint');
const errImage          = document.getElementById('errImage');
const uploadNewFeedback = document.getElementById('uploadNewFeedback');

let pendingWpFile  = null;
let pendingImgFile = null;

function openUploadModal() {
  // Reset form
  newMapName.value        = '';
  wpFileName.textContent  = 'Chưa chọn file';
  wpFileName.classList.remove('has-file');
  imgFileName.textContent = 'Chưa chọn file';
  imgFileName.classList.remove('has-file');
  newWaypointInput.value  = '';
  newImageInput.value     = '';
  pendingWpFile  = null;
  pendingImgFile = null;
  errMapName.textContent  = '';
  errWaypoint.textContent = '';
  errImage.textContent    = '';
  uploadNewFeedback.textContent = '';
  uploadNewFeedback.className   = 'feedback';
  uploadMapModal.style.display = 'flex';
  newMapName.focus();
}

function closeUploadModal(backToManager = false) {
  uploadMapModal.style.display = 'none';
  if (backToManager) openMapManager();
}

btnPickWaypoint.addEventListener('click', () => newWaypointInput.click());
btnPickImage.addEventListener('click',    () => newImageInput.click());

newWaypointInput.addEventListener('change', () => {
  const f = newWaypointInput.files[0];
  if (!f) return;
  pendingWpFile = f;
  wpFileName.textContent = f.name;
  wpFileName.classList.add('has-file');
  errWaypoint.textContent = '';
  // Auto-fill tên bản đồ nếu chưa nhập
  if (!newMapName.value.trim()) {
    newMapName.value = f.name.replace(/\.json$/i, '').replace(/_/g, ' ');
  }
});

newImageInput.addEventListener('change', () => {
  const f = newImageInput.files[0];
  if (!f) return;
  pendingImgFile = f;
  imgFileName.textContent = f.name;
  imgFileName.classList.add('has-file');
  errImage.textContent = '';
});

btnUploadClose.addEventListener('click',  () => closeUploadModal(true));
btnUploadCancel.addEventListener('click', () => closeUploadModal(true));
uploadMapModal.addEventListener('click', e => { if (e.target === uploadMapModal) closeUploadModal(true); });

btnUploadSubmit.addEventListener('click', async () => {
  // ── Validation ──
  let valid = true;

  const name = newMapName.value.trim();
  if (!name) {
    errMapName.textContent = 'Vui lòng nhập tên bản đồ.';
    valid = false;
  } else {
    errMapName.textContent = '';
  }

  if (!pendingWpFile) {
    errWaypoint.textContent = 'Vui lòng chọn file waypoint (.json).';
    valid = false;
  } else if (!pendingWpFile.name.toLowerCase().endsWith('.json')) {
    errWaypoint.textContent = 'File waypoint phải có định dạng .json.';
    valid = false;
  } else {
    errWaypoint.textContent = '';
  }

  if (!pendingImgFile) {
    errImage.textContent = 'Vui lòng chọn file ảnh bản đồ.';
    valid = false;
  } else if (!pendingImgFile.type.startsWith('image/')) {
    errImage.textContent = 'File ảnh không hợp lệ. Vui lòng chọn lại (png, jpg, …).';
    valid = false;
  } else {
    errImage.textContent = '';
  }

  if (!valid) return;

  // ── Upload ──
  btnUploadSubmit.disabled = true;
  setFeedback(uploadNewFeedback, 'Đang tải lên…', true);

  const fd = new FormData();
  fd.append('map_name', name);
  fd.append('waypoint', pendingWpFile);
  fd.append('image',    pendingImgFile);

  try {
    const res  = await fetch('/api/maps/upload', { method: 'POST', body: fd });
    const data = await res.json();

    if (data.ok) {
      addLog('info', `Đã upload bản đồ mới: ${data.name} (#${data.id})`);
      uploadMapModal.style.display = 'none';
      // Mở lại Map Manager để người dùng thấy bản đồ mới
      openMapManager();
    } else {
      setFeedback(uploadNewFeedback, data.error || 'Lỗi không xác định', false);
      btnUploadSubmit.disabled = false;
    }
  } catch (err) {
    setFeedback(uploadNewFeedback, 'Không thể kết nối server', false);
    btnUploadSubmit.disabled = false;
    addLog('error', `Upload error: ${err}`);
  } finally {
    if (!btnUploadSubmit.disabled) btnUploadSubmit.disabled = false;
  }
});

// ══════════════════════════════════════════════════════════════════════════════
//  KHỞI ĐỘNG: không auto-load bản đồ nào — người dùng phải tự chọn qua Select Map
// ══════════════════════════════════════════════════════════════════════════════

// ── ROS Status & Log Stream ───────────────────────────────────────────────────
// Bridge checker (bridge_check.py) chạy trên máy local, push heartbeat lên Flask.
// Frontend:
//   1. Poll /api/ros/status mỗi 6s để đồng bộ trạng thái (fallback).
//   2. Kết nối SSE /api/ros/log-stream để nhận log từ bridge_check.py NGAY LẬP TỨC.
//      Log từ ROS hiển thị màu khác (class 'ros') để phân biệt với log web.

// ── ROS Status Stream ─────────────────────────────────────────────────────────
let rosRunning      = false;
let rosStatusSource = null;

function setConnectedUI(running) {
  rosRunning        = running;
  connected         = running;   // alias dùng ở sendGoal/sendSpeed
  connDot.className = 'conn-dot ' + (running ? 'connected' : '');
  connStatus.textContent = running ? 'Connected' : 'Disconnected';
}

function startRosStatusStream() {
  if (rosStatusSource) return;
  rosStatusSource = new EventSource('/api/ros/status-stream');

  rosStatusSource.onmessage = (e) => {
    try {
      const { running } = JSON.parse(e.data);
      setConnectedUI(running);
    } catch { /* bỏ qua frame lỗi */ }
  };

  rosStatusSource.onerror = () => {
    // Browser tự reconnect — giữ nguyên UI
  };
}

// Nút "Check ROS" — mở lại stream nếu bị đóng
btnConnect.addEventListener('click', () => {
  if (!rosStatusSource || rosStatusSource.readyState === EventSource.CLOSED) {
    rosStatusSource = null;
    startRosStatusStream();
  }
  addLog('info', 'Đang chờ trạng thái từ bridge_check_node…');
});

// ── Gửi tọa độ ───────────────────────────────────────────────────────────────
// ── Navigate (gộp goal + speed) ──────────────────────────────────────────────
const navFeedback = document.getElementById('navFeedback');

function setNavFeedback(msg, type = '', timeout = 5000) {
  navFeedback.textContent = msg;
  navFeedback.className   = `feedback${type ? ' ' + type : ''}`;
  if (timeout) setTimeout(() => {
    if (navFeedback.textContent === msg) {
      navFeedback.textContent = 'Chọn một vị trí trên bản đồ hoặc nhập toạ độ';
      navFeedback.className   = 'feedback';
    }
  }, timeout);
}

async function navigate() {
  const x   = parseFloat(goalX.value);
  const y   = parseFloat(goalY.value);
  const kmh = parseFloat(speedKmh.value);

  if (isNaN(x) || isNaN(y)) {
    setNavFeedback('X và Y phải là số hợp lệ.', 'err', 4000);
    return;
  }
  if (isNaN(kmh) || kmh < 0) {
    setNavFeedback('Tốc độ phải là số >= 0.', 'err', 4000);
    return;
  }

  btnSendGoal.disabled = true;
  try {
    const res  = await fetch('/api/navigate', {
      method:  'POST',
      headers: { 'Content-Type': 'application/json' },
      body:    JSON.stringify({ x, y, speed_kmh: kmh }),
    });
    const data = await res.json();
    if (data.ok) {
      let detail = `(${x.toFixed(2)}, ${y.toFixed(2)}) — ${kmh.toFixed(1)} km/h`;
      if (data.snapped) detail += ` → snap ${data.dist} m`;
      setNavFeedback(`✓ Go: ${detail}`, 'ok');
      addLog('info', `Navigate → x=${x.toFixed(3)}, y=${y.toFixed(3)}, ${kmh.toFixed(1)} km/h`);
    } else {
      setNavFeedback(data.error || 'Server error', 'err');
      addLog('error', `Navigate failed: ${data.error}`);
    }
  } catch {
    setNavFeedback('Không thể kết nối server', 'err');
    addLog('error', 'Navigate: không thể kết nối server');
  } finally {
    btnSendGoal.disabled = false;
  }
}

async function stopVehicle() {
  btnStop.disabled = true;
  try {
    const res  = await fetch('/api/navigate', {
      method:  'POST',
      headers: { 'Content-Type': 'application/json' },
      body:    JSON.stringify({ stop: true }),
    });
    const data = await res.json();
    setNavFeedback('⚠ STOP — xe đã dừng', 'err');
    addLog('warn', 'STOP — speed=0, goal cancelled');
  } catch {
    setNavFeedback('Không thể kết nối server', 'err');
  } finally {
    btnStop.disabled = false;
  }
}

btnSendGoal.addEventListener('click', navigate);
[goalX, goalY, speedKmh].forEach(el =>
  el.addEventListener('keydown', e => { if (e.key === 'Enter') navigate(); })
);
btnStop.addEventListener('click', stopVehicle);

// ══════════════════════════════════════════════════════════════════════════════
//  SPAWN CAR
// ══════════════════════════════════════════════════════════════════════════════
// Luồng: bridge kết nối → backend tự lấy spawn point từ CARLA → đẩy xuống qua SSE
// (spawnPoints, ban đầu ẩn). Bấm "Spawn Car" → hiện các điểm xanh dương trên map.
// Chọn 1 điểm → ẩn lại các điểm → gửi (x, y, z, yaw) xuống backend để kill xe cũ
// (nếu có) và spawn xe mới tại vị trí đó.

function setSpawnFeedback(msg, type = '', timeout = 0) {
  spawnFeedback.textContent = msg;
  spawnFeedback.className   = `feedback${type ? ' ' + type : ''}`;
  if (timeout) setTimeout(() => {
    if (spawnFeedback.textContent === msg) {
      spawnFeedback.textContent = '';
      spawnFeedback.className   = 'feedback';
    }
  }, timeout);
}

function setSpawnButtonActive(active) {
  btnSpawnCar.classList.toggle('active', active);
  btnSpawnCar.textContent = active ? 'Chọn điểm trên bản đồ…' : 'Spawn Car';
}

btnSpawnCar.addEventListener('click', () => {
  if (!spawnPoints.length) {
    setSpawnFeedback('Chưa có spawn point (chờ bridge kết nối CARLA)…', 'err', 4000);
    addLog('warn', 'Spawn Car: chưa có spawn point nào từ CARLA.');
    return;
  }
  spawnPointsVisible = !spawnPointsVisible;
  setSpawnButtonActive(spawnPointsVisible);
  setSpawnFeedback(spawnPointsVisible ? `Chọn 1 trong ${spawnPoints.length} điểm xanh dương trên bản đồ…` : '');
  drawOverlay();
});

async function selectSpawnPoint(sp) {
  // Chọn xong → ẩn ngay các điểm, thoát chế độ chọn
  spawnPointsVisible = false;
  setSpawnButtonActive(false);
  drawOverlay();

  btnSpawnCar.disabled = true;
  setSpawnFeedback(`Đang spawn xe tại (${sp.x.toFixed(2)}, ${sp.y.toFixed(2)})…`);

  try {
    const res  = await fetch('/api/spawn/vehicle', {
      method:  'POST',
      headers: { 'Content-Type': 'application/json' },
      body:    JSON.stringify({ x: sp.x, y: sp.y, z: sp.z, yaw: sp.yaw }),
    });
    const data = await res.json();
    if (data.ok) {
      setSpawnFeedback(`✓ Đã gửi lệnh spawn tại (${sp.x.toFixed(2)}, ${sp.y.toFixed(2)})`, 'ok', 6000);
      addLog('info', `Spawn Car → x=${sp.x.toFixed(3)}, y=${sp.y.toFixed(3)}, z=${sp.z.toFixed(3)}, yaw=${sp.yaw.toFixed(3)}`);
    } else {
      setSpawnFeedback(data.error || 'Spawn thất bại', 'err');
      addLog('error', `Spawn Car failed: ${data.error}`);
    }
  } catch (err) {
    setSpawnFeedback('Không thể kết nối server', 'err');
    addLog('error', 'Spawn Car: không thể kết nối server');
  } finally {
    btnSpawnCar.disabled = false;
  }
}

// ── SSE: nhận danh sách spawn point mỗi khi backend fetch lại từ CARLA ───────
let spawnPointsSource = null;

function startSpawnPointsStream() {
  if (spawnPointsSource) return;
  spawnPointsSource = new EventSource('/api/spawn/points-stream');

  spawnPointsSource.onmessage = (e) => {
    try {
      const { points } = JSON.parse(e.data);
      const prevCount = spawnPoints.length;
      spawnPoints = Array.isArray(points) ? points : [];
      if (spawnPoints.length && spawnPoints.length !== prevCount) {
        addLog('info', `Đã nhận ${spawnPoints.length} spawn point từ CARLA.`);
        setSpawnFeedback(`Sẵn sàng — ${spawnPoints.length} spawn point.`, 'ok', 5000);
      }
      if (spawnPointsVisible) drawOverlay();
    } catch { /* bỏ qua frame lỗi */ }
  };

  spawnPointsSource.onerror = () => {
    // Browser tự reconnect — giữ nguyên UI
  };
}

// ── Lưu Địa Điểm ─────────────────────────────────────────────────────────────
const btnSaveLocation   = document.getElementById('btnSaveLocation');
const savedLocationList = document.getElementById('savedLocationList');
const saveLocationModal = document.getElementById('saveLocationModal');
const locationNameInput = document.getElementById('locationNameInput');
const btnLocationCancel = document.getElementById('btnLocationCancel');
const btnLocationSave   = document.getElementById('btnLocationSave');

let savedLocations = [];

function renderSavedLocations() {
  savedLocationList.innerHTML = '';
  savedLocations.forEach((loc, index) => {
    const li = document.createElement('li');
    li.className = 'saved-loc-item';
    li.innerHTML = `
      <div class="saved-loc-name">${index + 1}. ${escHtml(loc.name)}</div>
      <div class="saved-loc-coord">(${loc.wp.x.toFixed(3)}, ${loc.wp.y.toFixed(3)})</div>`;
    li.addEventListener('click', () => {
      selectedWaypoint = loc.wp;
      updateSelectedPin();
      goalX.value = loc.wp.x.toFixed(3);
      goalY.value = loc.wp.y.toFixed(3);
      document.getElementById('navFeedback').textContent = `Đã chọn lại địa điểm: ${loc.name}`;
      document.getElementById('navFeedback').className   = 'feedback ok';
      addLog('info', `Đã chọn lại điểm lưu: ${loc.name}`);
    });
    savedLocationList.appendChild(li);
  });
}

function closeLocationModal() {
  saveLocationModal.style.display = 'none';
  locationNameInput.value = '';
  locationNameError.style.display = 'none';
}

btnSaveLocation.addEventListener('click', () => {
  if (!selectedWaypoint) {
    alert('Vui lòng click chọn một waypoint trên bản đồ trước khi lưu!');
    return;
  }
  saveLocationModal.style.display = 'flex';
  locationNameInput.focus();
});

btnLocationCancel.addEventListener('click', closeLocationModal);
saveLocationModal.addEventListener('click', e => { if (e.target === saveLocationModal) closeLocationModal(); });

function executeSaveLocation() {
  const trimmedName = (locationNameInput.value || '').trim();
  locationNameError.style.display = 'none';

  if (trimmedName === '') {
    locationNameError.textContent = 'Tên địa điểm không được để trống.';
    locationNameError.style.display = 'block';
    return;
  }

  const isDuplicate = savedLocations.some(loc => loc.name.toLowerCase() === trimmedName.toLowerCase());
  if (isDuplicate) {
    locationNameError.textContent = 'Tên địa điểm đã tồn tại. Vui lòng chọn tên khác.';
    locationNameError.style.display = 'block';
    return;
  }

  savedLocations.push({ name: trimmedName, wp: selectedWaypoint });
  renderSavedLocations();
  closeLocationModal();
  addLog('info', `Đã lưu địa điểm mới: ${trimmedName}`);
}

btnLocationSave.addEventListener('click', executeSaveLocation);
locationNameInput.addEventListener('keydown', e => { if (e.key === 'Enter') executeSaveLocation(); });
locationNameInput.addEventListener('input', () => { locationNameError.style.display = 'none'; });

// ── Khởi động: không auto-load bản đồ — chờ người dùng chọn qua Select Map ──
addLog('info', 'Vui lòng chọn bản đồ qua nút "Select Map" để bắt đầu.');
startRosStatusStream();     // lắng nghe trạng thái ROS bridge qua SSE
startOdomStream();          // kết nối WebSocket nhận odom realtime
startSpawnPointsStream();   // lắng nghe danh sách spawn point từ CARLA qua SSE
startSpeedometerStream();   // kết nối WebSocket nhận tốc độ thực (Actual km/h) realtime