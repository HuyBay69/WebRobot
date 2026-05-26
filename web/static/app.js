'use strict';

// ── DOM ───────────────────────────────────────────────────────────────────────
const wsHostInput    = document.getElementById('wsHost');
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
const goalFeedback   = document.getElementById('goalFeedback');

const speedKmh       = document.getElementById('speedKmh');
const speedMpsEl     = document.getElementById('speedMps');
const btnSendSpeed   = document.getElementById('btnSendSpeed');
const btnStop        = document.getElementById('btnStop');
const speedFeedback  = document.getElementById('speedFeedback');

const logBody        = document.getElementById('logBody');
const locationNameError = document.getElementById('locationNameError');

let mapCanvasWrap = document.getElementById('mapCanvasWrap');
let overlayCanvas = document.getElementById('overlayCanvas');
let mapHud        = document.getElementById('mapHud');
let mapHint       = document.getElementById('mapHint');
let mapScaleLabel = document.getElementById('mapScaleLabel');

// ── State ─────────────────────────────────────────────────────────────────────
let ros       = null;
let goalPub   = null;
let speedPub  = null;
let connected = false;

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

function drawOverlay() {
  const ctx  = overlayCanvas.getContext('2d');
  const rect = mapCanvasWrap.getBoundingClientRect();
  ctx.clearRect(0, 0, rect.width, rect.height);
  if (!waypoints.length) return;
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
  drawOverlay();
  updateSelectedPin();
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
  drawOverlay();
  updateSelectedPin();
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

function handleWaypointClick(event) {
  const rect   = mapCanvasWrap.getBoundingClientRect();
  const clickX = event.clientX - rect.left;
  const clickY = event.clientY - rect.top;
  const hit    = findWaypointAt(clickX, clickY);
  if (!hit) return;
  selectedWaypoint = hit;
  updateSelectedPin();
  if (typeof hit.x === 'number' && typeof hit.y === 'number') {
    goalX.value = hit.x.toFixed(3);
    goalY.value = hit.y.toFixed(3);
    goalFeedback.textContent = 'Chọn vị trí thành công. Nhấn Send Goal để gửi.';
    goalFeedback.className   = 'feedback ok';
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
//  AUTO-LOAD KHI KHỞI ĐỘNG (Load bản đồ #1)
// ══════════════════════════════════════════════════════════════════════════════
async function autoLoadFirstMap() {
  try {
    const res  = await fetch('/api/maps');
    const data = await res.json();
    if (!data.ok || !data.maps.length) {
      addLog('info', 'Chưa có bản đồ nào. Hãy upload bản đồ qua Select Map.');
      return;
    }
    // Bản đồ có STT = 1 (đã được sort)
    const first = data.maps.find(m => m.id === 1) || data.maps[0];
    addLog('info', `Auto-load bản đồ #${first.id}: ${first.name}`);
    await loadMapById(first);
  } catch (err) {
    addLog('warn', `Auto-load thất bại: ${err}`);
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

// ── Gửi tọa độ ───────────────────────────────────────────────────────────────
async function sendGoal() {
  const x = parseFloat(goalX.value);
  const y = parseFloat(goalY.value);

  if (isNaN(x) || isNaN(y)) {
    goalFeedback.textContent = 'X và Y phải là số hợp lệ.';
    goalFeedback.className   = 'feedback err';
    return;
  }

  if (connected && goalPub) {
    goalPub.publish(new ROSLIB.Message({
      header: { frame_id: 'map' },
      pose: { position: { x, y, z: 0.0 }, orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 } },
    }));
    goalFeedback.textContent = `Đã gửi tọa độ (${x.toFixed(3)},${y.toFixed(3)}) thành công.`;
    goalFeedback.className   = 'feedback ok';
    setTimeout(() => {
      if (goalFeedback.className === 'feedback ok') {
        goalFeedback.textContent = 'Chọn một vị trí trên bản đồ';
        goalFeedback.className   = 'feedback';
      }
    }, 5000);
    addLog('info', `Goal [ROS] → x=${x.toFixed(3)}, y=${y.toFixed(3)}`);
    return;
  }

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
      goalFeedback.textContent = `Đã gửi tọa độ (${x.toFixed(3)},${y.toFixed(3)}) thành công.`;
      goalFeedback.className   = 'feedback ok';
      setTimeout(() => {
        if (goalFeedback.className === 'feedback ok') {
          goalFeedback.textContent = 'Chọn một vị trí trên bản đồ';
          goalFeedback.className   = 'feedback';
        }
      }, 5000);
      addLog('info', `Goal [API${via}] → ${detail}`);
    } else {
      goalFeedback.textContent = data.error || 'Server error';
      goalFeedback.className   = 'feedback err';
      addLog('error', `Goal failed: ${data.error}`);
    }
  } catch {
    goalFeedback.textContent = 'Không thể kết nối server';
    goalFeedback.className   = 'feedback err';
    addLog('error', 'Goal: không thể kết nối server');
  } finally {
    btnSendGoal.disabled = false;
  }
}

btnSendGoal.addEventListener('click', sendGoal);
[goalX, goalY].forEach(el => el.addEventListener('keydown', e => { if (e.key === 'Enter') sendGoal(); }));

// ── Gửi tốc độ ───────────────────────────────────────────────────────────────
async function sendSpeed(kmh) {
  if (connected && speedPub) {
    const mps = kmh / 3.6;
    speedPub.publish(new ROSLIB.Message({ data: mps }));
    speedFeedback.textContent = `Đã gửi tốc độ ${kmh.toFixed(2)} km/h thành công.`;
    speedFeedback.className   = 'feedback ok';
    setTimeout(() => {
      if (speedFeedback.className === 'feedback ok') {
        speedFeedback.textContent = 'Nhập tốc độ cần gửi';
        speedFeedback.className   = 'feedback';
      }
    }, 5000);
    addLog('info', `Speed [ROS] → ${kmh.toFixed(2)} km/h = ${mps.toFixed(3)} m/s`);
    return;
  }

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
      speedFeedback.textContent = `Đã gửi tốc độ ${kmh.toFixed(2)} km/h thành công.`;
      speedFeedback.className   = 'feedback ok';
      setTimeout(() => {
        if (speedFeedback.className === 'feedback ok') {
          speedFeedback.textContent = 'Nhập tốc độ cần gửi';
          speedFeedback.className   = 'feedback';
        }
      }, 5000);
      addLog('info', `Speed [API${via}] → ${kmh.toFixed(2)} km/h = ${data.speed_mps} m/s`);
    } else {
      speedFeedback.textContent = data.error || 'Server error';
      speedFeedback.className   = 'feedback err';
      addLog('error', `Speed failed: ${data.error}`);
    }
  } catch {
    speedFeedback.textContent = 'Không thể kết nối server';
    speedFeedback.className   = 'feedback err';
    addLog('error', 'Speed: không thể kết nối server');
  } finally {
    btnSendSpeed.disabled = false;
    btnStop.disabled      = false;
  }
}

btnSendSpeed.addEventListener('click', () => {
  const v = parseFloat(speedKmh.value);
  if (isNaN(v) || v < 0) {
    speedFeedback.textContent = 'Speed phải là số >= 0.';
    speedFeedback.className   = 'feedback err';
    return;
  }
  sendSpeed(v);
});

speedKmh.addEventListener('keydown', e => { if (e.key === 'Enter') btnSendSpeed.click(); });

btnStop.addEventListener('click', async () => {
  speedKmh.value         = '0';
  speedMpsEl.textContent = '0.000 m/s';
  await sendSpeed(0);
  if (connected) {
    speedFeedback.textContent = '⚠ Emergency stop sent!';
    speedFeedback.className   = 'feedback err';
    setTimeout(() => {
      if (speedFeedback.textContent === '⚠ Emergency stop sent!') {
        speedFeedback.textContent = 'Nhập tốc độ cần gửi';
        speedFeedback.className   = 'feedback';
      }
    }, 5000);
  }
  addLog('warn', 'EMERGENCY STOP — speed = 0');
});

// ── Tốc độ — sync m/s badge ──────────────────────────────────────────────────
speedKmh.addEventListener('input', () => {
  const v = parseFloat(speedKmh.value);
  speedMpsEl.textContent = isNaN(v) ? '— m/s' : `${(v / 3.6).toFixed(3)} m/s`;
});

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
      goalFeedback.textContent = `Đã chọn lại địa điểm: ${loc.name}`;
      goalFeedback.className   = 'feedback ok';
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

// ── Khởi động: auto-load bản đồ #1 ──────────────────────────────────────────
autoLoadFirstMap();