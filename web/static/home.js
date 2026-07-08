// ═══════════════════════════════════════════════════════════════════════════
// home.js — Trang chính (landing) — điều hướng 4 bước quy trình
// ═══════════════════════════════════════════════════════════════════════════

/**
 * Mở modal placeholder theo id.
 * Các bước 1, 2, 4 hiện chưa có tính năng thật — chỉ hiển thị cửa sổ tạm
 * với nút X để đóng. Sẽ bổ sung logic thật ở các bước phát triển sau.
 */
function openModal(modalId) {
  const modal = document.getElementById(modalId);
  if (modal) modal.style.display = 'flex';
}

function closeModal(modalId) {
  const modal = document.getElementById(modalId);
  if (modal) modal.style.display = 'none';
}

// ── Bước 1: Khởi động môi trường 3D CARLA ────────────────────────────────────
document.getElementById('btnStep1').addEventListener('click', () => openModal('modalStep1'));

(function initCarlaControl() {
  let state = 'idle';        // 'idle' | 'starting' | 'running' | 'stopping'
  let renderOn = false;      // mặc định: Tắt render

  const btnConfirm  = document.getElementById('btnStep1Confirm');
  const btnBack     = document.getElementById('btnStep1Back');
  const logBox      = document.getElementById('carlaLog');
  const toggleBtns  = document.querySelectorAll('#renderToggle .toggle-btn');

  function log(text) {
    const line = document.createElement('div');
    line.className = 'carla-log-line';
    line.textContent = text;
    logBox.appendChild(line);
    logBox.scrollTop = logBox.scrollHeight;
  }

  function setToggleDisabled(disabled) {
    toggleBtns.forEach(b => { b.disabled = disabled; });
  }

  toggleBtns.forEach(btn => {
    btn.addEventListener('click', () => {
      if (btn.disabled) return;
      toggleBtns.forEach(b => b.classList.remove('toggle-btn--active'));
      btn.classList.add('toggle-btn--active');
      renderOn = btn.dataset.value === 'on';
    });
  });

  function startFlow() {
    state = 'starting';
    btnConfirm.disabled = true;
    setToggleDisabled(true);
    log('Đang khởi động môi trường mô phỏng lái 3D CARLA . . .');

    fetch('/api/carla/start', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ render: renderOn })
    })
      .then(r => r.json())
      .then(data => {
        if (!data.ok) {
          log('Lỗi: ' + (data.error || 'Không thể khởi động'));
          state = 'idle';
          btnConfirm.disabled = false;
          setToggleDisabled(false);
          return;
        }
        setTimeout(() => {
          log('Khởi động thành công');
          state = 'running';
          btnConfirm.textContent = 'Tạm dừng';
          btnConfirm.classList.add('btn-confirm--danger');
          btnConfirm.disabled = false;
        }, 5000);
      })
      .catch(err => {
        log('Lỗi kết nối: ' + err.message);
        state = 'idle';
        btnConfirm.disabled = false;
        setToggleDisabled(false);
      });
  }

  function stopFlow() {
    state = 'stopping';
    btnConfirm.disabled = true;
    log('Đang tắt hệ thống mô phỏng lái 3D');

    // Bổ sung: tạm dừng CARLA cũng dọn dẹp luôn Carla ROS Bridge (bước 2)
    // nếu đang chạy, vì bridge phụ thuộc vào CARLA. Bỏ qua lỗi nếu bridge
    // chưa từng được khởi động.
    fetch('/api/rosbridge/stop', { method: 'POST' }).catch(() => {});

    fetch('/api/carla/stop', { method: 'POST' })
      .then(r => r.json())
      .then(() => {
        setTimeout(() => {
          log('Đã tắt');
          state = 'idle';
          btnConfirm.textContent = 'Khởi động';
          btnConfirm.classList.remove('btn-confirm--danger');
          btnConfirm.disabled = false;
          setToggleDisabled(false);
        }, 15000);
      })
      .catch(err => {
        log('Lỗi khi tắt: ' + err.message);
        btnConfirm.disabled = false;
      });
  }

  btnConfirm.addEventListener('click', () => {
    if (state === 'idle') startFlow();
    else if (state === 'running') stopFlow();
  });

  // "Quay lại" — đóng cửa sổ để trở về màn hình chính (không ảnh hưởng tiến
  // trình CARLA đang chạy nền, vì nó độc lập với giao diện).
  btnBack.addEventListener('click', () => closeModal('modalStep1'));
})();

// ── Bước 2: Khởi động cầu nối CARLA ROS BRIDGE ──────────────────────────────
document.getElementById('btnStep2').addEventListener('click', () => openModal('modalStep2'));

(function initRosBridgeControl() {
  let busy = false;

  const btnConfirm = document.getElementById('btnStep2Confirm');
  const btnBack     = document.getElementById('btnStep2Back');
  const logBox      = document.getElementById('rosBridgeLog');
  const mapSelect   = document.getElementById('rosBridgeMapSelect');
  const modeBtns    = document.querySelectorAll('#syncToggle .toggle-btn');

  function log(text) {
    const line = document.createElement('div');
    line.className = 'carla-log-line';
    line.textContent = text;
    logBox.appendChild(line);
    logBox.scrollTop = logBox.scrollHeight;
  }

  // Chỉ "Đồng bộ" được phép chọn — nút "Không đồng bộ" luôn disabled (chưa hỗ trợ).
  modeBtns.forEach(btn => {
    btn.addEventListener('click', () => {
      if (btn.disabled) return;
      modeBtns.forEach(b => b.classList.remove('toggle-btn--active'));
      btn.classList.add('toggle-btn--active');
    });
  });

  function startFlow() {
    if (busy) return;
    busy = true;

    const town = mapSelect.value;
    // STT bản đồ tương ứng lưu trong maps/ (vd: "1.Town01" → mapId = "1"),
    // lấy từ data-map-id gắn sẵn trên từng <option>.
    const selectedOption = mapSelect.options[mapSelect.selectedIndex];
    const mapId = selectedOption ? selectedOption.dataset.mapId : null;
    const mapLabel = selectedOption ? selectedOption.textContent : town;

    btnConfirm.disabled = true;
    mapSelect.disabled  = true;
    log('Đang khởi động Carla ROS Bridge . . .');

    fetch('/api/rosbridge/start', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ town: town, synchronous: true })
    })
      .then(r => r.json())
      .then(data => {
        if (!data.ok) {
          log('Lỗi: ' + (data.error || 'Không thể khởi động Carla ROS Bridge'));
          busy = false;
          btnConfirm.disabled = false;
          mapSelect.disabled  = false;
          return;
        }
        setTimeout(() => {
          log(`Đang load bản đồ ${mapLabel} . . .`);
          setTimeout(() => {
            log('Đã khởi động thành công');
            // Bridge đã chạy với bản đồ đã chọn — không cho đổi bản đồ giữa
            // chừng, nên ẩn luôn nút "Khởi động" thay vì bật lại.
            btnConfirm.style.display = 'none';
            log('Nếu muốn đổi bản đồ, hãy quay trở về làm lại từ bước 1.');

            // Ghi nhớ bản đồ đã chọn (khớp theo STT lưu trong maps/) để trang
            // điều khiển (bước 3) tự động load — thay cho "Select Map" cũ.
            linkActiveMap(mapId, mapLabel);
          }, 5000);
        }, 5000);
      })
      .catch(err => {
        log('Lỗi kết nối: ' + err.message);
        busy = false;
        btnConfirm.disabled = false;
        mapSelect.disabled  = false;
      });
  }

  /**
   * Lưu STT bản đồ đang active vào localStorage (đọc lại ở trang /control),
   * đồng thời báo ngay cho server để nó lấy spawn point mới từ CARLA.
   * Nếu bản đồ #mapId chưa từng được upload (chưa có waypoint/ảnh trong
   * maps/), server sẽ trả lỗi 404 — chỉ log cảnh báo, không chặn luồng.
   */
  function linkActiveMap(mapId, mapLabel) {
    if (!mapId) return;
    localStorage.setItem('activeMapId', mapId);
    fetch(`/api/maps/${mapId}/load`)
      .then(r => r.json())
      .then(d => {
        if (d.ok) {
          log(`Đã liên kết bản đồ: ${d.name} (#${d.id})`);
        } else {
          log(`Cảnh báo: chưa có dữ liệu bản đồ cho "${mapLabel}" (#${mapId}) — hãy tải lên bản đồ này trước.`);
        }
      })
      .catch(() => log('Không thể liên kết bản đồ với server.'));
  }

  btnConfirm.addEventListener('click', startFlow);

  // "Quay lại" — đóng cửa sổ, không ảnh hưởng bridge đang chạy nền.
  btnBack.addEventListener('click', () => closeModal('modalStep2'));
})();

// ── Bước 3: Điều khiển xe mô phỏng lái → sang trang điều khiển hiện tại ─────
document.getElementById('btnStep3').addEventListener('click', () => {
  window.location.href = '/control';
});

// ── Bước 4: Tạo đồ thị và phân tích dữ liệu ─────────────────────────────────
(function initPlotStep() {
  const btnStep4      = document.getElementById('btnStep4');
  const btnBack        = document.getElementById('btnStep4Back');
  const btnPlot        = document.getElementById('btnStep4Plot');
  const fileList        = document.getElementById('plotFileList');
  const fileListEmpty    = document.getElementById('plotFileListEmpty');

  let selectedFilename = null;

  function selectFile(filename, itemEl) {
    selectedFilename = filename;
    fileList.querySelectorAll('.plot-file-item').forEach(el => el.classList.remove('plot-file-item--selected'));
    itemEl.classList.add('plot-file-item--selected');
    btnPlot.disabled = false;
  }

  function renderFiles(files) {
    fileList.innerHTML = '';
    selectedFilename = null;
    btnPlot.disabled = true;

    if (!files.length) {
      fileListEmpty.textContent = 'Chưa có file dữ liệu nào trong recorded_data/. Hãy vào trang điều khiển (Bước 3) và bấm "Xuất dữ liệu" trước.';
      fileListEmpty.style.display = 'block';
      fileList.style.display = 'none';
      return;
    }

    fileListEmpty.style.display = 'none';
    fileList.style.display = 'flex';

    files.forEach(f => {
      const li = document.createElement('li');
      li.className = 'plot-file-item';
      li.innerHTML = `
        <span class="plot-file-radio"></span>
        <span class="plot-file-name">${f.display_name}</span>
        <span class="plot-file-size">${f.size_label}</span>
      `;
      li.addEventListener('click', () => selectFile(f.filename, li));
      fileList.appendChild(li);
    });
  }

  function loadFiles() {
    fileListEmpty.textContent = 'Đang tải danh sách file…';
    fileListEmpty.style.display = 'block';
    fileList.style.display = 'none';
    fileList.innerHTML = '';

    fetch('/api/datalogger/files')
      .then(r => r.json())
      .then(data => {
        if (data.ok) {
          renderFiles(data.files);
        } else {
          fileListEmpty.textContent = 'Lỗi tải danh sách: ' + (data.error || '');
        }
      })
      .catch(() => {
        fileListEmpty.textContent = 'Không thể kết nối server.';
      });
  }

  btnStep4.addEventListener('click', () => {
    openModal('modalStep4');
    loadFiles();
  });

  btnPlot.addEventListener('click', () => {
    if (!selectedFilename) return;
    btnPlot.disabled = true;
    btnPlot.textContent = 'Đang mở…';

    fetch('/api/datalogger/plot', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ filename: selectedFilename }),
    })
      .then(r => r.json())
      .then(data => {
        if (data.ok) {
          btnPlot.textContent = 'Đã mở — xem cửa sổ trên máy chủ';
          setTimeout(() => {
            btnPlot.textContent = 'Vẽ đồ thị';
            btnPlot.disabled = false;
          }, 3000);
        } else {
          alert('Lỗi: ' + (data.error || 'Không thể mở đồ thị'));
          btnPlot.textContent = 'Vẽ đồ thị';
          btnPlot.disabled = false;
        }
      })
      .catch(() => {
        alert('Không thể kết nối server.');
        btnPlot.textContent = 'Vẽ đồ thị';
        btnPlot.disabled = false;
      });
  });

  btnBack.addEventListener('click', () => closeModal('modalStep4'));
})();

// ── Đóng modal: nút X, click ra ngoài overlay, hoặc phím Esc ────────────────
document.querySelectorAll('.btn-modal-close[data-close]').forEach(btn => {
  btn.addEventListener('click', () => closeModal(btn.dataset.close));
});

document.querySelectorAll('.modal-overlay').forEach(overlay => {
  overlay.addEventListener('click', (e) => {
    if (e.target === overlay) overlay.style.display = 'none';
  });
});

document.addEventListener('keydown', (e) => {
  if (e.key !== 'Escape') return;
  document.querySelectorAll('.modal-overlay').forEach(overlay => {
    overlay.style.display = 'none';
  });
});