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

/**
 * Đánh dấu bước đang thực hiện (1-4): làm sáng nút bước + chấm tương ứng trong
 * pipeline bên phải. Mỗi lần bấm 1 nút, chỉ bước đó sáng (bỏ sáng các bước
 * khác) — thay cho việc cố định sáng bước 3 như trước. Gọi ngay khi người dùng
 * bấm vào 1 trong 4 nút bước.
 */
function setActiveStep(stepNumber) {
  // Nút bước
  for (let i = 1; i <= 5; i++) {
    const btn = document.getElementById('btnStep' + i);
    if (btn) btn.classList.toggle('step-btn--current', i === stepNumber);
  }
  // Chấm trong pipeline (thứ tự DOM của .pl-node khớp 1-5)
  const nodes = document.querySelectorAll('.pipeline .pl-node');
  nodes.forEach((node, idx) => {
    node.classList.toggle('pl-node--current', idx === stepNumber - 1);
  });
}

// ── Bước 1: Khởi động môi trường 3D CARLA ────────────────────────────────────
document.getElementById('btnStep1').addEventListener('click', () => { setActiveStep(1); openModal('modalStep1'); });

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
document.getElementById('btnStep2').addEventListener('click', () => { setActiveStep(2); openModal('modalStep2'); });

(function initRosBridgeControl() {
  let busy = false;

  const btnConfirm = document.getElementById('btnStep2Confirm');
  const btnBack     = document.getElementById('btnStep2Back');
  const logBox      = document.getElementById('rosBridgeLog');
  const mapSelect   = document.getElementById('rosBridgeMapSelect');
  const modeBtns    = document.querySelectorAll('#syncToggle .toggle-btn');
  const sensorSelect      = document.getElementById('sensorSelect');
  const sensorDescription = document.getElementById('sensorDescription');
  const sensorNote        = document.getElementById('sensorNote');

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

  // Mô tả cảm biến theo từng lựa chọn — cập nhật ngay khi đổi dropdown, chưa
  // gắn lệnh chạy thật khác nhau (sẽ bổ sung sau), hiện chỉ để thông tin.
  const SENSOR_DESCRIPTIONS = {
    default: 'Các cảm biến cơ bản: IMU, GNSS, Speedometer, Odometry, RGB View, Radar',
    aeb:     'Tích hợp thêm Depth và Segmentation sensor',
  };

  function updateSensorDescription() {
    sensorDescription.textContent = SENSOR_DESCRIPTIONS[sensorSelect.value] || '';
  }

  sensorSelect.addEventListener('change', updateSensorDescription);
  updateSensorDescription(); // hiện đúng mô tả ngay lúc mở modal

  // "Auto Emergency Brake" chỉ khả dụng với Town01 — các bản đồ khác chưa
  // được kiểm chứng nên tạm khoá lại (vô hiệu hoá <option>), luôn tự về
  // "Default" nếu đổi bản đồ khác trong lúc đang chọn AEB.
  const aebOption = sensorSelect.querySelector('option[value="aeb"]');

  function updateSensorAvailability() {
    const isTown01 = mapSelect.value === 'Town01_Opt';
    if (!isTown01 && sensorSelect.value === 'aeb') {
      sensorSelect.value = 'default';
      updateSensorDescription();
    }
    aebOption.disabled = !isTown01;
    sensorNote.style.display = isTown01 ? 'none' : 'block';
  }

  mapSelect.addEventListener('change', updateSensorAvailability);
  updateSensorAvailability(); // chạy ngay lúc mở modal, khớp với bản đồ đang chọn sẵn

  function startFlow() {
    if (busy) return;
    busy = true;

    const town = mapSelect.value;
    // STT bản đồ tương ứng lưu trong maps/ (vd: "1.Town01" → mapId = "1"),
    // lấy từ data-map-id gắn sẵn trên từng <option>.
    const selectedOption = mapSelect.options[mapSelect.selectedIndex];
    const mapId = selectedOption ? selectedOption.dataset.mapId : null;
    const mapLabel = selectedOption ? selectedOption.textContent : town;

    const sensorMode  = sensorSelect.value;
    const sensorLabel = sensorSelect.options[sensorSelect.selectedIndex].textContent;
    // Ghi nhớ lựa chọn cảm biến ngay lúc bấm Khởi động — trang điều khiển
    // (bước 3) sẽ đọc lại giá trị này để hiển thị bên cạnh nút Spawn Car.
    // Lưu ý: 2 lựa chọn hiện CHẠY CHUNG 1 lệnh khởi động (chưa phân biệt) —
    // sẽ bổ sung lệnh riêng cho từng chế độ sau.
    localStorage.setItem('sensorMode', sensorMode);

    btnConfirm.disabled = true;
    mapSelect.disabled  = true;
    log('Đang khởi động Carla ROS Bridge . . .');
    log(`Chế độ cảm biến: ${sensorLabel}`);

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
  setActiveStep(3);
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
    setActiveStep(4);
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

// ── Bước 5: Điều khiển xe ESP32 thực nghiệm ─────────────────────────────────
// Chiều 1 (Web → ESP32): gửi 1 lần toàn bộ chuỗi lệnh hình học qua HTTP POST.
// Chiều 2 (ESP32 → Web): ESP32 tự báo tiến độ về server (POST /api/esp32/progress),
// server phát lại qua SSE để trang này cập nhật real-time không cần polling.
(function initEsp32Control() {
  const btnStep5   = document.getElementById('btnStep5');
  const btnBack    = document.getElementById('btnStep5Back');
  const btnSend    = document.getElementById('btnStep5Send');
  const ipInput    = document.getElementById('esp32IpInput');
  const timeScaleInput = document.getElementById('esp32TimeScaleInput');
  const csvInput   = document.getElementById('esp32CsvFile');
  const csvFileName = document.getElementById('esp32CsvFileName');
  const btnAnalyze  = document.getElementById('btnEsp32Analyze');
  const convertStatus = document.getElementById('esp32ConvertStatus');
  const canvas     = document.getElementById('esp32PreviewCanvas');
  const listBox    = document.getElementById('esp32ProgressList');
  const listEmpty  = document.getElementById('esp32ProgressEmpty');
  const banner     = document.getElementById('esp32CompletedBanner');

  // Panel kết nối (cột trái)
  const connDot        = document.getElementById('esp32ConnDot');
  const connStatusText = document.getElementById('esp32ConnStatusText');
  const connIp          = document.getElementById('esp32ConnIp');
  const connRssi         = document.getElementById('esp32ConnRssi');
  const statsDivider     = document.getElementById('esp32StatsDivider');
  const packetField      = document.getElementById('esp32PacketField');
  const connPackets       = document.getElementById('esp32ConnPackets');
  const lossField          = document.getElementById('esp32LossField');
  const connLoss            = document.getElementById('esp32ConnLoss');
  const latencyField         = document.getElementById('esp32LatencyField');
  const connLatency           = document.getElementById('esp32ConnLatency');

  let lastHeartbeatAt = 0;  // Date.now() lần cuối nhận được heartbeat/tiến độ nào đó — tự phát hiện mất kết nối phía trình duyệt

  let parsedCommands = null;
  let progressSource = null;

  // Nhớ lại IP ESP32 đã nhập lần trước, khỏi phải gõ lại mỗi lần.
  ipInput.value = localStorage.getItem('esp32Ip') || '';
  ipInput.addEventListener('change', () => localStorage.setItem('esp32Ip', ipInput.value.trim()));

  timeScaleInput.value = localStorage.getItem('esp32TimeScale') || '1';
  timeScaleInput.addEventListener('change', () => localStorage.setItem('esp32TimeScale', timeScaleInput.value));

  // Mô tả từng lệnh — thời gian hiện theo đúng đơn vị gốc "time_ms" (mili-giây),
  // không quy đổi sang giây, để khớp chính xác giá trị thực sự gửi cho ESP32.
  function describeCommand(cmd) {
    if (cmd.command === 'straight') {
      return `Đi thẳng ${cmd.time_ms || 0} ms`;
    }
    if (cmd.command === 'turn') {
      const angle = cmd.turn_angle || 0;
      const direction = angle > 0 ? 'trái' : 'phải';
      return `Quay ${direction} ${Math.abs(angle)}°`;
    }
    return `Lệnh không rõ (${cmd.command})`;
  }

  function renderPendingList(commands) {
    listEmpty.style.display = 'none';
    banner.style.display = 'none';
    resetConnectionStatsDisplay();
    listBox.querySelectorAll('.esp32-progress-item').forEach(el => el.remove());
    commands.forEach((cmd, i) => {
      const row = document.createElement('div');
      row.className = 'esp32-progress-item';
      row.innerHTML = `
        <span class="esp32-step-label">Bước ${i + 1}: ${describeCommand(cmd)}</span>
        <span class="esp32-step-status" id="esp32StepStatus${i}">Chờ</span>
      `;
      listBox.appendChild(row);
    });
  }

  function updateStepStatus(index, status) {
    const el = document.getElementById('esp32StepStatus' + index);
    if (!el) return;
    if (status === 'running') {
      el.textContent = 'Đang chạy';
      el.className = 'esp32-step-status esp32-step-status--running';
    } else if (status === 'done') {
      el.textContent = 'Xong';
      el.className = 'esp32-step-status esp32-step-status--done';
    } else {
      el.textContent = 'Chờ';
      el.className = 'esp32-step-status';
    }
  }

  // Vẽ đường thẳng đã đơn giản hoá (kết quả cuối, KHÔNG vẽ quỹ đạo gốc) lên
  // canvas — tự co giãn theo khung hình chữ nhật bao quanh các điểm, chừa lề.
  function drawPreview(vertices) {
    if (!vertices || vertices.length < 2) {
      canvas.style.display = 'none';
      return;
    }
    canvas.style.display = 'block';

    const ctx = canvas.getContext('2d');
    const W = canvas.width, H = canvas.height;
    ctx.clearRect(0, 0, W, H);

    const xs = vertices.map(p => p[0]);
    const ys = vertices.map(p => p[1]);
    const minX = Math.min(...xs), maxX = Math.max(...xs);
    const minY = Math.min(...ys), maxY = Math.max(...ys);
    const spanX = Math.max(maxX - minX, 1e-6);
    const spanY = Math.max(maxY - minY, 1e-6);

    const PAD = 24;
    const scale = Math.min((W - PAD * 2) / spanX, (H - PAD * 2) / spanY);

    // Toạ độ Y màn hình đảo chiều so với Y toán học (Y lên trên) — đảo lại để
    // hình vẽ không bị lộn ngược so với hướng thật của quỹ đạo.
    function toScreen(p) {
      const sx = PAD + (p[0] - minX) * scale;
      const sy = H - PAD - (p[1] - minY) * scale;
      return [sx, sy];
    }

    ctx.strokeStyle = 'rgba(0, 200, 150, 0.95)';
    ctx.lineWidth = 3;
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';
    ctx.beginPath();
    vertices.forEach((p, i) => {
      const [sx, sy] = toScreen(p);
      if (i === 0) ctx.moveTo(sx, sy); else ctx.lineTo(sx, sy);
    });
    ctx.stroke();

    // Chấm điểm bắt đầu (xanh lá) / kết thúc (đỏ), các góc giữa (cam nhỏ)
    vertices.forEach((p, i) => {
      const [sx, sy] = toScreen(p);
      ctx.beginPath();
      ctx.arc(sx, sy, i === 0 || i === vertices.length - 1 ? 6 : 4, 0, Math.PI * 2);
      ctx.fillStyle = i === 0 ? '#2ecc71' : (i === vertices.length - 1 ? '#e74c3c' : '#f39c12');
      ctx.fill();
    });
  }

  function setConvertStatus(text, isError) {
    convertStatus.textContent = text;
    convertStatus.style.color = isError ? 'var(--danger)' : '';
  }

  // Chọn file — CHỈ cập nhật tên hiển thị + bật nút "Phân tích", KHÔNG tự động
  // upload. Cố ý tách riêng khỏi việc phân tích: input[type=file] không bắn
  // sự kiện "change" nếu chọn LẠI đúng file cũ (đặc điểm chuẩn của trình
  // duyệt) — nếu để tự động upload ngay trong handler này, người dùng đổi hệ
  // số thời gian rồi chọn lại file cũ sẽ không kích hoạt gì cả, tưởng nhầm là
  // hệ số "không cập nhật". Bấm nút riêng luôn đọc giá trị mới nhất, không
  // phụ thuộc sự kiện change nữa.
  csvInput.addEventListener('change', () => {
    const file = csvInput.files[0];
    csvFileName.textContent = file ? file.name : 'Chưa chọn file nào';
    btnAnalyze.disabled = !file;

    parsedCommands = null;
    btnSend.disabled = true;
    canvas.style.display = 'none';
    listEmpty.style.display = 'block';
    listBox.querySelectorAll('.esp32-progress-item').forEach(el => el.remove());
    setConvertStatus('Đã chọn file — chỉnh hệ số thời gian nếu cần rồi bấm "Phân tích quỹ đạo".', false);
  });

  btnAnalyze.addEventListener('click', () => {
    const file = csvInput.files[0];
    if (!file) {
      setConvertStatus('Chưa chọn file CSV nào.', true);
      return;
    }

    btnAnalyze.disabled = true;
    btnAnalyze.textContent = 'Đang phân tích…';
    setConvertStatus('Đang phân tích quỹ đạo…', false);

    const formData = new FormData();
    formData.append('csv', file);
    formData.append('time_scale', timeScaleInput.value.trim() || '1');

    fetch('/api/esp32/convert_csv', { method: 'POST', body: formData })
      .then(r => r.json())
      .then(data => {
        btnAnalyze.disabled = false;
        btnAnalyze.textContent = 'Phân tích quỹ đạo';
        if (!data.ok) {
          setConvertStatus('Lỗi: ' + data.error, true);
          return;
        }
        parsedCommands = data.commands;
        btnSend.disabled = false;
        setConvertStatus(`Đã phân tích thành ${data.commands.length} lệnh (${data.preview_vertices.length} đoạn thẳng) — hệ số thời gian ${timeScaleInput.value.trim() || '1'}.`, false);
        drawPreview(data.preview_vertices);
        renderPendingList(parsedCommands);
      })
      .catch(err => {
        btnAnalyze.disabled = false;
        btnAnalyze.textContent = 'Phân tích quỹ đạo';
        setConvertStatus('Không thể kết nối server: ' + err.message, true);
      });
  });

  // RSSI (dBm, thường -30 đến -90) → nhãn chất lượng dễ hiểu.
  function rssiToLabel(rssi) {
    if (rssi === null || rssi === undefined) return '—';
    if (rssi > -50) return `Rất tốt (${rssi} dBm)`;
    if (rssi > -60) return `Tốt (${rssi} dBm)`;
    if (rssi > -70) return `Trung bình (${rssi} dBm)`;
    if (rssi > -80) return `Yếu (${rssi} dBm)`;
    return `Rất yếu (${rssi} dBm)`;
  }

  function setConnectionOnline(ip, rssi) {
    lastHeartbeatAt = Date.now();
    connDot.className = 'esp32-conn-dot esp32-conn-dot--online';
    connStatusText.textContent = 'Đã kết nối';
    if (ip) connIp.textContent = ip;
    if (rssi !== undefined && rssi !== null) connRssi.textContent = rssiToLabel(rssi);
  }

  function setConnectionOffline() {
    connDot.className = 'esp32-conn-dot esp32-conn-dot--offline';
    connStatusText.textContent = 'Mất kết nối';
  }

  // Tự kiểm tra định kỳ — nếu quá lâu không nhận được heartbeat/tiến độ nào,
  // coi như mất kết nối NGAY CẢ KHI kết nối SSE tới Flask vẫn còn (vd ESP32 bị
  // rút nguồn/rớt Wi-Fi nhưng trình duyệt vẫn đang mở trang bình thường).
  const CONN_STALE_MS = 6000;
  setInterval(() => {
    if (lastHeartbeatAt && Date.now() - lastHeartbeatAt > CONN_STALE_MS) {
      setConnectionOffline();
    }
  }, 1000);

  function showCompletionSummary(summary) {
    banner.style.display = 'block';
    if (!summary) return;

    statsDivider.style.display = 'block';

    if (typeof summary.received_packets === 'number') {
      packetField.style.display = 'flex';
      const expected = summary.expected_packets;
      connPackets.textContent = expected
        ? `${summary.received_packets} / ${expected}`
        : `${summary.received_packets}`;
    }

    if (summary.loss_pct !== null && summary.loss_pct !== undefined) {
      lossField.style.display = 'flex';
      connLoss.textContent = `${summary.loss_pct}%`;
      connLoss.style.color = summary.loss_pct > 5 ? 'var(--danger)' : 'var(--accent)';
    }

    if (summary.avg_latency_ms !== null && summary.avg_latency_ms !== undefined) {
      latencyField.style.display = 'flex';
      connLatency.textContent = `${Math.round(summary.avg_latency_ms)} ms`;
    }
  }

  function resetConnectionStatsDisplay() {
    statsDivider.style.display = 'none';
    packetField.style.display = 'none';
    lossField.style.display = 'none';
    latencyField.style.display = 'none';
  }

  function connectProgressStream() {
    if (progressSource) progressSource.close();
    progressSource = new EventSource('/api/esp32/progress-stream');
    progressSource.onmessage = (e) => {
      let data;
      try { data = JSON.parse(e.data); } catch { return; }

      if (data.type === 'reset' && Array.isArray(data.progress) && data.progress.length) {
        // Server đã có tiến độ từ trước (vd mở lại modal giữa chừng) — không
        // ghi đè danh sách mô tả bước hiện có nếu người dùng chưa tải file mới.
        data.progress.forEach(p => updateStepStatus(p.index, p.status));
      } else if (data.type === 'update') {
        updateStepStatus(data.index, data.status);
        setConnectionOnline();
      } else if (data.type === 'heartbeat') {
        setConnectionOnline(data.ip, data.rssi);
      } else if (data.type === 'completed') {
        setConnectionOnline();
        showCompletionSummary(data.summary);
      }
    };
  }

  // Poll dự phòng lúc vừa mở modal — có ngay trạng thái kết nối hiện tại thay
  // vì phải chờ heartbeat tiếp theo (tối đa 2s) mới biết.
  function pollConnectionStatusOnce() {
    fetch('/api/esp32/connection_status')
      .then(r => r.json())
      .then(data => {
        if (data.ok && data.connected) {
          setConnectionOnline(data.ip, data.rssi);
        } else if (data.ok) {
          if (data.ip) connIp.textContent = data.ip;
          setConnectionOffline();
        }
      })
      .catch(() => {});
  }

  btnStep5.addEventListener('click', () => {
    setActiveStep(5);
    openModal('modalStep5');
    connectProgressStream();
    pollConnectionStatusOnce();
  });

  btnSend.addEventListener('click', () => {
    const ip = ipInput.value.trim();
    if (!ip) { alert('Nhập địa chỉ IP ESP32 trước.'); return; }
    if (!parsedCommands) { alert('Tải file CSV và chờ phân tích xong trước.'); return; }

    btnSend.disabled = true;
    btnSend.textContent = 'Đang gửi…';

    fetch('/api/esp32/send', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ esp32_ip: ip, commands: parsedCommands }),
    })
      .then(r => r.json())
      .then(data => {
        btnSend.textContent = 'Gửi lệnh tới ESP32';
        btnSend.disabled = false;
        if (!data.ok) alert('Lỗi: ' + data.error);
      })
      .catch(err => {
        btnSend.textContent = 'Gửi lệnh tới ESP32';
        btnSend.disabled = false;
        alert('Không thể kết nối server: ' + err.message);
      });
  });

  btnBack.addEventListener('click', () => closeModal('modalStep5'));
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