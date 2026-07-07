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

// ── Bước 2: Khởi động cầu nối CARLA ROS BRIDGE ──────────────────────────────
document.getElementById('btnStep2').addEventListener('click', () => openModal('modalStep2'));

// ── Bước 3: Điều khiển xe mô phỏng lái → sang trang điều khiển hiện tại ─────
document.getElementById('btnStep3').addEventListener('click', () => {
  window.location.href = '/control';
});

// ── Bước 4: Tạo đồ thị và phân tích dữ liệu ─────────────────────────────────
document.getElementById('btnStep4').addEventListener('click', () => openModal('modalStep4'));

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