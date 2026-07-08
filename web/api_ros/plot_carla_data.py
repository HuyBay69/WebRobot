#!/usr/bin/env python3
"""
plot_carla_data.py — Vẽ đồ thị dữ liệu đã ghi từ data_logger.py.

Phụ thuộc: numpy, matplotlib   (không cần pandas)

── Cách dùng ─────────────────────────────────────────────────────────────────
  python3 plot_carla_data.py                     # tự động chọn file MỚI NHẤT
                                                  # trong web/recorded_data/
  python3 plot_carla_data.py duong/dan/file.csv  # chỉ định file cụ thể

── Bố cục cửa sổ ────────────────────────────────────────────────────────────
  Cột trái:  [Quỹ đạo x-y (ô vuông, cao gấp đôi)]
             [Ga     ]
             [Phanh  ]
  Cột phải:  [v tổng hợp (v_lat + v_long)]
             [wz — tốc độ góc          ]
             [a lat — gia tốc ngang    ]
             [a long — gia tốc dọc     ]
  (Chỉ riêng đồ thị quỹ đạo vẽ theo 2 trục x-y; tất cả đồ thị còn lại vẽ theo
  trục ngang là THỜI GIAN kể từ lúc bắt đầu ghi.)

── Tương tác ─────────────────────────────────────────────────────────────────
  - Click vào 1 điểm trên quỹ đạo → đồng bộ vạch đứng (crosshair) + chấm giá
    trị trên TẤT CẢ đồ thị còn lại tại đúng mốc thời gian đó, kèm ô đọc số ở
    trên cùng cửa sổ.
  - Click vào bất kỳ đồ thị theo thời gian nào (ga/phanh/v/wz/a_lat/a_long)
    cũng nhảy tới mốc thời gian đó tương tự (tiện khi muốn xem 1 đoạn cụ thể
    mà không cần quay lại đồ thị quỹ đạo).
  - Phím ← / → : lùi / tiến đúng 1 frame (1 dòng dữ liệu).
  - Trên đồ thị "v tổng hợp": các chấm CAM đánh dấu từng đoạn xe đứng yên
    (is_idle) — click vào 1 chấm để hiện đã dừng bao nhiêu giây tại đó.
  - Nếu có file "<tên_csv>_waypoints.json" đi kèm (do navigate_node.py ghi mỗi
    lần "Chốt hành trình", được data_logger.py copy theo cặp khi Export), các
    điểm đích đã YÊU CẦU (A, B, C...) sẽ được đánh dấu bằng sao cam trên quỹ đạo.

── Gợi ý mở rộng sau này (chưa làm) ──────────────────────────────────────────
  - Nút Play/Pause tự động chạy qua từng frame theo đúng nhịp thời gian thực.
  - So sánh 2 file cạnh nhau (vd 2 lần chạy cùng 1 tuyến để đối chiếu).
  - Xuất ảnh PNG của khung hình đang chọn.
"""
import sys
import os
import re
import json

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


def _find_recorded_data_dir():
    """Dò thư mục recorded_data/ — script này ĐÚNG RA nên đặt tại web/ (ngang
    hàng app.py), nhưng phòng trường hợp lỡ đặt nhầm ở web/api_ros/ hoặc chỗ
    khác trong cây thư mục, tự thử thêm 1-2 cấp cha trước khi báo lỗi."""
    candidates = [
        os.path.join(SCRIPT_DIR, 'recorded_data'),
        os.path.join(SCRIPT_DIR, '..', 'recorded_data'),
        os.path.join(SCRIPT_DIR, '..', '..', 'recorded_data'),
    ]
    for c in candidates:
        c = os.path.abspath(c)
        if os.path.isdir(c):
            return c
    return os.path.abspath(candidates[0])  # không thấy cái nào — dùng cái đầu để báo lỗi có ý nghĩa


RECORDED_DATA_DIR = _find_recorded_data_dir()


# ── Helpers ──────────────────────────────────────────────────────────────────
# Chỉ khớp ĐÚNG định dạng do export_recorded_data() sinh ra: carla_data_YYYYMMDD_HHMMSS.csv
# — KHÔNG khớp file tạm carla_data_local.csv (dấu * của glob thường sẽ khớp nhầm
# "local" nếu lỡ nó bị copy/xuất hiện trong recorded_data/, dẫn tới đọc nhầm file
# tạm có thể ở schema cũ/khác).
_EXPORTED_FNAME_RE = re.compile(r'^carla_data_\d{8}_\d{6}\.csv$')


def find_latest_csv():
    if not os.path.isdir(RECORDED_DATA_DIR):
        return None
    files = [f for f in os.listdir(RECORDED_DATA_DIR) if _EXPORTED_FNAME_RE.match(f)]
    if not files:
        return None
    files.sort()  # tên có timestamp dạng số nên sort chuỗi = sort theo thời gian
    return os.path.join(RECORDED_DATA_DIR, files[-1])


def load_csv(path):
    data = np.genfromtxt(path, delimiter=',', names=True, dtype=None, encoding='utf-8')
    if data.ndim == 0:
        # File chỉ có đúng 1 dòng dữ liệu -> genfromtxt trả về mảng 0-chiều,
        # ép về mảng 1 phần tử để mọi chỗ dùng len()/indexing phía dưới còn đúng.
        data = np.array([data])
    return data


def load_waypoints_for(csv_path):
    """Tìm file '<csv không đuôi>_waypoints.json' đi kèm (do data_logger.py copy
    theo cặp khi Export). Trả về [] nếu không có — không coi là lỗi."""
    base = csv_path[:-4] if csv_path.endswith('.csv') else csv_path
    wp_path = base + '_waypoints.json'
    if os.path.isfile(wp_path):
        try:
            with open(wp_path) as f:
                d = json.load(f)
            return d.get('points', [])
        except Exception as e:
            print(f'[Cảnh báo] Không đọc được file waypoints kèm theo: {e}')
    return []


def detect_idle_clusters(rel_t, is_idle):
    """Gom các dòng is_idle liên tiếp thành từng cụm 'đứng yên', trả về list
    dict {start, end, duration} (index vào mảng dữ liệu + thời lượng giây)."""
    clusters = []
    n = len(rel_t)
    i = 0
    while i < n:
        if is_idle[i]:
            j = i
            while j < n and is_idle[j]:
                j += 1
            clusters.append({'start': i, 'end': j - 1, 'duration': float(rel_t[j - 1] - rel_t[i])})
            i = j
        else:
            i += 1
    return clusters


def format_duration(sec):
    sec = float(sec)
    if sec < 60:
        return f'{sec:.1f}s'
    m, s = divmod(sec, 60)
    return f'{int(m)}m {s:.1f}s'


# ── Lớp chính ────────────────────────────────────────────────────────────────
class Plotter:
    CROSSHAIR_COLOR = '#c800c8'  # magenta — điểm/khung hình đang chọn (khác màu waypoint & start/end)

    def __init__(self, csv_path):
        self.csv_path = csv_path
        self.data = load_csv(csv_path)
        self.n = len(self.data)
        if self.n == 0:
            raise ValueError('File CSV rỗng (không có dòng dữ liệu nào).')

        available = set(self.data.dtype.names or ())

        def _req(name):
            """Cột BẮT BUỘC — thiếu là báo lỗi rõ ràng thay vì traceback khó hiểu
            (thường do đang đọc nhầm file cũ ghi trước khi đổi schema)."""
            if name not in available:
                raise ValueError(
                    f'File CSV thiếu cột "{name}" — có thể đây là file cũ ghi từ '
                    f'phiên bản data_logger.py trước đó (schema khác). Hãy "Xuất dữ '
                    f'liệu" lại từ 1 phiên ghi MỚI rồi thử lại.\nFile: {csv_path}'
                )
            return self.data[name].astype(float)

        self.t = _req('timestamp')
        self.rel_t = self.t - self.t[0]

        self.x            = _req('x')
        self.y            = _req('y')
        self.v_total       = _req('v_total')
        self.w_z           = _req('w_z')
        self.a_lat         = _req('a_lat')
        self.a_long        = _req('a_long')
        self.throttle_real = _req('throttle_real')
        self.throttle_cmd  = _req('throttle_cmd')
        self.brake_real    = _req('brake_real')
        self.brake_cmd     = _req('brake_cmd')

        # is_idle: cột MỀM — file cũ hơn có thể chưa có, không cần chặn cả chương
        # trình vì việc này, chỉ mất tính năng đánh dấu chấm dừng trên đồ thị v_total.
        if 'is_idle' in available:
            self.is_idle = self.data['is_idle'].astype(int)
        else:
            print('[Cảnh báo] File không có cột "is_idle" (file cũ?) — bỏ qua '
                  'đánh dấu các đoạn đứng yên trên đồ thị vận tốc.')
            self.is_idle = np.zeros(self.n, dtype=int)

        self.idle_clusters = detect_idle_clusters(self.rel_t, self.is_idle)
        self.waypoints     = load_waypoints_for(csv_path)

        self.cur_idx = 0

        self._build_figure()
        self._connect_events()
        self._update_all()

    # ── Dựng layout + vẽ dữ liệu tĩnh (đường, marker cố định) ────────────────
    def _build_figure(self):
        self.fig = plt.figure(figsize=(13, 8.2))
        try:
            self.fig.canvas.manager.set_window_title(f'CARLA Data — {os.path.basename(self.csv_path)}')
        except Exception:
            pass

        gs = gridspec.GridSpec(
            4, 2, figure=self.fig,
            width_ratios=[1.15, 1.4],
            hspace=0.5, wspace=0.28,
            left=0.07, right=0.97, top=0.90, bottom=0.07,
        )

        self.ax_traj     = self.fig.add_subplot(gs[0:2, 0])   # quỹ đạo — cao gấp đôi ga/phanh
        self.ax_throttle = self.fig.add_subplot(gs[2, 0])
        self.ax_brake    = self.fig.add_subplot(gs[3, 0])

        self.ax_vtotal = self.fig.add_subplot(gs[0, 1])
        self.ax_wz     = self.fig.add_subplot(gs[1, 1])
        self.ax_alat   = self.fig.add_subplot(gs[2, 1])
        self.ax_along  = self.fig.add_subplot(gs[3, 1])

        self.time_axes = [self.ax_throttle, self.ax_brake, self.ax_vtotal,
                           self.ax_wz, self.ax_alat, self.ax_along]

        # ── Quỹ đạo (x, y) ──
        self.ax_traj.plot(self.x, self.y, '-', color='#3b7dd8', lw=1.3, zorder=2)
        self.ax_traj.plot(self.x[0], self.y[0], 'o', color='#2ecc71', ms=9,
                           mec='black', mew=0.5, label='Bắt đầu', zorder=4)
        self.ax_traj.plot(self.x[-1], self.y[-1], 's', color='#e74c3c', ms=9,
                           mec='black', mew=0.5, label='Kết thúc', zorder=4)

        if self.waypoints:
            wx = [p[0] for p in self.waypoints]
            wy = [p[1] for p in self.waypoints]
            self.ax_traj.plot(wx, wy, '*', color='#f39c12', ms=16, mec='black', mew=0.6,
                               linestyle='None', label='Điểm yêu cầu', zorder=5)
            for i, (px, py) in enumerate(self.waypoints):
                self.ax_traj.annotate(chr(65 + i), (px, py), textcoords='offset points',
                                       xytext=(7, 7), fontsize=10, fontweight='bold', color='#a0650a')

        self.ax_traj.set_aspect('equal', adjustable='datalim')
        self.ax_traj.set_xlabel('x (m)')
        self.ax_traj.set_ylabel('y (m)')
        self.ax_traj.set_title('Quỹ đạo')
        self.ax_traj.legend(loc='best', fontsize=8)
        self.ax_traj.grid(True, alpha=0.3)

        self.traj_marker, = self.ax_traj.plot([], [], 'o', color=self.CROSSHAIR_COLOR, ms=11,
                                               mec='white', mew=1.5, zorder=6)

        # ── Ga / Phanh (theo thời gian) ──
        self.ax_throttle.plot(self.rel_t, self.throttle_real, '-', color='#2ecc71', lw=1.1, label='Thực tế')
        self.ax_throttle.plot(self.rel_t, self.throttle_cmd, '--', color='#27ae60', lw=1.0, alpha=0.7, label='Lệnh')
        self.ax_throttle.set_ylabel('Ga')
        self.ax_throttle.legend(loc='upper right', fontsize=7)
        self.ax_throttle.grid(True, alpha=0.3)

        self.ax_brake.plot(self.rel_t, self.brake_real, '-', color='#e74c3c', lw=1.1, label='Thực tế')
        self.ax_brake.plot(self.rel_t, self.brake_cmd, '--', color='#c0392b', lw=1.0, alpha=0.7, label='Lệnh')
        self.ax_brake.set_ylabel('Phanh')
        self.ax_brake.set_xlabel('Thời gian (s)')
        self.ax_brake.legend(loc='upper right', fontsize=7)
        self.ax_brake.grid(True, alpha=0.3)

        # ── v tổng hợp (+ chấm đứng yên) ──
        self.ax_vtotal.plot(self.rel_t, self.v_total, '-', color='#3b7dd8', lw=1.1)
        self.ax_vtotal.set_ylabel('v tổng (m/s)')
        self.ax_vtotal.set_title('Vận tốc tổng hợp (v_lat + v_long)')
        self.ax_vtotal.grid(True, alpha=0.3)

        self.idle_dot_artists = []
        for c in self.idle_clusters:
            dot, = self.ax_vtotal.plot(
                self.rel_t[c['start']], self.v_total[c['start']],
                'o', color='#f39c12', ms=7, mec='black', mew=0.5,
                picker=8, zorder=5,
            )
            self.idle_dot_artists.append((dot, c))

        self.ax_wz.plot(self.rel_t, self.w_z, '-', color='#8e44ad', lw=1.1)
        self.ax_wz.set_ylabel('wz (rad/s)')
        self.ax_wz.set_title('Tốc độ góc (yaw rate)')
        self.ax_wz.grid(True, alpha=0.3)

        self.ax_alat.plot(self.rel_t, self.a_lat, '-', color='#16a085', lw=1.1)
        self.ax_alat.set_ylabel('a lat (m/s²)')
        self.ax_alat.set_title('Gia tốc ngang')
        self.ax_alat.grid(True, alpha=0.3)

        self.ax_along.plot(self.rel_t, self.a_long, '-', color='#d35400', lw=1.1)
        self.ax_along.set_ylabel('a dọc (m/s²)')
        self.ax_along.set_xlabel('Thời gian (s)')
        self.ax_along.set_title('Gia tốc dọc trục')
        self.ax_along.grid(True, alpha=0.3)

        # Crosshair: 1 vạch đứng + 1 chấm tại giao điểm, trên mỗi đồ thị theo thời gian
        self.vlines = {}
        self.hdots = {}
        for ax in self.time_axes:
            vline = ax.axvline(self.rel_t[0], color=self.CROSSHAIR_COLOR, lw=1, ls='-', alpha=0.75, zorder=3)
            dot,  = ax.plot([], [], 'o', color=self.CROSSHAIR_COLOR, ms=6,
                             mec='white', mew=1, zorder=4)
            self.vlines[ax] = vline
            self.hdots[ax]  = dot

        # Ô đọc giá trị (trên cùng cửa sổ)
        self.readout = self.fig.text(0.5, 0.965, '', ha='center', va='top',
                                      fontsize=9.5, family='monospace')

        # Annotation hiện thời lượng dừng khi click vào chấm cam (ẩn mặc định)
        self.idle_annotation = self.ax_vtotal.annotate(
            '', xy=(0, 0), xytext=(12, 16), textcoords='offset points',
            bbox=dict(boxstyle='round,pad=0.35', fc='#fff3cd', ec='#f39c12'),
            fontsize=8.5, visible=False, zorder=10,
        )

        self.fig.suptitle('') if False else None  # (giữ chỗ, không dùng suptitle để tránh đè lên readout)

    # ── Sự kiện chuột / bàn phím ─────────────────────────────────────────────
    def _connect_events(self):
        self.fig.canvas.mpl_connect('button_press_event', self._on_click)
        self.fig.canvas.mpl_connect('key_press_event', self._on_key)

    def _on_click(self, event):
        if event.inaxes is None or event.xdata is None:
            return

        # Ưu tiên kiểm tra click vào chấm "đứng yên" trên v_total trước
        if event.inaxes is self.ax_vtotal:
            for dot, cluster in self.idle_dot_artists:
                contains, _ = dot.contains(event)
                if contains:
                    self._show_idle_duration(cluster)
                    self.set_index(cluster['start'])
                    return

        if event.inaxes is self.ax_traj:
            dx = self.x - event.xdata
            dy = self.y - event.ydata
            idx = int(np.argmin(dx * dx + dy * dy))
            self.set_index(idx)
            return

        if event.inaxes in self.time_axes:
            idx = int(np.argmin(np.abs(self.rel_t - event.xdata)))
            self.set_index(idx)
            return

    def _on_key(self, event):
        if event.key == 'right':
            self.set_index(min(self.cur_idx + 1, self.n - 1))
        elif event.key == 'left':
            self.set_index(max(self.cur_idx - 1, 0))

    # ── Cập nhật trạng thái hiển thị ─────────────────────────────────────────
    def set_index(self, idx):
        self.cur_idx = idx
        self.idle_annotation.set_visible(False)
        self._update_all()

    def _show_idle_duration(self, cluster):
        i = cluster['start']
        self.idle_annotation.xy = (self.rel_t[i], self.v_total[i])
        self.idle_annotation.set_text(f"Dừng {format_duration(cluster['duration'])}")
        self.idle_annotation.set_visible(True)

    def _update_all(self):
        i = self.cur_idx
        t = self.rel_t[i]

        self.traj_marker.set_data([self.x[i]], [self.y[i]])

        for ax in self.time_axes:
            self.vlines[ax].set_xdata([t, t])

        self.hdots[self.ax_throttle].set_data([t], [self.throttle_real[i]])
        self.hdots[self.ax_brake].set_data([t], [self.brake_real[i]])
        self.hdots[self.ax_vtotal].set_data([t], [self.v_total[i]])
        self.hdots[self.ax_wz].set_data([t], [self.w_z[i]])
        self.hdots[self.ax_alat].set_data([t], [self.a_lat[i]])
        self.hdots[self.ax_along].set_data([t], [self.a_long[i]])

        self.readout.set_text(
            f"Frame {i + 1}/{self.n}   t={t:6.2f}s   x={self.x[i]:7.2f}  y={self.y[i]:7.2f}   "
            f"v={self.v_total[i]:5.2f} m/s   wz={self.w_z[i]:+.2f} rad/s   "
            f"a_lat={self.a_lat[i]:+.2f}  a_long={self.a_long[i]:+.2f} m/s²   "
            f"ga={self.throttle_real[i]:.2f}  phanh={self.brake_real[i]:.2f}"
        )

        self.fig.canvas.draw_idle()


def main():
    if len(sys.argv) > 1:
        csv_path = sys.argv[1]
        if not os.path.isfile(csv_path):
            print(f'Không tìm thấy file: {csv_path}')
            sys.exit(1)
    else:
        csv_path = find_latest_csv()
        if csv_path is None:
            print(f'Không tìm thấy file .csv nào trong: {RECORDED_DATA_DIR}')
            print('Hãy bấm "Xuất dữ liệu" trên trang điều khiển trước, hoặc chỉ định file:')
            print('  python3 plot_carla_data.py duong/dan/file.csv')
            sys.exit(1)
        print(f'Dùng file mới nhất: {csv_path}')

    try:
        plotter = Plotter(csv_path)
    except ValueError as e:
        print(f'\nLỗi: {e}')
        sys.exit(1)

    plt.show()


if __name__ == '__main__':
    main()