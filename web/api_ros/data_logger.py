#!/usr/bin/env python3
"""
data_logger.py — Ghi dữ liệu xe hero ra CSV, tích hợp trực tiếp với Web (giống
pattern navigate_node.py: 1 file vừa là ROS2 node vừa có sẵn hàm start/stop/export
+ Flask blueprint để app.py import dùng).

── Frame tham chiếu (ĐÃ KIỂM CHỨNG BẰNG THỰC NGHIỆM — giữ nguyên, không đổi) ────
  - twist (/odometry, msg.twist.twist.linear.x/y) ĐÃ Ở BODY FRAME (tương đối) sẵn
    — vx = dọc trục, vy = ngang, KHÔNG cần xoay theo yaw. Bằng chứng: khi xe vào
    cua, vy chỉ nhích nhẹ trong khi vx luôn áp đảo/ổn định quanh tốc độ thật —
    nếu ở world frame thì vx phải tụt mạnh về 0 khi xe đổi hướng.
  - acceleration (/vehicle_status, msg.acceleration.linear.x/y) VẪN Ở WORLD FRAME
    (tuyệt đối) — cần tự xoay theo yaw mới ra đúng dọc trục/ngang. Bằng chứng: ax
    chỉ nhảy khi xe chạy theo trục world-X, ay chỉ nhảy khi xe chạy theo trục
    world-Y — đúng đặc trưng world frame.

── Thay đổi lần này ─────────────────────────────────────────────────────────────
  1. Cột dữ liệu rút gọn theo đúng yêu cầu: x, y, yaw, v_total (tổng hợp v_lat +
     v_long), v_lat, v_long, w_z, a_lat, a_long (BỎ a_z, BỎ v_cmd/speed_command).
  2. Thêm cột `is_idle` (0/1) — đánh dấu xe có đang đứng yên hay không tại thời
     điểm ghi dòng đó (v_total < IDLE_SPEED_THRESHOLD_MPS). Mục đích: để 1 file
     vẽ đồ thị SAU NÀY (chưa làm ở đây) có thể lọc/tô các điểm đứng yên thành
     chấm tròn đỏ, thay vì phải tự đoán lại từ v_total.
  3. Vẫn giữ nguyên cơ chế lọc dòng trùng lặp hoàn toàn khi xe đứng yên
     (self.filter_idle) — is_idle không thay thế cơ chế này, chỉ bổ sung thêm
     thông tin cho dòng NÀO ĐƯỢC GHI biết nó có phải lúc đứng yên không.
  4. Đường dẫn file CSV giờ nhận qua tham số dòng lệnh (argv[1]) thay vì luôn
     cố định theo os.getcwd() — để phần start_data_logger() bên dưới kiểm soát
     chính xác ghi vào đâu (mặc định: web/data_record/carla_data_local.csv).
  5. Mọi giá trị số thực làm tròn 3 chữ số thập phân trước khi ghi CSV.
  6. Không ghi dòng nào tới khi đã nhận ít nhất 1 message odometry VÀ 1 message
     vehicle_status (tránh các dòng 0.0 giả ở đầu file).

── Tích hợp Web ─────────────────────────────────────────────────────────────────
  - start_data_logger(): app.js gọi ngay sau khi spawn xe hero thành công (xem
    startNavigationStack() cùng chỗ trong app.js — data logger cũng khởi động ở
    đúng điểm đó). Nếu đang có 1 phiên ghi cũ chạy, tự dừng sạch (đóng CSV đúng
    cách) rồi mới bắt đầu phiên mới — để không lẫn dữ liệu 2 lần spawn khác nhau
    vào cùng 1 file.
  - File tạm luôn ghi đè tại web/data_record/carla_data_local.csv — KHÔNG stream
    lên web, chỉ ghi nội bộ trên máy chạy Flask.
  - export_recorded_data(): nút "Xuất dữ liệu" trên trang điều khiển gọi
    POST /api/datalogger/export — copy file tạm hiện tại sang
    web/recorded_data/carla_data_<timestamp>.csv để giữ lại vĩnh viễn, không
    đụng tới file tạm (vẫn tiếp tục ghi bình thường sau khi export).
"""
import csv
import math
import os
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaEgoVehicleStatus
from flask import Blueprint, jsonify, request

IDLE_SPEED_THRESHOLD_MPS = 0.1  # v_total dưới ngưỡng này (m/s) coi như xe đứng yên
IDLE_REWRITE_PERIOD_SEC  = 1.0  # khi đứng yên không đổi, vẫn ghi lại mỗi bao nhiêu giây (giữ mốc thời gian cho việc tính thời lượng dừng)


class CarlaDataCollector(Node):
    def __init__(self, csv_path=None):
        super().__init__('carla_data_collector')

        # Đường dẫn file CSV — ưu tiên tham số truyền vào (do start_data_logger()
        # chỉ định), nếu không có thì fallback về thư mục hiện tại (chạy tay độc lập).
        self.csv_file_path = csv_path or os.path.join(os.getcwd(), 'carla_data_local.csv')

        # Bật/tắt lọc dòng trùng lặp hoàn toàn (chủ yếu xảy ra khi xe đứng yên)
        self.filter_idle = True

        # Định nghĩa các cột theo đúng thứ tự yêu cầu
        self.csv_headers = [
            'timestamp',
            'x', 'y', 'yaw',
            'v_total', 'v_lat', 'v_long',
            'w_z',
            'a_lat', 'a_long',
            'throttle_cmd', 'brake_cmd', 'steer_cmd', 'gear_cmd',
            'throttle_real', 'brake_real', 'steer_real', 'gear_real',
            'is_idle',
        ]

        # Khởi tạo file CSV
        os.makedirs(os.path.dirname(self.csv_file_path) or '.', exist_ok=True)
        self.csv_file = open(self.csv_file_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(self.csv_headers)

        # Khởi tạo biến lưu trữ tạm thời và khóa Thread Lock
        self.data_lock = threading.Lock()
        self.data = {key: 0.0 for key in self.csv_headers}
        self.data['gear_cmd'] = 0
        self.data['gear_real'] = 0
        self.data['is_idle'] = 0

        # Cờ đánh dấu đã nhận đủ dữ liệu cốt lõi (odometry + vehicle_status) chưa
        self._has_odom = False
        self._has_status = False
        self._last_content = None    # nội dung dòng đã ghi gần nhất (KHÔNG gồm timestamp), để lọc trùng lặp
        self._last_write_time = 0.0  # thời điểm (timestamp) ghi dòng gần nhất

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/carla/hero/odometry', self.odom_callback, 10)
        self.ctrl_cmd_sub = self.create_subscription(
            CarlaEgoVehicleControl, '/carla/hero/vehicle_control_cmd', self.ctrl_cmd_callback, 10)
        self.status_sub = self.create_subscription(
            CarlaEgoVehicleStatus, '/carla/hero/vehicle_status', self.status_callback, 10)

        # Timer ghi file tần suất 10Hz (tốc độ ghi thực tế phụ thuộc tốc độ publish
        # thật của /carla/hero/odometry)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info(f'Data Collector Node started. Saving to: {self.csv_file_path}')

    def euler_from_quaternion(self, x, y, z, w):
        """Chuyển đổi Quaternion sang Euler (chỉ lấy Yaw)"""
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        return yaw_z

    def odom_callback(self, msg):
        with self.data_lock:
            self.data['timestamp'] = msg.header.stamp.sec + (msg.header.stamp.nanosec / 1e9)

            # Pose: x, y, yaw
            self.data['x'] = msg.pose.pose.position.x
            self.data['y'] = msg.pose.pose.position.y
            orientation_q = msg.pose.pose.orientation
            yaw = self.euler_from_quaternion(
                orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
            )
            self.data['yaw'] = yaw

            # Twist ĐÃ ở body frame sẵn (xác nhận thực nghiệm) -> dùng thẳng, không xoay
            v_long = msg.twist.twist.linear.x
            v_lat = msg.twist.twist.linear.y
            self.data['v_long'] = v_long
            self.data['v_lat'] = v_lat
            self.data['v_total'] = math.sqrt(v_long ** 2 + v_lat ** 2)
            self.data['w_z'] = msg.twist.twist.angular.z

            self._has_odom = True

    def ctrl_cmd_callback(self, msg):
        with self.data_lock:
            self.data['throttle_cmd'] = msg.throttle
            self.data['brake_cmd'] = msg.brake
            self.data['steer_cmd'] = msg.steer
            self.data['gear_cmd'] = int(msg.gear)

    def status_callback(self, msg):
        with self.data_lock:
            # acceleration ở khung world/map -> xoay theo yaw hiện tại (đã cache từ
            # odometry) để ra dọc trục / ngang thật, giống twist ở trên
            yaw = self.data['yaw']
            ax = msg.acceleration.linear.x
            ay = msg.acceleration.linear.y
            self.data['a_long'] = ax * math.cos(yaw) + ay * math.sin(yaw)
            self.data['a_lat'] = -ax * math.sin(yaw) + ay * math.cos(yaw)

            self.data['throttle_real'] = msg.control.throttle
            self.data['brake_real'] = msg.control.brake
            self.data['steer_real'] = msg.control.steer
            self.data['gear_real'] = int(msg.control.gear)

            self._has_status = True

    def timer_callback(self):
        with self.data_lock:
            if not (self._has_odom and self._has_status):
                return

            self.data['is_idle'] = 1 if self.data['v_total'] < IDLE_SPEED_THRESHOLD_MPS else 0

            row = [
                round(self.data[key], 3) if isinstance(self.data[key], float) else self.data[key]
                for key in self.csv_headers
            ]

            # So sánh KHÔNG gồm timestamp (phần tử đầu tiên) — timestamp luôn khác
            # nhau giữa 2 mẫu liên tiếp nên nếu so cả timestamp thì "row == last"
            # không bao giờ đúng, khiến cơ chế lọc này trước đây thực chất không
            # lọc được gì (bug: xe đứng yên vẫn ghi liên tục 10 dòng/giây y hệt
            # nhau, chỉ khác mỗi timestamp).
            content = row[1:]
            now = self.data['timestamp']

            if self.filter_idle and self._last_content is not None and content == self._last_content:
                # Nội dung giống hệt dòng đã ghi gần nhất (xe đứng yên, không đổi
                # gì) — vẫn ghi định kỳ mỗi IDLE_REWRITE_PERIOD_SEC giây để giữ đủ
                # mốc thời gian tính thời lượng dừng (đồ thị cần biết lúc bắt đầu
                # VÀ lúc kết thúc mỗi đoạn đứng yên), thay vì ghi dồn dập không cần
                # thiết suốt cả lúc xe đứng im.
                if now - self._last_write_time < IDLE_REWRITE_PERIOD_SEC:
                    return

            self.csv_writer.writerow(row)
            self.csv_file.flush()
            self._last_content = content
            self._last_write_time = now

    def destroy_node(self):
        self.get_logger().info('Closing CSV file.')
        self.csv_file.close()
        super().destroy_node()


def main():
    csv_path = sys.argv[1] if len(sys.argv) > 1 else None
    rclpy.init()
    node = CarlaDataCollector(csv_path=csv_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()


# ── Spawn helpers + Flask blueprint — dùng bởi app.py ───────────────────────────
data_logger_bp = Blueprint('data_logger_bp', __name__)

# web/api_ros/data_logger.py → lên 1 cấp là web/
_BASE_DIR          = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA_RECORD_DIR    = os.path.join(_BASE_DIR, 'data_record')
RECORDED_DATA_DIR  = os.path.join(_BASE_DIR, 'recorded_data')
TEMP_CSV_PATH      = os.path.join(DATA_RECORD_DIR, 'carla_data_local.csv')
TEMP_WAYPOINTS_PATH = os.path.join(DATA_RECORD_DIR, 'mission_waypoints.json')  # ghi bởi navigate_node.py mỗi lần Chốt hành trình
DATA_LOGGER_SCRIPT = os.path.abspath(__file__)
PLOT_SCRIPT_PATH   = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'plot_carla_data.py')  # api_ros/plot_carla_data.py

# Chỉ chấp nhận đúng định dạng do export_recorded_data() sinh ra — chặn path
# traversal (vd "../../etc/passwd") khi nhận filename từ browser.
_EXPORTED_FNAME_RE = re.compile(r'^carla_data_\d{8}_\d{6}\.csv$')

_dl_lock = threading.Lock()
_dl_proc = None  # subprocess.Popen hiện tại (None nếu logger chưa chạy)
_dl_recording_start = None  # datetime — thời điểm bắt đầu phiên ghi hiện tại (None nếu chưa ghi)


def _dl_format_duration(total_seconds):
    total_seconds = int(total_seconds)
    h, rem = divmod(total_seconds, 3600)
    m, s = divmod(rem, 60)
    if h > 0:
        return f'{h} giờ {m} phút {s} giây'
    if m > 0:
        return f'{m} phút {s} giây'
    return f'{s} giây'


def _dl_log(msg):
    print(f'[DataLogger] {msg}', flush=True)


def _dl_get_descendant_pids(pid):
    """Quét /proc lấy toàn bộ PID con/cháu — cùng pattern với carla_node.py /
    ros_bridge_node.py / navigation_stack_node.py. data_logger.py là script đơn
    (không tự fork thêm gì) nên bình thường không cần, chỉ giữ làm lớp dự phòng."""
    descendants = []
    try:
        with open(f'/proc/{pid}/task/{pid}/children') as f:
            direct = [int(p) for p in f.read().split()]
    except (FileNotFoundError, ProcessLookupError, PermissionError):
        direct = []
    for child_pid in direct:
        descendants.append(child_pid)
        descendants.extend(_dl_get_descendant_pids(child_pid))
    return descendants


def is_running():
    with _dl_lock:
        return _dl_proc is not None and _dl_proc.poll() is None


def start_data_logger():
    """(Re)khởi động data_logger.py. Nếu đang có phiên ghi cũ chạy, dừng sạch
    trước (đóng CSV đúng cách qua SIGINT) rồi mới bắt đầu phiên mới — đảm bảo
    mỗi lần spawn xe là 1 phiên ghi log riêng biệt, không lẫn dữ liệu."""
    global _dl_proc, _dl_recording_start

    with _dl_lock:
        old_proc = _dl_proc
    if old_proc is not None and old_proc.poll() is None:
        _dl_log('Đã có phiên ghi cũ đang chạy — dừng sạch trước khi bắt đầu phiên mới.')
        stop_data_logger(wait_seconds=5, block=True)

    with _dl_lock:
        os.makedirs(DATA_RECORD_DIR, exist_ok=True)
        # Xoá file waypoints của phiên TRƯỚC (nếu có) — tránh lẫn toạ độ mission
        # cũ vào phiên ghi mới nếu chưa kịp Chốt hành trình nào trong phiên này.
        try:
            if os.path.isfile(TEMP_WAYPOINTS_PATH):
                os.remove(TEMP_WAYPOINTS_PATH)
        except Exception:
            pass
        _dl_log(f'Đang khởi chạy — ghi ra {TEMP_CSV_PATH}')
        try:
            _dl_proc = subprocess.Popen(
                [sys.executable, DATA_LOGGER_SCRIPT, TEMP_CSV_PATH],
                start_new_session=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except Exception as e:
            _dl_log(f'✗ Lỗi khởi chạy: {e}')
            return False, str(e)

        _dl_recording_start = datetime.now()  # mốc "bắt đầu ghi" cho phiên này — dùng khi export tính tổng thời gian
        _dl_log(f'✓ Đã chạy — PID={_dl_proc.pid}')
        return True, 'started'


def stop_data_logger(wait_seconds=10, block=True):
    """Dừng data logger đang chạy (nếu có). Dùng SIGINT (không phải SIGTERM/kill
    thẳng) để node tự chạy vào nhánh KeyboardInterrupt → destroy_node() → đóng
    file CSV đúng cách (flush + close), tránh mất/hỏng dữ liệu dòng cuối."""
    global _dl_proc
    with _dl_lock:
        proc = _dl_proc
        if proc is None or proc.poll() is not None:
            _dl_proc = None
            return False, 'Data logger không chạy'
        _dl_log(f'Đang dừng (PID={proc.pid}) — gửi SIGINT để đóng CSV đúng cách...')
        try:
            os.kill(proc.pid, signal.SIGINT)
        except ProcessLookupError:
            pass

    def _wait_and_force_kill():
        global _dl_proc
        try:
            proc.wait(timeout=wait_seconds)
            _dl_log('✓ Đã thoát sạch (CSV đã đóng đúng cách).')
        except subprocess.TimeoutExpired:
            _dl_log(f'✗ Không thoát kịp trong {wait_seconds}s — dọn cưỡng bức (có thể mất dòng CSV cuối).')
            descendants = _dl_get_descendant_pids(proc.pid)
            for pid in reversed(descendants):
                try:
                    os.kill(pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
            try:
                proc.kill()
                proc.wait(timeout=5)
            except Exception:
                pass
        except Exception:
            pass
        with _dl_lock:
            if _dl_proc is proc:
                _dl_proc = None

    if block:
        _wait_and_force_kill()
    else:
        threading.Thread(target=_wait_and_force_kill, daemon=True).start()

    return True, 'stopping'


def export_recorded_data():
    """Copy file CSV tạm (data_record/carla_data_local.csv) sang
    recorded_data/carla_data_<timestamp>.csv để giữ lại vĩnh viễn. Không đụng
    tới file tạm — logger (nếu đang chạy) vẫn tiếp tục ghi bình thường.

    "Kết thúc ghi" ở đây là THỜI ĐIỂM XUẤT (export), không phải lúc logger thật
    sự dừng — vì có thể export ngay trong lúc xe vẫn đang chạy/đang ghi tiếp.
    Trả về dict info để frontend hiển thị modal thông báo."""
    if not os.path.isfile(TEMP_CSV_PATH):
        return False, 'Chưa có dữ liệu nào được ghi (file tạm không tồn tại)', None

    os.makedirs(RECORDED_DATA_DIR, exist_ok=True)
    export_time = datetime.now()
    ts = export_time.strftime('%Y%m%d_%H%M%S')
    dest_name = f'carla_data_{ts}.csv'
    dest_path = os.path.join(RECORDED_DATA_DIR, dest_name)
    try:
        shutil.copyfile(TEMP_CSV_PATH, dest_path)
    except Exception as e:
        return False, str(e), None

    # Copy kèm file toạ độ mission (nếu navigate_node.py đã từng ghi) — đặt tên
    # theo cặp với CSV (carla_data_<ts>.csv ↔ carla_data_<ts>_waypoints.json) để
    # plot_carla_data.py tự tìm và đánh dấu đúng điểm đích đã yêu cầu. Không bắt
    # buộc phải có — nếu chưa từng Chốt hành trình nào thì bỏ qua, không lỗi.
    if os.path.isfile(TEMP_WAYPOINTS_PATH):
        try:
            shutil.copyfile(
                TEMP_WAYPOINTS_PATH,
                os.path.join(RECORDED_DATA_DIR, f'carla_data_{ts}_waypoints.json'),
            )
        except Exception as e:
            _dl_log(f'⚠ Không copy được file waypoints kèm theo: {e}')

    with _dl_lock:
        start_time = _dl_recording_start

    if start_time is not None:
        duration_sec = (export_time - start_time).total_seconds()
        info = {
            'filename':    dest_name,
            'path':        f'WebRobot/web/recorded_data/{dest_name}',
            'start_time':  start_time.strftime('%H:%M:%S %d/%m/%Y'),
            'end_time':    export_time.strftime('%H:%M:%S %d/%m/%Y'),
            'duration':    _dl_format_duration(duration_sec),
        }
    else:
        # Trường hợp hiếm: export được gọi mà chưa từng có phiên start_data_logger()
        # nào trong lần chạy Flask này (vd: Flask restart giữa chừng) — vẫn xuất
        # file bình thường, chỉ là không biết mốc bắt đầu thật sự.
        info = {
            'filename':   dest_name,
            'path':       f'WebRobot/web/recorded_data/{dest_name}',
            'start_time': 'không xác định (Flask có thể đã khởi động lại)',
            'end_time':   export_time.strftime('%H:%M:%S %d/%m/%Y'),
            'duration':   'không xác định',
        }

    _dl_log(f'✓ Đã xuất dữ liệu → {dest_path}')
    return True, 'exported', info


# ── Routes ─────────────────────────────────────────────────────────────────────
@data_logger_bp.route('/api/datalogger/start', methods=['POST'])
def api_data_logger_start():
    ok, msg = start_data_logger()
    if not ok:
        return jsonify({'ok': False, 'error': msg}), 400
    return jsonify({'ok': True})


@data_logger_bp.route('/api/datalogger/stop', methods=['POST'])
def api_data_logger_stop():
    ok, msg = stop_data_logger(wait_seconds=10, block=False)
    if not ok:
        return jsonify({'ok': False, 'error': msg}), 400
    return jsonify({'ok': True})


@data_logger_bp.route('/api/datalogger/status', methods=['GET'])
def api_data_logger_status():
    return jsonify({'ok': True, 'running': is_running()})


@data_logger_bp.route('/api/datalogger/export', methods=['POST'])
def api_data_logger_export():
    ok, msg, info = export_recorded_data()
    if not ok:
        return jsonify({'ok': False, 'error': msg}), 400
    return jsonify({'ok': True, **info})


def list_recorded_files():
    """Liệt kê các file .csv đã export trong recorded_data/, MỚI NHẤT lên đầu
    (tên file có timestamp nên sort chuỗi giảm dần = sort theo thời gian)."""
    if not os.path.isdir(RECORDED_DATA_DIR):
        return []

    items = []
    for fname in os.listdir(RECORDED_DATA_DIR):
        if not _EXPORTED_FNAME_RE.match(fname):
            continue
        full_path = os.path.join(RECORDED_DATA_DIR, fname)
        try:
            size_bytes = os.path.getsize(full_path)
        except OSError:
            size_bytes = 0

        # carla_data_20260708_150000.csv -> "15:00:00 08/07/2026"
        m = re.match(r'^carla_data_(\d{4})(\d{2})(\d{2})_(\d{2})(\d{2})(\d{2})\.csv$', fname)
        if m:
            yyyy, mm, dd, HH, MM, SS = m.groups()
            display_name = f'{HH}:{MM}:{SS} {dd}/{mm}/{yyyy}'
        else:
            display_name = fname

        size_kb = size_bytes / 1024.0
        size_label = f'{size_kb:.1f} KB' if size_kb < 1024 else f'{size_kb / 1024:.1f} MB'

        items.append({
            'filename': fname,
            'display_name': display_name,
            'size_label': size_label,
        })

    items.sort(key=lambda it: it['filename'], reverse=True)  # mới nhất lên đầu
    return items


def launch_plotter(filename):
    """Kích hoạt python3 plot_carla_data.py <đường dẫn file> dưới dạng subprocess
    NỀN, không chờ (fire-and-forget) — cửa sổ matplotlib sẽ mở ra trên MÁY ĐANG
    CHẠY Flask (không phải trên trình duyệt), tồn tại độc lập tới khi người dùng
    tự đóng, không bị ảnh hưởng nếu tắt trang web."""
    if not _EXPORTED_FNAME_RE.match(filename or ''):
        return False, 'Tên file không hợp lệ'

    full_path = os.path.join(RECORDED_DATA_DIR, filename)
    if not os.path.isfile(full_path):
        return False, f'Không tìm thấy file: {filename}'

    if not os.path.isfile(PLOT_SCRIPT_PATH):
        return False, f'Không tìm thấy plot_carla_data.py tại: {PLOT_SCRIPT_PATH}'

    try:
        subprocess.Popen(
            [sys.executable, PLOT_SCRIPT_PATH, full_path],
            start_new_session=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except Exception as e:
        return False, str(e)

    _dl_log(f'Đã mở cửa sổ đồ thị cho: {filename}')
    return True, 'launched'


@data_logger_bp.route('/api/datalogger/files', methods=['GET'])
def api_data_logger_files():
    return jsonify({'ok': True, 'files': list_recorded_files()})


@data_logger_bp.route('/api/datalogger/plot', methods=['POST'])
def api_data_logger_plot():
    data = request.get_json(force=True, silent=True) or {}
    filename = data.get('filename', '')
    ok, msg = launch_plotter(filename)
    if not ok:
        return jsonify({'ok': False, 'error': msg}), 400
    return jsonify({'ok': True})