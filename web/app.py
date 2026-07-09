#!/usr/bin/env python3
import json
import logging
import os
import re
import signal
import sys
import threading
import time

from flask import Flask, jsonify, render_template, request, send_from_directory, Response, stream_with_context
from flask_sock import Sock
from werkzeug.utils import secure_filename
from api_ros.ros_status_api import ros_status_bp, start_bridge_checker, stop_bridge_checker
from api_ros.navigate_node  import start_navigate_node, stop_navigate_node, send_navigate_command
from api_ros.odom_node      import start_odom_node, stop_odom_node, register_ws_client, unregister_ws_client
from api_ros.speedometer_node import (
    start_speedometer_node, stop_speedometer_node,
    register_ws_client as register_speedometer_ws_client,
    unregister_ws_client as unregister_speedometer_ws_client,
)
from api_ros.spawn_car_node import spawn_car_bp, stop_spawn_car_node, trigger_spawn_points_fetch
from api_ros.carla_node     import carla_bp, stop_carla
from api_ros.ros_bridge_node import ros_bridge_bp, stop_ros_bridge
from api_ros.navigation_stack_node import navstack_bp, stop_navigation_stack
from api_ros.data_logger import data_logger_bp, stop_data_logger
from api_ros.route_preview import route_preview_bp

app  = Flask(__name__)
sock = Sock(app)
app.register_blueprint(ros_status_bp)
app.register_blueprint(spawn_car_bp)
app.register_blueprint(carla_bp)
app.register_blueprint(ros_bridge_bp)
app.register_blueprint(navstack_bp)
app.register_blueprint(data_logger_bp)
app.register_blueprint(route_preview_bp)

# ── Ẩn log poll /api/ros/status (spam mỗi 6s) ────────────────────────────────
class _SuppressRosStatusLog(logging.Filter):
    def filter(self, record):
        return 'GET /api/ros/status' not in record.getMessage()

logging.getLogger('werkzeug').addFilter(_SuppressRosStatusLog())

# ── Cấu hình thư mục ──────────────────────────────────────────────────────────
BASE_DIR      = os.path.dirname(os.path.abspath(__file__))
MAPS_DIR      = os.path.join(BASE_DIR, 'maps')
TRACKING_LOG  = os.path.join(BASE_DIR, 'livetracking.log')
ODOM_LOG      = os.path.join(BASE_DIR, 'odom_check_log.txt')
app.config['MAX_CONTENT_LENGTH'] = 50 * 1024 * 1024  # 50 MB

ALLOWED_IMAGE_EXTENSIONS = {'png', 'jpg', 'jpeg', 'gif', 'bmp', 'webp'}

# ── ROS nodes ─────────────────────────────────────────────────────────────────
# bridge_check_node.py : subprocess — kiểm tra carla_ros_bridge, push heartbeat
# navigate_node.py     : subprocess — nhận GO/STOP từ stdin, publish ROS topics
# odom_node.py         : thread trong Flask  — subscribe /carla/markers, WebSocket broadcast


# ── Helpers ────────────────────────────────────────────────────────────────────
def _allowed_image(filename: str) -> bool:
    return '.' in filename and filename.rsplit('.', 1)[1].lower() in ALLOWED_IMAGE_EXTENSIONS


def _sanitize_name(name: str) -> str:
    """Chuyển tên bản đồ thành tên thư mục an toàn: khoảng trắng → _, bỏ ký tự đặc biệt."""
    name = name.strip()
    name = re.sub(r'[\s]+', '_', name)
    name = re.sub(r'[^\w\-.]', '', name)
    return name or 'map'


def _get_next_index() -> int:
    """Tìm STT tiếp theo dựa trên các thư mục đã tồn tại trong MAPS_DIR."""
    if not os.path.isdir(MAPS_DIR):
        return 1
    indices = []
    for d in os.listdir(MAPS_DIR):
        m = re.match(r'^(\d+)\.', d)
        if m and os.path.isdir(os.path.join(MAPS_DIR, d)):
            indices.append(int(m.group(1)))
    return max(indices, default=0) + 1


def _list_maps() -> list:
    """Trả về danh sách các bản đồ đã lưu, sắp xếp theo STT."""
    if not os.path.isdir(MAPS_DIR):
        return []
    maps = []
    for d in sorted(os.listdir(MAPS_DIR)):
        m = re.match(r'^(\d+)\.(.+)$', d)
        if not m:
            continue
        folder_path = os.path.join(MAPS_DIR, d)
        if not os.path.isdir(folder_path):
            continue
        idx = int(m.group(1))
        raw_name = m.group(2).replace('_', ' ')
        # Tìm file ảnh
        image_file = None
        for fname in os.listdir(folder_path):
            if fname.startswith('image.') and fname.rsplit('.', 1)[-1].lower() in ALLOWED_IMAGE_EXTENSIONS:
                image_file = fname
                break
        has_waypoint = os.path.isfile(os.path.join(folder_path, 'waypoints.json'))
        maps.append({
            'id':          idx,
            'folder':      d,
            'name':        raw_name,
            'has_image':   image_file is not None,
            'has_waypoint': has_waypoint,
            'image_url':   f'/maps/{d}/{image_file}' if image_file else None,
            'waypoint_url': f'/maps/{d}/waypoints.json' if has_waypoint else None,
        })
    maps.sort(key=lambda x: x['id'])
    return maps


# ── Routes ─────────────────────────────────────────────────────────────────────
@app.route('/')
def home():
    """Trang chính — quy trình 4 bước vận hành hệ thống."""
    return render_template('home.html')


@app.route('/control')
def index():
    """Trang điều khiển xe mô phỏng (bước 3 trong quy trình)."""
    return render_template('index.html')


# Serve file tĩnh từ thư mục maps/
@app.route('/maps/<path:filepath>')
def serve_map_file(filepath):
    directory = os.path.join(MAPS_DIR, os.path.dirname(filepath))
    filename  = os.path.basename(filepath)
    return send_from_directory(directory, filename)


# ── Map Manager API ────────────────────────────────────────────────────────────

@app.route('/api/maps', methods=['GET'])
def api_list_maps():
    """Lấy danh sách tất cả bản đồ đã lưu."""
    return jsonify({'ok': True, 'maps': _list_maps()})


@app.route('/api/maps/upload', methods=['POST'])
def api_upload_map():
    """
    Upload bản đồ mới.
    Form fields:
      - map_name  (str)       : tên bản đồ
      - waypoint  (file .json): file waypoint
      - image     (file ảnh)  : ảnh bản đồ
    """
    # --- Validate ---
    map_name = request.form.get('map_name', '').strip()
    if not map_name:
        return jsonify({'ok': False, 'error': 'Chưa nhập tên bản đồ'}), 400

    if 'waypoint' not in request.files or request.files['waypoint'].filename == '':
        return jsonify({'ok': False, 'error': 'Chưa chọn file waypoint (.json)'}), 400

    if 'image' not in request.files or request.files['image'].filename == '':
        return jsonify({'ok': False, 'error': 'Chưa chọn file ảnh bản đồ'}), 400

    wp_file  = request.files['waypoint']
    img_file = request.files['image']

    if not wp_file.filename.lower().endswith('.json'):
        return jsonify({'ok': False, 'error': 'File waypoint phải có định dạng .json'}), 400

    if not _allowed_image(img_file.filename):
        return jsonify({'ok': False, 'error': 'File ảnh không hợp lệ (png/jpg/jpeg/gif/bmp/webp)'}), 400

    # --- Tạo thư mục ---
    idx           = _get_next_index()
    safe_name     = _sanitize_name(map_name)
    folder_name   = f'{idx}.{safe_name}'
    folder_path   = os.path.join(MAPS_DIR, folder_name)
    os.makedirs(folder_path, exist_ok=True)

    # --- Lưu waypoint ---
    wp_file.save(os.path.join(folder_path, 'waypoints.json'))

    # --- Lưu ảnh (giữ đuôi gốc, đổi tên thành image.<ext>) ---
    img_ext = img_file.filename.rsplit('.', 1)[1].lower()
    img_file.save(os.path.join(folder_path, f'image.{img_ext}'))

    return jsonify({
        'ok':          True,
        'id':          idx,
        'folder':      folder_name,
        'name':        map_name,
        'image_url':   f'/maps/{folder_name}/image.{img_ext}',
        'waypoint_url': f'/maps/{folder_name}/waypoints.json',
    })


@app.route('/api/maps/<int:map_id>/load', methods=['GET'])
def api_load_map(map_id: int):
    """
    Trả về thông tin để load bản đồ theo ID (STT).
    Đồng thời là điểm "map vừa load thành công" — trigger lấy spawn point mới
    từ CARLA (nếu bridge đang kết nối). Được app.js gọi mỗi khi người dùng
    chọn/load một bản đồ trong Map Manager.
    """
    maps = _list_maps()
    found = next((m for m in maps if m['id'] == map_id), None)
    if not found:
        return jsonify({'ok': False, 'error': f'Không tìm thấy bản đồ #{map_id}'}), 404

    trigger_spawn_points_fetch()

    return jsonify({'ok': True, **found})


# ── Navigation API ─────────────────────────────────────────────────────────────

@sock.route('/ws/odom_stream')
def ws_odom_stream(ws):
    """WebSocket endpoint — push odom frame tới browser mỗi khi có dữ liệu mới."""
    register_ws_client(ws)
    try:
        while True:
            # Giữ connection sống — browser không cần gửi gì, chỉ nhận
            ws.receive(timeout=60)
    except Exception:
        pass
    finally:
        unregister_ws_client(ws)
# /api/odom/stream sẽ được thêm lại khi có odom_node.py


@sock.route('/ws/speedometer_stream')
def ws_speedometer_stream(ws):
    """WebSocket endpoint — push tốc độ thực (km/h, quy đổi từ m/s) tới browser
    mỗi khi speedometer_node.py có dữ liệu mới."""
    register_speedometer_ws_client(ws)
    try:
        while True:
            # Giữ connection sống — browser không cần gửi gì, chỉ nhận
            ws.receive(timeout=60)
    except Exception:
        pass
    finally:
        unregister_speedometer_ws_client(ws)


@app.route('/api/navigate', methods=['POST'])
def api_navigate():
    """
    Nhận lệnh từ browser, forward xuống navigate_node.py qua stdin.
    Body JSON:
      { "x": float, "y": float, "speed_kmh": float }  → GO
      { "stop": true }                                 → STOP
    """
    data = request.get_json(force=True, silent=True) or {}

    if data.get('stop'):
        send_navigate_command('STOP')
        return jsonify({'ok': True, 'stopped': True})

    try:
        x         = float(data['x'])
        y         = float(data['y'])
        speed_kmh = float(data['speed_kmh'])
    except (KeyError, ValueError, TypeError):
        return jsonify({'ok': False, 'error': 'Cần x, y (số) và speed_kmh (số)'}), 400

    if speed_kmh < 0:
        return jsonify({'ok': False, 'error': 'speed_kmh phải >= 0'}), 400

    send_navigate_command(f'GO {x} {y} {speed_kmh}')
    return jsonify({'ok': True, 'x': x, 'y': y, 'speed_kmh': speed_kmh})


@app.route('/api/mission/commit', methods=['POST'])
def api_mission_commit():
    """
    "Chốt hành trình" — nhận toàn bộ hàng đợi điểm (queue_pending_points, đã được
    quản lý hoàn toàn phía browser) và forward xuống navigate_node.py dưới dạng
    1 lệnh MISSION duy nhất. Node sẽ tự lo phần đi lần lượt qua từng điểm.
    Body JSON:
      { "points": [{"x": float, "y": float}, ...], "speed_kmh": float }
    """
    data = request.get_json(force=True, silent=True) or {}

    try:
        speed_kmh = float(data['speed_kmh'])
    except (KeyError, ValueError, TypeError):
        return jsonify({'ok': False, 'error': 'Cần speed_kmh (số)'}), 400

    if speed_kmh < 0:
        return jsonify({'ok': False, 'error': 'speed_kmh phải >= 0'}), 400

    points = data.get('points')
    if not isinstance(points, list) or not points:
        return jsonify({'ok': False, 'error': 'Cần ít nhất 1 điểm trong hành trình'}), 400

    coords = []
    for i, p in enumerate(points):
        try:
            coords.append(float(p['x']))
            coords.append(float(p['y']))
        except (KeyError, ValueError, TypeError):
            return jsonify({'ok': False, 'error': f'Điểm thứ {i + 1} thiếu x/y hợp lệ'}), 400

    cmd = 'MISSION ' + ' '.join(str(v) for v in [speed_kmh] + coords)
    send_navigate_command(cmd)

    return jsonify({'ok': True, 'count': len(points), 'speed_kmh': speed_kmh})


# ── Bootstrap ──────────────────────────────────────────────────────────────────
_cleanup_done = False


def _cleanup():
    """Dọn dẹp tất cả node/subprocess nền — kể cả CARLA. Idempotent: chỉ chạy 1 lần
    dù được gọi từ finally (Ctrl+C / SIGINT) hay từ signal handler (SIGTERM)."""
    global _cleanup_done
    if _cleanup_done:
        return
    _cleanup_done = True
    print('\n[Web] Đang đóng Web Server...')
    stop_speedometer_node()  # dọn trước — khởi động sau cùng nên dọn trước tiên (LIFO)
    stop_odom_node()
    stop_navigate_node()
    stop_bridge_checker()
    stop_spawn_car_node()
    stop_navigation_stack(wait_seconds=10, block=True)  # waypoint_publisher/ad_agent/navigation_hmi
    stop_data_logger(wait_seconds=10, block=True)        # đóng CSV đúng cách trước khi thoát hẳn
    # Tắt CARLA UE4 TRƯỚC rồi mới tới ROS Bridge — không làm ngược lại nữa.
    # Lý do đổi: cố "dọn êm" ROS Bridge trong lúc CARLA UE4 vẫn còn sống (đợi
    # nó tự disconnect/destroy actor đúng trình tự) hay bị treo/timeout do
    # CARLA phản hồi chậm khi đang tự thoát dở dang → dọn không sạch, để lại
    # tiến trình mồ côi. Tắt CARLA trước (double-tap Ctrl+C, xem carla_node.py)
    # rồi mới tắt ROS Bridge: lúc này bridge chỉ còn phát hiện mất kết nối và
    # thoát nhanh (hoặc bị SIGTERM/SIGKILL ép thoát), không còn phải chờ CARLA.
    stop_carla(wait_seconds=10, block=True)
    stop_ros_bridge(wait_seconds=10, block=True)
    print('[Web] Đã dọn dẹp xong.')


def _handle_sigterm(signum, frame):
    """Bắt SIGTERM (vd: `kill`, `systemctl stop`) — Ctrl+C (SIGINT) đã được xử lý
    tự nhiên qua KeyboardInterrupt/finally bên dưới."""
    _cleanup()
    sys.exit(0)


if __name__ == '__main__':
    os.makedirs(MAPS_DIR, exist_ok=True)
    signal.signal(signal.SIGTERM, _handle_sigterm)

    start_bridge_checker()
    start_navigate_node()
    start_odom_node(ODOM_LOG)
    start_speedometer_node()  # khởi động sau cùng — ưu tiên thấp hơn các node khác

    try:
        app.run(host='0.0.0.0', port=5000, debug=True, use_reloader=False, threaded=True)
    finally:
        _cleanup()