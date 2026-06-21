#!/usr/bin/env python3
import json
import math
import os
import queue
import re
import threading
import time

from flask import Flask, jsonify, render_template, request, send_from_directory, Response, stream_with_context, Response, stream_with_context
from werkzeug.utils import secure_filename

app = Flask(__name__)

# ── Cấu hình thư mục ──────────────────────────────────────────────────────────
BASE_DIR      = os.path.dirname(os.path.abspath(__file__))
MAPS_DIR      = os.path.join(BASE_DIR, 'maps')
TRACKING_LOG  = os.path.join(BASE_DIR, 'livetracking.log')
app.config['MAX_CONTENT_LENGTH'] = 50 * 1024 * 1024  # 50 MB

ALLOWED_IMAGE_EXTENSIONS = {'png', 'jpg', 'jpeg', 'gif', 'bmp', 'webp'}

# ── Optional ROS integration ───────────────────────────────────────────────────
ROS_AVAILABLE = False
_ros_node     = None
_ros_executor = None
_ros_thread   = None

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from geometry_msgs.msg import PoseStamped
    from visualization_msgs.msg import MarkerArray
    from std_msgs.msg import Float64
    from nav_msgs.msg import Odometry
    ROS_AVAILABLE = True
except ImportError:
    pass


if ROS_AVAILABLE:
    class RobotROSNode(Node):
        def __init__(self):
            super().__init__('robot_web_node')

            self.declare_parameter('role_name', 'hero')
            self.role_name = (
                self.get_parameter('role_name').get_parameter_value().string_value
            )

            self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

            speed_qos = QoSProfile(depth=10)
            speed_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.speed_pub = self.create_publisher(
                Float64,
                f'/carla/{self.role_name}/target_speed',
                speed_qos,
            )

            road_qos = QoSProfile(
                depth=1,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.road_sub = self.create_subscription(
                MarkerArray,
                'carla_road_network',
                self._road_callback,
                road_qos,
            )

            self._markers = []
            self._markers_lock = threading.Lock()

            # ── Vehicle marker subscriber (/carla/markers) ─────────────────
            # Nguồn chính: vị trí + heading, publish liên tục kể cả khi đứng yên
            self._vehicle_marker      = None
            self._vehicle_marker_lock = threading.Lock()
            self._odom_event          = threading.Event()   # báo SSE ngay khi có frame mới
            self.vehicle_marker_sub   = self.create_subscription(
                MarkerArray,
                '/carla/markers',
                self._vehicle_marker_callback,
                1,
            )

            # ── Odometry subscriber ────────────────────────────────────────
            # Chỉ lấy twist.linear (tốc độ); pose lấy từ /carla/markers
            self._odom_speed      = None   # (vx, vy, vz)
            self._odom_speed_lock = threading.Lock()
            self.odom_sub = self.create_subscription(
                Odometry,
                f'/carla/{self.role_name}/odometry',
                self._odom_callback,
                1,
            )

            # ── Log writer thread (không block callback) ───────────────────
            self._log_queue = queue.Queue()
            threading.Thread(target=self._log_writer, daemon=True).start()

            self.get_logger().info('RobotROSNode started.')

        def _road_callback(self, msg: MarkerArray):
            with self._markers_lock:
                self._markers = list(msg.markers)

        def _vehicle_marker_callback(self, msg: MarkerArray):
            """Cập nhật vị trí + heading từ /carla/markers, push SSE ngay lập tức."""
            if not msg.markers:
                return
            marker = msg.markers[0]
            with self._vehicle_marker_lock:
                self._vehicle_marker = marker
            # Báo SSE có frame mới — không cần poll nữa
            self._odom_event.set()
            # Đẩy log vào queue để thread riêng ghi, không block callback
            p = marker.pose.position
            o = marker.pose.orientation
            ts = time.strftime('%Y-%m-%d %H:%M:%S')
            self._log_queue.put(
                f"{ts} | x={p.x:.4f} y={p.y:.4f} z={p.z:.4f}"
                f" | qx={o.x:.6f} qy={o.y:.6f} qz={o.z:.6f} qw={o.w:.6f}\n"
            )

        def _odom_callback(self, msg: Odometry):
            """Chỉ lưu twist.linear để tính tốc độ."""
            l = msg.twist.twist.linear
            with self._odom_speed_lock:
                self._odom_speed = (l.x, l.y, l.z)

        def _log_writer(self):
            """Thread riêng ghi livetracking.log — không block ROS callback."""
            with open(TRACKING_LOG, 'a') as f:
                while True:
                    line = self._log_queue.get()
                    f.write(line)
                    f.flush()

        def get_odom(self):
            """Trả về dict tracking mới nhất:
            - pose từ /carla/markers (luôn có, kể cả khi đứng yên)
            - tốc độ từ /carla/hero/odometry (0.0 nếu xe đứng yên)
            """
            with self._vehicle_marker_lock:
                if self._vehicle_marker is None:
                    return None
                p = self._vehicle_marker.pose.position
                o = self._vehicle_marker.pose.orientation
            with self._odom_speed_lock:
                spd = self._odom_speed
            vx, vy, vz = spd if spd is not None else (0.0, 0.0, 0.0)
            return {
                'x':  p.x,  'y':  p.y,  'z':  p.z,
                'qx': o.x,  'qy': o.y,  'qz': o.z, 'qw': o.w,
                'vx': vx,   'vy': vy,   'vz': vz,
            }

        def _find_nearest_marker(self, x: float, y: float):
            with self._markers_lock:
                if not self._markers:
                    return None, None
                nearest, min_dist = None, float('inf')
                for m in self._markers:
                    d = math.hypot(m.pose.position.x - x, m.pose.position.y - y)
                    if d < min_dist:
                        min_dist, nearest = d, m
                return nearest, min_dist

        def send_goal(self, x: float, y: float):
            nearest, dist = self._find_nearest_marker(x, y)

            msg = PoseStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.orientation.w = 1.0

            if nearest is not None:
                msg.pose.position.x = nearest.pose.position.x
                msg.pose.position.y = nearest.pose.position.y
                msg.pose.position.z = nearest.pose.position.z
                snap = {
                    'snapped': True,
                    'wx': nearest.pose.position.x,
                    'wy': nearest.pose.position.y,
                    'dist': round(dist, 4),
                }
            else:
                msg.pose.position.x = x
                msg.pose.position.y = y
                msg.pose.position.z = 0.0
                snap = {'snapped': False}

            self.goal_pub.publish(msg)
            return snap

        def send_speed(self, speed_kmh: float):
            speed_mps = speed_kmh / 3.6
            msg = Float64()
            msg.data = speed_mps
            self.speed_pub.publish(msg)
            return round(speed_mps, 4)


def _start_ros():
    global _ros_node, _ros_executor, _ros_thread
    try:
        rclpy.init()
        _ros_node = RobotROSNode()
        _ros_executor = rclpy.executors.SingleThreadedExecutor()
        _ros_executor.add_node(_ros_node)
        _ros_thread = threading.Thread(target=_ros_executor.spin, daemon=True)
        _ros_thread.start()
        print('[ROS] Node started successfully.')
    except Exception as e:
        print(f'[ROS] Failed to start node: {e}')


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
def index():
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
    """Trả về thông tin để load bản đồ theo ID (STT)."""
    maps = _list_maps()
    found = next((m for m in maps if m['id'] == map_id), None)
    if not found:
        return jsonify({'ok': False, 'error': f'Không tìm thấy bản đồ #{map_id}'}), 404
    return jsonify({'ok': True, **found})


# ── Navigation API ─────────────────────────────────────────────────────────────

@app.route('/api/ros-status', methods=['GET'])
def api_ros_status():
    """
    Kiểm tra /carla_ros_bridge node có đang chạy không.
    Browser poll endpoint này định kỳ để hiện trạng thái Connected/Disconnected.
    """
    if not ROS_AVAILABLE or _ros_node is None:
        return jsonify({'ok': True, 'running': False, 'reason': 'ROS node not initialized'})
    try:
        node_names = _ros_node.get_node_names()
        running = 'carla_ros_bridge' in node_names
        return jsonify({'ok': True, 'running': running, 'nodes': node_names})
    except Exception as e:
        return jsonify({'ok': True, 'running': False, 'reason': str(e)})


@app.route('/api/odom', methods=['GET'])
def api_odom():
    """Trả về odom mới nhất từ /carla/hero/odometry."""
    if _ros_node is None:
        return jsonify({'ok': False, 'error': 'ROS node not running'}), 503
    data = _ros_node.get_odom()
    if data is None:
        return jsonify({'ok': False, 'error': 'No odom data yet'}), 204
    return jsonify({'ok': True, **data})

@app.route('/api/odom/stream')
def api_odom_stream():
    """SSE endpoint — push ngay khi /carla/markers callback kích hoạt Event.
    Không poll, không delay thêm — latency chỉ còn đúng 1 network round-trip.
    """
    def generate():
        while True:
            if _ros_node is None:
                time.sleep(0.1)
                continue
            # Block đến khi callback báo có frame mới, timeout 1s để không treo mãi
            triggered = _ros_node._odom_event.wait(timeout=1.0)
            if not triggered:
                continue   # timeout, thử lại
            _ros_node._odom_event.clear()
            data = _ros_node.get_odom()
            if data is not None:
                yield f"data: {json.dumps(data)}\n\n"

    return Response(
        stream_with_context(generate()),
        mimetype='text/event-stream',
        headers={
            'Cache-Control':     'no-cache',
            'X-Accel-Buffering': 'no',
        }
    )


@app.route('/api/send-goal', methods=['POST'])
def api_send_goal():
    data = request.get_json(force=True, silent=True) or {}
    try:
        x = float(data['x'])
        y = float(data['y'])
    except (KeyError, ValueError, TypeError):
        return jsonify({'ok': False, 'error': 'x và y phải là số hợp lệ'}), 400

    if _ros_node is not None:
        try:
            snap = _ros_node.send_goal(x, y)
            return jsonify({'ok': True, 'ros': True, 'x': x, 'y': y, **snap})
        except Exception as e:
            return jsonify({'ok': False, 'error': str(e)}), 500

    return jsonify({'ok': True, 'ros': False, 'snapped': False, 'x': x, 'y': y})


@app.route('/api/send-speed', methods=['POST'])
def api_send_speed():
    data = request.get_json(force=True, silent=True) or {}
    try:
        speed_kmh = float(data['speed_kmh'])
    except (KeyError, ValueError, TypeError):
        return jsonify({'ok': False, 'error': 'speed_kmh phải là số hợp lệ'}), 400

    if speed_kmh < 0:
        return jsonify({'ok': False, 'error': 'Tốc độ phải >= 0'}), 400

    speed_mps = round(speed_kmh / 3.6, 4)

    if _ros_node is not None:
        try:
            speed_mps = _ros_node.send_speed(speed_kmh)
            return jsonify({'ok': True, 'ros': True, 'speed_kmh': speed_kmh, 'speed_mps': speed_mps})
        except Exception as e:
            return jsonify({'ok': False, 'error': str(e)}), 500

    return jsonify({'ok': True, 'ros': False, 'speed_kmh': speed_kmh, 'speed_mps': speed_mps})


@app.route('/api/navigate', methods=['POST'])
def api_navigate():
    """
    Gộp goal + speed vào 1 lần gọi.
    Body JSON:
      { "x": float, "y": float, "speed_kmh": float }   → Go
      { "stop": true }                                  → STOP (speed=0, goal=(0,0,0))
    """
    data = request.get_json(force=True, silent=True) or {}

    # ── STOP ──────────────────────────────────────────────────────────────────
    if data.get('stop'):
        result = {'ok': True, 'stopped': True}
        if _ros_node is not None:
            try:
                _ros_node.send_speed(0.0)
                # Gửi goal tại vị trí hiện tại để hủy đích cũ
                odom = _ros_node.get_odom()
                cx = odom['x'] if odom else 0.0
                cy = odom['y'] if odom else 0.0
                _ros_node.send_goal(cx, cy)
                result['ros'] = True
            except Exception as e:
                result['ros_error'] = str(e)
        return jsonify(result)

    # ── GO ────────────────────────────────────────────────────────────────────
    try:
        x         = float(data['x'])
        y         = float(data['y'])
        speed_kmh = float(data['speed_kmh'])
    except (KeyError, ValueError, TypeError):
        return jsonify({'ok': False, 'error': 'Cần cung cấp x, y (số) và speed_kmh (số)'}), 400

    if speed_kmh < 0:
        return jsonify({'ok': False, 'error': 'Tốc độ phải >= 0'}), 400

    result = {'ok': True, 'x': x, 'y': y, 'speed_kmh': speed_kmh}

    if _ros_node is not None:
        try:
            snap      = _ros_node.send_goal(x, y)
            speed_mps = _ros_node.send_speed(speed_kmh)
            result.update({'ros': True, 'speed_mps': speed_mps, **snap})
        except Exception as e:
            return jsonify({'ok': False, 'error': str(e)}), 500
    else:
        result.update({'ros': False, 'snapped': False, 'speed_mps': round(speed_kmh / 3.6, 4)})

    return jsonify(result)


# ── Bootstrap ──────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    os.makedirs(MAPS_DIR, exist_ok=True)

    if ROS_AVAILABLE and os.environ.get('WERKZEUG_RUN_MAIN') != 'false':
        _start_ros()

    # Bọc quá trình chạy Web trong khối try...finally để bắt sự kiện tắt ứng dụng
    try:
        app.run(host='0.0.0.0', port=5000, debug=True, use_reloader=False, threaded=True)
    finally:
        # Đoạn code này LUÔN LUÔN được chạy khi bạn bấm Ctrl+C hoặc Web bị tắt
        if ROS_AVAILABLE:
            print('\n[Web] Đang đóng Web Server...')
            print('[ROS] Đang giải phóng bộ nhớ đệm FastDDS và hủy Node ngầm...')
            try:
                if _ros_executor is not None:
                    _ros_executor.shutdown()  # Bước 1: Ra lệnh dừng vòng lặp spin() đang chạy ngầm
                if rclpy.ok():
                    rclpy.shutdown()          # Bước 2: Tắt hẳn hệ thống rclpy, dọn sạch Shared Memory
                print('[ROS] Đã dọn dẹp gọn gàng và đóng sạch sẽ!')
            except Exception as e:
                print(f'[ROS] Có lỗi xảy ra khi dọn dẹp: {e}')