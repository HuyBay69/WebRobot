#!/usr/bin/env python3
import math
import os
import threading

from flask import Flask, jsonify, render_template, request, send_from_directory
from werkzeug.utils import secure_filename

app = Flask(__name__)
app.config['UPLOAD_FOLDER'] = os.path.join(os.path.dirname(__file__), 'uploads')
app.config['MAX_CONTENT_LENGTH'] = 50 * 1024 * 1024  # 50 MB

ALLOWED_EXTENSIONS = {'png', 'jpg', 'jpeg', 'gif', 'bmp', 'webp'}

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

            road_qos = QoSProfile(depth=10)
            self.road_sub = self.create_subscription(
                MarkerArray,
                'carla_road_network',
                self._road_callback,
                road_qos,
            )

            self._markers = []
            self._markers_lock = threading.Lock()
            self.get_logger().info('RobotROSNode started.')

        def _road_callback(self, msg: MarkerArray):
            with self._markers_lock:
                self._markers = list(msg.markers)

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
def _allowed(filename: str) -> bool:
    return '.' in filename and filename.rsplit('.', 1)[1].lower() in ALLOWED_EXTENSIONS


# ── Routes ─────────────────────────────────────────────────────────────────────
@app.route('/')
def index():
    return render_template('index.html')


@app.route('/api/upload-map', methods=['POST'])
def api_upload_map():
    if 'file' not in request.files:
        return jsonify({'ok': False, 'error': 'Không có file'}), 400

    f = request.files['file']
    map_name = request.form.get('map_name', '').strip() or f.filename

    if f.filename == '':
        return jsonify({'ok': False, 'error': 'Chưa chọn file'}), 400

    if not _allowed(f.filename):
        return jsonify({'ok': False, 'error': 'Chỉ hỗ trợ file ảnh (png/jpg/gif/bmp/webp)'}), 400

    filename = secure_filename(f.filename)
    save_path = os.path.join(app.config['UPLOAD_FOLDER'], filename)
    f.save(save_path)

    return jsonify({'ok': True, 'filename': filename, 'map_name': map_name, 'url': f'/uploads/{filename}'})


@app.route('/uploads/<filename>')
def uploaded_file(filename):
    return send_from_directory(app.config['UPLOAD_FOLDER'], filename)


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

    # ROS không khả dụng — chỉ xác nhận đã nhận
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


if __name__ == '__main__':
    os.makedirs(app.config['UPLOAD_FOLDER'], exist_ok=True)

    # Khởi động ROS node nền (chỉ trong process chính, tránh double-init khi debug reloader)
    if ROS_AVAILABLE and os.environ.get('WERKZEUG_RUN_MAIN') != 'false':
        _start_ros()

    app.run(host='0.0.0.0', port=5000, debug=True, use_reloader=False)
