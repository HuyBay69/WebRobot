"""
odom_node.py — ROS2 node subscribe /carla/markers, chạy trong thread của Flask.

Chọn viết chung trong Flask (không phải subprocess riêng) vì:
- Topic 10-20Hz cần latency thấp nhất, không muốn overhead HTTP mỗi frame
- Callback chỉ update 1 dict + ghi queue → cực nhẹ, không ảnh hưởng Flask
- Nếu ROS không có → node không khởi động, Flask vẫn chạy bình thường

Dữ liệu lấy: markers[0].pose → position (x, y, z) + orientation (quaternion → yaw)

Log format (odom_check_log.txt):
    [ros_time_sec] [x] [y] [yaw_rad] [delta_t_ms]
"""

import math
import os
import queue
import threading
import time
import json

# ── Shared state ───────────────────────────────────────────────────────────────
_latest: dict | None = None
_latest_lock = threading.Lock()

# Queue ghi file
_log_queue: queue.Queue = queue.Queue(maxsize=200)

# WebSocket clients — mỗi tab browser là 1 entry
_ws_clients: set = set()
_ws_lock = threading.Lock()

_node    = None
_thread  = None
_running = False


# ── Helpers ────────────────────────────────────────────────────────────────────
def _quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Quaternion → yaw (rad), trục Z lên."""
    return math.atan2(2.0 * (qw * qz + qx * qy),
                      1.0 - 2.0 * (qy * qy + qz * qz))


# ── ROS2 Node ──────────────────────────────────────────────────────────────────
def _build_node(log_path: str):
    """Khởi tạo rclpy node. Gọi trong thread riêng sau khi rclpy.init()."""
    import rclpy
    from rclpy.node import Node
    from visualization_msgs.msg import MarkerArray

    class OdomNode(Node):
        def __init__(self):
            super().__init__('odom_check')
            self._prev_ros_t: float | None = None

            from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
            qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )

            self.create_subscription(
                MarkerArray,
                '/carla/markers',
                self._cb,
                qos,
            )
            self.get_logger().info('Đã khởi chạy.')

        def _cb(self, msg: MarkerArray):
            if not msg.markers:
                return

            m   = msg.markers[0]
            p   = m.pose.position
            o   = m.pose.orientation
            yaw = _quat_to_yaw(o.x, o.y, o.z, o.w)

            # ROS time từ header marker (giây, float)
            ros_t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

            # Delta t so với frame trước
            if self._prev_ros_t is not None:
                delta_ms = (ros_t - self._prev_ros_t) * 1000.0
            else:
                delta_ms = 0.0
            self._prev_ros_t = ros_t

            frame = {
                'x':        p.x,
                'y':        p.y,
                'z':        p.z,
                'yaw':      yaw,
                'ros_t':    ros_t,
                'delta_ms': delta_ms,
            }

            # Cập nhật latest
            with _latest_lock:
                global _latest
                _latest = frame

            # Broadcast WebSocket — round để giảm bytes truyền
            _broadcast_ws({
                'x':        round(p.x,   3),
                'y':        round(p.y,   3),
                'yaw':      round(yaw,   4),
                'ros_t':    round(ros_t, 3),
                'delta_ms': round(delta_ms, 2),
            })

            # Đẩy vào log queue (non-blocking)
            try:
                _log_queue.put_nowait(frame)
            except queue.Full:
                pass

    return OdomNode()


def _writer_thread(log_path: str):
    """Thread riêng chỉ ghi file — đọc từ queue, không bao giờ block callback ROS."""
    with open(log_path, 'w', buffering=1) as f:   # buffering=1: line-buffered
        f.write('# ros_time(s)          x(m)      y(m)      yaw(rad)  delta_t(ms)\n')
        while _running or not _log_queue.empty():
            try:
                fr = _log_queue.get(timeout=0.5)
                f.write(
                    f"{fr['ros_t']:>20.6f}  "
                    f"{fr['x']:>8.3f}  "
                    f"{fr['y']:>8.3f}  "
                    f"{fr['yaw']:>8.4f}  "
                    f"{fr['delta_ms']:>10.2f}\n"
                )
            except queue.Empty:
                continue


def _broadcast_ws(payload: dict):
    """Gửi JSON frame tới tất cả WebSocket client đang kết nối."""
    msg = json.dumps(payload)
    with _ws_lock:
        dead = set()
        for ws in _ws_clients:
            try:
                ws.send(msg)
            except Exception:
                dead.add(ws)
        for ws in dead:
            _ws_clients.discard(ws)   # discard thay vì -= để tránh rebind local


def register_ws_client(ws):
    with _ws_lock:
        _ws_clients.add(ws)


def unregister_ws_client(ws):
    with _ws_lock:
        _ws_clients.discard(ws)


# ── Public API — gọi từ app.py ─────────────────────────────────────────────────
def start_odom_node(log_path: str):
    """
    Khởi động ROS2 node trong thread riêng.
    Nếu rclpy không cài → bỏ qua, không crash Flask.
    """
    global _node, _thread, _running

    try:
        import rclpy
    except ImportError:
        print('rclpy không tìm thấy — bỏ qua odom node.')
        return

    if _thread is not None and _thread.is_alive():
        return

    _running = True

    def _run():
        global _node
        try:
            rclpy.init()
            _node = _build_node(log_path)

            wt = threading.Thread(target=_writer_thread, args=(log_path,), daemon=True)
            wt.start()

            rclpy.spin(_node)
        except Exception as e:
            print(f'Lỗi: {type(e).__name__}: {e}')
        finally:
            if _node is not None:
                _node.destroy_node()
            try:
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception:
                pass

    _thread = threading.Thread(target=_run, daemon=True, name='odom-node')
    _thread.start()
    print('Thread đã khởi động.')


def stop_odom_node():
    """Dừng node khi Flask tắt."""
    global _running, _node

    _running = False

    try:
        import rclpy
        if _node is not None:
            _node.destroy_node()
            _node = None
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass

    if _thread is not None and _thread.is_alive():
        _thread.join(timeout=3)

    print('[OdomNode] Đã dọn dẹp xong.')


def get_latest() -> dict | None:
    """Trả về frame mới nhất — dùng cho SSE odom sau này."""
    with _latest_lock:
        return dict(_latest) if _latest else None