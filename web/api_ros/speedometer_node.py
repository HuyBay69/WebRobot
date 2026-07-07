"""
speedometer_node.py — ROS2 node subscribe /carla/hero/speedometer, chạy trong
thread của Flask (giống pipeline odom_node.py), nhưng khởi động SAU các node
khác (ưu tiên thấp hơn) vì đây chỉ là dữ liệu hiển thị, không ảnh hưởng điều
khiển xe.

Chọn viết chung trong Flask (không phải subprocess riêng) vì:
- Topic tần suất cao, cần latency thấp, không muốn overhead HTTP mỗi frame
- Callback chỉ quy đổi m/s → km/h rồi broadcast → cực nhẹ
- Nếu ROS không có → node không khởi động, Flask vẫn chạy bình thường

QUAN TRỌNG — Context ROS2 riêng:
    odom_node.py gọi rclpy.init() KHÔNG chỉ định context, tức là dùng context
    global mặc định. Nếu speedometer_node.py cũng gọi rclpy.init() mặc định
    trong cùng 1 process Flask, ROS2 sẽ báo lỗi "rcl_init already called" vì
    2 node cùng tranh 1 context global.
    → Node này tự tạo 1 rclpy.Context() RIÊNG, hoàn toàn độc lập với context
      mà odom_node.py đang dùng — khởi động/dừng theo thứ tự nào cũng không
      xung đột.

Dữ liệu lấy: /carla/hero/speedometer — std_msgs/Float32, giá trị m/s (tốc độ
tức thời dọc trục xe, chuẩn carla_ros_bridge) → quy đổi km/h = m/s * 3.6
trước khi broadcast xuống browser.

QoS: dùng BEST_EFFORT (giống odom_node.py) vì đây là quy tắc tương thích an
toàn nhất — subscriber BEST_EFFORT luôn kết nối được dù publisher là
RELIABLE hay BEST_EFFORT; chiều ngược lại (subscriber RELIABLE) có thể bị từ
chối nếu publisher là BEST_EFFORT.

Tần suất broadcast: throttle xuống tối đa 5Hz (không phụ thuộc tần suất
publish gốc của topic) để giảm tải tính toán — mỗi callback vẫn cập nhật
_latest_kmh/_latest_mps ngay lập tức, nhưng chỉ gửi xuống WebSocket tối đa
5 lần/giây.

Giá trị luôn không âm: lấy abs(msg.data) trước khi quy đổi km/h — vừa đảm
bảo tốc độ hiển thị luôn dương, vừa triệt tiêu hiện tượng "-0.00" khi giá
trị gốc là số âm rất nhỏ do nhiễu cảm biến.
"""

import json
import threading
import time

# ── Shared state ───────────────────────────────────────────────────────────────
_latest_kmh: float | None = None
_latest_mps: float | None = None
_last_update_t = 0.0
_latest_lock = threading.Lock()

# WebSocket clients — mỗi tab browser là 1 entry
_ws_clients: set = set()
_ws_lock = threading.Lock()

_node     = None
_thread   = None
_running  = False
_context  = None   # rclpy.Context() riêng — xem docstring ở trên

_BROADCAST_HZ       = 5
_BROADCAST_INTERVAL = 1.0 / _BROADCAST_HZ   # 0.2s — throttle tốc độ gửi WebSocket


# ── ROS2 Node ──────────────────────────────────────────────────────────────────
def _build_node(context):
    """Khởi tạo rclpy node trên context riêng. Gọi trong thread riêng sau khi
    rclpy.init(context=context)."""
    from rclpy.node import Node
    from std_msgs.msg import Float32
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

    class SpeedometerNode(Node):
        def __init__(self):
            super().__init__('speedometer_check', context=context)
            self._last_broadcast_t = 0.0   # để throttle xuống BROADCAST_HZ

            qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )

            self.create_subscription(
                Float32,
                '/carla/hero/speedometer',
                self._cb,
                qos,
            )
            self.get_logger().info('Đã khởi chạy.')

        def _cb(self, msg: Float32):
            # abs(): tốc độ hiển thị luôn là số dương (không quan tâm chiều
            # tiến/lùi), đồng thời triệt tiêu luôn hiện tượng "-0.00" khi
            # msg.data là số âm rất nhỏ do nhiễu cảm biến (vd -0.0001 m/s).
            mps = abs(msg.data)
            kmh = mps * 3.6
            now = time.time()

            global _latest_kmh, _latest_mps, _last_update_t
            with _latest_lock:
                _latest_kmh    = kmh
                _latest_mps    = mps
                _last_update_t = now

            # Throttle xuống tối đa BROADCAST_HZ lần/giây — topic gốc có thể
            # tới nhanh hơn nhiều (vd 20Hz+), nhưng browser chỉ cần 5Hz là đủ
            # mượt, giảm tải JSON-serialize + WebSocket send + render phía JS.
            if now - self._last_broadcast_t >= _BROADCAST_INTERVAL:
                self._last_broadcast_t = now
                _broadcast_ws({
                    'mps': round(mps, 3),
                    'kmh': round(kmh, 2),
                })

    return SpeedometerNode()


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
def start_speedometer_node():
    """
    Khởi động ROS2 node trong thread riêng, dùng context riêng để không đụng
    context global mà odom_node.py đang dùng.
    Nếu rclpy không cài → bỏ qua, không crash Flask.
    """
    global _node, _thread, _running, _context

    try:
        import rclpy
    except ImportError:
        print('rclpy không tìm thấy — bỏ qua speedometer node.')
        return

    if _thread is not None and _thread.is_alive():
        return

    _running = True

    def _run():
        global _node, _context
        try:
            _context = rclpy.Context()
            rclpy.init(context=_context)
            _node = _build_node(_context)

            from rclpy.executors import SingleThreadedExecutor
            executor = SingleThreadedExecutor(context=_context)
            executor.add_node(_node)
            executor.spin()
        except Exception as e:
            print(f'[SpeedometerNode] Lỗi: {type(e).__name__}: {e}')
        finally:
            if _node is not None:
                _node.destroy_node()
            try:
                if _context is not None and _context.ok():
                    rclpy.shutdown(context=_context)
            except Exception:
                pass

    _thread = threading.Thread(target=_run, daemon=True, name='speedometer-node')
    _thread.start()
    print('[SpeedometerNode] Thread đã khởi động.')


def stop_speedometer_node():
    """Dừng node khi Flask tắt."""
    global _running, _node

    _running = False

    try:
        import rclpy
        if _node is not None:
            _node.destroy_node()
            _node = None
        if _context is not None and _context.ok():
            rclpy.shutdown(context=_context)
    except Exception:
        pass

    if _thread is not None and _thread.is_alive():
        _thread.join(timeout=3)

    print('[SpeedometerNode] Đã dọn dẹp xong.')


def get_latest_kmh() -> dict | None:
    """Trả về giá trị mới nhất — dùng nếu cần expose thêm qua REST/SSE sau này."""
    with _latest_lock:
        if _latest_kmh is None:
            return None
        return {
            'kmh': _latest_kmh,
            'mps': _latest_mps,
            'age_sec': time.time() - _last_update_t,
        }