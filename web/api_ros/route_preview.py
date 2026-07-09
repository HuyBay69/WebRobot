#!/usr/bin/env python3
"""
route_preview.py — Tính TRƯỚC (preview) tuyến đường dự kiến giữa các điểm trong
hàng đợi hành trình (queue_pending_points), để vẽ lên bản đồ web ngay khi người
dùng thêm/xoá điểm — KHÔNG publish lên bất kỳ ROS topic nào, không đụng gì tới
navigate_node.py / navigation stack thật đang chạy.

Dùng ĐÚNG thuật toán carla_waypoint_publisher.py đang dùng để tính route thật:
GlobalRoutePlanner (agents.navigation.global_route_planner), sampling_resolution=1.
Khác biệt duy nhất: module này chạy TRỰC TIẾP trong tiến trình Flask (không qua
subprocess/ROS2), kết nối thẳng CARLA Python API, trả kết quả về ngay trong cùng
1 request HTTP — vì đây là tác vụ "tính rồi trả kết quả ngay", không hợp với
kiểu giao tiếp 1 chiều qua stdin mà navigate_node.py / navigation_stack_node.py
đang dùng (gửi lệnh xuống nhưng không có đường trả kết quả real-time ngược lên).

── LƯU Ý MÔI TRƯỜNG (đọc kỹ nếu gặp lỗi import) ─────────────────────────────
`agents.navigation.global_route_planner` là 1 phần PythonAPI của CARLA (thư mục
PythonAPI/carla trong bản cài CARLA) — thường chỉ nằm sẵn trong sys.path khi
chạy qua môi trường đã "source" ROS2 workspace (vd ros2 launch/ros2 run). Vì
Flask (start_web.sh) là tiến trình Python ĐỘC LẬP, không chắc có cùng sys.path
đó. Module này tự thử thêm đường dẫn PythonAPI đoán theo vị trí CarlaUE4.sh đã
biết (~/CARLA/carla_packed_linux/PythonAPI/carla) — SỬA LẠI _EXTRA_CARLA_PYTHONAPI_PATH
bên dưới nếu máy bạn cài CARLA ở chỗ khác hoặc PythonAPI nằm ở vị trí khác.
Gọi GET /api/route_preview/status để kiểm tra import có thành công hay không.
"""
import os
import sys
import threading

from flask import Blueprint, jsonify, request

route_preview_bp = Blueprint('route_preview_bp', __name__)

# Thử thêm đường dẫn PythonAPI của CARLA vào sys.path nếu chưa có sẵn — đoán
# theo vị trí CarlaUE4.sh đã biết từ carla_node.py. Sửa lại cho khớp máy bạn
# nếu không đúng (vd nếu cài CARLA .whl qua pip thì có thể không cần dòng này).
_EXTRA_CARLA_PYTHONAPI_PATH = os.path.expanduser('~/CARLA/carla_packed_linux/PythonAPI/carla')
if os.path.isdir(_EXTRA_CARLA_PYTHONAPI_PATH) and _EXTRA_CARLA_PYTHONAPI_PATH not in sys.path:
    sys.path.append(_EXTRA_CARLA_PYTHONAPI_PATH)

_import_error = None
try:
    import carla
    from agents.navigation.global_route_planner import GlobalRoutePlanner
except Exception as e:  # noqa: BLE001 — cố ý bắt rộng, chỉ để báo lỗi rõ ràng qua API
    carla = None
    GlobalRoutePlanner = None
    _import_error = f'{type(e).__name__}: {e}'


def _log(msg):
    print(f'[RoutePreview] {msg}', flush=True)


CARLA_HOST = 'localhost'
CARLA_PORT = 2000
CARLA_TIMEOUT_SEC = 5.0
SAMPLING_RESOLUTION = 1.0  # giống hệt carla_waypoint_publisher.py hiện tại (comment gốc: "Tang tu 1")

_lock = threading.Lock()
_carla_map = None
_grp = None  # GlobalRoutePlanner đã build sẵn graph của bản đồ — cache lại, không tạo mới mỗi lần


def _ensure_connected_locked():
    """Kết nối CARLA + build GlobalRoutePlanner nếu chưa có. PHẢI gọi trong lúc
    đã giữ _lock. Build route planner khá tốn (dựng graph từ toàn bộ bản đồ) nên
    cache lại, chỉ làm lại khi lỗi giữa chừng (xem _reset_connection_locked)."""
    global _carla_map, _grp

    if _import_error is not None:
        raise RuntimeError(
            f'Không import được carla / GlobalRoutePlanner: {_import_error} — '
            f'kiểm tra lại sys.path cho tiến trình Flask (xem docstring đầu file '
            f'route_preview.py). Gọi GET /api/route_preview/status để xem chi tiết.'
        )

    if _grp is not None:
        return

    client = carla.Client(CARLA_HOST, CARLA_PORT)
    client.set_timeout(CARLA_TIMEOUT_SEC)
    world = client.get_world()
    _carla_map = world.get_map()
    _grp = GlobalRoutePlanner(_carla_map, sampling_resolution=SAMPLING_RESOLUTION)
    _log('Đã kết nối CARLA + build xong GlobalRoutePlanner.')


def _reset_connection_locked():
    """Xoá cache — gọi khi tính route lỗi giữa chừng (vd CARLA restart / đổi bản
    đồ khác khiến route planner cũ không còn khớp). PHẢI gọi trong lúc giữ _lock."""
    global _carla_map, _grp
    _carla_map, _grp = None, None


def _compute_segment_locked(x1, y1, x2, y2):
    """Tính waypoints từ (x1,y1) tới (x2,y2) — toạ độ vào/ra đều ở hệ ROS (map
    frame). PHẢI gọi trong lúc giữ _lock.

    Đảo dấu y khi vào/ra CARLA: đúng quy ước carla_ros_bridge đang dùng trong
    toàn bộ hệ thống (x_ros = x_carla, y_ros = -y_carla) — carla_waypoint_publisher.py
    làm y hệt vậy qua trans.ros_pose_to_carla_transform() / carla_transform_to_ros_pose()
    (ở đây chỉ cần đảo (x,y), không cần xử lý phần xoay yaw vì chỉ dùng để vẽ)."""
    loc1 = carla.Location(x=x1, y=-y1, z=0.0)
    loc2 = carla.Location(x=x2, y=-y2, z=0.0)

    route = _grp.trace_route(loc1, loc2)

    points = []
    for wp, _road_option in route:
        loc = wp.transform.location
        points.append([loc.x, -loc.y])  # đảo lại về hệ ROS
    return points


def compute_segments(points_ros):
    """points_ros: list[[x,y], ...] ở hệ ROS — điểm đầu tiên là mốc bắt đầu
    (thường là vị trí hiện tại của xe), các điểm sau là hàng đợi A, B, C...
    theo đúng thứ tự. Trả về list CÁC ĐOẠN RIÊNG BIỆT — mỗi đoạn là
    list[[x,y],...] ứng với 1 chặng (điểm[i] → điểm[i+1]) — KHÔNG nối chung
    thành 1 đường nữa, để frontend tô màu so le từng đoạn (cam/xanh xen kẽ)."""
    with _lock:
        _ensure_connected_locked()

        segments = []
        for i in range(len(points_ros) - 1):
            x1, y1 = points_ros[i]
            x2, y2 = points_ros[i + 1]
            try:
                segment = _compute_segment_locked(float(x1), float(y1), float(x2), float(y2))
            except Exception as e:
                # Có thể do bản đồ vừa đổi / CARLA vừa restart — thử kết nối lại
                # 1 lần rồi tính lại, không được nữa thì để lỗi bay lên cho route
                # gọi biết mà báo cho browser.
                _log(f'Lỗi tính route (thử kết nối lại 1 lần): {e}')
                _reset_connection_locked()
                _ensure_connected_locked()
                segment = _compute_segment_locked(float(x1), float(y1), float(x2), float(y2))

            segments.append(segment)

        return segments


# ── Routes ─────────────────────────────────────────────────────────────────────
@route_preview_bp.route('/api/route_preview', methods=['POST'])
def api_route_preview():
    """
    Body JSON: { "points": [[x0,y0], [x1,y1], [x2,y2], ...] }
    points[0] = vị trí hiện tại của xe (browser tự lấy từ odometry đang có sẵn),
    các phần tử sau là các điểm trong hàng đợi theo đúng thứ tự A, B, C...

    Trả về: { "ok": true, "segments": [ [[x,y],...], [[x,y],...], ... ] } —
    MỖI CHẶNG (xe→A, A→B, B→C...) là 1 mảng riêng, theo hệ ROS (map frame),
    để frontend vẽ so le màu từng đoạn.
    """
    data = request.get_json(force=True, silent=True) or {}
    points = data.get('points')

    if not isinstance(points, list) or len(points) < 2:
        return jsonify({'ok': False, 'error': 'Cần ít nhất 2 điểm (vị trí hiện tại + 1 điểm hàng đợi)'}), 400

    try:
        segments = compute_segments(points)
    except Exception as e:
        return jsonify({'ok': False, 'error': str(e)}), 500

    return jsonify({'ok': True, 'segments': segments})


@route_preview_bp.route('/api/route_preview/status', methods=['GET'])
def api_route_preview_status():
    return jsonify({
        'ok': True,
        'carla_module_available': _import_error is None,
        'import_error': _import_error,
        'connected': _grp is not None,
        'extra_pythonapi_path_used': _EXTRA_CARLA_PYTHONAPI_PATH if os.path.isdir(_EXTRA_CARLA_PYTHONAPI_PATH) else None,
    })