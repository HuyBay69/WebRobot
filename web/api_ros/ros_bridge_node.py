#!/usr/bin/env python3
"""
ros_bridge_node.py — Quản lý tiến trình Carla ROS Bridge (Bước 2: Khởi động cầu nối
CARLA ROS BRIDGE).

Chạy `ros2 launch carla_ros_bridge carla_ros_bridge.launch.py town:=<map>` dưới dạng
subprocess nền (không blocking request Flask). Cung cấp API để home.js start / stop
cầu nối, chọn bản đồ (town) và chế độ (hiện chỉ hỗ trợ đồng bộ).

Được tách thành hàm riêng (không gắn chặt vào route) theo cùng pattern với
carla_node.py, để có thể gọi lại từ nơi khác — ví dụ: khi "Tạm dừng" CARLA ở
Bước 1, hoặc khi Web Server tắt (Ctrl+C / SIGTERM) — nhằm dọn dẹp sạch bridge
nếu đang chạy, vì bridge phụ thuộc vào CARLA.
"""
import os
import signal
import subprocess
import threading

from flask import Blueprint, jsonify, request

ros_bridge_bp = Blueprint('ros_bridge_bp', __name__)

# web/api_ros/ros_bridge_node.py → lên 1 cấp là web/ → ghi log vào web/logs/
_BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOG_DIR = os.path.join(_BASE_DIR, 'logs')
ROS_BRIDGE_LOG_PATH = os.path.join(LOG_DIR, 'ros_bridge.log')

_lock = threading.Lock()
_proc = None          # subprocess.Popen hiện tại (None nếu bridge chưa chạy)
_current_town = None  # bản đồ (town) đang chạy cùng bridge, nếu có


def _log(msg):
    print(f'[RosBridgeNode] {msg}', flush=True)


def _get_descendant_pids(pid):
    """Trả về list toàn bộ PID con/cháu (đệ quy) của `pid`, đọc qua /proc.

    Dùng thay cho os.killpg vì `ros2 launch` tự gọi setsid() cho TỪNG node nó
    spawn — mỗi node nằm ở process group/session RIÊNG, khác với chính tiến
    trình `ros2 launch`. killpg theo group của ros2 launch vì vậy không chạm
    tới các node con — đây chính là lý do carla_ros_bridge / carla_ad_agent
    còn sống sót sau khi bridge "dừng". Quan hệ cha/con (fork) qua /proc thì
    vẫn giữ nguyên bất kể setsid(), nên đây là cách duy nhất chắc chắn dọn
    sạch được toàn bộ cây tiến trình.
    """
    descendants = []
    try:
        with open(f'/proc/{pid}/task/{pid}/children') as f:
            direct = [int(p) for p in f.read().split()]
    except (FileNotFoundError, ProcessLookupError, PermissionError):
        direct = []
    for child_pid in direct:
        descendants.append(child_pid)
        descendants.extend(_get_descendant_pids(child_pid))
    return descendants


def is_running():
    with _lock:
        return _proc is not None and _proc.poll() is None


def start_ros_bridge(town: str, synchronous: bool = True):
    """Khởi động carla_ros_bridge với bản đồ (town) được chỉ định.

    Hiện chỉ hỗ trợ chế độ đồng bộ (synchronous=True); tham số được giữ lại
    để dễ mở rộng khi hỗ trợ chế độ không đồng bộ sau này.
    """
    global _proc, _current_town
    with _lock:
        if _proc is not None and _proc.poll() is None:
            _log('Đã đang chạy, bỏ qua lệnh khởi động mới.')
            return False, 'Carla ROS Bridge đã đang chạy'

        cmd = [
            'ros2', 'launch', 'carla_ros_bridge', 'carla_ros_bridge.launch.py',
            f'town:={town}',
            f'synchronous_mode:={"True" if synchronous else "False"}',
        ]

        _log(f'Đang khởi chạy: {" ".join(cmd)}  — log: {ROS_BRIDGE_LOG_PATH}')
        try:
            os.makedirs(LOG_DIR, exist_ok=True)
            log_f = open(ROS_BRIDGE_LOG_PATH, 'w')
            _proc = subprocess.Popen(
                cmd,
                start_new_session=True,  # process group riêng để kill sạch (kể cả tiến trình con)
                stdout=log_f,
                stderr=subprocess.STDOUT,
            )
        except Exception as e:
            _log(f'✗ Lỗi khởi chạy: {e}')
            return False, str(e)

        _current_town = town
        _log(f'✓ Đã chạy — PID={_proc.pid}, town={town}')
        return True, 'started'


def stop_ros_bridge(wait_seconds=10, block=True):
    """Dừng Carla ROS Bridge đang chạy (nếu có).

    block=True  → chờ tối đa `wait_seconds` giây cho tiến trình thoát hẳn rồi mới
                  return (dùng khi tắt hẳn Web Server, hoặc khi tạm dừng CARLA ở
                  Bước 1, cần dọn dẹp sạch trước).
    block=False → gửi tín hiệu dừng rồi return ngay (không chặn request Flask);
                  một thread nền sẽ tiếp tục theo dõi và dọn cưỡng bức nếu quá hạn.

    QUAN TRỌNG: gửi SIGINT (không phải SIGTERM) tới đúng tiến trình `ros2 launch`
    — đây là tín hiệu launch tự nhận diện để cascade shutdown xuống các node nó
    quản lý (carla_ros_bridge, carla_ad_agent...), giống hệt Ctrl+C thật. Nếu
    dùng SIGTERM hoặc killpg theo group của launch, các node con sẽ KHÔNG bị
    dừng vì mỗi node nằm ở process group/session riêng do launch tự setsid().
    """
    global _proc, _current_town
    with _lock:
        proc = _proc
        if proc is None or proc.poll() is not None:
            _proc = None
            _current_town = None
            _log('Không có tiến trình nào đang chạy, bỏ qua lệnh dừng.')
            return False, 'Carla ROS Bridge không chạy'
        _log(f'Đang dừng (PID={proc.pid}) — gửi SIGINT...')
        try:
            os.kill(proc.pid, signal.SIGINT)
        except ProcessLookupError:
            pass

    def _wait_and_force_kill():
        global _proc, _current_town
        try:
            proc.wait(timeout=wait_seconds)
            _log('✓ Đã thoát sạch.')
        except subprocess.TimeoutExpired:
            # ros2 launch không tự thoát kịp trong wait_seconds — dọn cưỡng
            # bức: quét TOÀN BỘ cây con/cháu thật sự qua /proc (không phụ
            # thuộc process group) rồi SIGKILL từng cái, con trước cha sau.
            _log(f'✗ Không thoát kịp trong {wait_seconds}s — dọn cưỡng bức cả cây con/cháu.')
            descendants = _get_descendant_pids(proc.pid)
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
            _log('✓ Đã dọn cưỡng bức xong.')
        except Exception:
            pass
        with _lock:
            if _proc is proc:
                _proc = None
                _current_town = None

    if block:
        _wait_and_force_kill()
    else:
        threading.Thread(target=_wait_and_force_kill, daemon=True).start()

    return True, 'stopping'


# ── Routes ─────────────────────────────────────────────────────────────────────
@ros_bridge_bp.route('/api/rosbridge/start', methods=['POST'])
def api_ros_bridge_start():
    data = request.get_json(force=True, silent=True) or {}
    town = (data.get('town') or '').strip()
    synchronous = bool(data.get('synchronous', True))

    if not town:
        return jsonify({'ok': False, 'error': 'Chưa chọn bản đồ'}), 400

    ok, msg = start_ros_bridge(town, synchronous)
    if not ok:
        return jsonify({'ok': False, 'error': msg}), 400
    return jsonify({'ok': True, 'town': town, 'synchronous': synchronous})


@ros_bridge_bp.route('/api/rosbridge/stop', methods=['POST'])
def api_ros_bridge_stop():
    # block=False: trả lời ngay, không chờ bridge thoát hẳn (giống pattern
    # /api/carla/stop) — dùng cho nút "Tạm dừng" ở Bước 1.
    ok, msg = stop_ros_bridge(wait_seconds=10, block=False)
    if not ok:
        return jsonify({'ok': False, 'error': msg}), 400
    return jsonify({'ok': True})


@ros_bridge_bp.route('/api/rosbridge/status', methods=['GET'])
def api_ros_bridge_status():
    return jsonify({'ok': True, 'running': is_running(), 'town': _current_town})