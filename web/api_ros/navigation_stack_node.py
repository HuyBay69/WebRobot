#!/usr/bin/env python3
"""
navigation_stack_node.py — Quản lý cụm tiến trình "navigation stack" phụ trợ cho xe
hero: carla_waypoint_publisher, carla_ad_agent (bundle ad_agent + local_planner),
và navigation_hmi_1.

Trước đây 3 tiến trình này được người dùng chạy tay bằng script navigate1.sh:
  ros2 launch carla_waypoint_publisher carla_waypoint_publisher.launch.py &
  ros2 launch carla_ad_agent carla_ad_agent.launch.py &
  ros2 run navigation navigation_hmi_1
(kèm trap Ctrl+C để kill $(jobs -p) dọn dẹp)

Module này thay thế script đó — khởi động tự động ngay sau khi spawn xe thành
công (app.js gọi POST /api/navstack/start), giữ nguyên thứ tự + độ trễ 1s giữa
các bước như script gốc.

CÙNG VẤN ĐỀ với ros_bridge_node.py: carla_waypoint_publisher và carla_ad_agent
chạy qua `ros2 launch`, mà `ros2 launch` tự setsid() cho TỪNG node con nó
spawn (mỗi node ở process group/session RIÊNG) — killpg theo group của chính
tiến trình launch sẽ KHÔNG chạm tới các node con, để lại tiến trình mồ côi.
Dùng lại đúng pattern đã sửa: SIGINT trực tiếp vào tiến trình `ros2 launch`
(để nó tự cascade dọn dẹp node con theo đúng cơ chế nội bộ của nó), dự phòng
bằng quét toàn bộ cây con/cháu qua /proc rồi SIGKILL nếu quá hạn.
"""
import os
import signal
import subprocess
import threading
import time

from flask import Blueprint, jsonify

navstack_bp = Blueprint('navstack_bp', __name__)

# web/api_ros/navigation_stack_node.py → lên 1 cấp là web/ → ghi log vào web/logs/
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOG_DIR  = os.path.join(BASE_DIR, 'logs')

# Thứ tự PHẢI giữ nguyên như navigate1.sh gốc — mỗi bước cách nhau 1s để node
# trước có thời gian khởi tạo (waypoint_publisher cần sẵn sàng trước khi
# ad_agent subscribe /carla/hero/waypoints, v.v.)
_PROCESS_SPECS = [
    {
        'name': 'waypoint_publisher',
        'cmd': ['ros2', 'launch', 'carla_waypoint_publisher', 'carla_waypoint_publisher.launch.py'],
    },
    {
        'name': 'ad_agent',
        'cmd': ['ros2', 'launch', 'carla_ad_agent', 'carla_ad_agent.launch.py'],
    },
    {
        'name': 'navigation_hmi',
        'cmd': ['ros2', 'run', 'navigation', 'navigation_hmi_1'],
    },
]
_STEP_DELAY_SEC = 1.0  # giống sleep 1 trong navigate1.sh gốc

_lock  = threading.Lock()
_procs = {}   # name -> subprocess.Popen (chỉ chứa tiến trình đang/đã chạy trong lần start gần nhất)


def _log(msg):
    print(f'[NavStack] {msg}', flush=True)


def _get_descendant_pids(pid):
    """Quét /proc lấy toàn bộ PID con/cháu (đệ quy) — không phụ thuộc process
    group/session, vì `ros2 launch` tự setsid() cho từng node nó spawn nên
    killpg thông thường không chạm tới được (xem giải thích ở đầu file)."""
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
        return any(p is not None and p.poll() is None for p in _procs.values())


def start_navigation_stack():
    """Khởi động lần lượt 3 tiến trình theo đúng thứ tự navigate1.sh gốc.
    Nếu 1 bước lỗi giữa chừng, tự dọn các bước đã lỡ chạy trước đó rồi báo lỗi
    — tránh để lại rác nửa chừng (vd: waypoint_publisher chạy nhưng ad_agent
    launch lỗi thì không nên để waypoint_publisher chạy mồ côi 1 mình)."""
    with _lock:
        if any(p is not None and p.poll() is None for p in _procs.values()):
            return False, 'Navigation stack đã đang chạy'

        os.makedirs(LOG_DIR, exist_ok=True)
        _procs.clear()

        for spec in _PROCESS_SPECS:
            name, cmd = spec['name'], spec['cmd']
            log_path = os.path.join(LOG_DIR, f'navstack_{name}.log')
            _log(f'Đang khởi chạy {name}: {" ".join(cmd)}  (log: {log_path})')

            try:
                log_f = open(log_path, 'w')
                proc = subprocess.Popen(
                    cmd,
                    start_new_session=True,  # process group riêng (xem giải thích đầu file)
                    stdout=log_f,
                    stderr=subprocess.STDOUT,
                )
            except Exception as e:
                _log(f'✗ Lỗi khởi chạy {name}: {e}')
                _stop_all_locked(wait_seconds=5)
                return False, f'Lỗi khởi chạy {name}: {e}'

            _procs[name] = proc
            _log(f'✓ {name} đã chạy — PID={proc.pid}')
            time.sleep(_STEP_DELAY_SEC)

        _log('Navigation stack đã khởi động xong toàn bộ 3 tiến trình.')
        return True, 'started'


def _stop_all_locked(wait_seconds=10):
    """Dừng tất cả tiến trình đang track. PHẢI gọi trong lúc đã giữ _lock."""
    for name, proc in list(_procs.items()):
        if proc is None or proc.poll() is not None:
            continue
        _log(f'Đang dừng {name} (PID={proc.pid}) — gửi SIGINT...')
        try:
            os.kill(proc.pid, signal.SIGINT)
        except ProcessLookupError:
            pass

    deadline = time.time() + wait_seconds
    for name, proc in list(_procs.items()):
        if proc is None:
            continue
        remaining = max(0, deadline - time.time())
        try:
            proc.wait(timeout=remaining)
            _log(f'✓ {name} đã thoát sạch.')
        except subprocess.TimeoutExpired:
            _log(f'✗ {name} không thoát kịp trong {wait_seconds}s — dọn cưỡng bức cả cây con/cháu.')
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
        except Exception:
            pass

    _procs.clear()


def stop_navigation_stack(wait_seconds=10, block=True):
    """Dừng toàn bộ navigation stack (nếu đang chạy).

    block=True  → chờ dọn sạch xong mới return (dùng khi tắt Web Server).
    block=False → return ngay, dọn dẹp chạy nền (dùng cho nút bấm trên UI).
    """
    def _do_stop():
        with _lock:
            if not _procs:
                return
            _stop_all_locked(wait_seconds=wait_seconds)

    if block:
        _do_stop()
    else:
        threading.Thread(target=_do_stop, daemon=True).start()

    return True, 'stopping'


# ── Routes ─────────────────────────────────────────────────────────────────────
@navstack_bp.route('/api/navstack/start', methods=['POST'])
def api_navstack_start():
    # Lưu ý: request này mất ~3s+ để trả lời (3 tiến trình x 1s delay giữa mỗi
    # bước, giống hệt navigate1.sh gốc) — cố ý làm đồng bộ cho đơn giản/dễ debug,
    # frontend gọi cái này tách biệt với luồng Spawn Car nên không chặn UI khác.
    ok, msg = start_navigation_stack()
    if not ok:
        return jsonify({'ok': False, 'error': msg}), 400
    return jsonify({'ok': True})


@navstack_bp.route('/api/navstack/stop', methods=['POST'])
def api_navstack_stop():
    ok, msg = stop_navigation_stack(wait_seconds=10, block=False)
    if not ok:
        return jsonify({'ok': False, 'error': msg}), 400
    return jsonify({'ok': True})


@navstack_bp.route('/api/navstack/status', methods=['GET'])
def api_navstack_status():
    with _lock:
        processes = {name: (p is not None and p.poll() is None) for name, p in _procs.items()}
    return jsonify({'ok': True, 'running': is_running(), 'processes': processes})
