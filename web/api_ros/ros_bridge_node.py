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

_lock = threading.Lock()
_proc = None          # subprocess.Popen hiện tại (None nếu bridge chưa chạy)
_current_town = None  # bản đồ (town) đang chạy cùng bridge, nếu có


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
            return False, 'Carla ROS Bridge đã đang chạy'

        cmd = [
            'ros2', 'launch', 'carla_ros_bridge', 'carla_ros_bridge.launch.py',
            f'town:={town}',
            f'synchronous_mode:={"True" if synchronous else "False"}',
        ]

        try:
            _proc = subprocess.Popen(
                cmd,
                start_new_session=True,  # process group riêng để kill sạch (kể cả tiến trình con)
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except Exception as e:
            return False, str(e)

        _current_town = town
        return True, 'started'


def stop_ros_bridge(wait_seconds=10, block=True):
    """Dừng Carla ROS Bridge đang chạy (nếu có).

    block=True  → chờ tối đa `wait_seconds` giây cho tiến trình thoát hẳn rồi mới
                  return (dùng khi tắt hẳn Web Server, hoặc khi tạm dừng CARLA ở
                  Bước 1, cần dọn dẹp sạch trước).
    block=False → gửi tín hiệu dừng rồi return ngay (không chặn request Flask);
                  một thread nền sẽ tiếp tục theo dõi và SIGKILL nếu quá hạn.
    """
    global _proc, _current_town
    with _lock:
        proc = _proc
        if proc is None or proc.poll() is not None:
            _proc = None
            _current_town = None
            return False, 'Carla ROS Bridge không chạy'
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except (ProcessLookupError, PermissionError):
            pass

    def _wait_and_force_kill():
        global _proc, _current_town
        try:
            proc.wait(timeout=wait_seconds)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except Exception:
                pass
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
