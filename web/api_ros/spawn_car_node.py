"""
spawn_car_node.py — Flask Blueprint quản lý tính năng "Spawn Car".

Gồm 2 phần:
  1. SPAWN POINTS: khi bridge kết nối thành công (do ros_status_api.py báo qua
     trigger_spawn_points_fetch()), chạy fetch_spawn_points.py như 1 subprocess
     riêng để lấy danh sách spawn point từ CARLA, cache lại và đẩy xuống browser
     qua SSE /api/spawn/points-stream.
  2. SPAWN VEHICLE: nhận toạ độ (x, y, z, yaw) người dùng chọn trên bản đồ, kill
     tiến trình `ros2 launch carla_spawn_objects carla_example_ego_vehicle...`
     đang chạy (nếu có) để dọn xe cũ, rồi launch tiến trình mới để spawn xe tại
     vị trí mới. roll = pitch luôn = 0.

Endpoints:
    GET  /api/spawn/points          ← lấy cache hiện tại (dùng khi mới load trang)
    GET  /api/spawn/points-stream   ← SSE, đẩy danh sách spawn point mỗi khi có mới
    POST /api/spawn/vehicle         ← { x, y, z, yaw } → kill xe cũ + spawn xe mới
"""

import json
import os
import queue
import signal
import subprocess
import sys
import threading
import time

from flask import Blueprint, jsonify, request, Response, stream_with_context

BASE_DIR     = os.path.dirname(os.path.abspath(__file__))
FETCH_SCRIPT = os.path.join(BASE_DIR, 'fetch_spawn_points.py')

# ── Cấu hình ──────────────────────────────────────────────────────────────────
# Python dùng để chạy fetch_spawn_points.py. SỬA LẠI hằng số này nếu package
# `carla` được cài ở một môi trường/venv khác với Flask (VD: python3.7 kèm theo
# CARLA .egg riêng) — ví dụ: CARLA_PYTHON_EXEC = '/usr/bin/python3.7'
CARLA_PYTHON_EXEC = sys.executable

CARLA_HOST = 'localhost'
CARLA_PORT = 2000

# ID của xe được định nghĩa trong file .json cấu hình objects (carla_spawn_objects) —
# chỉ dùng để log/đối chiếu. Việc map đúng "spawn_point_ego_vehicle" (tên argument
# launch bên ngoài, giữ nguyên cho quen thuộc) sang đúng ROS param "spawn_point_<id>"
# mà node thực sự đọc (id='hero' trong objects.json) đã được xử lý BÊN TRONG
# carla_spawn_objects.launch.py (dùng OpaqueFunction dựa theo role_name) — không cần
# đổi tên argument ở đây nữa.
EGO_VEHICLE_ID = 'hero'

# Thời gian chờ (giây) sau khi kill xe cũ trước khi launch xe mới — cho CARLA
# kịp dọn sạch actor cũ (destroy service, sensor cleanup...) trước khi spawn mới,
# tránh trường hợp xe mới spawn đè lên lúc xe cũ chưa kịp dọn xong.
POST_KILL_DELAY_SEC = 2.0

# Thời gian chờ orderly shutdown (SIGINT chỉ gửi cho PID ros2 launch, không phải
# cả process group) trước khi cưỡng chế killpg. Giá trị này phụ thuộc tải CARLA
# server lúc destroy actor — chỉnh lại nếu môi trường của bạn cần nhiều/ít thời
# gian hơn.
SIGINT_GRACE_SEC = 6.0

spawn_car_bp = Blueprint('spawn_car', __name__)

# ── Cache spawn point + SSE subscribers ───────────────────────────────────────
_spawn_points_cache: list = []
_cache_lock = threading.Lock()

_subscribers: list[queue.Queue] = []
_sub_lock = threading.Lock()


def _broadcast(payload: dict):
    with _sub_lock:
        dead = []
        for q in _subscribers:
            try:
                q.put_nowait(payload)
            except queue.Full:
                dead.append(q)
        for q in dead:
            _subscribers.remove(q)


def _fetch_spawn_points_sync() -> tuple[bool, str]:
    """Chạy fetch_spawn_points.py đồng bộ (blocking). Trả về (ok, message)."""
    try:
        result = subprocess.run(
            [CARLA_PYTHON_EXEC, FETCH_SCRIPT, '--host', CARLA_HOST, '--port', str(CARLA_PORT)],
            capture_output=True, text=True, timeout=15,
        )
    except subprocess.TimeoutExpired:
        return False, 'Timeout khi kết nối CARLA để lấy spawn point.'
    except Exception as e:
        return False, f'Lỗi khi chạy fetch_spawn_points.py: {e}'

    # Lấy dòng cuối cùng có nội dung (script chỉ in đúng 1 dòng JSON)
    lines = [l for l in result.stdout.strip().splitlines() if l.strip()]
    last_line = lines[-1] if lines else ''

    if result.returncode != 0:
        try:
            err = json.loads(last_line)
            return False, err.get('error', result.stderr or 'Unknown error')
        except Exception:
            return False, (result.stderr or result.stdout or 'Unknown error').strip()

    try:
        data = json.loads(last_line)
    except Exception as e:
        return False, f'Không parse được kết quả spawn point: {e}'

    points = data.get('points', [])
    with _cache_lock:
        global _spawn_points_cache
        _spawn_points_cache = points

    _broadcast({'points': points})
    return True, f'Đã lấy {len(points)} spawn point từ CARLA.'


def trigger_spawn_points_fetch():
    """Chạy fetch spawn point trong thread nền — gọi khi bridge vừa kết nối."""
    def _run():
        ok, msg = _fetch_spawn_points_sync()
        level = 'OK' if ok else 'LỖI'
        print(f'[SpawnCar] ({level}) {msg}', flush=True)
    threading.Thread(target=_run, daemon=True).start()


@spawn_car_bp.route('/api/spawn/points', methods=['GET'])
def api_spawn_points():
    """Fallback GET — lấy cache hiện tại (dùng khi browser mới load trang)."""
    with _cache_lock:
        points = list(_spawn_points_cache)
    return jsonify({'ok': True, 'points': points})


@spawn_car_bp.route('/api/spawn/points-stream')
def api_spawn_points_stream():
    """SSE — đẩy danh sách spawn point mỗi khi fetch lại thành công."""
    q = queue.Queue(maxsize=5)
    with _sub_lock:
        _subscribers.append(q)
    with _cache_lock:
        current = list(_spawn_points_cache)

    def generate():
        try:
            yield f'data: {json.dumps({"points": current})}\n\n'
            while True:
                try:
                    payload = q.get(timeout=25)
                    yield f'data: {json.dumps(payload)}\n\n'
                except queue.Empty:
                    yield ': keep-alive\n\n'
        finally:
            with _sub_lock:
                try:
                    _subscribers.remove(q)
                except ValueError:
                    pass

    return Response(
        stream_with_context(generate()),
        mimetype='text/event-stream',
        headers={'Cache-Control': 'no-cache', 'X-Accel-Buffering': 'no'},
    )


# ── Quản lý tiến trình `ros2 launch ... ` để spawn ego vehicle ───────────────
_vehicle_process: subprocess.Popen | None = None
_vehicle_lock = threading.Lock()


def _kill_current_vehicle_locked():
    """PHẢI giữ _vehicle_lock trước khi gọi. Kill sạch process group xe cũ (nếu có)."""
    global _vehicle_process
    if _vehicle_process is None or _vehicle_process.poll() is not None:
        _vehicle_process = None
        return

    pid = _vehicle_process.pid
    print(f'[SpawnCar] Đang dọn xe cũ (PID={pid})...', flush=True)
    try:
        # ── Bước 1: SIGINT CHỈ gửi cho đúng PID của `ros2 launch` (KHÔNG killpg) ──
        # Nếu gửi cho cả process group, các node con (rclpy) nhận SIGINT trực tiếp
        # và tự chạy shutdown handler mặc định của chúng CÙNG LÚC với việc
        # `ros2 launch` cũng đang điều phối tắt chính node đó theo trình tự nội bộ
        # → hai luồng shutdown đụng nhau → lỗi "rcl_shutdown already called".
        # Chỉ gửi cho PID launch để nó tự lo tắt node con đúng thứ tự.
        os.kill(pid, signal.SIGINT)
        try:
            _vehicle_process.wait(timeout=SIGINT_GRACE_SEC)
        except subprocess.TimeoutExpired:
            # ── Bước 2: orderly shutdown không kịp → cưỡng chế cả process group ──
            print('[SpawnCar] SIGINT (orderly) chưa đủ, killpg SIGTERM...', flush=True)
            pgid = os.getpgid(pid)
            os.killpg(pgid, signal.SIGTERM)
            try:
                _vehicle_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                print('[SpawnCar] SIGTERM chưa đủ, SIGKILL...', flush=True)
                os.killpg(pgid, signal.SIGKILL)
                _vehicle_process.wait(timeout=3)
    except ProcessLookupError:
        pass  # process đã tự thoát
    except Exception as e:
        print(f'[SpawnCar] Lỗi khi kill xe cũ: {e}', flush=True)
    finally:
        _vehicle_process = None
        print('[SpawnCar] Đã dọn dẹp xe cũ xong.', flush=True)


def spawn_vehicle(x: float, y: float, z: float, yaw: float) -> tuple[bool, str]:
    """
    Kill xe hiện tại (nếu có), sau đó launch xe mới tại (x, y, z, yaw).
    roll = pitch = 0 cố định. Thứ tự truyền vào launch: x,y,z,roll,pitch,yaw.
    """
    global _vehicle_process
    spawn_str = f'{x},{y},{z},0,0,{yaw}'

    with _vehicle_lock:
        had_old_vehicle = _vehicle_process is not None and _vehicle_process.poll() is None
        _kill_current_vehicle_locked()

        if had_old_vehicle:
            # Chờ cho CARLA dọn sạch actor cũ (destroy service, sensor, v.v.)
            # trước khi launch xe mới — tránh xung đột lúc xe cũ chưa kịp dọn xong.
            print(f'[SpawnCar] Chờ {POST_KILL_DELAY_SEC}s để CARLA dọn sạch xe cũ...', flush=True)
            time.sleep(POST_KILL_DELAY_SEC)

        cmd = [
            'ros2', 'launch', 'carla_spawn_objects', 'carla_example_ego_vehicle.launch.py',
            f'spawn_point_ego_vehicle:={spawn_str}',
        ]
        try:
            _vehicle_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1,
                preexec_fn=os.setsid,   # tạo process group riêng → kill sạch toàn bộ cây con sau này
            )
        except Exception as e:
            _vehicle_process = None
            return False, f'Không chạy được ros2 launch: {e}'

        proc = _vehicle_process

    def _pipe_log():
        for line in proc.stdout:
            print(f'[SpawnCar] {line}', end='', flush=True)

    threading.Thread(target=_pipe_log, daemon=True).start()

    print(f'[SpawnCar] Đang spawn xe mới tại {spawn_str}', flush=True)
    return True, f'Đã gửi lệnh spawn xe tại ({x:.2f}, {y:.2f}, {z:.2f}), yaw={yaw:.2f}'


@spawn_car_bp.route('/api/spawn/vehicle', methods=['POST'])
def api_spawn_vehicle():
    """Body JSON: { "x": float, "y": float, "z": float, "yaw": float }"""
    data = request.get_json(force=True, silent=True) or {}
    try:
        x   = float(data['x'])
        y   = float(data['y'])
        z   = float(data.get('z', 0.0))
        yaw = float(data.get('yaw', 0.0))
    except (KeyError, ValueError, TypeError):
        return jsonify({'ok': False, 'error': 'Cần x, y (số); z, yaw tùy chọn (mặc định 0)'}), 400

    ok, msg = spawn_vehicle(x, y, z, yaw)
    if not ok:
        return jsonify({'ok': False, 'error': msg}), 500
    return jsonify({'ok': True, 'message': msg, 'x': x, 'y': y, 'z': z, 'yaw': yaw})


def stop_spawn_car_node():
    """Gọi khi Flask tắt — dọn sạch xe đang spawn (nếu có) để không bỏ rơi actor."""
    with _vehicle_lock:
        _kill_current_vehicle_locked()