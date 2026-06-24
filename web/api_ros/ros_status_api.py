"""
ros_status_api.py — Flask Blueprint cho ROS bridge status.

Tách hoàn toàn khỏi app.py. Đăng ký bằng:
    from api_ros.ros_status_api import ros_status_bp, start_bridge_checker
    app.register_blueprint(ros_status_bp)

Endpoints:
    POST /api/ros/heartbeat   ← nhận push từ bridge_check.py (auth bằng X-Bridge-Token)
    GET  /api/ros/status      ← frontend poll, trả { running, log_msg, log_level }
    GET  /api/ros/log-stream  ← SSE stream để đẩy log ngay khi nhận, không cần frontend poll
"""

import subprocess
import sys
import os
import time
import threading
import json
from flask import Blueprint, jsonify, request, Response, stream_with_context

# ── Cấu hình ────────────────────────────────────────────────────────────────────
BRIDGE_TOKEN   = 'bridge-check-secret'   # phải khớp với --token trong bridge_check.py
STATUS_TTL     = 15.0                    # giây — coi là offline nếu không nhận heartbeat
CHECK_INTERVAL = 5                       # giây — truyền sang bridge_check.py khi spawn
BRIDGE_SCRIPT  = os.path.join(os.path.dirname(__file__), 'bridge_check.py')

# ── Shared state ─────────────────────────────────────────────────────────────────
_status_lock = threading.Lock()
_status = {
    'running':    False,
    'log_msg':    None,
    'log_level':  None,
    'updated_at': 0.0,
}

# SSE queue: mỗi subscriber là một queue.Queue()
_sse_subscribers: list = []
_sse_lock = threading.Lock()

# Handle process con
_checker_process: subprocess.Popen | None = None


# ── Blueprint ────────────────────────────────────────────────────────────────────
ros_status_bp = Blueprint('ros_status', __name__)


@ros_status_bp.route('/api/ros/heartbeat', methods=['POST'])
def api_ros_heartbeat():
    """Nhận heartbeat từ bridge_check.py chạy trên máy local."""
    token = request.headers.get('X-Bridge-Token', '')
    if token != BRIDGE_TOKEN:
        return jsonify({'ok': False, 'error': 'Unauthorized'}), 403

    data = request.get_json(force=True, silent=True) or {}
    running    = bool(data.get('running', False))
    log_msg    = data.get('log_msg')
    log_level  = data.get('log_level', 'info')

    with _status_lock:
        _status['running']    = running
        _status['log_msg']    = log_msg
        _status['log_level']  = log_level
        _status['updated_at'] = time.time()

    # Đẩy log qua SSE tới tất cả subscriber đang mở
    if log_msg:
        _broadcast_log(log_msg, log_level)

    return jsonify({'ok': True})


@ros_status_bp.route('/api/ros/status', methods=['GET'])
def api_ros_status():
    """
    Frontend poll endpoint này (thay thế /api/ros-status cũ).
    Trả về trạng thái hiện tại. Nếu quá TTL → coi là offline.
    """
    with _status_lock:
        stale   = (time.time() - _status['updated_at']) > STATUS_TTL
        running = _status['running'] and not stale

    return jsonify({'ok': True, 'running': running})


@ros_status_bp.route('/api/ros/log-stream')
def api_ros_log_stream():
    """
    SSE endpoint để đẩy log từ bridge_check.py xuống frontend ngay lập tức.
    Frontend kết nối một lần, nhận event mỗi khi bridge_check.py push heartbeat.

    Event format:
        data: {"msg": "...", "level": "info|warn|error", "source": "ros"}
    """
    q = __import__('queue').Queue(maxsize=50)
    with _sse_lock:
        _sse_subscribers.append(q)

    def generate():
        try:
            # Gửi comment keep-alive ngay khi connect để browser biết stream sống
            yield ': connected\n\n'
            while True:
                try:
                    payload = q.get(timeout=25)   # timeout để gửi keep-alive
                    yield f'data: {json.dumps(payload)}\n\n'
                except __import__('queue').Empty:
                    yield ': keep-alive\n\n'      # ngăn proxy/browser timeout
        finally:
            with _sse_lock:
                try:
                    _sse_subscribers.remove(q)
                except ValueError:
                    pass

    return Response(
        stream_with_context(generate()),
        mimetype='text/event-stream',
        headers={
            'Cache-Control':     'no-cache',
            'X-Accel-Buffering': 'no',
        }
    )


# ── Internal helpers ─────────────────────────────────────────────────────────────
def _broadcast_log(msg: str, level: str):
    """Đẩy log event vào tất cả SSE subscriber đang kết nối."""
    payload = {'msg': msg, 'level': level, 'source': 'ros'}
    with _sse_lock:
        dead = []
        for q in _sse_subscribers:
            try:
                q.put_nowait(payload)
            except Exception:
                dead.append(q)
        for q in dead:
            _sse_subscribers.remove(q)


# ── Spawn / cleanup bridge_check_node.py ────────────────────────────────────────
def start_bridge_checker():
    """Spawn bridge_check_node.py ngay lập tức (đồng bộ).
    Log của node được in ra sau Flask banner nhờ thread pipe_log delay nhỏ.
    """
    global _checker_process

    if _checker_process is not None and _checker_process.poll() is None:
        return

    cmd = [
        sys.executable, BRIDGE_SCRIPT,
        '--flask-url', 'http://localhost:5000',
        '--interval',  str(CHECK_INTERVAL),
        '--token',     BRIDGE_TOKEN,
    ]
    _checker_process = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )

    # Delay chỉ áp dụng cho việc IN log ra terminal — không ảnh hưởng process
    def _pipe_log():
        time.sleep(0.5)   # đợi Flask banner in xong rồi mới bắt đầu in log node
        for line in _checker_process.stdout:
            print(f'[BridgeChecker] {line}', end='', flush=True)

    threading.Thread(target=_pipe_log, daemon=True).start()


def stop_bridge_checker():
    """Kill bridge_check_node.py khi Flask tắt."""
    global _checker_process
    if _checker_process is None:
        return
    if _checker_process.poll() is None:
        print(f'[BridgeChecker] Đang kill PID={_checker_process.pid}...')
        _checker_process.terminate()
        try:
            _checker_process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            _checker_process.kill()
        print('[BridgeChecker] Đã dọn dẹp xong.')
    _checker_process = None