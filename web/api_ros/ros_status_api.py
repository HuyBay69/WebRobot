"""
ros_status_api.py — Flask Blueprint cho ROS bridge status.

Endpoints:
    POST /api/ros/heartbeat     ← nhận push từ bridge_check_node.py khi state đổi
    GET  /api/ros/status-stream ← SSE, browser lắng nghe thay đổi connected/disconnected
"""

import json
import os
import queue
import subprocess
import sys
import threading
import time

from flask import Blueprint, jsonify, request, Response, stream_with_context

# ── Cấu hình ──────────────────────────────────────────────────────────────────
BRIDGE_TOKEN   = 'bridge-check-secret'
CHECK_INTERVAL = 5
BRIDGE_SCRIPT  = os.path.join(os.path.dirname(__file__), 'bridge_check.py')

# ── Shared state ───────────────────────────────────────────────────────────────
_running      = False
_running_lock = threading.Lock()

# SSE subscribers — mỗi browser tab là 1 queue
_subscribers: list[queue.Queue] = []
_sub_lock = threading.Lock()

# Process con
_checker_process: subprocess.Popen | None = None

# ── Blueprint ──────────────────────────────────────────────────────────────────
ros_status_bp = Blueprint('ros_status', __name__)


@ros_status_bp.route('/api/ros/heartbeat', methods=['POST'])
def api_ros_heartbeat():
    """Nhận push từ bridge_check_node.py khi trạng thái carla_ros_bridge thay đổi."""
    if request.headers.get('X-Bridge-Token') != BRIDGE_TOKEN:
        return jsonify({'ok': False}), 403

    data    = request.get_json(force=True, silent=True) or {}
    running = bool(data.get('running', False))

    with _running_lock:
        global _running
        _running = running

    # Broadcast xuống tất cả browser đang mở SSE
    _broadcast({'running': running})

    return jsonify({'ok': True})


@ros_status_bp.route('/api/ros/status-stream')
def api_ros_status_stream():
    """
    SSE — browser kết nối 1 lần, nhận event mỗi khi bridge state thay đổi.
    Event format:  data: {"running": true|false}
    """
    q = queue.Queue(maxsize=10)
    with _sub_lock:
        _subscribers.append(q)

    # Gửi trạng thái hiện tại ngay khi browser connect
    with _running_lock:
        current = _running

    def generate():
        try:
            yield f'data: {json.dumps({"running": current})}\n\n'
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


# ── Internal ───────────────────────────────────────────────────────────────────
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


# ── Spawn / cleanup ────────────────────────────────────────────────────────────
def start_bridge_checker():
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
        cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
    )

    def _pipe_log():
        time.sleep(0.5)
        for line in _checker_process.stdout:
            print(f'[BridgeChecker] {line}', end='', flush=True)

    threading.Thread(target=_pipe_log, daemon=True).start()


def stop_bridge_checker():
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