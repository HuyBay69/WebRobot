#!/usr/bin/env python3
"""
carla_node.py — Quản lý tiến trình CARLA simulator (Bước 1: Khởi động môi trường 3D CARLA).

Chạy CarlaUE4.sh dưới dạng subprocess nền (không blocking request Flask).
Cung cấp API để home.js start / stop / kiểm tra trạng thái môi trường CARLA.

Placeholder cho tương lai: `stop_carla()` được tách thành hàm riêng (không gắn
chặt vào route) để có thể được gọi lại từ một nơi khác — ví dụ nút tắt hệ thống
ở trang điều khiển (Bước 3) — khi tính năng đó được bổ sung sau này.
"""
import os
import signal
import subprocess
import threading
import time

from flask import Blueprint, jsonify, request

carla_bp = Blueprint('carla_bp', __name__)

CARLA_SH_PATH = os.path.expanduser('~/CARLA/carla_packed_linux/CarlaUE4.sh')

_lock = threading.Lock()
_proc = None  # subprocess.Popen hiện tại (None nếu CARLA chưa chạy)


def is_running():
    with _lock:
        return _proc is not None and _proc.poll() is None


def start_carla(render: bool):
    """Khởi động CarlaUE4.sh dưới dạng subprocess nền, chạy liên tục cho đến khi bị dừng.
    render=True  → chạy có hiển thị màn hình (mặc định).
    render=False → thêm cờ -RenderOffScreen, chạy ngầm, giảm tải cho máy.
    """
    global _proc
    with _lock:
        if _proc is not None and _proc.poll() is None:
            return False, 'CARLA đã đang chạy'

        if not os.path.isfile(CARLA_SH_PATH):
            return False, f'Không tìm thấy file: {CARLA_SH_PATH}'

        env = os.environ.copy()
        env['__NV_PRIME_RENDER_OFFLOAD'] = '1'
        env['__GLX_VENDOR_LIBRARY_NAME'] = 'nvidia'

        cmd = ['bash', CARLA_SH_PATH, '-quality-level=Low', '-vulkan']
        if not render:
            cmd.append('-RenderOffScreen')

        try:
            _proc = subprocess.Popen(
                cmd,
                env=env,
                start_new_session=True,  # tạo process group riêng để có thể kill sạch (kể cả tiến trình con)
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except Exception as e:
            return False, str(e)

        return True, 'started'


def stop_carla(wait_seconds=15, block=True):
    """Dừng CARLA đang chạy (nếu có).

    CarlaUE4 (Unreal Engine) thường KHÔNG thoát ngay với 1 lần Ctrl+C — SIGINT
    đầu tiên chỉ kích hoạt quy trình thoát "êm" của engine. Giống hệt thao tác
    thủ công trên terminal: phải bấm Ctrl+C thêm 1 lần nữa NGAY SAU đó (cách
    nhau một khoảng ngắn, không phải chờ vài giây) thì CarlaUE4 mới chấp nhận
    thoát ngay lập tức. Nếu khoảng cách giữa 2 lần quá xa, lần SIGINT thứ 2 chỉ
    bị coi như một lần bấm bình thường khác chứ không có tác dụng "double-tap"
    ép thoát — đây là lý do cách cũ (3 lần, cách nhau 2s) không ổn định.

    Vì vậy ở đây gửi SIGINT (đúng tín hiệu Ctrl+C) 2 lần, cách nhau một
    khoảng ngắn (_DOUBLE_TAP_GAP); nếu sau đó vẫn còn sống mới fallback sang
    SIGTERM rồi SIGKILL để đảm bảo dọn sạch.

    block=True  → chờ tối đa `wait_seconds` giây cho tiến trình thoát hẳn rồi mới return
                  (dùng khi tắt hẳn Web Server, cần dọn dẹp sạch trước khi thoát).
    block=False → gửi tín hiệu dừng rồi return ngay (không chặn request Flask); một thread
                  nền sẽ tiếp tục theo dõi và gửi thêm tín hiệu / SIGKILL nếu quá hạn. Dùng
                  cho nút "Tạm dừng" trên giao diện — nơi phần hiển thị "đang tắt" / "đã tắt"
                  do frontend tự đếm thời gian.
    """
    global _proc
    with _lock:
        proc = _proc
        if proc is None or proc.poll() is not None:
            _proc = None
            return False, 'CARLA không chạy'
        try:
            pgid = os.getpgid(proc.pid)
        except ProcessLookupError:
            pgid = None

    def _send(sig):
        try:
            if pgid is not None:
                os.killpg(pgid, sig)
            else:
                proc.send_signal(sig)
        except (ProcessLookupError, PermissionError):
            pass

    _DOUBLE_TAP_GAP = 0.3  # giây — khoảng cách giữa 2 lần Ctrl+C, mô phỏng thao tác bấm tay nhanh

    def _wait_and_force_kill():
        global _proc

        # "Double-tap" Ctrl+C: SIGINT lần 1 để engine bắt đầu thoát êm, SIGINT
        # lần 2 gửi ngay sau đó (cách nhau rất ngắn) để ép thoát thật sự —
        # giống hệt cách bấm tay. Bỏ qua lần 2 nếu tiến trình đã thoát rồi.
        _send(signal.SIGINT)
        time.sleep(_DOUBLE_TAP_GAP)
        if proc.poll() is None:
            _send(signal.SIGINT)

        try:
            remaining = max(wait_seconds - _DOUBLE_TAP_GAP, 5)
            proc.wait(timeout=remaining)
        except subprocess.TimeoutExpired:
            # Vẫn còn sống sau double-tap Ctrl+C — leo thang: SIGTERM rồi SIGKILL.
            _send(signal.SIGTERM)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                _send(signal.SIGKILL)
        except Exception:
            pass

        with _lock:
            if _proc is proc:
                _proc = None

    if block:
        _wait_and_force_kill()
    else:
        threading.Thread(target=_wait_and_force_kill, daemon=True).start()

    return True, 'stopping'


# ── Routes ─────────────────────────────────────────────────────────────────────
@carla_bp.route('/api/carla/start', methods=['POST'])
def api_carla_start():
    data = request.get_json(force=True, silent=True) or {}
    render = bool(data.get('render', True))
    ok, msg = start_carla(render)
    if not ok:
        return jsonify({'ok': False, 'error': msg}), 400
    return jsonify({'ok': True})


@carla_bp.route('/api/carla/stop', methods=['POST'])
def api_carla_stop():
    # block=False: trả lời ngay, không chờ CARLA thoát hẳn — khớp với việc
    # frontend tự hiển thị log "đang tắt" rồi đợi 15s theo timer riêng của nó.
    ok, msg = stop_carla(wait_seconds=15, block=False)
    if not ok:
        return jsonify({'ok': False, 'error': msg}), 400
    return jsonify({'ok': True})


@carla_bp.route('/api/carla/status', methods=['GET'])
def api_carla_status():
    # Placeholder cho tương lai — có thể được nút tắt hệ thống ở trang điều
    # khiển (Bước 3) gọi để biết CARLA có đang chạy hay không trước khi tắt.
    return jsonify({'ok': True, 'running': is_running()})