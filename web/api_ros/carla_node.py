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

from flask import Blueprint, jsonify, request

carla_bp = Blueprint('carla_bp', __name__)

CARLA_SH_PATH = os.path.expanduser('~/CARLA/carla_packed_linux/CarlaUE4.sh')

_lock = threading.Lock()
_proc = None  # subprocess.Popen hiện tại (None nếu CARLA chưa chạy)


def _log(msg):
    print(f'[CarlaNode] {msg}', flush=True)


def _get_descendant_pids(pid):
    """Quét /proc lấy toàn bộ PID con/cháu (đệ quy) của `pid`. Dùng làm lớp dự
    phòng khi SIGTERM không đủ để CarlaUE4.sh + tiến trình UE4 thật sự thoát
    hết (vd: crash reporter/shader-compile worker còn sót) — không phụ thuộc
    process group nên chắc chắn quét được toàn bộ cây, giống pattern đã dùng
    ở ros_bridge_node.py / navigation_stack_node.py."""
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


def start_carla(render: bool):
    """Khởi động CarlaUE4.sh dưới dạng subprocess nền, chạy liên tục cho đến khi bị dừng.
    render=True  → chạy có hiển thị màn hình (mặc định).
    render=False → thêm cờ -RenderOffScreen, chạy ngầm, giảm tải cho máy.
    """
    global _proc
    with _lock:
        if _proc is not None and _proc.poll() is None:
            _log('Đã đang chạy, bỏ qua lệnh khởi động mới.')
            return False, 'CARLA đã đang chạy'

        if not os.path.isfile(CARLA_SH_PATH):
            _log(f'✗ Không tìm thấy file: {CARLA_SH_PATH}')
            return False, f'Không tìm thấy file: {CARLA_SH_PATH}'

        env = os.environ.copy()
        env['__NV_PRIME_RENDER_OFFLOAD'] = '1'
        env['__GLX_VENDOR_LIBRARY_NAME'] = 'nvidia'

        cmd = ['bash', CARLA_SH_PATH, '-quality-level=Low', '-vulkan']
        if not render:
            cmd.append('-RenderOffScreen')

        _log(f'Đang khởi chạy: {" ".join(cmd)}  (render={render})')
        try:
            _proc = subprocess.Popen(
                cmd,
                env=env,
                start_new_session=True,  # tạo process group riêng để có thể kill sạch (kể cả tiến trình con)
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except Exception as e:
            _log(f'✗ Lỗi khởi chạy: {e}')
            return False, str(e)

        _log(f'✓ Đã chạy — PID={_proc.pid}')
        return True, 'started'


def stop_carla(wait_seconds=15, block=True):
    """Dừng CARLA đang chạy (nếu có).

    block=True  → chờ tối đa `wait_seconds` giây cho tiến trình thoát hẳn rồi mới return
                  (dùng khi tắt hẳn Web Server, cần dọn dẹp sạch trước khi thoát).
    block=False → gửi tín hiệu dừng rồi return ngay (không chặn request Flask); một thread
                  nền sẽ tiếp tục theo dõi và SIGKILL nếu quá hạn. Dùng cho nút "Tạm dừng"
                  trên giao diện — nơi phần hiển thị "đang tắt" / "đã tắt" do frontend tự
                  đếm thời gian.
    """
    global _proc
    with _lock:
        proc = _proc
        if proc is None or proc.poll() is not None:
            _proc = None
            _log('Không có tiến trình nào đang chạy, bỏ qua lệnh dừng.')
            return False, 'CARLA không chạy'
        _log(f'Đang dừng (PID={proc.pid}) — gửi SIGTERM...')
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except (ProcessLookupError, PermissionError):
            pass

    def _wait_and_force_kill():
        global _proc
        try:
            proc.wait(timeout=wait_seconds)
            _log('✓ Đã thoát sạch.')
        except subprocess.TimeoutExpired:
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