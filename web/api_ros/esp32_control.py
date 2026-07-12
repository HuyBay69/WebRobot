#!/usr/bin/env python3
"""
esp32_control.py — Gửi chuỗi lệnh hình học (đi thẳng / quay trái / quay phải)
xuống xe ESP32 thật qua HTTP, và nhận lại tiến độ chạy để hiển thị real-time
trên web (trang chính, Bước 5).

QUAN TRỌNG (xem TihcHop.txt): KHÔNG đồng bộ thời gian thực với CARLA — đây là
demo pipeline chuyển quỹ đạo → phần cứng thật + demo độ ổn định kết nối không
dây, KHÔNG dùng để kiểm chứng thuật toán điều khiển (ad_agent/AEB vẫn đánh giá
riêng trong CARLA).

── Định dạng lệnh (JSON) — KHỚP ĐÚNG file carla_csv_to_esp32_json.py xuất ra ──
File xuất ra là 1 MẢNG JSON TRẦN (không bọc trong {"commands": [...]}), mỗi
phần tử có 4 trường:
[
  {"index": 0, "command": "straight", "time_ms": 12000, "turn_angle": 0},
  {"index": 1, "command": "turn",     "time_ms": 0,     "turn_angle": 90},
  {"index": 2, "command": "straight", "time_ms": 8500,  "turn_angle": 0},
  {"index": 3, "command": "turn",     "time_ms": 0,     "turn_angle": -90}
]
- command: "straight" | "turn"
- straight: dùng "time_ms" (mili-giây), "turn_angle" luôn = 0 (không dùng).
- turn: dùng "turn_angle" (độ, CÓ DẤU — dương = quay trái, âm = quay phải),
  "time_ms" luôn = 0 (không dùng — ESP32 tự canh thời gian quay bằng MPU).

── Chiều 1 — Web → ESP32 (gửi 1 lần toàn bộ) ────────────────────────────────
POST /api/esp32/send  { "esp32_ip": "10.42.0.205", "commands": [...] }
  → forward nguyên MẢNG TRẦN [...] (KHÔNG bọc {"commands":...}) tới http://<esp32_ip>/commands
    (HTTP POST, timeout 5s). ESP32 nhận 1 lần, tự chạy tuần tự từng lệnh.

── Chiều 2 — ESP32 → Web (báo tiến độ) ───────────────────────────────────────
ESP32 tự POST về mỗi khi BẮT ĐẦU và khi HOÀN THÀNH từng bước:
POST /api/esp32/progress  { "step_index": 0, "status": "running" }
POST /api/esp32/progress  { "step_index": 0, "status": "done" }
  → lưu lại + phát ngay qua SSE cho mọi trình duyệt đang mở Bước 5. Khi bước
    cuối cùng chuyển "done", tự phát thêm sự kiện {"type": "completed"}.

GET  /api/esp32/progress-stream   (SSE — browser lắng nghe, tự nhận ngay trạng
                                    thái hiện tại lúc vừa kết nối)
GET  /api/esp32/status            (poll dự phòng nếu trình duyệt không hỗ trợ SSE)
"""
import json
import queue
import threading
import time
import urllib.error
import urllib.request

from flask import Blueprint, jsonify, request, Response

from api_ros.trajectory_converter import convert_csv_to_commands, ConversionError

esp32_bp = Blueprint('esp32_bp', __name__)

ESP32_SEND_TIMEOUT_SEC = 5.0
SSE_KEEPALIVE_SEC = 15.0  # gửi comment rỗng định kỳ để giữ kết nối SSE sống qua proxy/timeout trình duyệt

_lock = threading.Lock()
_progress = []       # list[{index, status, description}] — trạng thái hiện tại từng bước
_subscribers = []    # list[queue.Queue] — mỗi trình duyệt đang mở Bước 5 có 1 hàng đợi riêng

# ── Trạng thái kết nối ESP32 — cập nhật bởi heartbeat + mọi báo cáo tiến độ ──
CONNECTION_TIMEOUT_SEC = 5.0  # quá lâu không nhận được gì -> coi là mất kết nối (frontend tự kiểm tra qua last_seen)

_connection = {
    'ip': None,
    'rssi': None,       # dBm, càng gần 0 càng mạnh (vd -50 tốt, -80 yếu)
    'last_seen': 0.0,   # time.time() lần cuối nhận được BẤT KỲ gói nào (heartbeat hoặc progress)
    # Bù lệch đồng hồ ESP32 (millis(), mốc 0 = lúc ESP32 khởi động) so với đồng
    # hồ hệ thống Flask (time.time(), mốc 0 = epoch) — thiết lập 1 lần tại gói
    # ĐẦU TIÊN nhận được mỗi phiên, dùng để ước lượng "thời điểm ESP32 gửi" quy
    # về cùng hệ quy chiếu với Flask mà KHÔNG cần đồng bộ giờ NTP giữa 2 máy.
    'esp32_baseline_millis': None,
    'flask_baseline_time': None,
}
_received_packet_indices = set()  # các packet_index đã THỰC SỰ nhận được (dùng phát hiện gói bị rớt)
_latency_samples_ms = []          # ước lượng độ trễ từng gói (ms), tính theo baseline offset ở trên


def _log(msg):
    print(f'[ESP32Control] {msg}', flush=True)


def _describe_command(cmd):
    """Mô tả người-đọc-được cho 1 lệnh — dùng để hiển thị trên danh sách tiến độ.
    Thời gian hiện theo đúng đơn vị gốc của "time_ms" trong dữ liệu đầu ra (ms),
    không quy đổi sang giây."""
    ctype = cmd.get('command')
    if ctype == 'straight':
        ms = cmd.get('time_ms') or 0
        return f'Đi thẳng {ms} ms'
    if ctype == 'turn':
        angle = cmd.get('turn_angle', 0)
        direction = 'trái' if angle > 0 else 'phải'
        return f'Quay {direction} {abs(angle):g}°'
    if ctype == 'finish':
        return 'Hoàn thành hành trình'
    return f'Lệnh không rõ ({ctype})'


def _broadcast(event):
    """Đẩy 1 sự kiện JSON tới mọi trình duyệt đang lắng nghe SSE."""
    with _lock:
        subs = list(_subscribers)
    for q in subs:
        try:
            q.put_nowait(event)
        except Exception:
            pass


def _record_packet(data):
    """Cập nhật trạng thái kết nối/gói tin/độ trễ từ 1 gói bất kỳ nhận được từ
    ESP32 (heartbeat hoặc báo cáo tiến độ) — gọi ở MỌI route ESP32 gửi lên để
    theo dõi được liên tục, không chỉ lúc đang chạy lệnh."""
    now = time.time()
    with _lock:
        if data.get('ip'):
            _connection['ip'] = data['ip']
        if 'rssi' in data:
            _connection['rssi'] = data['rssi']
        _connection['last_seen'] = now

        packet_index = data.get('packet_index')
        if packet_index is not None:
            _received_packet_indices.add(packet_index)

        esp32_millis = data.get('esp32_millis')
        if esp32_millis is not None:
            if _connection['esp32_baseline_millis'] is None:
                # Gói đầu tiên của phiên — lấy làm mốc quy đổi, không tính độ trễ cho chính nó.
                _connection['esp32_baseline_millis'] = esp32_millis
                _connection['flask_baseline_time'] = now
            else:
                estimated_send_time = (
                    _connection['flask_baseline_time']
                    + (esp32_millis - _connection['esp32_baseline_millis']) / 1000.0
                )
                latency_ms = (now - estimated_send_time) * 1000.0
                # Lọc bỏ giá trị vô lý (vd millis() tràn số sau ~49 ngày, hoặc
                # gói tới trước cả mốc ước lượng do sai số nhỏ ban đầu).
                if -1000.0 < latency_ms < 30000.0:
                    _latency_samples_ms.append(latency_ms)


def _reset_connection_stats():
    """Xoá thống kê gói tin/độ trễ của phiên trước — gọi mỗi khi bắt đầu gửi
    hành trình mới (KHÔNG xoá trạng thái kết nối ip/rssi/last_seen, vì ESP32
    vẫn đang kết nối bình thường, chỉ là bắt đầu đếm lại từ đầu cho phiên mới)."""
    with _lock:
        _received_packet_indices.clear()
        _latency_samples_ms.clear()
        _connection['esp32_baseline_millis'] = None
        _connection['flask_baseline_time'] = None


# ── Chiều 1: Web → ESP32 ─────────────────────────────────────────────────────
@esp32_bp.route('/api/esp32/convert_csv', methods=['POST'])
def api_esp32_convert_csv():
    """
    Nhận file CSV quỹ đạo (multipart/form-data, field name "csv"), chạy thuật
    toán trong trajectory_converter.py (port từ carla_csv_to_esp32_json.py),
    trả về chuỗi lệnh + toạ độ để web vẽ preview trước khi gửi ESP32.

    Body form field tuỳ chọn "time_scale" (0.0-1.0) — hệ số co giãn thời gian
    mỗi đoạn thẳng, gửi từ ô nhập trên web. Bỏ trống = dùng mặc định 1.0.

    Trả về: { "ok": true, "commands": [...], "preview_vertices": [[x,y],...] }
    hoặc     { "ok": false, "error": "..." } (400) nếu file không hợp lệ.
    """
    if 'csv' not in request.files or request.files['csv'].filename == '':
        return jsonify({'ok': False, 'error': 'Chưa chọn file CSV'}), 400

    csv_file = request.files['csv']

    time_scale = None
    raw_time_scale = request.form.get('time_scale', '').strip()
    if raw_time_scale:
        try:
            time_scale = float(raw_time_scale)
        except ValueError:
            return jsonify({'ok': False, 'error': f'Hệ số thời gian không hợp lệ: "{raw_time_scale}"'}), 400
        if not 0.0 <= time_scale <= 1.0:
            return jsonify({'ok': False, 'error': 'Hệ số thời gian phải nằm trong khoảng 0.0 - 1.0'}), 400

    try:
        commands, preview_vertices = convert_csv_to_commands(csv_file, time_scale=time_scale)
    except ConversionError as e:
        return jsonify({'ok': False, 'error': str(e)}), 400
    except Exception as e:
        _log(f'✗ Lỗi không xác định khi convert CSV: {e}')
        return jsonify({'ok': False, 'error': f'Lỗi không xác định khi xử lý file: {e}'}), 500

    _log(f'✓ Đã convert CSV → {len(commands)} lệnh, {len(preview_vertices)} điểm preview.')
    return jsonify({'ok': True, 'commands': commands, 'preview_vertices': preview_vertices})


@esp32_bp.route('/api/esp32/send', methods=['POST'])
def api_esp32_send():
    global _progress

    data = request.get_json(force=True, silent=True) or {}
    esp32_ip = (data.get('esp32_ip') or '').strip()
    commands = data.get('commands')

    if not esp32_ip:
        return jsonify({'ok': False, 'error': 'Thiếu địa chỉ IP ESP32'}), 400
    if not isinstance(commands, list) or not commands:
        return jsonify({'ok': False, 'error': 'Danh sách lệnh rỗng hoặc không hợp lệ'}), 400

    with _lock:
        # Lệnh "finish" (luôn ở cuối, do trajectory_converter.py tự thêm) không
        # hiện trong danh sách "Bước N" — nó không phải bước di chuyển, chỉ là
        # tín hiệu kết thúc + báo cáo tổng kết, xử lý riêng ở nhánh status="finish"
        # bên dưới (ESP32 không gọi reportProgress() theo step_index cho nó, nên
        # bỏ khỏi _progress không làm lệch chỉ số các bước còn lại — "finish"
        # luôn ở vị trí cuối cùng nên bỏ nó đi không dịch chuyển index nào khác).
        _progress = [
            {'index': i, 'status': 'pending', 'description': _describe_command(c)}
            for i, c in enumerate(commands)
            if c.get('command') != 'finish'
        ]
        progress_copy = list(_progress)

    _reset_connection_stats()
    _broadcast({'type': 'reset', 'progress': progress_copy})

    url = f'http://{esp32_ip}/commands'
    body = json.dumps(commands).encode('utf-8')  # MẢNG TRẦN — khớp đúng định dạng carla_csv_to_esp32_json.py xuất ra, ESP32 firmware parse trực tiếp doc.as<JsonArray>()
    req = urllib.request.Request(
        url, data=body, headers={'Content-Type': 'application/json'}, method='POST'
    )

    try:
        with urllib.request.urlopen(req, timeout=ESP32_SEND_TIMEOUT_SEC) as resp:
            resp_body = resp.read().decode('utf-8', errors='replace')
        _log(f'✓ Đã gửi {len(commands)} lệnh tới {esp32_ip} — phản hồi: {resp_body[:200]}')
    except urllib.error.URLError as e:
        _log(f'✗ Không kết nối được ESP32 tại {esp32_ip}: {e}')
        return jsonify({'ok': False, 'error': f'Không kết nối được ESP32 tại {esp32_ip}: {e}'}), 502
    except Exception as e:
        _log(f'✗ Lỗi không xác định khi gửi lệnh: {e}')
        return jsonify({'ok': False, 'error': str(e)}), 500

    return jsonify({'ok': True, 'sent': len(commands)})


# ── Chiều 2: ESP32 → Web ─────────────────────────────────────────────────────
@esp32_bp.route('/api/esp32/progress', methods=['POST'])
def api_esp32_progress():
    """ESP32 tự gọi endpoint này để báo tiến độ — KHÔNG phải trình duyệt gọi.

    status="finish" là tín hiệu ĐẶC BIỆT báo đã tới điểm cuối cùng của hành
    trình (ứng với lệnh command="finish" luôn ở cuối chuỗi lệnh) — không gắn
    với step_index nào, mang theo "total_packets_sent" để tính tỉ lệ mất gói,
    và là nơi DUY NHẤT phát sự kiện "completed" (không còn tự suy luận "đã
    xong" qua việc mọi bước đều done như trước — ESP32 nói chính xác lúc nào
    thật sự kết thúc)."""
    data = request.get_json(force=True, silent=True) or {}
    status = data.get('status')

    _record_packet(data)  # cập nhật ip/rssi/độ trễ/gói tin cho MỌI báo cáo, kể cả finish

    if status == 'finish':
        with _lock:
            expected = data.get('total_packets_sent')
            received = len(_received_packet_indices)
            loss_pct = None
            if isinstance(expected, int) and expected > 0:
                loss_pct = round(max(0.0, 1.0 - received / expected) * 100, 1)
            avg_latency_ms = (
                round(sum(_latency_samples_ms) / len(_latency_samples_ms), 1)
                if _latency_samples_ms else None
            )
            summary = {
                'expected_packets': expected,
                'received_packets': received,
                'loss_pct': loss_pct,
                'avg_latency_ms': avg_latency_ms,
            }

        _broadcast({'type': 'completed', 'summary': summary})
        _log(f'✓ ESP32 báo hoàn thành hành trình — {summary}')
        return jsonify({'ok': True})

    step_index = data.get('step_index')
    if not isinstance(step_index, int) or status not in ('running', 'done'):
        return jsonify({'ok': False, 'error': 'Cần step_index (int) và status ("running"|"done"|"finish")'}), 400

    with _lock:
        if not (0 <= step_index < len(_progress)):
            return jsonify({
                'ok': False,
                'error': f'step_index {step_index} ngoài phạm vi (đang theo dõi {len(_progress)} bước — có thể chưa gửi lệnh hoặc ESP32 đang chạy chuỗi lệnh cũ)',
            }), 400
        _progress[step_index]['status'] = status

    _broadcast({'type': 'update', 'index': step_index, 'status': status})
    return jsonify({'ok': True})


@esp32_bp.route('/api/esp32/heartbeat', methods=['POST'])
def api_esp32_heartbeat():
    """ESP32 tự gọi định kỳ (vd mỗi 2s) để báo còn sống + chất lượng Wi-Fi —
    độc lập với việc có đang chạy lệnh hay không, để panel kết nối trên web
    luôn cập nhật (kể cả lúc ESP32 rảnh, chưa nhận chuỗi lệnh nào)."""
    data = request.get_json(force=True, silent=True) or {}
    _record_packet(data)
    with _lock:
        ip, rssi = _connection['ip'], _connection['rssi']
    _broadcast({'type': 'heartbeat', 'ip': ip, 'rssi': rssi})
    return jsonify({'ok': True})


@esp32_bp.route('/api/esp32/connection_status', methods=['GET'])
def api_esp32_connection_status():
    """Poll dự phòng cho panel kết nối, nếu SSE lỗi."""
    with _lock:
        connected = bool(_connection['last_seen']) and (time.time() - _connection['last_seen']) < CONNECTION_TIMEOUT_SEC
        return jsonify({
            'ok': True,
            'connected': connected,
            'ip': _connection['ip'],
            'rssi': _connection['rssi'],
        })


@esp32_bp.route('/api/esp32/status', methods=['GET'])
def api_esp32_status():
    """Poll dự phòng — trạng thái hiện tại toàn bộ danh sách bước."""
    with _lock:
        return jsonify({'ok': True, 'progress': list(_progress)})


@esp32_bp.route('/api/esp32/progress-stream', methods=['GET'])
def api_esp32_progress_stream():
    """SSE — trình duyệt lắng nghe để cập nhật tiến độ real-time, không cần polling."""
    client_queue = queue.Queue()
    with _lock:
        _subscribers.append(client_queue)
        initial_progress = list(_progress)

    def stream():
        # Gửi ngay trạng thái hiện tại cho client vừa kết nối, để không phải
        # chờ tới sự kiện tiếp theo mới thấy gì (vd mở lại modal giữa chừng).
        yield f'data: {json.dumps({"type": "reset", "progress": initial_progress})}\n\n'
        try:
            while True:
                try:
                    event = client_queue.get(timeout=SSE_KEEPALIVE_SEC)
                    yield f'data: {json.dumps(event)}\n\n'
                except queue.Empty:
                    yield ': keep-alive\n\n'
        except GeneratorExit:
            pass
        finally:
            with _lock:
                if client_queue in _subscribers:
                    _subscribers.remove(client_queue)

    return Response(stream(), mimetype='text/event-stream')