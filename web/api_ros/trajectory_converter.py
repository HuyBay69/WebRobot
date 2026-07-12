#!/usr/bin/env python3
"""
trajectory_converter.py — Port của carla_csv_to_esp32_json.py (do người dùng
cung cấp) thành module dùng trong Flask: nhận file CSV upload trực tiếp từ
trình duyệt (không cần lưu ra đĩa), chạy ĐÚNG thuật toán gốc (đọc waypoint →
làm mượt → ước lượng góc trục → phân loại hướng từng đoạn → lọc nhiễu bằng
run-length + majority filter → gộp thành các đoạn thẳng → tính điểm góc cua →
xuất chuỗi lệnh), trả về JSON cho web vẽ preview + gửi ESP32.

CỐ Ý giữ nguyên tên hàm/logic/tham số mặc định giống hệt file gốc người dùng
gửi (TURN_RADIUS, SMOOTH_WINDOW, DIRECTION_SPAN, MIN_RUN_EDGES...) — không đổi
gì về mặt thuật toán, chỉ đổi phần đọc/ghi I/O cho phù hợp Flask.
"""
from __future__ import annotations

import io
from typing import Optional

import numpy as np
import pandas as pd

# ============================================================
# CẤU HÌNH — giữ nguyên y hệt giá trị mặc định trong file gốc
# ============================================================
TIME_SCALE = 1.0
TIMESTAMP_UNIT = "s"
INVERT_TURN_DIRECTION = False
TURN_RADIUS = 7.0
AUTO_RADIUS_MARGIN = 0.45
SMOOTH_WINDOW = 5
DIRECTION_SPAN = 4
DIRECTION_VOTE_WINDOW = 7
MIN_RUN_EDGES = 5
FORCE_AXIS_ALIGNED = False


class ConversionError(ValueError):
    """Lỗi có thông điệp đã sẵn sàng hiển thị trực tiếp cho người dùng trên web."""
    pass


# ============================================================
# ĐỌC DỮ LIỆU (đổi từ đọc file trên đĩa -> đọc file-like object từ upload)
# ============================================================

def normalize_column_name(name: object) -> str:
    return (
        str(name)
        .strip()
        .lower()
        .replace(" ", "")
        .replace("_", "")
        .replace("-", "")
        .replace("(", "")
        .replace(")", "")
        .replace("[", "")
        .replace("]", "")
    )


def choose_xy_columns(df: pd.DataFrame) -> tuple[object, object]:
    normalized = {col: normalize_column_name(col) for col in df.columns}

    x_names = {
        "x", "posx", "positionx", "pointx", "coordx", "coordinatex",
        "xm", "xmeter", "xmeters",
    }
    y_names = {
        "y", "posy", "positiony", "pointy", "coordy", "coordinatey",
        "ym", "ymeter", "ymeters",
    }

    x_candidates = [col for col, name in normalized.items() if name in x_names]
    y_candidates = [col for col, name in normalized.items() if name in y_names]

    if x_candidates and y_candidates:
        return x_candidates[0], y_candidates[0]

    numeric_columns: list[object] = []
    for col in df.columns:
        converted = pd.to_numeric(df[col], errors="coerce")
        if converted.notna().sum() >= 2:
            numeric_columns.append(col)

    if len(numeric_columns) < 2:
        raise ConversionError(
            'Không tìm thấy cột toạ độ X, Y hợp lệ. File CSV cần có 2 cột số '
            'chứa toạ độ (đặt tên "x"/"y" hoặc tương tự để nhận diện chắc chắn '
            'hơn — hiện có các cột: ' + ', '.join(str(c) for c in df.columns) + ').'
        )

    return numeric_columns[0], numeric_columns[1]


def choose_timestamp_column(df: pd.DataFrame, x_col: object, y_col: object) -> object:
    normalized = {col: normalize_column_name(col) for col in df.columns}
    timestamp_names = {
        "t", "time", "timestamp", "stamp", "elapsed", "elapsedtime",
        "simtime", "simulationtime", "gametime", "rostime",
        "times", "timesecond", "timeseconds", "timeinseconds",
        "timems", "timestampms", "elapsedms", "milliseconds",
        "timeus", "timestampus", "timens", "timestampns",
    }

    candidates = [
        col for col, name in normalized.items()
        if col not in {x_col, y_col} and name in timestamp_names
    ]
    if candidates:
        return candidates[0]

    numeric_remaining: list[object] = []
    for col in df.columns:
        if col in {x_col, y_col}:
            continue
        converted = pd.to_numeric(df[col], errors="coerce")
        if converted.notna().sum() >= 2:
            numeric_remaining.append(col)

    if len(numeric_remaining) == 1:
        return numeric_remaining[0]

    raise ConversionError(
        f'Không xác định được cột thời gian (timestamp). File cần có đúng 1 cột '
        f'thời gian ngoài 2 cột toạ độ "{x_col}"/"{y_col}" — đặt tên "timestamp" '
        f'hoặc "t" để nhận diện chắc chắn hơn.'
    )


def parse_timestamps(series: pd.Series) -> np.ndarray:
    numeric = pd.to_numeric(series, errors="coerce")
    if numeric.notna().sum() >= 2:
        return numeric.to_numpy(dtype=float)

    datetime_values = pd.to_datetime(series, errors="coerce")
    if datetime_values.notna().sum() < 2:
        raise ConversionError('Cột thời gian không có đủ dữ liệu hợp lệ để đọc.')

    first_valid = datetime_values.dropna().iloc[0]
    return (datetime_values - first_valid).dt.total_seconds().to_numpy(dtype=float)


def read_waypoints(csv_stream) -> tuple[np.ndarray, np.ndarray, object]:
    try:
        df = pd.read_csv(csv_stream)
    except Exception as e:
        raise ConversionError(f'Không đọc được file CSV: {e}')

    if df.empty:
        raise ConversionError('File CSV rỗng, không có dòng dữ liệu nào.')

    x_col, y_col = choose_xy_columns(df)
    timestamp_col = choose_timestamp_column(df, x_col, y_col)

    x = pd.to_numeric(df[x_col], errors="coerce")
    y = pd.to_numeric(df[y_col], errors="coerce")
    timestamps_raw = pd.Series(parse_timestamps(df[timestamp_col]), index=df.index)

    valid = x.notna() & y.notna() & timestamps_raw.notna()
    points = np.column_stack(
        (
            x.loc[valid].to_numpy(dtype=float),
            y.loc[valid].to_numpy(dtype=float),
        )
    )
    timestamps = timestamps_raw.loc[valid].to_numpy(dtype=float)

    if len(points) < 3:
        raise ConversionError(
            f'Cần ít nhất 3 waypoint có đủ X, Y và thời gian hợp lệ — file hiện '
            f'chỉ có {len(points)} điểm hợp lệ sau khi lọc.'
        )

    delta = np.diff(points, axis=0)
    keep = np.r_[True, np.linalg.norm(delta, axis=1) > 1e-12]
    points = points[keep]
    timestamps = timestamps[keep]

    if len(points) < 3:
        raise ConversionError(
            f'Sau khi bỏ các điểm trùng vị trí, chỉ còn {len(points)} điểm — '
            f'cần ít nhất 3 điểm khác vị trí nhau để tạo được quỹ đạo.'
        )

    if np.any(np.diff(timestamps) < 0):
        raise ConversionError('Thời gian (timestamp) không tăng dần theo thứ tự các điểm trong file.')

    return points, timestamps, timestamp_col


# ============================================================
# XỬ LÝ QUỸ ĐẠO — giữ nguyên y hệt thuật toán gốc
# ============================================================

def moving_average(values: np.ndarray, window: int) -> np.ndarray:
    if window <= 1 or len(values) < 3:
        return values.copy()

    window = min(window, len(values))
    if window % 2 == 0:
        window -= 1
    if window <= 1:
        return values.copy()

    pad = window // 2
    padded = np.pad(values, (pad, pad), mode="edge")
    kernel = np.ones(window, dtype=float) / window
    return np.convolve(padded, kernel, mode="valid")


def smooth_points(points: np.ndarray, window: int) -> np.ndarray:
    result = np.empty_like(points, dtype=float)
    result[:, 0] = moving_average(points[:, 0], window)
    result[:, 1] = moving_average(points[:, 1], window)
    return result


def estimate_orthogonal_angle(points: np.ndarray) -> float:
    if FORCE_AXIS_ALIGNED:
        return 0.0

    steps = np.diff(points, axis=0)
    lengths = np.linalg.norm(steps, axis=1)
    valid = lengths > 1e-12

    if not np.any(valid):
        return 0.0

    steps = steps[valid]
    lengths = lengths[valid]
    angles = np.arctan2(steps[:, 1], steps[:, 0])

    sin_sum = np.sum(lengths * np.sin(4.0 * angles))
    cos_sum = np.sum(lengths * np.cos(4.0 * angles))
    return 0.25 * np.arctan2(sin_sum, cos_sum)


def rotation_matrix(angle: float) -> np.ndarray:
    c = np.cos(angle)
    s = np.sin(angle)
    return np.array([[c, -s], [s, c]], dtype=float)


def rotate_points(points: np.ndarray, angle: float, center: np.ndarray) -> np.ndarray:
    return (points - center) @ rotation_matrix(angle).T + center


def classify_edge_directions(points: np.ndarray, span: int) -> np.ndarray:
    labels = np.zeros(len(points) - 1, dtype=np.int8)
    span = max(1, int(span))

    for edge_index in range(len(points) - 1):
        left = max(0, edge_index - span + 1)
        right = min(len(points) - 1, edge_index + span)
        dx = points[right, 0] - points[left, 0]
        dy = points[right, 1] - points[left, 1]
        labels[edge_index] = 0 if abs(dx) >= abs(dy) else 1

    return labels


def majority_filter(labels: np.ndarray, window: int) -> np.ndarray:
    if len(labels) == 0 or window <= 1:
        return labels.copy()

    window = min(window, len(labels))
    if window % 2 == 0:
        window -= 1
    if window <= 1:
        return labels.copy()

    radius = window // 2
    result = labels.copy()

    for i in range(len(labels)):
        left = max(0, i - radius)
        right = min(len(labels), i + radius + 1)
        neighborhood = labels[left:right]
        ones = int(np.sum(neighborhood))
        zeros = len(neighborhood) - ones
        result[i] = 1 if ones > zeros else 0

    return result


def run_length_encode(labels: np.ndarray) -> list[list[int]]:
    if len(labels) == 0:
        return []

    runs: list[list[int]] = []
    start = 0
    current = int(labels[0])

    for i in range(1, len(labels)):
        label = int(labels[i])
        if label != current:
            runs.append([start, i - 1, current])
            start = i
            current = label

    runs.append([start, len(labels) - 1, current])
    return runs


def merge_adjacent_same_runs(runs: list[list[int]]) -> list[list[int]]:
    if not runs:
        return []

    merged = [runs[0].copy()]
    for start, end, label in runs[1:]:
        if merged[-1][2] == label:
            merged[-1][1] = end
        else:
            merged.append([start, end, label])
    return merged


def remove_short_runs(labels: np.ndarray, min_run_edges: int) -> np.ndarray:
    result = labels.copy()
    min_run_edges = max(1, int(min_run_edges))

    for _ in range(100):
        runs = run_length_encode(result)
        short_index = next(
            (
                i for i, (start, end, _label) in enumerate(runs)
                if end - start + 1 < min_run_edges
            ),
            None,
        )

        if short_index is None or len(runs) == 1:
            break

        start, end, _label = runs[short_index]

        if short_index == 0:
            replacement = runs[1][2]
        elif short_index == len(runs) - 1:
            replacement = runs[-2][2]
        else:
            left_length = runs[short_index - 1][1] - runs[short_index - 1][0] + 1
            right_length = runs[short_index + 1][1] - runs[short_index + 1][0] + 1
            replacement = (
                runs[short_index - 1][2]
                if left_length >= right_length
                else runs[short_index + 1][2]
            )

        result[start:end + 1] = replacement

    return result


def fit_mean_lines(rotated_points: np.ndarray, labels: np.ndarray) -> tuple[np.ndarray, list[dict]]:
    runs = merge_adjacent_same_runs(run_length_encode(labels))
    if not runs:
        raise ConversionError(
            'Không tạo được đoạn thẳng nào từ quỹ đạo — dữ liệu có thể quá ngắn '
            'hoặc quá nhiễu (xe lái không theo mẫu hình các đoạn thẳng + góc vuông rõ ràng).'
        )

    line_data: list[dict] = []

    for segment_index, (start_edge, end_edge, label) in enumerate(runs):
        start_point = start_edge
        end_point = end_edge + 1
        segment_points = rotated_points[start_point:end_point + 1]

        if label == 0:
            fixed_coordinate = float(np.mean(segment_points[:, 1]))
            direction_name = "horizontal"
        else:
            fixed_coordinate = float(np.mean(segment_points[:, 0]))
            direction_name = "vertical"

        line_data.append(
            {
                "segment_index": segment_index,
                "start_point": start_point,
                "end_point": end_point,
                "label": label,
                "direction": direction_name,
                "fixed_coordinate": fixed_coordinate,
            }
        )

    vertices: list[np.ndarray] = []

    first = line_data[0]
    first_point = rotated_points[int(first["start_point"])].copy()
    if int(first["label"]) == 0:
        first_point[1] = float(first["fixed_coordinate"])
    else:
        first_point[0] = float(first["fixed_coordinate"])
    vertices.append(first_point)

    for previous, current in zip(line_data[:-1], line_data[1:]):
        if int(previous["label"]) == int(current["label"]):
            raise ConversionError('Lỗi nội bộ: hai đoạn liên tiếp cùng hướng sau khi gộp.')

        if int(previous["label"]) == 0:
            x_value = float(current["fixed_coordinate"])
            y_value = float(previous["fixed_coordinate"])
        else:
            x_value = float(previous["fixed_coordinate"])
            y_value = float(current["fixed_coordinate"])

        vertices.append(np.array([x_value, y_value], dtype=float))

    last = line_data[-1]
    last_point = rotated_points[int(last["end_point"])].copy()
    if int(last["label"]) == 0:
        last_point[1] = float(last["fixed_coordinate"])
    else:
        last_point[0] = float(last["fixed_coordinate"])
    vertices.append(last_point)

    return np.vstack(vertices), line_data


def normalize_vector(vector: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(vector)
    if norm <= 1e-12:
        raise ConversionError('Có 2 điểm liên tiếp trùng vị trí nhau (vector độ dài 0) khi tính điểm góc cua.')
    return vector / norm


def build_corner_tangents(vertices: np.ndarray) -> list[dict]:
    corners: list[dict] = []

    for i in range(1, len(vertices) - 1):
        previous = vertices[i - 1]
        corner = vertices[i]
        following = vertices[i + 1]

        vector_in = corner - previous
        vector_out = following - corner
        length_in = np.linalg.norm(vector_in)
        length_out = np.linalg.norm(vector_out)

        if length_in <= 1e-9 or length_out <= 1e-9:
            continue

        direction_in = normalize_vector(vector_in)
        direction_out = normalize_vector(vector_out)

        if abs(float(np.dot(direction_in, direction_out))) > 1e-3:
            continue

        radius = min(TURN_RADIUS, AUTO_RADIUS_MARGIN * min(length_in, length_out))
        if radius <= 1e-9:
            continue

        tangent_a = corner - radius * direction_in
        tangent_b = corner + radius * direction_out
        cross = float(direction_in[0] * direction_out[1] - direction_in[1] * direction_out[0])

        corners.append(
            {
                "corner_index": i,
                "tangent_a": tangent_a,
                "tangent_b": tangent_b,
                "turn_type": "left" if cross > 0 else "right",
            }
        )

    return corners


# ============================================================
# TẠO CHUỖI LỆNH JSON
# ============================================================

def timestamp_multiplier_to_ms(timestamp_column: object) -> float:
    unit = TIMESTAMP_UNIT.strip().lower()

    if unit == "auto":
        name = normalize_column_name(timestamp_column)
        if "ns" in name or "nanosecond" in name:
            unit = "ns"
        elif "us" in name or "microsecond" in name:
            unit = "us"
        elif "ms" in name or "millisecond" in name:
            unit = "ms"
        else:
            unit = "s"

    multipliers = {
        "s": 1000.0, "sec": 1000.0, "second": 1000.0, "seconds": 1000.0,
        "ms": 1.0, "millisecond": 1.0, "milliseconds": 1.0,
        "us": 0.001, "microsecond": 0.001, "microseconds": 0.001,
        "ns": 0.000001, "nanosecond": 0.000001, "nanoseconds": 0.000001,
    }

    if unit not in multipliers:
        raise ConversionError("TIMESTAMP_UNIT phải là 'auto', 's', 'ms', 'us' hoặc 'ns'.")

    return multipliers[unit]


def nearest_waypoint_index(points: np.ndarray, target: np.ndarray, start_index: int, end_index: int) -> int:
    start_index = max(0, int(start_index))
    end_index = min(len(points) - 1, int(end_index))

    if start_index > end_index:
        raise ConversionError(f'Khoảng tìm waypoint không hợp lệ: {start_index} > {end_index}.')

    local_points = points[start_index:end_index + 1]
    distances = np.linalg.norm(local_points - target, axis=1)
    return start_index + int(np.argmin(distances))


def make_time_ms(start_timestamp: float, end_timestamp: float, timestamp_column: object, time_scale: float) -> int:
    if not 0.0 <= time_scale <= 1.0:
        raise ConversionError('Hệ số thời gian (time_scale) phải nằm trong khoảng từ 0.0 đến 1.0.')

    delta = float(end_timestamp - start_timestamp)
    if delta < 0:
        raise ConversionError('Thời gian kết thúc nhỏ hơn thời gian bắt đầu khi tính 1 đoạn thẳng.')

    raw_ms = delta * timestamp_multiplier_to_ms(timestamp_column)
    return max(0, int(round(raw_ms * time_scale)))


def build_commands(
    points: np.ndarray,
    timestamps: np.ndarray,
    timestamp_column: object,
    corner_data: list[dict],
    line_data: list[dict],
    time_scale: float,
) -> list[dict]:
    commands: list[dict] = []
    current_waypoint_index = 0

    for corner in corner_data:
        corner_index = int(corner["corner_index"])
        tangent_a = np.asarray(corner["tangent_a"], dtype=float)
        tangent_b = np.asarray(corner["tangent_b"], dtype=float)

        incoming_line = line_data[corner_index - 1]
        outgoing_line = line_data[corner_index]

        waypoint_a = nearest_waypoint_index(
            points, tangent_a,
            max(current_waypoint_index, int(incoming_line["start_point"])),
            int(incoming_line["end_point"]),
        )
        waypoint_b = nearest_waypoint_index(
            points, tangent_b,
            max(waypoint_a, int(outgoing_line["start_point"])),
            int(outgoing_line["end_point"]),
        )

        commands.append({
            "index": len(commands),
            "command": "straight",
            "time_ms": make_time_ms(timestamps[current_waypoint_index], timestamps[waypoint_a], timestamp_column, time_scale),
            "turn_angle": 0,
        })

        turn_angle = 90 if str(corner["turn_type"]) == "left" else -90
        if INVERT_TURN_DIRECTION:
            turn_angle *= -1

        commands.append({
            "index": len(commands),
            "command": "turn",
            "time_ms": 0,
            "turn_angle": turn_angle,
        })

        current_waypoint_index = waypoint_b

    commands.append({
        "index": len(commands),
        "command": "straight",
        "time_ms": make_time_ms(timestamps[current_waypoint_index], timestamps[-1], timestamp_column, time_scale),
        "turn_angle": 0,
    })

    # Lệnh kết thúc — luôn ở cuối cùng. Khi ESP32 chạy tới đây, KHÔNG di chuyển
    # gì thêm, chỉ gửi báo cáo tổng kết (số gói tin, tỉ lệ mất gói, độ trễ
    # trung bình) về Web rồi dừng hẳn — xem esp32_control.py + esp32_wifi_led.ino.
    commands.append({
        "index": len(commands),
        "command": "finish",
        "time_ms": 0,
        "turn_angle": 0,
    })

    return commands


# ============================================================
# HÀM CHÍNH — dùng trong Flask route
# ============================================================

def convert_csv_to_commands(csv_stream, time_scale: Optional[float] = None) -> tuple[list[dict], list[list[float]]]:
    """
    csv_stream: file-like object (vd request.files['csv'] từ Flask, hoặc
    io.BytesIO/StringIO) chứa nội dung CSV.

    time_scale: hệ số co giãn thời gian (0.0-1.0) — vd 0.01 sẽ khiến mọi đoạn
    thẳng chạy bằng 1/100 thời gian ghi được trong CARLA. Truyền từ web mỗi
    lần convert — KHÔNG còn đọc hằng số TIME_SCALE cố định trong file này nữa,
    để đổi giá trị không cần sửa code + khởi động lại Flask. None = dùng mặc
    định TIME_SCALE ở đầu file (1.0).

    Trả về (commands, preview_vertices):
      - commands: list dict [{"index","command","time_ms","turn_angle"}, ...]
        — đúng định dạng gửi cho ESP32.
      - preview_vertices: list [[x,y], ...] — các điểm góc của tuyến đường
        thẳng đã đơn giản hoá, để vẽ preview trên web (nối các điểm này lại
        bằng đường thẳng là ra đúng hình dạng lộ trình cuối cùng).

    Ném ConversionError với thông điệp đã sẵn sàng hiển thị cho người dùng nếu
    dữ liệu đầu vào không hợp lệ ở bất kỳ bước nào.
    """
    if time_scale is None:
        time_scale = TIME_SCALE

    points, timestamps, timestamp_column = read_waypoints(csv_stream)

    smoothed = smooth_points(points, SMOOTH_WINDOW)
    center = np.mean(smoothed, axis=0)
    base_angle = estimate_orthogonal_angle(smoothed)
    rotated = rotate_points(smoothed, -base_angle, center)

    labels = classify_edge_directions(rotated, DIRECTION_SPAN)
    labels = majority_filter(labels, DIRECTION_VOTE_WINDOW)
    labels = remove_short_runs(labels, MIN_RUN_EDGES)

    rotated_vertices, line_data = fit_mean_lines(rotated, labels)
    vertices = rotate_points(rotated_vertices, base_angle, center)
    corner_data = build_corner_tangents(vertices)

    commands = build_commands(points, timestamps, timestamp_column, corner_data, line_data, time_scale)
    preview_vertices = vertices.tolist()

    return commands, preview_vertices