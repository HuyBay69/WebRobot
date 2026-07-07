#!/usr/bin/env python3
"""
fetch_spawn_points.py — Kết nối tới CARLA, lấy danh sách spawn point của bản đồ
hiện tại (world đang chạy), trích x, y, z, yaw (roll = pitch = 0 theo yêu cầu),
in ra stdout dưới dạng MỘT dòng JSON duy nhất — KHÔNG ghi ra file.

LƯU Ý: giá trị `y` trả về đã bị ĐẢO DẤU (y = -sp.location.y) để khớp quy ước
y_new = -y_ros dùng khi sinh waypoints.json / vẽ map ở app.js (rosToPixel()).
Nếu không đảo, spawn point sẽ hiển thị sai vị trí (đối xứng qua trục X) trên map.

`yaw` cũng bị ĐẢO DẤU (yaw = -sp.rotation.yaw) vì lý do tương tự: lật trục Y
(phản chiếu qua trục X) làm đảo luôn chiều quay của góc — nếu chỉ đảo Y mà
không đảo yaw, hướng xe sẽ bị lệch so với vị trí đã lật, khiến xe spawn quay
sai hướng. Đây cũng đúng quy ước carla_ros_bridge dùng khi chuyển hệ toạ độ
CARLA (trái) sang ROS (phải): yaw_ros = -yaw_carla.

Được spawn_car_node.py gọi như một subprocess riêng biệt (không chạy chung tiến
trình với Flask/ROS), vì thư viện `carla` có thể yêu cầu môi trường Python khác
với môi trường chạy Flask/rclpy.

Chạy thử độc lập:
    python3 fetch_spawn_points.py --host localhost --port 2000

Output thành công (1 dòng, in ra stdout):
    {"points": [{"id": 0, "x": 1.23, "y": -4.5, "z": 0.3, "yaw": 90.0}, ...]}

Output lỗi (in ra stdout, exit code != 0):
    {"error": "mô tả lỗi"}
"""

import argparse
import json
import sys


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default='localhost', help='Địa chỉ CARLA server')
    parser.add_argument('--port', type=int, default=2000, help='Port CARLA server')
    parser.add_argument('--timeout', type=float, default=5.0, help='Timeout kết nối (giây)')
    args = parser.parse_args()

    import carla  # import trễ để bắt lỗi "module not found" gọn trong try/except bên dưới

    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)

    world = client.get_world()
    spawn_points = world.get_map().get_spawn_points()

    points = []
    for i, sp in enumerate(spawn_points):
        points.append({
            'id':  i,
            'x':   round(sp.location.x, 4),
            # Đảo dấu Y — khớp quy ước y_new = -y_ros đã dùng khi sinh waypoints.json
            # (xem app.js: rosToPixel()). Không đảo thì spawn point sẽ vẽ sai vị trí trên map.
            'y':   round(-sp.location.y, 4),
            'z':   round(sp.location.z, 4),
            # Đảo dấu yaw — cùng lý do đảo Y ở trên: lật trục Y làm đảo chiều quay
            # của góc, không đảo yaw thì hướng xe sẽ sai lệch so với vị trí đã lật.
            # roll = pitch = 0 theo yêu cầu — chỉ giữ trị tuyệt đối, đảo dấu yaw.
            'yaw': round(-sp.rotation.yaw, 4),
        })

    print(json.dumps({'points': points}))


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        # In lỗi ra stdout (không phải stderr) để parent process parse thống nhất bằng JSON
        print(json.dumps({'error': f'{type(e).__name__}: {e}'}))
        sys.exit(1)