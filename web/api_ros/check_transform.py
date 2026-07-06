#!/usr/bin/env python3
"""
check_transform.py — Kết nối tới CARLA, in ra transform thực tế (x, y, z, yaw)
của TẤT CẢ vehicle actor đang có trong world, kèm role_name.

Dùng để debug: chạy script này TRƯỚC và SAU khi publish /init_pose để xem
tọa độ thật trong CARLA có đổi đúng như kỳ vọng không — không phụ thuộc RViz.

Chạy:
    python3 check_transform.py
    python3 check_transform.py --host localhost --port 2000
"""

import argparse
import sys
import traceback


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default='localhost')
    parser.add_argument('--port', type=int, default=2000)
    parser.add_argument('--timeout', type=float, default=5.0)
    args = parser.parse_args()

    print(f'[check_transform] Đang kết nối CARLA tại {args.host}:{args.port} ...', flush=True)

    import carla
    print(f'[check_transform] Dùng module carla tại: {carla.__file__}', flush=True)

    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)

    world = client.get_world()
    print(f'[check_transform] Map hiện tại: {world.get_map().name}', flush=True)

    all_actors = world.get_actors()
    print(f'[check_transform] Tổng số actor (mọi loại): {len(all_actors)}', flush=True)

    vehicles = [a for a in all_actors if a.type_id.startswith('vehicle.')]
    print(f'[check_transform] Số vehicle actor: {len(vehicles)}', flush=True)
    print('-' * 70, flush=True)

    if not vehicles:
        print('[check_transform] KHÔNG tìm thấy vehicle actor nào trong world.', flush=True)
        return

    for a in vehicles:
        t = a.get_transform()
        role_name = a.attributes.get('role_name', None)
        print(
            f'id={a.id:<6} type={a.type_id:<35} role_name={role_name!r:<10} '
            f'x={t.location.x:>10.4f}  y={t.location.y:>10.4f}  z={t.location.z:>8.4f}  '
            f'yaw={t.rotation.yaw:>9.4f}',
            flush=True,
        )


if __name__ == '__main__':
    try:
        main()
    except Exception:
        print('[check_transform] LỖI:', flush=True)
        traceback.print_exc()
        sys.exit(1)