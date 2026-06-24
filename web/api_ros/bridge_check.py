#!/usr/bin/env python3
"""
bridge_check.py — ROS2 node chạy trên máy local.

Chức năng:
  - Kiểm tra /carla_ros_bridge có đang sống trong ROS graph không (mỗi 5 giây).
  - Chỉ gửi HTTP POST lên Flask KHI trạng thái thay đổi (edge-triggered).
  - Gửi kèm log message để hiển thị trên bảng log của web.
  - Không gửi gì nếu trạng thái vẫn như cũ.

Cách chạy (do Flask tự spawn, hoặc chạy tay để debug):
  python3 bridge_check.py
  python3 bridge_check.py --flask-url http://localhost:5000 --interval 5 --token mytoken
"""

import argparse
import sys
import time
import requests
import rclpy
from rclpy.node import Node

# ── Defaults (khớp với ros_status_api.py) ──────────────────────────────────────
DEFAULT_FLASK_URL = 'http://localhost:5000'
DEFAULT_INTERVAL  = 5        # giây giữa mỗi lần check
DEFAULT_TOKEN     = 'bridge-check-secret'
TARGET_NODE       = 'carla_ros_bridge'

ENDPOINT_HEARTBEAT = '/api/ros/heartbeat'


class BridgeCheckerNode(Node):
    def __init__(self, flask_url: str, interval: float, token: str):
        super().__init__('bridge_checker')
        self._flask_url = flask_url.rstrip('/')
        self._interval  = interval
        self._token     = token
        self._last_state: bool | None = None   # None = chưa biết (lần đầu)

        self.get_logger().info('[BridgeChecker] Đã khởi chạy.')

        # Timer ROS2 — gọi _check() đúng mỗi interval giây
        self.create_timer(interval, self._check)

    # ── Kiểm tra và so sánh state ───────────────────────────────────────────────
    def _check(self):
        try:
            node_names = self.get_node_names()
            running    = TARGET_NODE in node_names
        except Exception as e:
            self.get_logger().warn(f'[BridgeChecker] get_node_names() lỗi: {e}')
            running = False

        # Chỉ xử lý khi state THAY ĐỔI (hoặc lần đầu khởi động)
        if running == self._last_state:
            return

        prev_state       = self._last_state
        self._last_state = running

        if running:
            log_msg   = f'[ROS Bridge] carla_ros_bridge detected — trạng thái: ONLINE'
            log_level = 'info'
            self.get_logger().info(log_msg)
        else:
            if prev_state is None:
                log_msg   = '[ROS Bridge] Checking — carla_ros_bridge chưa chạy'
                log_level = 'warn'
            else:
                log_msg   = '[ROS Bridge] carla_ros_bridge đã tắt — trạng thái: OFFLINE'
                log_level = 'warn'
            self.get_logger().warn(log_msg)

        self._push(running, log_msg, log_level)

    # ── Gửi HTTP POST lên Flask ──────────────────────────────────────────────────
    def _push(self, running: bool, log_msg: str, log_level: str):
        url     = self._flask_url + ENDPOINT_HEARTBEAT
        payload = {
            'running':    running,
            'log_msg':    log_msg,
            'log_level':  log_level,   # 'info' | 'warn' | 'error'
            'source':     'bridge_checker',
        }
        headers = {'X-Bridge-Token': self._token, 'Content-Type': 'application/json'}
        try:
            resp = requests.post(url, json=payload, headers=headers, timeout=2)
            if resp.status_code != 200:
                self.get_logger().warn(
                    f'[BridgeChecker] Flask trả về HTTP {resp.status_code}'
                )
        except requests.exceptions.ConnectionError:
            self.get_logger().warn('[BridgeChecker] Không thể kết nối Flask — sẽ thử lại sau')
        except Exception as e:
            self.get_logger().warn(f'[BridgeChecker] Lỗi gửi heartbeat: {e}')


# ── Entry point ─────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description='ROS2 bridge checker node')
    parser.add_argument('--flask-url', default=DEFAULT_FLASK_URL,
                        help=f'URL Flask server (default: {DEFAULT_FLASK_URL})')
    parser.add_argument('--interval', type=float, default=DEFAULT_INTERVAL,
                        help=f'Giây giữa mỗi lần check (default: {DEFAULT_INTERVAL})')
    parser.add_argument('--token', default=DEFAULT_TOKEN,
                        help='Auth token khớp với Flask (default: bridge-check-secret)')

    # rclpy truyền args riêng sau '--', tách ra để argparse không bị confuse
    argv = []
    for a in sys.argv[1:]:
        if a == '--ros-args':
            break
        argv.append(a)
    args = parser.parse_args(argv)

    rclpy.init()
    node = BridgeCheckerNode(
        flask_url=args.flask_url,
        interval=args.interval,
        token=args.token,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()