#!/usr/bin/env python3
"""
navigate_node.py — ROS2 node nhận lệnh điều hướng từ Flask qua stdin.
Phần cuối file chứa start_navigate_node / stop_navigate_node / send_navigate_command
để app.py import và dùng.

Định dạng lệnh (mỗi dòng một lệnh):
  GO x y speed_kmh       → publish /goal_pose + /carla/hero/target_speed
  STOP                   → publish speed=0, goal tại (0,0,0)

Flask spawn node này lúc start, ghi lệnh vào stdin mỗi khi browser ấn GO/STOP.
Node chạy độc lập, không biết gì về web.

Mọi xử lý dữ liệu (quy đổi đơn vị, validate, filter waypoint sau này) đều nằm ở đây.
"""

import sys
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64

ROLE_NAME = 'hero'   # có thể đổi thành arg nếu cần


class NavigateNode(Node):
    def __init__(self):
        super().__init__('navigate_node')

        self.declare_parameter('role_name', ROLE_NAME)
        self.role_name = (
            self.get_parameter('role_name').get_parameter_value().string_value
        )

        # ── Publisher /goal_pose ───────────────────────────────────────────────
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # ── Publisher /carla/<role>/target_speed ──────────────────────────────
        speed_qos = QoSProfile(depth=10)
        speed_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.speed_pub = self.create_publisher(
            Float64,
            f'/carla/{self.role_name}/target_speed',
            speed_qos,
        )

        self.get_logger().info('Đã khởi chạy.')

    # ── Xử lý lệnh ──────────────────────────────────────────────────────────────
    def handle_command(self, line: str):
        """Parse và thực thi một dòng lệnh từ stdin."""
        parts = line.strip().split()
        if not parts:
            return

        cmd = parts[0].upper()

        if cmd == 'GO':
            self._handle_go(parts[1:])
        elif cmd == 'STOP':
            self._handle_stop()
        else:
            self.get_logger().warn(f'Lệnh không hợp lệ: {line.strip()!r}')

    def _handle_go(self, args: list):
        """GO x y speed_kmh"""
        if len(args) != 3:
            self.get_logger().warn('GO cần đúng 3 tham số: x y speed_kmh')
            return
        try:
            x         = float(args[0])
            y         = float(args[1])
            speed_kmh = float(args[2])
        except ValueError:
            self.get_logger().warn('GO: x, y, speed_kmh phải là số')
            return

        if speed_kmh < 0:
            self.get_logger().warn('GO: speed_kmh phải >= 0')
            return

        self._publish_goal(x, y)
        self._publish_speed(speed_kmh)

        self.get_logger().info(
            f'GO ! x={x:.3f} y={y:.3f} | '
            f'{speed_kmh:.1f} km/h ({speed_kmh / 3.6:.3f} m/s)'
        )

    def _handle_stop(self):
        """STOP: speed=0, goal tại (0,0,0)"""
        self._publish_speed(0.0)
        self._publish_goal(0.0, 0.0)
        self.get_logger().info('STOP !')

    # ── Publish helpers ──────────────────────────────────────────────────────────
    def _publish_goal(self, x: float, y: float, z: float = 0.0):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0   # identity quaternion
        self.goal_pub.publish(msg)

    def _publish_speed(self, speed_kmh: float):
        """Quy đổi km/h → m/s rồi publish."""
        speed_mps   = speed_kmh / 3.6
        msg         = Float64()
        msg.data    = speed_mps
        self.speed_pub.publish(msg)


# ── Stdin reader ─────────────────────────────────────────────────────────────────
def _stdin_loop(node: NavigateNode):
    """
    Chạy trong thread riêng — đọc từng dòng từ stdin và gọi handle_command.
    Khi stdin đóng (Flask process die hoặc pipe bị cắt) thì thoát.
    """
    for line in sys.stdin:
        if not rclpy.ok():
            break
        node.handle_command(line)


# ── Entry point ──────────────────────────────────────────────────────────────────
def main():
    rclpy.init()
    node = NavigateNode()

    # Thread đọc stdin — không block ROS spin
    reader = threading.Thread(target=_stdin_loop, args=(node,), daemon=True)
    reader.start()

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


# ── Spawn helpers — dùng bởi app.py ─────────────────────────────────────────────
import os
import time
import subprocess
import threading as _threading

_navigate_process: subprocess.Popen | None = None
_navigate_lock = _threading.Lock()

NAVIGATE_SCRIPT = os.path.abspath(__file__)


def start_navigate_node():
    """Spawn navigate_node.py với stdin=PIPE ngay lập tức (đồng bộ)."""
    global _navigate_process

    with _navigate_lock:
        if _navigate_process is not None and _navigate_process.poll() is None:
            return

        _navigate_process = subprocess.Popen(
            [sys.executable, NAVIGATE_SCRIPT],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )

    def _pipe_log():
        time.sleep(0.5)   # đợi Flask banner in xong rồi mới in log node
        for line in _navigate_process.stdout:
            print(f'[NavigateNode] {line}', end='', flush=True)

    _threading.Thread(target=_pipe_log, daemon=True).start()


def stop_navigate_node():
    """Kill navigate_node.py khi Flask tắt."""
    global _navigate_process
    with _navigate_lock:
        if _navigate_process is None:
            return
        if _navigate_process.poll() is None:
            print(f'[NavigateNode] Đang kill PID={_navigate_process.pid}...')
            _navigate_process.terminate()
            try:
                _navigate_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                _navigate_process.kill()
            print('[NavigateNode] Đã dọn dẹp xong.')
        _navigate_process = None


def send_navigate_command(cmd: str):
    """
    Ghi một dòng lệnh vào stdin của navigate_node.py.
    cmd ví dụ: 'GO 12.5 -8.2 30.0' hoặc 'STOP'
    """
    with _navigate_lock:
        if _navigate_process is None or _navigate_process.poll() is not None:
            print(f'[NavigateNode] Process không chạy, bỏ lệnh: {cmd!r}')
            return
        try:
            _navigate_process.stdin.write(cmd + '\n')
            _navigate_process.stdin.flush()
        except BrokenPipeError:
            print('[NavigateNode] Broken pipe — process đã chết?')