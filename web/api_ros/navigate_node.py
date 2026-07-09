#!/usr/bin/env python3
"""
navigate_node.py — ROS2 node nhận lệnh điều hướng từ Flask qua stdin.
Phần cuối file chứa start_navigate_node / stop_navigate_node / send_navigate_command
để app.py import và dùng.

Định dạng lệnh (mỗi dòng một lệnh):
  GO x y speed_kmh                              → publish /goal_pose + /carla/hero/target_speed
                                                   (giữ nguyên — tương thích ngược, đi 1 điểm duy nhất)
  STOP                                           → publish speed=0, goal tại (0,0,0),
                                                   đồng thời HUỶ hành trình (mission) đang chạy nếu có
  MISSION speed_kmh x1 y1 x2 y2 ... xn yn        → CHỐT HÀNH TRÌNH nhiều điểm (A, B, C...)
                                                   Xe sẽ tự động đi lần lượt qua từng điểm.

── Cơ chế MISSION (hành trình nhiều điểm) ───────────────────────────────────────
- `active_mission_points`: danh sách điểm đang "chốt", xe bám theo danh sách này.
- Khi có lệnh MISSION mới: ghi đè toàn bộ active_mission_points bằng danh sách mới.
  Nếu điểm đầu tiên của danh sách mới KHÁC điểm xe đang hướng tới hiện tại,
  publish ngay goal_pose mới (ROS Bridge sẽ tự tính lại tuyến đường từ vị trí
  hiện tại của xe). Nếu điểm đầu giống hệt điểm đang chạy tới thì không làm gì
  thêm (xe cứ tiếp tục chạy), chỉ có phần đuôi danh sách được cập nhật cho các
  lượt "tới điểm kế tiếp" sau này.
- Node subscribe /carla/<role>/odometry để biết vị trí + tốc độ hiện tại của xe,
  VÀ subscribe /carla/<role>/waypoints (Path do ROS Bridge tự tính khi nhận
  /goal_pose mới) để biết CHÍNH XÁC điểm cuối cùng của tuyến đường đang chạy.
  Lý do phải dùng điểm cuối của Path thay vì toạ độ mình yêu cầu ban đầu: bộ
  lập tuyến của CARLA Bridge snap goal vào waypoint gần nhất trên đường, có
  thể lệch 1-2m so với toạ độ yêu cầu — nếu so khoảng cách với toạ độ yêu cầu
  gốc, có thể không bao giờ đạt ngưỡng dù xe đã thực sự dừng hẳn tại đó.
  Liên tục tính khoảng cách Euclidean từ xe tới điểm cuối tuyến đường. Khi
  khoảng cách < ARRIVAL_DIST_M và tốc độ < ARRIVAL_SPEED_KMH (xe coi như đã
  dừng hẳn — trùng với lúc local_planner.py tự phanh do hết waypoint trong
  buffer), khởi động timer 1 lần ARRIVAL_HOLD_SEC giây. Hết giờ: bỏ điểm vừa
  tới khỏi active_mission_points, lấy điểm kế tiếp, publish goal_pose mới.
  Hết danh sách thì coi như hoàn thành hành trình (publish speed=0 cho chắc).

── Lưu ý khi chỉnh ARRIVAL_DIST_M (đang để rộng, 4.0m) ──────────────────────────
  1) Nếu 2 điểm liền nhau trong hàng đợi cách nhau GẦN HƠN ARRIVAL_DIST_M, xe
     có thể vừa "tới" điểm A xong đã lọt luôn vào bán kính điểm B ngay cả khi
     chưa thực sự chạy qua đó — nên đặt các điểm cách nhau tối thiểu ~2x
     ARRIVAL_DIST_M để an toàn.
  2) Nếu ngay gần điểm đích có đèn đỏ / vật cản khiến ad_agent.py buộc xe dừng
     hẳn (BLOCKED_RED_LIGHT / BLOCKED_BY_VEHICLE) mà vị trí dừng đó lại nằm
     trong bán kính ARRIVAL_DIST_M của đích — hệ thống sẽ hiểu nhầm là "đã tới"
     dù xe chỉ đang dừng đèn đỏ, rồi tự chuyển sang điểm kế tiếp sớm. Ngưỡng
     càng rộng thì rủi ro này càng cao. Nếu gặp phải, báo lại để bổ sung thêm
     tín hiệu phân biệt (vd: kiểm tra trạng thái đèn giao thông) thay vì chỉ
     dựa khoảng cách + tốc độ.

Flask spawn node này lúc start, ghi lệnh vào stdin mỗi khi browser bấm nút.
Node chạy độc lập, không biết gì về web.
"""

import math
import os
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64

ROLE_NAME = 'hero'   # có thể đổi thành arg nếu cần

# ── Ngưỡng nhận diện "đã tới điểm" ────────────────────────────────────────────
ARRIVAL_DIST_M    = 4.0   # khoảng cách Euclidean (m) tới điểm CUỐI của route hiện tại
                          # — nới từ 1.5m lên 4.0m vì xe thường "trôi/đỗ" cách đích thật
                          # 1.5-2m (do PID + độ trễ phanh), 1.5m cũ quá sát nên có lúc xe
                          # dừng hẳn mà vẫn không qua ngưỡng. Xem lưu ý 2 tình huống đặc
                          # biệt cần để ý ở docstring phía trên khi chỉnh số này.
ARRIVAL_SPEED_KMH = 0.8   # tốc độ (km/h) coi như xe đã dừng hẳn — giữ nguyên, log cho thấy
                          # xe dừng hẳn về đúng 0.00km/h, không cần nới thêm.
ARRIVAL_HOLD_SEC  = 3.0   # giữ nguyên trạng thái dừng bao lâu trước khi chuyển điểm kế tiếp
POINT_EPS         = 1e-6  # sai số so sánh 2 điểm (float) coi là "cùng 1 điểm"
DEBUG_LOG_PERIOD_SEC = 2.0  # log chẩn đoán (dist/speed) mỗi bao nhiêu giây khi đang chờ tới đích
MIN_TRAVEL_TIME_SEC  = 2.0  # thời gian tối thiểu "đang di chuyển" trước khi được phép coi là đã tới
                            # — lưới an toàn thứ 2, độc lập với khoảng cách, chặn false-positive
                            # tức thời ngay sau khi vừa đổi điểm đích (vd: do route_end cũ sót lại)


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

        # ── Subscriber /carla/<role>/odometry — để theo dõi hành trình ────────
        self._odom_sub = self.create_subscription(
            Odometry,
            f'/carla/{self.role_name}/odometry',
            self._odometry_cb,
            10,
        )

        # ── Subscriber /carla/<role>/waypoints — Path do ROS Bridge tự tính
        #    sau mỗi lần nhận /goal_pose mới. Dùng điểm CUỐI của Path này làm
        #    mốc "đã tới đích" thay vì toạ độ mình yêu cầu ban đầu (xem giải
        #    thích ở đầu file). QoS phải khớp TRANSIENT_LOCAL như ad_agent.py/
        #    local_planner.py đang dùng, nếu không sẽ không nhận được gì.
        self._path_sub = self.create_subscription(
            Path,
            f'/carla/{self.role_name}/waypoints',
            self._path_cb,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )

        # ── Trạng thái hành trình (mission) — dùng chung giữa thread stdin và
        #    thread ROS spin nên cần lock ─────────────────────────────────────
        self._lock = threading.Lock()
        self._active_points = []       # list[(x, y)] — hàng đợi đã CHỐT, xe đang bám theo
        self._current_target = None    # (x, y) hoặc None — điểm xe đang hướng tới (toạ độ yêu cầu)
        self._route_end = None         # (x, y) hoặc None — điểm cuối THẬT của route hiện tại (từ Path)
        self._waiting_at_point = False  # True trong lúc đang giữ 3s tại điểm vừa tới
        self._arrival_timer = None      # rclpy Timer đang chờ (None nếu không có)
        self._target_set_time = 0.0     # time.time() lúc đặt current_target — dùng cho MIN_TRAVEL_TIME_SEC
        self._last_debug_log_time = 0.0
        self._odom_received = False     # để log 1 lần đầu xác nhận odometry có tới hay không

        self.get_logger().info('Đã khởi chạy.')

    # ── Xử lý lệnh từ stdin ──────────────────────────────────────────────────────
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
        elif cmd == 'MISSION':
            self._handle_mission(parts[1:])
        else:
            self.get_logger().warn(f'Lệnh không hợp lệ: {line.strip()!r}')

    def _handle_go(self, args: list):
        """GO x y speed_kmh — đi 1 điểm duy nhất (giữ nguyên hành vi cũ, không qua hàng đợi)."""
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

        # GO thủ công sẽ thay thế luôn mission đang chạy (nếu có) bằng 1 điểm duy nhất
        with self._lock:
            self._cancel_arrival_timer_locked()
            self._active_points = [(x, y)]
            self._set_target_locked((x, y))
            self._waiting_at_point = False

        self._publish_goal(x, y)
        self._publish_speed(speed_kmh)

        self.get_logger().info(
            f'GO ! x={x:.3f} y={y:.3f} | '
            f'{speed_kmh:.1f} km/h ({speed_kmh / 3.6:.3f} m/s)'
        )

    def _handle_stop(self):
        """STOP: speed=0, goal tại (0,0,0) — đồng thời huỷ hành trình (mission) đang chạy."""
        with self._lock:
            self._cancel_arrival_timer_locked()
            self._active_points = []
            self._current_target = None
            self._route_end = None
            self._target_set_time = 0.0
            self._waiting_at_point = False

        self._publish_speed(0.0)
        self._publish_goal(0.0, 0.0)
        self.get_logger().info('STOP ! (đã huỷ hành trình đang chạy nếu có)')

    def _handle_mission(self, args: list):
        """MISSION speed_kmh x1 y1 x2 y2 ... xn yn — chốt hành trình nhiều điểm."""
        if len(args) < 3:
            self.get_logger().warn('MISSION cần: speed_kmh rồi ít nhất 1 cặp x y')
            return

        try:
            speed_kmh = float(args[0])
            coords    = [float(v) for v in args[1:]]
        except ValueError:
            self.get_logger().warn('MISSION: tất cả tham số phải là số')
            return

        if speed_kmh < 0:
            self.get_logger().warn('MISSION: speed_kmh phải >= 0')
            return

        if len(coords) % 2 != 0:
            self.get_logger().warn('MISSION: số toạ độ lẻ — thiếu 1 giá trị x hoặc y')
            return

        new_points = [(coords[i], coords[i + 1]) for i in range(0, len(coords), 2)]
        if not new_points:
            self.get_logger().warn('MISSION: danh sách điểm rỗng')
            return

        with self._lock:
            # Huỷ timer chờ đang chạy (nếu có) — Chốt lại thì tính lại từ đầu
            self._cancel_arrival_timer_locked()
            self._waiting_at_point = False

            new_first = new_points[0]
            target_changed = (
                self._current_target is None or
                not self._points_equal(new_first, self._current_target)
            )

            self._active_points = new_points

            if target_changed:
                self._set_target_locked(new_first)

        # Luôn republish target_speed (rẻ, vô hại) — cho phép đổi tốc độ giữa
        # đường mà không cần đổi điểm đích.
        self._publish_speed(speed_kmh)

        if target_changed:
            self._publish_goal(*new_first)
            self.get_logger().info(
                f'MISSION ! Chốt {len(new_points)} điểm, đích mới → '
                f'x={new_first[0]:.3f} y={new_first[1]:.3f} | {speed_kmh:.1f} km/h'
            )
        else:
            self.get_logger().info(
                f'MISSION ! Chốt {len(new_points)} điểm (điểm đầu không đổi, '
                f'xe tiếp tục chạy) | {speed_kmh:.1f} km/h'
            )

    # ── Theo dõi tuyến đường (Path) do ROS Bridge tự tính ────────────────────────
    def _path_cb(self, msg: Path):
        with self._lock:
            if msg.poses:
                p = msg.poses[-1].pose.position
                self._route_end = (p.x, p.y)
            else:
                self._route_end = None

    # ── Theo dõi hành trình qua odometry ──────────────────────────────────────────
    def _odometry_cb(self, msg: Odometry):
        if not self._odom_received:
            self._odom_received = True
            self.get_logger().info('Đã nhận odometry đầu tiên — subscription hoạt động bình thường.')

        with self._lock:
            target        = self._current_target
            route_end     = self._route_end
            waiting       = self._waiting_at_point
            has_mission   = bool(self._active_points)
            target_set_at = self._target_set_time

        if target is None or waiting or not has_mission:
            return

        # Ưu tiên điểm cuối THẬT của route (khớp với lúc local_planner tự phanh);
        # nếu chưa nhận được Path nào (ví dụ ngay sau khi publish goal, Bridge
        # chưa kịp trả route) thì tạm dùng toạ độ yêu cầu ban đầu.
        arrival_ref = route_end if route_end is not None else target

        pos = msg.pose.pose.position
        dx  = arrival_ref[0] - pos.x
        dy  = arrival_ref[1] - pos.y
        dist = math.hypot(dx, dy)

        tw = msg.twist.twist.linear
        speed_kmh = math.sqrt(tw.x ** 2 + tw.y ** 2 + tw.z ** 2) * 3.6

        elapsed = time.time() - target_set_at

        now = time.time()
        if now - self._last_debug_log_time > DEBUG_LOG_PERIOD_SEC:
            self._last_debug_log_time = now
            src = 'route_end' if route_end is not None else 'target (chưa có Path)'
            cooldown = f', còn cooldown {MIN_TRAVEL_TIME_SEC - elapsed:.1f}s' if elapsed < MIN_TRAVEL_TIME_SEC else ''
            self.get_logger().info(
                f'[debug] dist={dist:.2f}m speed={speed_kmh:.2f}km/h '
                f'ref=({arrival_ref[0]:.2f},{arrival_ref[1]:.2f}) [{src}]{cooldown}'
            )

        # Lưới an toàn độc lập với khoảng cách: mới đặt đích chưa đủ lâu thì
        # CHƯA cho phép coi là đã tới, dù dist/speed có thoả điều kiện gì đi nữa.
        if elapsed < MIN_TRAVEL_TIME_SEC:
            return

        if dist < ARRIVAL_DIST_M and speed_kmh < ARRIVAL_SPEED_KMH:
            with self._lock:
                # Kiểm tra lại 1 lần nữa trong lock, tránh race giữa 2 lần callback gọi liên tiếp
                if self._waiting_at_point or self._current_target != target:
                    return
                self._waiting_at_point = True
                self._arrival_timer = self.create_timer(ARRIVAL_HOLD_SEC, self._on_arrival_timeout)
            self.get_logger().info(
                f'Đã tới điểm ({target[0]:.3f}, {target[1]:.3f}) — '
                f'giữ {ARRIVAL_HOLD_SEC:.0f}s trước khi chuyển điểm kế tiếp.'
            )

    def _on_arrival_timeout(self):
        """Hết 3s giữ tại điểm vừa tới — chuyển sang điểm kế tiếp (nếu còn)."""
        with self._lock:
            self._cancel_arrival_timer_locked()
            self._waiting_at_point = False

            if self._active_points:
                self._active_points.pop(0)  # bỏ điểm vừa hoàn thành

            if self._active_points:
                next_point = self._active_points[0]
                self._set_target_locked(next_point)
            else:
                next_point = None
                self._current_target = None
                self._route_end = None

        if next_point is not None:
            self._publish_goal(*next_point)
            self.get_logger().info(
                f'Chuyển điểm kế tiếp → x={next_point[0]:.3f} y={next_point[1]:.3f}'
            )
        else:
            self._publish_speed(0.0)
            self.get_logger().info('Hoàn thành hành trình — hết điểm trong danh sách.')

    def _cancel_arrival_timer_locked(self):
        """Huỷ timer chờ đang chạy. PHẢI gọi trong lúc đã giữ self._lock."""
        if self._arrival_timer is not None:
            self._arrival_timer.cancel()
            self.destroy_timer(self._arrival_timer)
            self._arrival_timer = None

    @staticmethod
    def _points_equal(p1, p2) -> bool:
        return abs(p1[0] - p2[0]) < POINT_EPS and abs(p1[1] - p2[1]) < POINT_EPS

    def _set_target_locked(self, new_target):
        """Đặt điểm đích mới. PHẢI gọi trong lúc đã giữ self._lock.
        LUÔN xoá route_end cũ (route trước đó không còn liên quan tới đích mới)
        và ghi lại thời điểm đặt đích — đây là chỗ duy nhất được phép đổi
        current_target, để không bao giờ quên đồng bộ 2 giá trị này với nhau
        (nguyên nhân bug trước đó: quên xoá route_end khi tự động chuyển điểm)."""
        self._current_target = new_target
        self._route_end = None
        self._target_set_time = time.time()

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
    cmd ví dụ: 'GO 12.5 -8.2 30.0' hoặc 'STOP' hoặc 'MISSION 40.0 1.0 2.0 3.0 4.0'
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