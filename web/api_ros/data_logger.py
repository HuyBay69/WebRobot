import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaEgoVehicleStatus
import csv
import math
import os
import threading

"""
data_logger.py - bản đã sửa lỗi so với file gốc:

0. [ĐÃ KIỂM CHỨNG BẰNG THỰC NGHIỆM TRÊN CHÍNH BRIDGE CỦA BẠN]
   - twist (/odometry, msg.twist.twist.linear.x/y) ĐÃ Ở BODY FRAME (tương đối) sẵn -
     vx = dọc trục, vy = ngang, KHÔNG cần xoay theo yaw nữa. Bằng chứng: khi xe vào
     cua, vy chỉ nhích nhẹ trong khi vx luôn áp đảo/ổn định quanh tốc độ thật - nếu
     ở world frame thì vx phải tụt mạnh về 0 khi xe đổi hướng.
   - acceleration (/vehicle_status, msg.acceleration.linear.x/y) VẪN Ở WORLD FRAME
     (tuyệt đối) - cần tự xoay theo yaw mới ra đúng dọc trục/ngang. Bằng chứng: ax
     chỉ nhảy khi xe chạy theo trục world-X, ay chỉ nhảy khi xe chạy theo trục
     world-Y - đúng đặc trưng world frame (không phải "luôn ax là dọc trục").
   (Lượt sửa trước của tôi xoay CẢ HAI theo yaw - sai một nửa, phần v_long/v_lat
   đã bị xoay thừa. Đã sửa lại đúng ở đây.)
2. v_cmd quy đổi từ km/h (đơn vị gốc của /speed_command, theo ad_agent.py: *3.6)
   sang m/s để cùng đơn vị với v_total/v_long/v_lat.
3. Mọi giá trị số thực làm tròn 3 chữ số thập phân trước khi ghi CSV.
4. Không ghi dòng nào cho tới khi đã nhận được ít nhất 1 message odometry VÀ
   1 message vehicle_status (tránh ghi các dòng 0.0 giả ở đầu file).
5. Bỏ qua dòng nếu giống hệt dòng vừa ghi trước đó (lọc chuỗi dòng đứng yên lặp
   lại ở cuối file) - có thể tắt bằng self.filter_idle = False.
6. a_z giữ nguyên là acceleration.linear.z (gia tốc thẳng đứng) như bạn đã chủ
   động sửa - vì acceleration.angular.z do carla_ros_bridge luôn trả về 0.0.
"""


class CarlaDataCollector(Node):
    def __init__(self):
        super().__init__('carla_data_collector')

        # Cấu hình đường dẫn file CSV
        self.csv_file_path = os.path.join(os.getcwd(), 'carla_data_local.csv')

        # Bật/tắt lọc dòng trùng lặp hoàn toàn (chủ yếu xảy ra khi xe đứng yên)
        self.filter_idle = True

        # Định nghĩa các cột theo đúng thứ tự yêu cầu
        self.csv_headers = [
            'timestamp',
            'x', 'y', 'yaw',
            'v_cmd',
            'v_total', 'v_long', 'v_lat',
            'w_z',
            'a_long', 'a_lat', 'a_z',
            'throttle_cmd', 'brake_cmd', 'steer_cmd', 'gear_cmd',
            'throttle_real', 'brake_real', 'steer_real', 'gear_real'
        ]

        # Khởi tạo file CSV
        self.csv_file = open(self.csv_file_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(self.csv_headers)

        # Khởi tạo biến lưu trữ tạm thời và khóa Thread Lock
        self.data_lock = threading.Lock()
        self.data = {key: 0.0 for key in self.csv_headers}
        self.data['gear_cmd'] = 0
        self.data['gear_real'] = 0

        # Cờ đánh dấu đã nhận đủ dữ liệu cốt lõi (odometry + vehicle_status) chưa
        self._has_odom = False
        self._has_status = False
        self._last_row = None  # dòng đã ghi gần nhất, để lọc trùng lặp

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/carla/hero/odometry', self.odom_callback, 10)
        self.v_cmd_sub = self.create_subscription(
            Float64, '/carla/hero/speed_command', self.vcmd_callback, 10)
        self.ctrl_cmd_sub = self.create_subscription(
            CarlaEgoVehicleControl, '/carla/hero/vehicle_control_cmd', self.ctrl_cmd_callback, 10)
        self.status_sub = self.create_subscription(
            CarlaEgoVehicleStatus, '/carla/hero/vehicle_status', self.status_callback, 10)

        # Timer ghi file tần suất 10Hz (tốc độ ghi thực tế phụ thuộc tốc độ publish
        # thật của /carla/hero/odometry - xem ghi chú ở đầu file)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info(f'Data Collector Node started. Saving to: {self.csv_file_path}')

    def euler_from_quaternion(self, x, y, z, w):
        """Chuyển đổi Quaternion sang Euler (chỉ lấy Yaw)"""
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        return yaw_z

    def odom_callback(self, msg):
        with self.data_lock:
            # Lấy timestamp từ header của odometry
            self.data['timestamp'] = msg.header.stamp.sec + (msg.header.stamp.nanosec / 1e9)

            # Pose: x, y, yaw
            self.data['x'] = msg.pose.pose.position.x
            self.data['y'] = msg.pose.pose.position.y
            orientation_q = msg.pose.pose.orientation
            yaw = self.euler_from_quaternion(
                orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
            )
            self.data['yaw'] = yaw

            # Twist ĐÃ ở body frame sẵn (xác nhận thực nghiệm) -> dùng thẳng, không xoay
            v_long = msg.twist.twist.linear.x
            v_lat = msg.twist.twist.linear.y
            self.data['v_long'] = v_long
            self.data['v_lat'] = v_lat
            self.data['v_total'] = math.sqrt(v_long ** 2 + v_lat ** 2)
            self.data['w_z'] = msg.twist.twist.angular.z

            self._has_odom = True

    def vcmd_callback(self, msg):
        with self.data_lock:
            # /speed_command ở km/h (xem ad_agent.py: target_speed*3.6) -> đổi về m/s
            # cho cùng đơn vị với v_total/v_long/v_lat
            self.data['v_cmd'] = msg.data / 3.6

    def ctrl_cmd_callback(self, msg):
        with self.data_lock:
            # Lệnh điều khiển yêu cầu
            self.data['throttle_cmd'] = msg.throttle
            self.data['brake_cmd'] = msg.brake
            self.data['steer_cmd'] = msg.steer
            self.data['gear_cmd'] = int(msg.gear)

    def status_callback(self, msg):
        with self.data_lock:
            # acceleration ở khung world/map -> xoay theo yaw hiện tại (đã cache từ
            # odometry) để ra dọc trục / ngang thật, giống twist ở trên
            yaw = self.data['yaw']
            ax = msg.acceleration.linear.x
            ay = msg.acceleration.linear.y
            self.data['a_long'] = ax * math.cos(yaw) + ay * math.sin(yaw)
            self.data['a_lat'] = -ax * math.sin(yaw) + ay * math.cos(yaw)
            # angular.z luôn = 0.0 (carla_ros_bridge không tính) nên dùng linear.z
            # (gia tốc thẳng đứng) thay thế, như bạn đã chủ động sửa
            self.data['a_z'] = msg.acceleration.linear.z

            # Trạng thái điều khiển thực tế
            self.data['throttle_real'] = msg.control.throttle
            self.data['brake_real'] = msg.control.brake
            self.data['steer_real'] = msg.control.steer
            self.data['gear_real'] = int(msg.control.gear)

            self._has_status = True

    def timer_callback(self):
        with self.data_lock:
            # Chờ đủ dữ liệu cốt lõi trước khi bắt đầu ghi, tránh các dòng 0.0 giả
            if not (self._has_odom and self._has_status):
                return

            # Làm tròn 3 chữ số cho các giá trị số thực, giữ nguyên số nguyên (gear)
            row = [
                round(self.data[key], 3) if isinstance(self.data[key], float) else self.data[key]
                for key in self.csv_headers
            ]

            # Bỏ qua nếu trùng hệt dòng vừa ghi (chủ yếu xảy ra khi xe đứng yên)
            if self.filter_idle and row == self._last_row:
                return

            self.csv_writer.writerow(row)
            self.csv_file.flush()
            self._last_row = row

    def destroy_node(self):
        self.get_logger().info('Closing CSV file.')
        self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CarlaDataCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()