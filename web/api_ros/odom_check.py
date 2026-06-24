#!/usr/bin/env python3
import math
import time
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray

class OdomCheckNode(Node):
    def __init__(self):
        super().__init__('odom_check_node')
        
        # Đăng ký subscribe vào topic hệ thống theo yêu cầu
        self.subscription = self.create_subscription(
            MarkerArray,
            '/carla/markers',
            self.marker_callback,
            10
        )
        
        # Biến lưu thời gian để tính delta s
        self.last_time = None
        
        self.get_logger().info('Node odom_check da khoi dong. Dang cho tin nhan...')

    def marker_callback(self, msg: MarkerArray):
        # Kiểm tra nếu mảng marker trống thì bỏ qua
        if not msg.markers:
            return
        
        # Lấy dữ liệu của marker đầu tiên đại diện cho xe
        marker = msg.markers[0]
        position = marker.pose.position
        orientation = marker.pose.orientation

        # --- Tính toán góc xoay (Yaw) từ Quaternion ---
        # Công thức: yaw = atan2(2(wz + xy), 1 - 2(y^2 + z^2))
        siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        yaw_deg = math.degrees(yaw_rad) # Đổi sang độ để hiển thị trực quan

        # --- Tính toán thời gian thực tế nhận tin nhắn và delta s ---
        now = time.time()
        time_str = time.strftime('%H:%M:%S')
        
        if self.last_time is not None:
            delta_s = now - self.last_time
        else:
            delta_s = 0.0 # Tin nhắn đầu tiên chưa có delta s
            
        self.last_time = now

        # --- Lọc và đóng gói dữ liệu vào cấu trúc filter_msg ---
        filter_msg = {
            'x': position.x,
            'y': position.y,
            'yaw': yaw_deg
        }

        # --- In thông thường ra màn hình theo đúng định dạng ---
        print(f"[{time_str}] x: {filter_msg['x']:.3f} y: {filter_msg['y']:.3f} yaw: {filter_msg['yaw']:.2f} [{delta_s:.3f}s]")


def main(args=None):
    rclpy.init(args=args)
    node = OdomCheckNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nDang dung node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()