#!/usr/bin/env python3
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from rclpy.qos import QoSProfile, DurabilityPolicy


class SpeedInputTerminal(Node):
    def __init__(self):
        super().__init__('speed_input_terminal')

        self.declare_parameter('role_name', 'hero')
        self.role_name = self.get_parameter('role_name').get_parameter_value().string_value

        qos = QoSProfile(depth=10)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.speed_pub = self.create_publisher(
            Float64,
            f'/carla/{self.role_name}/target_speed',
            qos
        )

        self.get_logger().info(
            'SpeedInputTerminal started.'
        )

    def publish_speed_kmh(self, speed_kmh: float):
        speed_mps = speed_kmh / 3.6

        msg = Float64()
        msg.data = speed_mps
        self.speed_pub.publish(msg)

        self.get_logger().info(
            f'Published target_speed = {speed_kmh:.2f} km/h ({speed_mps:.2f} m/s)'
        )


def main(args=None):
    rclpy.init(args=args)

    node = SpeedInputTerminal()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    # Thread chạy spin
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        while rclpy.ok():
            raw = input('Nhap toc do cua xe (km/h), hoac q de thoat: ').strip()

            if raw.lower() in ['q', 'quit', 'exit']:
                break

            try:
                speed_kmh = float(raw)
            except ValueError:
                print('Toc do phai la so.')
                continue

            if speed_kmh < 0.0:
                print('Toc do phai >= 0.')
                continue

            node.publish_speed_kmh(speed_kmh)

    except KeyboardInterrupt:
        pass
    finally:
        # 🔥 QUAN TRỌNG: dừng executor trước
        executor.shutdown()

        # đợi thread kết thúc
        spin_thread.join()

        # destroy node
        node.destroy_node()

        # shutdown ROS
        rclpy.shutdown()


if __name__ == '__main__':
    main()
