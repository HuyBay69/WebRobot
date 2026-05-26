#!/usr/bin/env python3
import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray


class GoalInputTerminal(Node):
    def __init__(self):
        super().__init__('goal_input_terminal')

        self.declare_parameter('role_name', 'hero')
        self.role_name = self.get_parameter('role_name').get_parameter_value().string_value

        # Nhận toàn bộ waypoint/road network từ navigation_hmi
        road_qos = QoSProfile(depth=10)

        self.road_sub = self.create_subscription(
            MarkerArray,
            'carla_road_network',
            self.road_callback,
            road_qos
        )

        # Publish sang navigation_hmi input
        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )

        self.markers = []
        self.markers_lock = threading.Lock()

        self.get_logger().info('GoalInputTerminal started.')
        self.get_logger().info('Waiting for carla_road_network markers...')

    def road_callback(self, msg: MarkerArray):
        with self.markers_lock:
            self.markers = list(msg.markers)

    def find_nearest_marker(self, x, y):
        with self.markers_lock:
            if not self.markers:
                return None

            nearest = None
            min_dist = float('inf')

            for marker in self.markers:
                mx = marker.pose.position.x
                my = marker.pose.position.y
                dist = math.hypot(mx - x, my - y)
                if dist < min_dist:
                    min_dist = dist
                    nearest = marker

            return nearest, min_dist

    def publish_goal_nearest_waypoint(self, x, y):
        result = self.find_nearest_marker(x, y)
        if result is None:
            self.get_logger().warn('Chua nhan duoc marker nao tu carla_road_network.')
            return

        nearest_marker, dist = result

        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.position.x = nearest_marker.pose.position.x
        goal_msg.pose.position.y = nearest_marker.pose.position.y
        goal_msg.pose.position.z = nearest_marker.pose.position.z

        # Orientation mặc định
        goal_msg.pose.orientation.x = 0.0
        goal_msg.pose.orientation.y = 0.0
        goal_msg.pose.orientation.z = 0.0
        goal_msg.pose.orientation.w = 1.0

        self.goal_pub.publish(goal_msg)

        self.get_logger().info(
            f'Goal goc: ({x:.3f}, {y:.3f}) | '
            f'Waypoint gan nhat: ({goal_msg.pose.position.x:.3f}, {goal_msg.pose.position.y:.3f}) | '
            f'Khoang cach: {dist:.3f} m'
        )


def main(args=None):
    rclpy.init(args=args)
    node = GoalInputTerminal()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        while rclpy.ok():
            raw = input('Nhap toa do dich den (x y), hoac q de thoat: ').strip()

            if raw.lower() in ['q', 'quit', 'exit']:
                break

            parts = raw.split()
            if len(parts) != 2:
                print('Nhap dung dinh dang: x y')
                continue

            try:
                x = float(parts[0])
                y = float(parts[1])
            except ValueError:
                print('Gia tri x y phai la so.')
                continue

            node.publish_goal_nearest_waypoint(x, y)

    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
