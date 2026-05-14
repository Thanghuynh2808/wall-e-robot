#!/usr/bin/env python3
"""
set_initial_pose.py
-------------------
Set vị trí ban đầu của robot (AMCL initial pose) qua topic /initialpose.
Dùng khi bạn muốn chỉ định chính xác vị trí robot trên bản đồ mà không cần RViz2.

Cách dùng:
    python3 set_initial_pose.py                         # mặc định (0, 0, hướng 0°)
    python3 set_initial_pose.py --x 1.0 --y 0.5 --yaw 90
    python3 set_initial_pose.py --x -0.5 --y 1.2 --yaw 180

Lưu ý:
    --yaw là góc theo độ (degree), 0° = nhìn theo trục X+, 90° = nhìn theo trục Y+
"""

import argparse
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


def deg_to_quat_z_w(yaw_deg: float):
    """Convert yaw in degrees to quaternion (z, w) components."""
    yaw_rad = math.radians(yaw_deg)
    return math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0)


class InitialPoseSetter(Node):
    def __init__(self, x: float, y: float, yaw_deg: float):
        super().__init__('initial_pose_setter')
        self.pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )
        self.x = x
        self.y = y
        self.yaw_deg = yaw_deg
        # Đợi publisher sẵn sàng rồi publish ngay
        self.timer = self.create_timer(0.5, self.publish_once)
        self.published = False

    def publish_once(self):
        if self.published:
            return

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0

        qz, qw = deg_to_quat_z_w(self.yaw_deg)
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        # Covariance mặc định (diagonal nhỏ = tự tin)
        msg.pose.covariance[0]  = 0.25   # xx
        msg.pose.covariance[7]  = 0.25   # yy
        msg.pose.covariance[35] = 0.068  # yaw-yaw

        self.pub.publish(msg)
        self.get_logger().info(
            f'[InitialPose] Published: x={self.x:.3f}, y={self.y:.3f}, yaw={self.yaw_deg:.1f}°'
        )
        self.published = True
        # Publish lại lần 2 sau 0.5s để AMCL chắc chắn nhận được
        self.timer = self.create_timer(0.5, self.publish_again)

    def publish_again(self):
        if not hasattr(self, '_second_done'):
            self._second_done = True
            msg = PoseWithCovarianceStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.pose.pose.position.x = self.x
            msg.pose.pose.position.y = self.y
            msg.pose.pose.position.z = 0.0
            qz, qw = deg_to_quat_z_w(self.yaw_deg)
            msg.pose.pose.orientation.z = qz
            msg.pose.pose.orientation.w = qw
            msg.pose.covariance[0]  = 0.25
            msg.pose.covariance[7]  = 0.25
            msg.pose.covariance[35] = 0.068
            self.pub.publish(msg)
            self.get_logger().info('[InitialPose] Second publish sent. Done!')


def main():
    parser = argparse.ArgumentParser(description='Set AMCL initial pose for Wall-E robot')
    parser.add_argument('--x',   type=float, default=0.0,  help='X position on map (meters)')
    parser.add_argument('--y',   type=float, default=0.0,  help='Y position on map (meters)')
    parser.add_argument('--yaw', type=float, default=0.0,  help='Yaw angle in DEGREES (0=facing +X)')
    args = parser.parse_args()

    print(f'Setting initial pose → x={args.x}, y={args.y}, yaw={args.yaw}°')
    print('(Tip: Lấy tọa độ chính xác bằng cách click vào điểm trong RViz2 → "Publish Point" hoặc đọc /amcl_pose)')

    rclpy.init()
    node = InitialPoseSetter(args.x, args.y, args.yaw)
    # Spin 3 giây là đủ
    import time
    start = time.time()
    while rclpy.ok() and (time.time() - start) < 3.0:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()
    print('Initial pose set complete.')


if __name__ == '__main__':
    main()
