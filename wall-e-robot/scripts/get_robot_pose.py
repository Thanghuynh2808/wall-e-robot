#!/usr/bin/env python3
"""
get_robot_pose.py
-----------------
In ra vị trí hiện tại của robot trên bản đồ (từ /amcl_pose).
Dùng để lấy tọa độ chính xác của từng điểm A, B, C khi bạn
đặt robot tay vào vị trí đó, rồi copy tọa độ vào script.

Cách dùng:
    python3 get_robot_pose.py
    python3 get_robot_pose.py --once      # Chỉ in 1 lần rồi thoát
    python3 get_robot_pose.py --count 5   # In 5 lần rồi thoát
"""

import argparse
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


def quat_to_yaw_deg(qx, qy, qz, qw) -> float:
    """Convert quaternion to yaw in degrees."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw_rad = math.atan2(siny_cosp, cosy_cosp)
    return math.degrees(yaw_rad)


class PoseReader(Node):
    def __init__(self, max_count: int):
        super().__init__('pose_reader')
        self.max_count = max_count
        self.count = 0
        self.sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )
        self.get_logger().info('Đang lắng nghe /amcl_pose... (Ctrl+C để dừng)')
        print('\n📍 Vị trí robot hiện tại:\n')
        print(f'  {"#":>3}  {"X (m)":>8}  {"Y (m)":>8}  {"Yaw (°)":>8}')
        print(f'  {"-"*3}  {"-"*8}  {"-"*8}  {"-"*8}')

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose
        x = p.position.x
        y = p.position.y
        yaw = quat_to_yaw_deg(
            p.orientation.x, p.orientation.y,
            p.orientation.z, p.orientation.w
        )
        self.count += 1
        print(f'  {self.count:>3}  {x:>8.4f}  {y:>8.4f}  {yaw:>8.2f}')
        print(f'       → copy vào script: ({x:.4f}, {y:.4f}, {yaw:.2f})')

        if self.max_count > 0 and self.count >= self.max_count:
            self.get_logger().info(f'Đã đọc {self.count} lần. Thoát.')
            raise SystemExit(0)


def main():
    parser = argparse.ArgumentParser(description='Đọc vị trí robot từ /amcl_pose')
    parser.add_argument('--once', action='store_true', help='Chỉ in 1 lần rồi thoát')
    parser.add_argument('--count', type=int, default=0,
                        help='Số lần in rồi thoát (0 = liên tục)')
    args = parser.parse_args()

    max_count = 1 if args.once else args.count

    rclpy.init()
    node = PoseReader(max_count=max_count)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print('\n[Done]')


if __name__ == '__main__':
    main()
