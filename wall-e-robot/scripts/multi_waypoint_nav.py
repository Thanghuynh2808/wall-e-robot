#!/usr/bin/env python3
"""
multi_waypoint_nav.py
---------------------
Điều hướng robot Wall-E đến nhiều điểm (A → B → C ...) tuần tự,
với delay tùy chỉnh giữa mỗi điểm. KHÔNG cần RViz2.

Cách dùng:
    # Đi qua 3 điểm, delay mặc định 5 giây mỗi điểm:
    python3 multi_waypoint_nav.py

    # Chỉ định tọa độ qua command line:
    python3 multi_waypoint_nav.py \
        --waypoints "1.0,0.5,0 | 2.0,1.0,90 | 0.0,0.0,180" \
        --delay 8

    # Định dạng mỗi waypoint: "x,y,yaw_degree"
    # Ví dụ: "1.5,2.0,45" = vị trí (1.5m, 2.0m), hướng 45°

Lưu ý:
    - Nav2 phải đang chạy (nav_bringup.launch.py)
    - Robot phải đã có initial pose (set_initial_pose.py hoặc RViz2)
    - Tọa độ theo frame MAP (lấy từ RViz2 hoặc /amcl_pose)
"""

import argparse
import math
import time
import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose, FollowWaypoints


# ──────────────────────────────────────────────────────────────
# Cấu hình mặc định (chỉnh sửa tại đây nếu không dùng CLI args)
# ──────────────────────────────────────────────────────────────
DEFAULT_WAYPOINTS = [
    # (x_m, y_m, yaw_degree)   ← tọa độ trên bản đồ (map frame)
    (1.0,  0.0,  0.0),   # Điểm A
    (1.0,  1.0, 90.0),   # Điểm B
    (0.0,  1.0, 180.0),  # Điểm C
]

DEFAULT_DELAY_SECONDS = 5.0   # Thời gian dừng tại mỗi điểm (giây)
DEFAULT_GOAL_TIMEOUT  = 120.0 # Timeout tối đa cho mỗi goal (giây)
# ──────────────────────────────────────────────────────────────


def deg_to_quat(yaw_deg: float):
    """Convert yaw degrees → quaternion (x, y, z, w)."""
    yaw_rad = math.radians(yaw_deg)
    return (0.0, 0.0, math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0))


def make_pose_stamped(x: float, y: float, yaw_deg: float) -> PoseStamped:
    """Tạo PoseStamped message từ x, y, yaw."""
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0
    qx, qy, qz, qw = deg_to_quat(yaw_deg)
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


class WaypointNavigator(Node):
    """Node điều hướng qua nhiều waypoint tuần tự với delay."""

    def __init__(self):
        super().__init__('waypoint_navigator')
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def navigate_to(self, x: float, y: float, yaw_deg: float, label: str = '') -> bool:
        """
        Gửi goal đến NavigateToPose action server và đợi kết quả.
        Trả về True nếu thành công, False nếu thất bại.
        """
        self.get_logger().info(f'Đang chờ Nav2 action server...')
        if not self._nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Nav2 action server không phản hồi sau 10 giây!')
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = make_pose_stamped(x, y, yaw_deg)
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        self.get_logger().info(
            f'→ Gửi goal {label}: x={x:.3f}, y={y:.3f}, yaw={yaw_deg:.1f}°'
        )

        send_goal_future = self._nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'Goal {label} bị từ chối bởi Nav2!')
            return False

        self.get_logger().info(f'Goal {label} được chấp nhận. Đang điều hướng...')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=DEFAULT_GOAL_TIMEOUT)

        result = result_future.result()
        if result is None:
            self.get_logger().error(f'Goal {label} timeout sau {DEFAULT_GOAL_TIMEOUT}s!')
            return False

        status = result.status
        # GoalStatus: SUCCEEDED=4, ABORTED=6, CANCELED=5
        if status == 4:
            self.get_logger().info(f'✓ Goal {label} hoàn thành!')
            return True
        else:
            self.get_logger().error(f'✗ Goal {label} thất bại! Status={status}')
            return False


def parse_waypoints_string(wp_str: str):
    """
    Parse chuỗi waypoint từ CLI.
    Định dạng: "x1,y1,yaw1 | x2,y2,yaw2 | x3,y3,yaw3"
    """
    waypoints = []
    for part in wp_str.split('|'):
        part = part.strip()
        if not part:
            continue
        vals = [v.strip() for v in part.split(',')]
        if len(vals) != 3:
            raise ValueError(f'Waypoint "{part}" phải có đúng 3 giá trị: x,y,yaw')
        waypoints.append((float(vals[0]), float(vals[1]), float(vals[2])))
    return waypoints


def main():
    global DEFAULT_GOAL_TIMEOUT

    parser = argparse.ArgumentParser(
        description='Multi-waypoint navigation cho Wall-E robot',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Ví dụ:
  # Dùng waypoints mặc định trong script (chỉnh DEFAULT_WAYPOINTS)
  python3 multi_waypoint_nav.py

  # Chỉ định waypoints qua CLI (x,y,yaw_degrees | ...)
  python3 multi_waypoint_nav.py --waypoints "1.0,0.0,0 | 1.0,1.5,90 | 0.0,0.0,180" --delay 8

  # Chỉ đi đến 1 điểm
  python3 multi_waypoint_nav.py --waypoints "2.0,1.0,45" --delay 0
        """
    )
    parser.add_argument(
        '--waypoints',
        type=str,
        default=None,
        help='Chuỗi waypoints: "x1,y1,yaw1 | x2,y2,yaw2 | ..." (yaw tính bằng degrees)'
    )
    parser.add_argument(
        '--delay',
        type=float,
        default=DEFAULT_DELAY_SECONDS,
        help=f'Thời gian dừng tại mỗi waypoint (giây, mặc định={DEFAULT_DELAY_SECONDS})'
    )
    parser.add_argument(
        '--timeout',
        type=float,
        default=DEFAULT_GOAL_TIMEOUT,
        help=f'Timeout tối đa cho mỗi goal (giây, mặc định={DEFAULT_GOAL_TIMEOUT})'
    )
    args = parser.parse_args()

    # Cập nhật timeout global
    DEFAULT_GOAL_TIMEOUT = args.timeout

    # Lấy danh sách waypoints
    if args.waypoints:
        try:
            waypoints = parse_waypoints_string(args.waypoints)
        except ValueError as e:
            print(f'[LỖI] {e}')
            sys.exit(1)
    else:
        waypoints = DEFAULT_WAYPOINTS

    if not waypoints:
        print('[LỖI] Không có waypoint nào!')
        sys.exit(1)

    # ── In tóm tắt hành trình ──
    print('\n' + '='*55)
    print('  Wall-E Multi-Waypoint Navigation')
    print('='*55)
    labels = [chr(65 + i) for i in range(len(waypoints))]  # A, B, C, D...
    for i, (x, y, yaw) in enumerate(waypoints):
        print(f'  Điểm {labels[i]}: x={x:6.3f}m, y={y:6.3f}m, hướng={yaw:.1f}°')
    print(f'\n  Delay giữa các điểm : {args.delay:.1f} giây')
    print(f'  Timeout mỗi goal    : {DEFAULT_GOAL_TIMEOUT:.0f} giây')
    print('='*55 + '\n')

    # ── Khởi động ROS2 ──
    rclpy.init()
    navigator = WaypointNavigator()

    try:
        total = len(waypoints)
        for i, (x, y, yaw) in enumerate(waypoints):
            label = labels[i]
            print(f'\n[{i+1}/{total}] Đang di chuyển đến điểm {label}...')

            success = navigator.navigate_to(x, y, yaw, label=f'Điểm {label}')

            if success:
                print(f'  ✓ Đã đến điểm {label}!')
                if i < total - 1:
                    print(f'  ⏳ Dừng {args.delay:.1f} giây trước khi đến điểm {labels[i+1]}...')
                    time.sleep(args.delay)
            else:
                print(f'  ✗ Không thể đến điểm {label}. Bỏ qua và thử điểm tiếp theo...')
                if i < total - 1:
                    print(f'  ⏳ Dừng {args.delay:.1f} giây...')
                    time.sleep(args.delay)

        print('\n' + '='*55)
        print('  Hành trình hoàn thành!')
        print('='*55 + '\n')

    except KeyboardInterrupt:
        print('\n[!] Người dùng dừng chương trình (Ctrl+C)')

    finally:
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
