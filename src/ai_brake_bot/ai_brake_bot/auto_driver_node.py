"""一定速度で道路の中央を走り続ける「ドライバー役」のノード（評価・デモ用）。

前方の人や標識は一切見ていない（＝よそ見をしているドライバー）。
止まるかどうかは AI ブレーキ（safety_brake）次第、という状況を再現する。

購読: odom (nav_msgs/Odometry)
配信: cmd_vel_teleop (geometry_msgs/Twist)
"""

import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class AutoDriver(Node):
    def __init__(self):
        super().__init__('auto_driver')
        self.declare_parameters('', [
            ('speed', 0.5),          # [m/s] 巡航速度
            ('goal_x', 40.0),        # [m]   ここまで来たら止まる
            ('lane_y', 0.0),         # [m]   走行レーンの中心
            ('k_lateral', 1.0),      # 横ずれのゲイン
            ('k_heading', 2.0),      # 向きのずれのゲイン
            ('max_angular', 0.5),    # [rad/s]
            ('start_delay', 0.0),    # [s] 走り出すまでの待ち時間（評価で歩行者との出会い方を変える）
            ('rate', 10.0),
        ])
        get = lambda name: self.get_parameter(name).value  # noqa: E731
        self.speed, self.goal_x, self.lane_y = get('speed'), get('goal_x'), get('lane_y')
        self.k_lat, self.k_head, self.max_w = get('k_lateral'), get('k_heading'), get('max_angular')
        self.start_delay = get('start_delay')
        self.start_time = None
        self.pose = None
        self.reached = False
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel_teleop', 10)
        self.create_timer(1.0 / get('rate'), self.on_timer)
        self.get_logger().info(f'自動運転: {self.speed} m/s で x={self.goal_x} m まで走行')

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))

    def on_timer(self):
        cmd = Twist()
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.start_time is None:
            self.start_time = now
        if self.pose is not None and now - self.start_time >= self.start_delay:
            x, y, yaw = self.pose
            if x >= self.goal_x:
                if not self.reached:
                    self.get_logger().info('ゴールに到着')
                    self.reached = True
            else:
                w = -self.k_lat * (y - self.lane_y) - self.k_head * yaw
                cmd.linear.x = self.speed
                cmd.angular.z = max(-self.max_w, min(self.max_w, w))
        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = AutoDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
