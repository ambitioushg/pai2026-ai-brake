"""操縦者の速度指令に AI ブレーキを掛けてロボットへ送るノード。

購読: cmd_vel_teleop      (geometry_msgs/Twist)  … キーボード操作 or 自動運転の指令
      ai_brake/detections (std_msgs/String, JSON) … yolo_detector の検出結果
      scan                (sensor_msgs/LaserScan) … 近距離の障害物（カメラの死角対策）
配信: cmd_vel             (geometry_msgs/Twist)  … ブレーキ適用後の指令（ロボットへ）
      ai_brake/state      (std_msgs/String, JSON) … 状態（表示・評価用）
"""

from dataclasses import fields
import json

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from .brake_logic import BrakeConfig, BrakeController, Detection
from .geometry import nearest_in_corridor


class SafetyBrake(Node):
    def __init__(self):
        super().__init__('safety_brake')
        defaults = BrakeConfig()
        config = {f.name: self.declare_parameter(f.name, getattr(defaults, f.name)).value
                  for f in fields(BrakeConfig)}
        rate = self.declare_parameter('rate', 20.0).value
        self.controller = BrakeController(BrakeConfig(**config))
        self.cmd = (0.0, 0.0)
        self.last_state = None

        self.create_subscription(Twist, 'cmd_vel_teleop', self.on_cmd, 10)
        self.create_subscription(String, 'ai_brake/detections', self.on_detections, 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_state = self.create_publisher(String, 'ai_brake/state', 10)
        self.create_timer(1.0 / rate, self.on_timer)
        self.get_logger().info(f'AI ブレーキ開始: {config}')

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_cmd(self, msg: Twist):
        self.cmd = (msg.linear.x, msg.angular.z)

    def on_detections(self, msg: String):
        data = json.loads(msg.data)
        detections = [Detection(d['label'], d['confidence'], d['distance'], d['lateral'])
                      for d in data['detections']]
        self.controller.update(detections, self.now())

    def on_scan(self, msg: LaserScan):
        nearest = nearest_in_corridor(msg.ranges, msg.angle_min, msg.angle_increment,
                                      self.controller.cfg.obstacle_half_width,
                                      msg.range_min, msg.range_max)
        self.controller.update_scan(nearest, self.now())

    def on_timer(self):
        now = self.now()
        out = self.controller.compute(self.cmd[0], self.cmd[1], now)

        twist = Twist()
        twist.linear.x = out.linear
        twist.angular.z = out.angular
        self.pub_cmd.publish(twist)

        state = out.to_dict()
        state.update(cmd_linear=self.cmd[0], cmd_angular=self.cmd[1], stamp=now)
        self.pub_state.publish(String(data=json.dumps(state)))

        if out.state != self.last_state:
            dist = f' {out.distance:.2f} m' if out.distance is not None else ''
            self.get_logger().info(f'{self.last_state} -> {out.state} {out.reason}{dist}')
            self.last_state = out.state


def main():
    rclpy.init()
    node = SafetyBrake()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
