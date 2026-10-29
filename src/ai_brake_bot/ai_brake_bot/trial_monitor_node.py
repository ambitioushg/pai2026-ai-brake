"""自動運転 1 回分（1 試行）の結果を記録・判定するノード（評価用）。

購読: odom, ai_brake/state
出力: <results_dir>/<trial_name>.csv … 時系列ログ
      <results_dir>/summary.csv      … 1 試行 1 行の結果

判定項目:
- sign_ok      : 止まれ標識で一時停止したか（SIGN_WAIT 中に速度がほぼ 0 になったか）
- no_false_stop: 歩道の人に反応して誤ってブレーキしなかったか
- crossing_gap : 横断歩道の歩行者が進路上にいる間、最も近づいたときの距離（負なら衝突）
                 ※ Gazebo の歩行者（actor）には当たり判定がないため、台本の軌道から位置を計算する
- final_gap    : レーン上に立つ人の手前、何 m で止まれたか
試行は「人の手前で止まって settle_time 秒経過」または timeout で終了し、ノードも終了する。
"""

import csv
import json
import math
from pathlib import Path

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

SUMMARY_FIELDS = ['trial', 'speed', 'result', 'success', 'sign_ok', 'sign_stop_x',
                  'no_false_stop', 'crossing_stops', 'crossing_gap', 'final_x', 'final_gap',
                  'min_gap', 'duration']


def interpolate(t: float, times: list, values: list) -> float:
    """台本（時刻と値の折れ線）を周期 times[-1] でループさせたときの値。"""
    t = t % times[-1]
    for t0, t1, v0, v1 in zip(times, times[1:], values, values[1:]):
        if t0 <= t <= t1:
            return v0 if t1 == t0 else v0 + (v1 - v0) * (t - t0) / (t1 - t0)
    return values[-1]


class TrialMonitor(Node):
    def __init__(self):
        super().__init__('trial_monitor')
        self.declare_parameters('', [
            ('trial_name', 'trial'),
            ('results_dir', 'results'),
            ('speed', 0.0),                     # 記録用（auto_driver の巡航速度）
            ('lane_person_x', 24.0),            # [m] レーン上に立つ人の位置（ワールドと合わせる）
            ('robot_front_offset', 0.25),       # [m] ロボット中心から前端まで
            ('sidewalk_zone', [4.0, 10.5]),     # [m] 歩道の人だけが近くに見える区間
            ('crossing_zone', [10.5, 17.0]),    # [m] 横断する人に反応しうる区間
            ('end_min_x', 18.0),                # [m] これより先で停止したら試行終了
            ('settle_time', 4.0),               # [s]
            ('timeout', 300.0),                 # [s]
            ('success_gap', 0.5),               # [m] 立っている人との停止距離がこれ以上なら成功
            # 横断者は止まったロボットの前を横切ることもあるので「接触しない（余裕 0.2 m）」で判定
            ('crossing_success_gap', 0.2),
            # 横断歩道の歩行者の台本（worlds/town_road.sdf の actor と合わせる）
            ('crossing_x', 16.0),
            ('crossing_times', [0.0, 6.0, 18.0, 24.0, 25.0, 31.0, 43.0, 49.0, 50.0]),
            ('crossing_y', [3.0, 0.0, 0.0, -3.0, -3.0, 0.0, 0.0, 3.0, 3.0]),
            ('crossing_path_half_width', 0.5),  # [m] ロボット半幅 + 人の半幅
            ('person_half_depth', 0.2),         # [m]
            ('robot_length', 0.5),              # [m]
        ])
        get = lambda name: self.get_parameter(name).value  # noqa: E731
        self.p = {name: get(name) for name in [
            'trial_name', 'results_dir', 'speed', 'lane_person_x', 'robot_front_offset',
            'sidewalk_zone', 'crossing_zone', 'end_min_x', 'settle_time', 'timeout',
            'success_gap', 'crossing_success_gap', 'crossing_x', 'crossing_times', 'crossing_y',
            'crossing_path_half_width', 'person_half_depth', 'robot_length']}
        self.crossing_gap = math.inf
        self.odom = None
        self.state = None
        self.rows = []
        self.start = None
        self.still_since = None
        self.prev_state = None
        self.sign_stop_x = None
        self.sign_min_speed = math.inf
        self.false_stop = False
        self.crossing_stops = 0
        self.done = False
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_subscription(String, 'ai_brake/state', self.on_state, 10)
        self.create_timer(0.1, self.on_timer)

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, msg: Odometry):
        self.odom = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                     msg.twist.twist.linear.x)

    def on_state(self, msg: String):
        self.state = json.loads(msg.data)

    def on_timer(self):
        if self.done or self.odom is None or self.state is None:
            return
        t = self.now()
        if self.start is None:
            self.start = t
        x, y, v = self.odom
        s = self.state
        name, reason = s['state'], s['reason']
        actor_y = interpolate(t, self.p['crossing_times'], self.p['crossing_y'])
        self.rows.append([round(t - self.start, 2), round(x, 3), round(y, 3), round(v, 3), name,
                          reason, s['distance'], s['cmd_linear'], s['linear'],
                          round(actor_y, 3)])

        # 横断歩道の歩行者が進路上にいて、ロボットがまだ通り過ぎていなければ距離を記録
        front = x + self.p['robot_front_offset']
        rear = front - self.p['robot_length']
        if (abs(actor_y - y) < self.p['crossing_path_half_width']
                and rear <= self.p['crossing_x'] + self.p['person_half_depth']):
            gap = self.p['crossing_x'] - self.p['person_half_depth'] - front
            self.crossing_gap = min(self.crossing_gap, gap)

        if name == 'SIGN_WAIT':
            if self.sign_stop_x is None:
                self.sign_stop_x = x
            self.sign_min_speed = min(self.sign_min_speed, abs(v))
        lo, hi = self.p['sidewalk_zone']
        if lo <= x <= hi and name in ('STOP', 'SLOW') and reason == 'person':
            self.false_stop = True
        lo, hi = self.p['crossing_zone']
        if lo <= x <= hi and name == 'STOP' and self.prev_state != 'STOP':
            self.crossing_stops += 1
        self.prev_state = name

        self.still_since = (self.still_since or t) if abs(v) < 0.02 else None
        settled = (x >= self.p['end_min_x'] and self.still_since is not None
                   and t - self.still_since >= self.p['settle_time'])
        if settled or t - self.start >= self.p['timeout']:
            self.finish('stopped' if settled else 'timeout')

    def gap(self, x: float) -> float:
        return self.p['lane_person_x'] - (x + self.p['robot_front_offset'])

    def finish(self, result: str):
        self.done = True
        final_x = self.odom[0]
        sign_ok = self.sign_stop_x is not None and self.sign_min_speed < 0.05
        final_gap = self.gap(final_x)
        crossing_gap = None if math.isinf(self.crossing_gap) else round(self.crossing_gap, 3)
        crossing_ok = crossing_gap is None or crossing_gap >= self.p['crossing_success_gap']
        summary = {
            'trial': self.p['trial_name'],
            'speed': self.p['speed'],
            'result': result,
            'success': result == 'stopped' and sign_ok and not self.false_stop
                       and crossing_ok and final_gap >= self.p['success_gap'],
            'sign_ok': sign_ok,
            'sign_stop_x': None if self.sign_stop_x is None else round(self.sign_stop_x, 2),
            'no_false_stop': not self.false_stop,
            'crossing_stops': self.crossing_stops,
            'crossing_gap': crossing_gap,
            'final_x': round(final_x, 3),
            'final_gap': round(final_gap, 3),
            'min_gap': round(min(self.gap(r[1]) for r in self.rows), 3),
            'duration': self.rows[-1][0],
        }
        out = Path(self.p['results_dir'])
        out.mkdir(parents=True, exist_ok=True)
        with open(out / f"{self.p['trial_name']}.csv", 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'x', 'y', 'v', 'state', 'reason', 'distance', 'cmd_linear',
                        'out_linear', 'crossing_person_y'])
            w.writerows(self.rows)
        summary_path = out / 'summary.csv'
        new_file = not summary_path.exists()
        with open(summary_path, 'a', newline='') as f:
            w = csv.DictWriter(f, fieldnames=SUMMARY_FIELDS)
            if new_file:
                w.writeheader()
            w.writerow(summary)
        self.get_logger().info('試行結果: ' + json.dumps(summary, ensure_ascii=False))


def main():
    rclpy.init()
    node = TrialMonitor()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
