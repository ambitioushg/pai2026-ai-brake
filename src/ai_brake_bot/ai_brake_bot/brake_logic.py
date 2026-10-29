"""AI 自動ブレーキの判断ロジック（ROS 非依存なので単体テストできる）。

入力: 操縦者（キーボード or 自動運転）の速度指令と、YOLO の検出結果（距離つき）
出力: ブレーキを反映した速度指令と状態

状態（優先度の高い順）:
- FAILSAFE : AI の検出結果か LiDAR が一定時間届かない → 完全停止（センサが止まったら車も止める）
- STOP     : (reason=obstacle) 進路のすぐ前に何かある（LiDAR）→ 前進を禁止
             カメラの死角（近すぎて全身が映らない等）を補う最後の砦
- SIGN_WAIT: 止まれ標識を検出 → 一定時間の一時停止
- STOP     : (reason=person) レーン内の近くに人 → 前進を禁止（後退・旋回は許可）
- SLOW     : レーン内の少し先に人 → 距離に応じて減速
- CLEAR    : 指令をそのまま通す
"""

from dataclasses import asdict, dataclass
import math
from typing import Iterable, Optional

PERSON = 'person'
STOP_SIGN = 'stop sign'
OBSTACLE = 'obstacle'


@dataclass
class Detection:
    label: str
    confidence: float
    distance: Optional[float]   # [m] 推定できなければ None
    lateral: Optional[float]    # [m] 右が正


@dataclass
class BrakeConfig:
    # 人に対するブレーキ
    corridor_half_width: float = 0.6     # [m] 走行レーンの半幅（車体半幅 + 余裕）
    person_min_confidence: float = 0.35
    stop_distance: float = 1.5           # [m] これより近いと停止
    release_margin: float = 0.3          # [m] 停止後は stop_distance + これ を超えるまで解除しない
    slow_distance: float = 4.0           # [m] これより近いと減速開始
    min_speed_ratio: float = 0.25        # 減速中の最低速度比
    person_hold_time: float = 1.0        # [s] 人を見失っても停止を保つ時間
    # 止まれ標識
    sign_min_confidence: float = 0.6
    sign_trigger_distance: float = 3.0   # [m] これより近づいたら一時停止
    sign_confirm_frames: int = 2         # 連続何フレーム検出したら一時停止するか（誤検出対策）
    sign_wait_time: float = 3.0          # [s] 一時停止の時間
    sign_ignore_time: float = 8.0        # [s] 一時停止後、同じ標識を無視する時間（通過用）
    # 近距離の障害物（LiDAR）
    use_lidar: bool = True
    obstacle_stop_distance: float = 0.8  # [m] LiDAR から前方へこの距離以内に何かあれば停止
    obstacle_half_width: float = 0.35    # [m] 車体半幅 0.18 m + 余裕
    obstacle_hold_time: float = 0.5      # [s]
    # フェイルセーフ
    detection_timeout: float = 1.0       # [s] 検出結果がこれ以上途絶えたら停止
    scan_timeout: float = 1.0            # [s] LiDAR がこれ以上途絶えたら停止


@dataclass
class BrakeOutput:
    linear: float
    angular: float
    state: str
    reason: str = ''
    distance: Optional[float] = None
    speed_ratio: float = 1.0
    sign_wait_remaining: float = 0.0

    def to_dict(self) -> dict:
        return asdict(self)


class BrakeController:
    def __init__(self, config: Optional[BrakeConfig] = None):
        self.cfg = config or BrakeConfig()
        self._last_detection_time: Optional[float] = None
        self._nearest_person = math.inf
        self._person_seen_at = -math.inf
        self._person_stop_until = -math.inf
        self._sign_frames = 0
        self._sign_distance: Optional[float] = None
        self._sign_wait_until = -math.inf
        self._sign_ignore_until = -math.inf
        self._last_scan_time: Optional[float] = None
        self._obstacle_distance: Optional[float] = None
        self._obstacle_stop_until = -math.inf

    def update_scan(self, nearest_obstacle: float, now: float) -> None:
        """LiDAR のスキャンが届くたびに、レーン内で最も近い障害物までの距離を渡す。"""
        self._last_scan_time = now
        if nearest_obstacle <= self.cfg.obstacle_stop_distance:
            self._obstacle_distance = nearest_obstacle
            self._obstacle_stop_until = now + self.cfg.obstacle_hold_time

    def in_corridor(self, det: Detection) -> bool:
        return det.lateral is not None and abs(det.lateral) <= self.cfg.corridor_half_width

    def update(self, detections: Iterable[Detection], now: float) -> None:
        """YOLO の検出結果が届くたびに呼ぶ。"""
        cfg = self.cfg
        detections = list(detections)
        self._last_detection_time = now

        # 人: レーン内で最も近い人の距離。見失っても person_hold_time の間は直前の値を保持
        # （レーンの境界付近で 減速/解除 が細かく切り替わるのを防ぐ）
        nearest = min(
            (d.distance for d in detections
             if d.label == PERSON and d.confidence >= cfg.person_min_confidence
             and d.distance is not None and self.in_corridor(d)),
            default=math.inf)
        if math.isfinite(nearest):
            self._nearest_person = nearest
            self._person_seen_at = now
            stopping = now < self._person_stop_until
            threshold = cfg.stop_distance + (cfg.release_margin if stopping else 0.0)
            if nearest <= threshold:
                self._person_stop_until = now + cfg.person_hold_time
        elif now - self._person_seen_at > cfg.person_hold_time:
            self._nearest_person = math.inf

        # 止まれ標識: 近くで連続検出したら一時停止を開始
        if now < self._sign_ignore_until:
            return
        near_signs = [d.distance for d in detections
                      if d.label == STOP_SIGN and d.confidence >= cfg.sign_min_confidence
                      and d.distance is not None and d.distance <= cfg.sign_trigger_distance]
        if not near_signs:
            self._sign_frames = 0
            return
        self._sign_frames += 1
        self._sign_distance = min(near_signs)
        if self._sign_frames >= cfg.sign_confirm_frames:
            self._sign_wait_until = now + cfg.sign_wait_time
            self._sign_ignore_until = self._sign_wait_until + cfg.sign_ignore_time
            self._sign_frames = 0

    def compute(self, linear_cmd: float, angular_cmd: float, now: float) -> BrakeOutput:
        """操縦者の指令に AI ブレーキを適用した指令を返す（一定周期で呼ぶ）。"""
        cfg = self.cfg
        if (self._last_detection_time is None
                or now - self._last_detection_time > cfg.detection_timeout):
            return BrakeOutput(0.0, 0.0, 'FAILSAFE', reason='no AI input', speed_ratio=0.0)
        if cfg.use_lidar and (self._last_scan_time is None
                              or now - self._last_scan_time > cfg.scan_timeout):
            return BrakeOutput(0.0, 0.0, 'FAILSAFE', reason='no LiDAR', speed_ratio=0.0)

        forward = max(linear_cmd, 0.0)
        backward = min(linear_cmd, 0.0)   # 後退は止めない（前方センサの判断なので）
        nearest = self._nearest_person if math.isfinite(self._nearest_person) else None

        if cfg.use_lidar and now < self._obstacle_stop_until:
            return BrakeOutput(backward, angular_cmd, 'STOP', reason=OBSTACLE,
                               distance=self._obstacle_distance, speed_ratio=0.0)

        if now < self._sign_wait_until:
            return BrakeOutput(backward, angular_cmd, 'SIGN_WAIT', reason=STOP_SIGN,
                               distance=self._sign_distance, speed_ratio=0.0,
                               sign_wait_remaining=self._sign_wait_until - now)

        if now < self._person_stop_until:
            return BrakeOutput(backward, angular_cmd, 'STOP', reason=PERSON,
                               distance=nearest, speed_ratio=0.0)

        if nearest is not None and nearest < cfg.slow_distance:
            ratio = (nearest - cfg.stop_distance) / (cfg.slow_distance - cfg.stop_distance)
            ratio = min(1.0, max(cfg.min_speed_ratio, ratio))
            return BrakeOutput(forward * ratio + backward, angular_cmd, 'SLOW', reason=PERSON,
                               distance=nearest, speed_ratio=ratio)

        return BrakeOutput(linear_cmd, angular_cmd, 'CLEAR')
