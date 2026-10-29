import math

import pytest

from ai_brake_bot.brake_logic import (
    OBSTACLE, PERSON, STOP_SIGN, BrakeConfig, BrakeController, Detection)


def person(distance, lateral=0.0, conf=0.9):
    return Detection(PERSON, conf, distance, lateral)


def sign(distance, conf=0.9):
    return Detection(STOP_SIGN, conf, distance, 1.8)


@pytest.fixture
def ctrl():
    """カメラ（YOLO）側の判断だけを確かめるため LiDAR は無効にする。"""
    return BrakeController(BrakeConfig(use_lidar=False))


@pytest.fixture
def ctrl_lidar():
    return BrakeController(BrakeConfig(use_lidar=True))


def test_failsafe_without_detections(ctrl):
    out = ctrl.compute(0.5, 0.2, now=0.0)
    assert out.state == 'FAILSAFE'
    assert (out.linear, out.angular) == (0.0, 0.0)


def test_failsafe_when_detections_stop(ctrl):
    ctrl.update([], now=0.0)
    assert ctrl.compute(0.5, 0.0, now=0.5).state == 'CLEAR'
    assert ctrl.compute(0.5, 0.0, now=1.5).state == 'FAILSAFE'


def test_clear_passes_command(ctrl):
    ctrl.update([], now=0.0)
    out = ctrl.compute(0.5, 0.3, now=0.1)
    assert (out.state, out.linear, out.angular) == ('CLEAR', 0.5, 0.3)


def test_slow_down_in_proportion_to_distance(ctrl):
    ctrl.update([person(3.0)], now=0.0)
    out = ctrl.compute(0.5, 0.0, now=0.05)
    assert out.state == 'SLOW'
    assert out.speed_ratio == pytest.approx((3.0 - 1.5) / (4.0 - 1.5))
    assert out.linear == pytest.approx(0.5 * out.speed_ratio)


def test_slow_keeps_minimum_speed(ctrl):
    ctrl.update([person(1.6)], now=0.0)
    out = ctrl.compute(0.5, 0.0, now=0.05)
    assert out.state == 'SLOW'
    assert out.speed_ratio == pytest.approx(0.25)


def test_stop_blocks_forward_but_allows_reverse_and_turn(ctrl):
    ctrl.update([person(1.2)], now=0.0)
    fwd = ctrl.compute(0.5, 0.4, now=0.05)
    assert (fwd.state, fwd.linear, fwd.angular) == ('STOP', 0.0, 0.4)
    back = ctrl.compute(-0.2, 0.0, now=0.05)
    assert back.linear == -0.2


def test_person_outside_lane_is_ignored(ctrl):
    ctrl.update([person(1.0, lateral=2.0)], now=0.0)
    assert ctrl.compute(0.5, 0.0, now=0.05).state == 'CLEAR'


def test_low_confidence_person_is_ignored(ctrl):
    ctrl.update([person(1.0, conf=0.2)], now=0.0)
    assert ctrl.compute(0.5, 0.0, now=0.05).state == 'CLEAR'


def test_stop_is_held_after_person_disappears(ctrl):
    ctrl.update([person(1.2)], now=0.0)
    ctrl.update([], now=0.5)                       # 一瞬見失っても
    assert ctrl.compute(0.5, 0.0, now=0.6).state == 'STOP'
    ctrl.update([], now=1.1)
    assert ctrl.compute(0.5, 0.0, now=1.2).state == 'CLEAR'   # 保持時間 1.0 s 後に解除


def test_slow_is_held_briefly_when_person_flickers(ctrl):
    ctrl.update([person(3.0)], now=0.0)
    ctrl.update([person(3.0, lateral=0.7)], now=0.5)   # レーン境界の外に一瞬出ても
    out = ctrl.compute(0.5, 0.0, now=0.6)
    assert (out.state, out.distance) == ('SLOW', 3.0)  # 直前の距離で減速を続ける
    ctrl.update([], now=1.2)
    assert ctrl.compute(0.5, 0.0, now=1.3).state == 'CLEAR'


def test_release_margin_prevents_creeping(ctrl):
    ctrl.update([person(1.4)], now=0.0)
    ctrl.update([person(1.7)], now=0.9)            # 停止中は 1.5 + 0.3 まで停止を継続
    assert ctrl.compute(0.5, 0.0, now=1.5).state == 'STOP'
    ctrl.update([person(1.9)], now=1.9)            # 1.8 m を超えたら（保持時間後に）減速へ
    assert ctrl.compute(0.5, 0.0, now=2.0).state == 'SLOW'


def test_stop_sign_wait_then_ignore(ctrl):
    ctrl.update([sign(2.8)], now=0.0)
    assert ctrl.compute(0.5, 0.0, now=0.05).state == 'CLEAR'  # 1 フレームだけでは止まらない
    ctrl.update([sign(2.7)], now=0.1)
    out = ctrl.compute(0.5, 0.0, now=0.2)
    assert out.state == 'SIGN_WAIT'
    assert out.linear == 0.0
    assert out.sign_wait_remaining == pytest.approx(2.9)
    ctrl.update([sign(2.6)], now=3.0)
    assert ctrl.compute(0.5, 0.0, now=3.05).state == 'SIGN_WAIT'
    ctrl.update([sign(2.5)], now=3.2)              # 一時停止後は同じ標識を無視して発進
    assert ctrl.compute(0.5, 0.0, now=3.25).state == 'CLEAR'
    ctrl.update([sign(2.5)], now=11.2)             # 無視時間（8 s）を過ぎたら再び有効
    ctrl.update([sign(2.5)], now=11.3)
    assert ctrl.compute(0.5, 0.0, now=11.35).state == 'SIGN_WAIT'


def test_lidar_stops_when_camera_misses_close_person(ctrl_lidar):
    # 近すぎてカメラ（YOLO）には人が映っていないが、LiDAR が 0.6 m 先に何かを捉えている
    ctrl_lidar.update([], now=0.0)
    ctrl_lidar.update_scan(0.6, now=0.0)
    out = ctrl_lidar.compute(0.5, 0.2, now=0.05)
    assert (out.state, out.reason, out.linear, out.angular) == ('STOP', OBSTACLE, 0.0, 0.2)
    ctrl_lidar.update([], now=0.6)
    ctrl_lidar.update_scan(math.inf, now=0.6)      # 障害物がなくなり、保持時間 0.5 s を過ぎたら解除
    assert ctrl_lidar.compute(0.5, 0.0, now=0.65).state == 'CLEAR'


def test_lidar_ignores_far_obstacles(ctrl_lidar):
    ctrl_lidar.update([], now=0.0)
    ctrl_lidar.update_scan(2.0, now=0.0)
    assert ctrl_lidar.compute(0.5, 0.0, now=0.05).state == 'CLEAR'


def test_failsafe_without_lidar(ctrl_lidar):
    ctrl_lidar.update([], now=0.0)
    out = ctrl_lidar.compute(0.5, 0.0, now=0.05)
    assert (out.state, out.reason) == ('FAILSAFE', 'no LiDAR')


def test_far_stop_sign_is_ignored(ctrl):
    ctrl.update([sign(5.0)], now=0.0)
    ctrl.update([sign(4.5)], now=0.1)
    assert ctrl.compute(0.5, 0.0, now=0.2).state == 'CLEAR'
