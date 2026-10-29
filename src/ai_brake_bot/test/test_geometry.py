import math

import pytest

from ai_brake_bot.geometry import (
    CameraModel, corridor_polygon, distance_from_width, ground_point_from_pixel,
    ground_position_from_bbox, nearest_in_corridor, project_ground_point, suppress_nested_boxes)

CAM = CameraModel(width=640, height=480, hfov=1.5, mount_height=0.7)


def test_focal_length():
    assert CAM.fx == pytest.approx(320.0 / math.tan(0.75))
    assert CAM.fy == CAM.fx


@pytest.mark.parametrize('distance,lateral', [(1.5, 0.0), (3.0, 0.5), (8.0, -1.2)])
def test_project_and_back(distance, lateral):
    u, v = project_ground_point(CAM, distance, lateral)
    d, lat = ground_point_from_pixel(CAM, u, v)
    assert d == pytest.approx(distance)
    assert lat == pytest.approx(lateral)


def test_above_horizon_is_none():
    assert ground_point_from_pixel(CAM, 320, CAM.cy) is None
    assert ground_point_from_pixel(CAM, 320, 100) is None


def test_bbox_bottom_gives_feet_distance():
    u, v = project_ground_point(CAM, 2.5, 0.3)
    d, lat = ground_position_from_bbox(CAM, (u - 30, 50, u + 30, v))
    assert d == pytest.approx(2.5)
    assert lat == pytest.approx(0.3)


def test_cut_off_feet_is_treated_as_closest():
    d, _ = ground_position_from_bbox(CAM, (300, 0, 340, 479.5))
    assert d == pytest.approx(CAM.min_ground_distance)
    assert d < 1.1  # 高さ 0.7 m, 垂直画角 ±35° 付近 → 約 1.0 m


def test_distance_from_width():
    real_width = 0.6
    w_px = CAM.fx * real_width / 3.0   # 3 m 先なら何 px になるか
    d, lat = distance_from_width(CAM, (320 - w_px / 2, 100, 320 + w_px / 2, 160), real_width)
    assert d == pytest.approx(3.0)
    assert lat == pytest.approx(0.0, abs=1e-9)
    assert distance_from_width(CAM, (100, 100, 100.5, 120), real_width) is None


def test_nearest_in_corridor():
    n = 181
    angle_min, inc = -math.pi / 2, math.pi / (n - 1)    # -90° 〜 +90° を 1° 刻み
    ranges = [math.inf] * n
    ranges[90] = 1.2                  # 正面 1.2 m
    ranges[90 + 60] = 0.5             # 左 60°, 0.5 m → 横に 0.43 m ずれているのでレーン外
    ranges[90 - 10] = 0.9             # 右 10°, 0.9 m → 前方 0.886 m, 横 0.156 m（レーン内）
    d = nearest_in_corridor(ranges, angle_min, inc, half_width=0.35, range_min=0.08,
                            range_max=10.0)
    assert d == pytest.approx(0.9 * math.cos(math.radians(10)))
    assert nearest_in_corridor([math.inf] * n, angle_min, inc, 0.35) == math.inf


def test_suppress_nested_boxes_keeps_confident_one():
    sign = {'label': 'stop sign', 'confidence': 0.83, 'bbox': (432, 167, 478, 207)}
    sign_and_pole = {'label': 'stop sign', 'confidence': 0.41, 'bbox': (388, 130, 483, 258)}
    person = {'label': 'person', 'confidence': 0.30, 'bbox': (440, 170, 470, 200)}
    kept = suppress_nested_boxes([sign_and_pole, sign, person])
    assert sign in kept and person in kept      # 別クラスは消さない
    assert sign_and_pole not in kept


def test_suppress_nested_boxes_keeps_separate_objects():
    a = {'label': 'person', 'confidence': 0.9, 'bbox': (100, 100, 150, 250)}
    b = {'label': 'person', 'confidence': 0.8, 'bbox': (140, 100, 190, 250)}  # 少し重なるだけ
    assert len(suppress_nested_boxes([a, b])) == 2


def test_corridor_polygon_is_symmetric():
    pts = corridor_polygon(CAM, 0.6, 1.5, 10.0)
    (ul_n, v_n), (ul_f, v_f), (ur_f, _), (ur_n, _) = pts
    assert v_n > v_f                     # 近い方が画面の下
    assert ul_n < ul_f < ur_f < ur_n     # 遠いほど幅が狭くなる台形
    assert (ul_n + ur_n) / 2 == pytest.approx(CAM.cx, abs=1)
