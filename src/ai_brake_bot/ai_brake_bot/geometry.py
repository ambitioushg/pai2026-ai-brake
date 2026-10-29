"""単眼カメラの幾何計算（ROS 非依存なので単体テストできる）。

YOLO の検出枠（ピクセル）から、物体までの距離と左右のずれ（メートル）を推定する。

座標の約束:
- 画像: u は右向き、v は下向き [px]
- distance: カメラから前方への距離 [m]
- lateral : カメラの光軸からの左右のずれ [m]（右が正）

カメラは水平（ピッチ角 0）に取り付けられている前提。
"""

from dataclasses import dataclass
import math
from typing import List, Optional, Tuple


@dataclass(frozen=True)
class CameraModel:
    width: int            # 画像の幅 [px]
    height: int           # 画像の高さ [px]
    hfov: float           # 水平画角 [rad]
    mount_height: float   # 地面からカメラまでの高さ [m]

    @property
    def fx(self) -> float:
        return (self.width / 2.0) / math.tan(self.hfov / 2.0)

    @property
    def fy(self) -> float:
        return self.fx  # 正方画素

    @property
    def cx(self) -> float:
        return self.width / 2.0

    @property
    def cy(self) -> float:
        return self.height / 2.0

    @property
    def min_ground_distance(self) -> float:
        """画像の下端に映る地面までの距離。これより近い足元は画面外になる。"""
        return self.mount_height * self.fy / (self.height - self.cy)


def ground_point_from_pixel(cam: CameraModel, u: float, v: float
                            ) -> Optional[Tuple[float, float]]:
    """地面上の点が画像の (u, v) に映っているとき、その (distance, lateral) を返す。

    v が地平線（cy）より上なら地面と交わらないので None。
    """
    dv = v - cam.cy
    if dv <= 1e-6:
        return None
    distance = cam.mount_height * cam.fy / dv
    lateral = (u - cam.cx) * distance / cam.fx
    return distance, lateral


def ground_position_from_bbox(cam: CameraModel, bbox: Tuple[float, float, float, float],
                              bottom_margin_px: float = 2.0
                              ) -> Optional[Tuple[float, float]]:
    """人など地面に立つ物体の距離を、検出枠の下端（足元）から推定する。

    物体が近すぎて足元が画面の下端で切れている場合は、
    「見えている地面の最短距離以下」なので min_ground_distance を返す（安全側）。
    """
    x1, _, x2, y2 = bbox
    u = (x1 + x2) / 2.0
    if y2 >= cam.height - bottom_margin_px:
        d = cam.min_ground_distance
        return d, (u - cam.cx) * d / cam.fx
    return ground_point_from_pixel(cam, u, y2)


def distance_from_width(cam: CameraModel, bbox: Tuple[float, float, float, float],
                        real_width: float) -> Optional[Tuple[float, float]]:
    """大きさが既知の物体（標識など）の距離を、検出枠の幅から推定する。"""
    x1, _, x2, _ = bbox
    w = x2 - x1
    if w <= 1.0:
        return None
    distance = cam.fx * real_width / w
    lateral = ((x1 + x2) / 2.0 - cam.cx) * distance / cam.fx
    return distance, lateral


def nearest_in_corridor(ranges, angle_min: float, angle_increment: float, half_width: float,
                        range_min: float = 0.0, range_max: float = math.inf) -> float:
    """2D LiDAR の点のうち、前方の走行レーン（左右 ±half_width）内で最も近い前方距離 [m]。

    レーン内に何もなければ inf。LiDAR 座標系は x が前、y が左。
    """
    nearest = math.inf
    for i, r in enumerate(ranges):
        if not (range_min <= r <= range_max) or math.isnan(r):
            continue
        a = angle_min + i * angle_increment
        x, y = r * math.cos(a), r * math.sin(a)
        if x > 0.0 and abs(y) <= half_width:
            nearest = min(nearest, x)
    return nearest


def suppress_nested_boxes(detections: List[dict], overlap: float = 0.8) -> List[dict]:
    """同じクラスの枠がほぼ内側に重なっているとき、信頼度の高い方だけを残す。

    YOLO 標準の重複除去（IoU 基準）は「大きな枠の中に小さな枠」がある場合を消せない。
    例: 標識だけを囲む正しい枠と、標識＋ポールを囲む誤った大きな枠が両方出る。
    detections の要素は 'label', 'confidence', 'bbox'(x1, y1, x2, y2) を持つ辞書。
    """
    def area(b):
        return max(0.0, b[2] - b[0]) * max(0.0, b[3] - b[1])

    kept: List[dict] = []
    for d in sorted(detections, key=lambda d: d['confidence'], reverse=True):
        b = d['bbox']
        duplicated = False
        for k in kept:
            if k['label'] != d['label']:
                continue
            kb = k['bbox']
            inter = area((max(b[0], kb[0]), max(b[1], kb[1]), min(b[2], kb[2]), min(b[3], kb[3])))
            if inter > overlap * min(area(b), area(kb)):
                duplicated = True
                break
        if not duplicated:
            kept.append(d)
    return kept


def project_ground_point(cam: CameraModel, distance: float, lateral: float
                         ) -> Tuple[float, float]:
    """地面上の点 (distance, lateral) を画像座標 (u, v) に投影する。"""
    u = cam.cx + cam.fx * lateral / distance
    v = cam.cy + cam.fy * cam.mount_height / distance
    return u, v


def corridor_polygon(cam: CameraModel, half_width: float, near: float, far: float
                     ) -> List[Tuple[int, int]]:
    """走行レーン（左右 ±half_width、前方 near〜far）を画像上の四角形で返す（描画用）。"""
    pts = [
        project_ground_point(cam, near, -half_width),
        project_ground_point(cam, far, -half_width),
        project_ground_point(cam, far, half_width),
        project_ground_point(cam, near, half_width),
    ]
    return [(int(round(u)), int(round(v))) for u, v in pts]
