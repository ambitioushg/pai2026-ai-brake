"""YOLO で人と止まれ標識を検出し、距離を推定して配信するノード。

購読: camera/image_raw   (sensor_msgs/Image)
      ai_brake/state     (std_msgs/String, JSON)  … 画面表示用
配信: ai_brake/detections (std_msgs/String, JSON)
      ai_brake/debug_image (sensor_msgs/Image)    … 検出枠・距離・ブレーキ状態を描いた画像
"""

import json
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .geometry import (
    CameraModel, corridor_polygon, distance_from_width, ground_position_from_bbox,
    suppress_nested_boxes)

try:
    import cv2
    from ultralytics import YOLO
except ImportError as e:  # venv の有効化忘れを分かりやすく知らせる
    raise SystemExit(f'{e}\nultralytics が見つかりません。'
                     '先に `source scripts/env.sh`（.venv の有効化）をしてください。')

# 描画色 (BGR)
GREEN, YELLOW, ORANGE, RED, MAGENTA, PURPLE, WHITE, BLACK = (
    (70, 190, 70), (0, 210, 255), (0, 150, 255), (50, 50, 235), (230, 0, 230),
    (200, 70, 200), (255, 255, 255), (0, 0, 0))
STATE_COLORS = {'CLEAR': GREEN, 'SLOW': YELLOW, 'STOP': RED, 'SIGN_WAIT': RED,
                'FAILSAFE': PURPLE}


def image_to_bgr(msg: Image) -> np.ndarray:
    if msg.encoding not in ('rgb8', 'bgr8'):
        raise ValueError(f'未対応の画像形式です: {msg.encoding}')
    arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    arr = arr[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
    if msg.encoding == 'rgb8':
        arr = arr[:, :, ::-1]
    return np.ascontiguousarray(arr)


def put_text(img, text, org, color, scale=0.55, thickness=1, bg=BLACK):
    """背景付きの文字（動画で読みやすくするため）。"""
    (w, h), base = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, thickness)
    x, y = org
    y = max(y, h + 4)
    cv2.rectangle(img, (x, y - h - 4), (x + w + 4, y + base), bg, -1)
    cv2.putText(img, text, (x + 2, y - 2), cv2.FONT_HERSHEY_SIMPLEX, scale, color, thickness,
                cv2.LINE_AA)


class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        self.declare_parameters('', [
            ('model', 'yolo11n.pt'),
            ('device', ''),                 # '' なら自動（GPU があれば GPU）
            ('confidence', 0.25),
            ('image_size', 640),
            ('target_classes', ['person', 'stop sign']),
            ('camera_hfov', 1.5),           # [rad] ロボットの SDF と合わせる
            ('camera_height', 0.7),         # [m]   ロボットの SDF と合わせる
            ('stop_sign_width', 0.6),       # [m]   標識の対辺距離
            ('corridor_half_width', 0.6),   # [m]   描画用（判断は safety_brake 側）
            ('publish_debug_image', True),
        ])
        get = lambda name: self.get_parameter(name).value  # noqa: E731
        self.conf = get('confidence')
        self.imgsz = get('image_size')
        self.device = get('device') or None
        self.hfov = get('camera_hfov')
        self.cam_height = get('camera_height')
        self.sign_width = get('stop_sign_width')
        self.corridor = get('corridor_half_width')
        self.publish_debug = get('publish_debug_image')

        model_path = get('model')
        if not Path(model_path).exists():
            self.get_logger().warn(f'{model_path} が無いので {Path(model_path).name} を自動ダウンロードします')
            model_path = Path(model_path).name
        self.model = YOLO(model_path)
        self.model_name = Path(model_path).stem
        self.class_ids = [i for i, n in self.model.names.items() if n in get('target_classes')]
        # 最初の推論は遅いので起動時に一度動かしておく
        self.model.predict(np.zeros((640, 640, 3), np.uint8), imgsz=self.imgsz,
                           device=self.device, verbose=False)
        self.get_logger().info(
            f'YOLO 準備完了: {self.model_name} on {self.model.predictor.device}, '
            f'classes={[self.model.names[i] for i in self.class_ids]}')

        self.state = None
        self.state_time = None
        self.infer_ms = []
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT)  # 常に最新フレームを処理
        self.create_subscription(Image, 'camera/image_raw', self.on_image, qos)
        self.create_subscription(String, 'ai_brake/state', self.on_state, 10)
        self.pub_det = self.create_publisher(String, 'ai_brake/detections', 10)
        self.pub_img = self.create_publisher(Image, 'ai_brake/debug_image', 1)
        self.create_timer(10.0, self.report_speed)

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_state(self, msg: String):
        self.state = json.loads(msg.data)
        self.state_time = self.now()

    def on_image(self, msg: Image):
        bgr = image_to_bgr(msg)
        cam = CameraModel(msg.width, msg.height, self.hfov, self.cam_height)

        t0 = time.perf_counter()
        result = self.model.predict(bgr, conf=self.conf, imgsz=self.imgsz, device=self.device,
                                    classes=self.class_ids, verbose=False)[0]
        infer_ms = (time.perf_counter() - t0) * 1000.0
        self.infer_ms.append(infer_ms)

        boxes = result.boxes
        detections = suppress_nested_boxes([
            {'label': self.model.names[int(cls)], 'confidence': round(conf, 3),
             'bbox': [round(v, 1) for v in bbox]}
            for bbox, conf, cls in zip(boxes.xyxy.tolist(), boxes.conf.tolist(),
                                       boxes.cls.tolist())])
        for d in detections:
            if d['label'] == 'stop sign':
                pos = distance_from_width(cam, d['bbox'], self.sign_width)
            else:
                pos = ground_position_from_bbox(cam, d['bbox'])
            d['distance'], d['lateral'] = pos if pos else (None, None)

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.pub_det.publish(String(data=json.dumps({
            'stamp': stamp, 'image_size': [msg.width, msg.height],
            'inference_ms': round(infer_ms, 2), 'detections': detections})))

        if self.publish_debug:
            out = self.draw(bgr, cam, detections, infer_ms)
            img = Image(header=msg.header, height=out.shape[0], width=out.shape[1],
                        encoding='bgr8', is_bigendian=0, step=out.shape[1] * 3,
                        data=out.tobytes())
            self.pub_img.publish(img)

    def draw(self, bgr, cam, detections, infer_ms):
        img = bgr.copy()
        # 走行レーン（この中の人だけがブレーキ対象）
        near = max(cam.min_ground_distance, 1.0)
        poly = np.array(corridor_polygon(cam, self.corridor, near, 15.0), np.int32)
        overlay = img.copy()
        cv2.fillPoly(overlay, [poly], GREEN)
        img = cv2.addWeighted(overlay, 0.22, img, 0.78, 0)
        cv2.polylines(img, [poly], True, GREEN, 1, cv2.LINE_AA)

        for d in detections:
            x1, y1, x2, y2 = (int(v) for v in d['bbox'])
            if d['label'] == 'stop sign':
                color = MAGENTA
            elif d['lateral'] is not None and abs(d['lateral']) <= self.corridor:
                color = RED
            else:
                color = ORANGE
            cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
            dist = f"{d['distance']:.1f}m" if d['distance'] is not None else '--'
            label_y = y1 - 2 if y1 > 80 else y1 + 18   # 上端のバナーに隠れないように
            put_text(img, f"{d['label']} {d['confidence']:.2f} {dist}", (x1, label_y), WHITE,
                     scale=0.5, bg=color)

        # ブレーキ状態のバナー
        state = self.state if (self.state_time is not None
                               and self.now() - self.state_time < 1.0) else None
        if state is None:
            banner, color = 'AI BRAKE: (waiting)', PURPLE
        else:
            s = state['state']
            color = STATE_COLORS.get(s, WHITE)
            dist = f" {state['distance']:.1f} m" if state.get('distance') is not None else ''
            if s == 'SLOW':
                banner = f"AI BRAKE: SLOW x{state['speed_ratio']:.2f}  (person{dist})"
            elif s == 'STOP' and state['reason'] == 'obstacle':
                banner = f'SAFETY STOP  (LiDAR obstacle{dist})'
            elif s == 'STOP':
                banner = f'AI BRAKE: STOP  (person{dist})'
            elif s == 'SIGN_WAIT':
                banner = f"AI BRAKE: STOP SIGN  wait {state['sign_wait_remaining']:.1f} s"
            elif s == 'FAILSAFE':
                banner = f"FAILSAFE  ({state['reason']})"
            else:
                banner = 'AI BRAKE: CLEAR'
        cv2.rectangle(img, (0, 0), (img.shape[1], 34), color, -1)
        cv2.putText(img, banner, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.75, BLACK, 2, cv2.LINE_AA)
        if state is not None:
            put_text(img, f"cmd {state['cmd_linear']:+.2f} m/s -> out {state['linear']:+.2f} m/s",
                     (8, 60), WHITE)
        put_text(img, f'{self.model_name}  {infer_ms:.1f} ms', (8, img.shape[0] - 8), WHITE,
                 scale=0.5)
        return img

    def report_speed(self):
        if self.infer_ms:
            self.get_logger().info(
                f'推論 {len(self.infer_ms) / 10.0:.1f} fps, 平均 {np.mean(self.infer_ms):.1f} ms')
            self.infer_ms.clear()


def main():
    rclpy.init()
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
