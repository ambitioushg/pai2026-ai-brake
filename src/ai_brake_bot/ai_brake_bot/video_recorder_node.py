"""デモ動画を MP4 に書き出すノード（提出動画の素材づくり用）。

左: 追従カメラ（外から見た様子）  右: ロボットのカメラ + YOLO の判断（debug_image）
下: AI ブレーキの判断を日本語の字幕で表示

購読: chase_camera/image_raw, ai_brake/debug_image, ai_brake/state
"""

import json
from pathlib import Path

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

try:
    import cv2
    from PIL import Image as PILImage, ImageDraw, ImageFont
except ImportError as e:
    raise SystemExit(f'{e}\n先に `source scripts/env.sh`（.venv の有効化）をしてください。')

from .yolo_detector_node import image_to_bgr

PANEL = 640                      # 左右それぞれの一辺 [px]
BAR = 80                         # 字幕の帯の高さ [px]
FONT_PATHS = ['/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc',
              '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc']
# 字幕の色 (RGB)
COLORS = {'CLEAR': (90, 200, 90), 'SLOW': (255, 200, 0), 'STOP': (240, 70, 70),
          'SIGN_WAIT': (240, 70, 70), 'FAILSAFE': (200, 120, 220), None: (200, 200, 200)}


def caption(state):
    if state is None:
        return '準備中…'
    s, d = state['state'], state.get('distance')
    dist = f'（{d:.1f} m 先）' if d is not None else ''
    if s == 'STOP' and state.get('reason') == 'obstacle':
        return f'すぐ前に障害物（LiDAR）→ 停止{dist}'
    return {
        'CLEAR': '走行中：前方のレーンに危険なし',
        'SLOW': f'レーン内に人を検知 → 減速{dist}',
        'STOP': f'レーン内に人を検知 → 停止{dist}',
        'SIGN_WAIT': f"止まれ標識を検知 → 一時停止（あと {state['sign_wait_remaining']:.1f} 秒）",
        'FAILSAFE': 'センサの入力が途絶 → 安全のため停止',
    }.get(s, s)


class VideoRecorder(Node):
    def __init__(self):
        super().__init__('video_recorder')
        self.path = Path(self.declare_parameter('output', 'videos/demo.mp4').value)
        self.fps = self.declare_parameter('fps', 15.0).value
        self.path.parent.mkdir(parents=True, exist_ok=True)
        size = (PANEL * 2, PANEL + BAR)
        self.writer = cv2.VideoWriter(str(self.path), cv2.VideoWriter_fourcc(*'mp4v'),
                                      self.fps, size)
        font = next((p for p in FONT_PATHS if Path(p).exists()), None)
        if font is None:
            raise SystemExit('日本語フォント (Noto Sans CJK) が見つかりません')
        self.font_big = ImageFont.truetype(font, 30, index=0)    # index 0 = 日本語 (JP)
        self.font_small = ImageFont.truetype(font, 18, index=0)
        self.labels = self.render_labels()
        self.bar_cache = {}

        self.chase = None
        self.ai = None
        self.state = None
        self.frames = 0
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, 'chase_camera/image_raw', self.on_chase, qos)
        self.create_subscription(Image, 'ai_brake/debug_image', self.on_ai, qos)
        self.create_subscription(String, 'ai_brake/state', self.on_state, 10)
        self.create_timer(1.0 / self.fps, self.on_timer)
        self.get_logger().info(f'録画開始: {self.path}')

    def on_chase(self, msg):
        self.chase = cv2.resize(image_to_bgr(msg), (PANEL, PANEL))

    def on_ai(self, msg):
        self.ai = cv2.resize(image_to_bgr(msg), (PANEL, PANEL))

    def on_state(self, msg):
        self.state = json.loads(msg.data)

    def render_labels(self):
        """各画面の左上に付ける見出し（日本語）。"""
        labels = []
        for text in ['外から見た様子', 'ロボットのカメラ + AI（YOLO）の判断']:
            img = PILImage.new('RGB', (PANEL, 30), (0, 0, 0))
            ImageDraw.Draw(img).text((8, 3), text, font=self.font_small, fill=(255, 255, 255))
            labels.append(cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR))
        return labels

    def render_bar(self, state):
        text = caption(state)
        speed = ''
        if state is not None:
            speed = (f"操縦者の速度指令 {state['cmd_linear']:.2f} m/s"
                     f"  →  AI ブレーキ後 {state['linear']:.2f} m/s")
        key = (text, speed)
        if key not in self.bar_cache:
            img = PILImage.new('RGB', (PANEL * 2, BAR), (20, 20, 20))
            draw = ImageDraw.Draw(img)
            color = COLORS.get(state['state'] if state else None, (255, 255, 255))
            draw.rectangle([0, 0, 12, BAR], fill=color)
            draw.text((28, 6), text, font=self.font_big, fill=color)
            draw.text((30, 50), speed, font=self.font_small, fill=(220, 220, 220))
            if len(self.bar_cache) > 500:
                self.bar_cache.clear()
            self.bar_cache[key] = cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)
        return self.bar_cache[key]

    def on_timer(self):
        if self.chase is None or self.ai is None:
            return
        left, right = self.chase.copy(), self.ai.copy()
        left[:30, :] = self.labels[0]
        right[34:64, :] = self.labels[1]   # 右は上端に AI ブレーキのバナーがあるのでその下
        frame = np.vstack([np.hstack([left, right]), self.render_bar(self.state)])
        self.writer.write(frame)
        self.frames += 1

    def close(self):
        self.writer.release()
        self.get_logger().info(
            f'録画終了: {self.path}（{self.frames} フレーム, {self.frames / self.fps:.1f} 秒）')


def main():
    rclpy.init()
    node = VideoRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
