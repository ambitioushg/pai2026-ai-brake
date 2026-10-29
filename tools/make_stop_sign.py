#!/usr/bin/env python3
"""止まれ標識（八角形・STOP）の 3D メッシュ (OBJ) とテクスチャ (PNG) を生成する。

Gazebo Fuel の既存の標識モデルは旧 Gazebo 用のマテリアル指定のため、
新しい Gazebo (Harmonic) ではテクスチャが表示されない。そこで自作する。

出力: src/ai_brake_bot/models/stop_sign/meshes/ 以下
  stop_sign.obj / stop_sign.mtl / stop_sign_texture.png

使い方: .venv/bin/python tools/make_stop_sign.py
"""

import argparse
import glob
import math
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
OUT_DIR = ROOT / 'src/ai_brake_bot/models/stop_sign/meshes'

FONT_CANDIDATES = [
    '/usr/share/fonts/truetype/liberation*/LiberationSansNarrow-Bold.ttf',
    '/usr/share/fonts/truetype/dejavu/DejaVuSansCondensed-Bold.ttf',
    '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf',
]
RED = (196, 18, 48)
WHITE = (250, 250, 250)


def octagon(cx, cy, r):
    """平らな辺が上下に来る正八角形の頂点（反時計回り、y 上向きの数学座標）。"""
    return [(cx + r * math.cos(math.radians(22.5 + 45 * k)),
             cy + r * math.sin(math.radians(22.5 + 45 * k))) for k in range(8)]


def find_font():
    for pattern in FONT_CANDIDATES:
        hits = glob.glob(pattern)
        if hits:
            return hits[0]
    raise FileNotFoundError('太字フォントが見つかりません')


def make_texture(path: Path, size: int = 1024):
    """八角形の外接円がテクスチャの正方形に内接するように描く（UV と対応させるため）。"""
    img = Image.new('RGB', (size, size), (120, 120, 120))
    draw = ImageDraw.Draw(img)
    c = size / 2
    # 画像座標は y 下向きなので、y を反転して描く
    def to_px(pts):
        return [(x, size - y) for x, y in pts]
    draw.polygon(to_px(octagon(c, c, c)), fill=WHITE)            # 白い縁
    draw.polygon(to_px(octagon(c, c, c * 0.94)), fill=RED)        # 赤地
    draw.polygon(to_px(octagon(c, c, c * 0.86)), outline=WHITE, width=int(size * 0.012))

    font = ImageFont.truetype(find_font(), int(size * 0.34))
    text = 'STOP'
    left, top, right, bottom = draw.textbbox((0, 0), text, font=font)
    tw, th = right - left, bottom - top
    target_w = size * 0.70                                        # 文字幅を標識幅の約 7 割に
    if tw > target_w:
        font = ImageFont.truetype(find_font(), int(size * 0.34 * target_w / tw))
        left, top, right, bottom = draw.textbbox((0, 0), text, font=font)
        tw, th = right - left, bottom - top
    draw.text((c - tw / 2 - left, c - th / 2 - top), text, font=font, fill=WHITE)
    img.save(path)


def make_obj(obj_path: Path, mtl_path: Path, texture_name: str, width: float, thickness: float):
    """表面（+X 向き）にテクスチャ、裏面と側面は灰色の八角形の板。"""
    r = width / (2 * math.cos(math.radians(22.5)))   # 対辺距離 width → 外接円半径
    ring = octagon(0.0, 0.0, r)                       # (y, z) 平面上
    xf, xb = thickness / 2, -thickness / 2

    lines = [f'# stop sign: width across flats {width} m, thickness {thickness} m',
             f'mtllib {mtl_path.name}']
    for y, z in ring:
        lines.append(f'v {xf:.5f} {y:.5f} {z:.5f}')     # 1..8: 表
    for y, z in ring:
        lines.append(f'v {xb:.5f} {y:.5f} {z:.5f}')     # 9..16: 裏
    for y, z in ring:                                   # UV（外接円 = テクスチャの正方形）
        lines.append(f'vt {0.5 + y / (2 * r):.5f} {0.5 + z / (2 * r):.5f}')
    lines += ['vn 1 0 0', 'vn -1 0 0']
    for k in range(8):
        a = math.radians(22.5 + 45 * k + 22.5)
        lines.append(f'vn 0 {math.cos(a):.5f} {math.sin(a):.5f}')  # 3..10: 側面

    lines += ['o sign_face', 'usemtl stop_face']
    lines.append('f ' + ' '.join(f'{i}/{i}/1' for i in range(1, 9)))     # 反時計回り（+X から見て）
    lines += ['o sign_back', 'usemtl sign_metal']
    lines.append('f ' + ' '.join(f'{i}//2' for i in range(16, 8, -1)))
    for k in range(8):
        f0, f1 = k + 1, (k + 1) % 8 + 1
        b0, b1 = f0 + 8, f1 + 8
        n = k + 3
        lines.append(f'f {f0}//{n} {b0}//{n} {b1}//{n} {f1}//{n}')
    obj_path.write_text('\n'.join(lines) + '\n')

    mtl_path.write_text('\n'.join([
        'newmtl stop_face',
        'Ka 1.0 1.0 1.0', 'Kd 1.0 1.0 1.0', 'Ks 0.1 0.1 0.1', 'Ns 10',
        f'map_Kd {texture_name}',
        '',
        'newmtl sign_metal',
        'Ka 0.55 0.55 0.55', 'Kd 0.6 0.6 0.6', 'Ks 0.2 0.2 0.2', 'Ns 20',
    ]) + '\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--width', type=float, default=0.60, help='対辺距離 [m]')
    parser.add_argument('--thickness', type=float, default=0.01, help='板厚 [m]')
    args = parser.parse_args()

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    texture = OUT_DIR / 'stop_sign_texture.png'
    make_texture(texture)
    make_obj(OUT_DIR / 'stop_sign.obj', OUT_DIR / 'stop_sign.mtl', texture.name,
             args.width, args.thickness)
    print(f'生成しました: {OUT_DIR}')


if __name__ == '__main__':
    main()
