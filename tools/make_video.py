#!/usr/bin/env python3
"""録画したデモ（video_recorder の MP4）から、提出用の動画を作る。

- 冒頭にタイトル画面、最後にまとめ画面（評価グラフを貼れる）を付ける
- クリップは「ファイル@開始-終了#見出し」の形で、区間の切り出しと見出し画面の挿入ができる
- --speed で倍速にでき、その場合は画面に「◯倍速」と表示する（課題の要件: 倍速は明記）
- YouTube 向けに H.264 (MP4) で書き出し、長さが 1〜3 分に収まっているか確認する

使い方:
  python3 tools/make_video.py \
      "videos/demo.mp4#デモ：自動運転 0.5 m/s" \
      "videos/camera_only.mp4@40-70#比較：カメラ（AI）だけの場合" \
      -o videos/submission.mp4 --ending-image results/comparison.png --speed 1.5
"""

import argparse
from pathlib import Path
import shutil
import subprocess
import tempfile

from PIL import Image, ImageDraw, ImageFont

W, H = 1280, 720
FONT = '/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc'
FONT_REGULAR = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
BG = (24, 28, 36)

DEFAULT_TITLE = 'AI 自動ブレーキ付きロボット'
DEFAULT_SUBTITLE = ['操縦者の指令に AI（YOLO）が「待った」をかける',
                    '人を見つけたら減速・停止、止まれ標識では一時停止']
DEFAULT_POINTS = ['YOLO11（COCO 学習済み）の推論のみで人と標識を認識（追加学習なし）',
                  '単眼カメラで距離を推定し、走行レーン内の人だけにブレーキ',
                  'カメラの死角（近すぎる人）は近距離 LiDAR で補う二重の安全設計',
                  '速度・タイミングを変えた自動評価で効果を確認']


def font(size, bold=True):
    return ImageFont.truetype(FONT if bold else FONT_REGULAR, size, index=0)


def title_card(path, title, subtitle, footer):
    img = Image.new('RGB', (W, H), BG)
    d = ImageDraw.Draw(img)
    d.rectangle([0, 0, 16, H], fill=(230, 60, 60))
    d.text((90, 210), title, font=font(64), fill=(255, 255, 255))
    for i, line in enumerate(subtitle):
        d.text((92, 330 + i * 52), line, font=font(32, bold=False), fill=(210, 215, 225))
    d.text((92, H - 90), footer, font=font(22, bold=False), fill=(150, 160, 175))
    img.save(path)


def ending_card(path, points, image):
    """横長の画像は箇条書きの下に、そうでなければ右側に置く。"""
    img = Image.new('RGB', (W, H), BG)
    d = ImageDraw.Draw(img)
    d.text((70, 50), 'まとめ', font=font(48), fill=(255, 255, 255))
    chart, below = None, False
    text_w = W - 170
    if image:
        chart = Image.open(image).convert('RGB')
        below = chart.width / chart.height > 1.8
        if below:
            chart.thumbnail((W - 140, 300))
        else:
            chart.thumbnail((560, 540))
            text_w = W - chart.width - 100 - 80
    y = 140
    for p in points:
        d.text((70, y), '•', font=font(26), fill=(230, 60, 60))
        for line in wrap(d, p, font(26, bold=False), text_w):
            d.text((100, y), line, font=font(26, bold=False), fill=(220, 225, 235))
            y += 40
        y += 18
    if chart is not None:
        if below:
            img.paste(chart, ((W - chart.width) // 2, max(y + 10, H - chart.height - 30)))
        else:
            img.paste(chart, (W - chart.width - 50, (H - chart.height) // 2 + 30))
    img.save(path)


def wrap(draw, text, fnt, width):
    lines, line = [], ''
    for ch in text:
        if draw.textlength(line + ch, font=fnt) > width:
            lines.append(line)
            line = ch
        else:
            line += ch
    return lines + [line]


def caption_card(path, text):
    """クリップの前に挟む見出し画面。「|」で区切ると 2 行目を小さく表示する。"""
    title, _, sub = text.partition('|')
    img = Image.new('RGB', (W, H), BG)
    d = ImageDraw.Draw(img)
    d.rectangle([0, 0, 16, H], fill=(230, 60, 60))
    w = d.textlength(title, font=font(48))
    d.text(((W - w) / 2, 300 if sub else 330), title, font=font(48), fill=(255, 255, 255))
    if sub:
        w = d.textlength(sub, font=font(28, bold=False))
        d.text(((W - w) / 2, 380), sub, font=font(28, bold=False), fill=(210, 215, 225))
    img.save(path)


def parse_clip(spec):
    """'path@start-end#caption' → (path, start, end, caption)"""
    spec, _, caption = spec.partition('#')
    path, _, span = spec.partition('@')
    start = end = None
    if span:
        a, _, b = span.partition('-')
        start, end = float(a), float(b)
    return path, start, end, caption or None


def badge(path, speed):
    text = f'{speed:g}倍速'
    fnt = font(30)
    img = Image.new('RGBA', (170, 56), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)
    d.rounded_rectangle([0, 0, 169, 55], radius=12, fill=(0, 0, 0, 170))
    w = d.textlength(text, font=fnt)
    d.text(((170 - w) / 2, 6), text, font=fnt, fill=(255, 220, 0))
    img.save(path)


def duration(path):
    out = subprocess.run(['ffprobe', '-v', 'error', '-show_entries', 'format=duration',
                          '-of', 'default=nw=1:nk=1', str(path)],
                         capture_output=True, text=True, check=True)
    return float(out.stdout.strip())


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    p.add_argument('clips', nargs='+',
                   help='録画した MP4。"ファイル@開始秒-終了秒#見出し" で切り出し・見出しを指定')
    p.add_argument('-o', '--output', default='videos/submission.mp4')
    p.add_argument('--speed', type=float, default=1.0, help='デモ部分の再生速度（2 倍まで）')
    p.add_argument('--title', default=DEFAULT_TITLE)
    p.add_argument('--subtitle', nargs='*', default=DEFAULT_SUBTITLE)
    p.add_argument('--footer', default='ROS 2 Jazzy + Gazebo Harmonic + Ultralytics YOLO11')
    p.add_argument('--points', nargs='*', default=DEFAULT_POINTS, help='まとめ画面の箇条書き')
    p.add_argument('--ending-image', help='まとめ画面に貼る画像（評価グラフなど）')
    p.add_argument('--title-sec', type=float, default=5.0)
    p.add_argument('--caption-sec', type=float, default=2.5)
    p.add_argument('--ending-sec', type=float, default=8.0)
    args = p.parse_args()
    if not 1.0 <= args.speed <= 2.0:
        p.error('--speed は 1〜2 にしてください（課題の要件は 2 倍速まで）')
    for tool in ('ffmpeg', 'ffprobe'):
        if shutil.which(tool) is None:
            p.error(f'{tool} が見つかりません')

    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        inputs, segments = [], []    # segments: ('card' | 'clip', 入力番号)

        def add_input(opts):
            inputs.extend(opts)
            return sum(1 for o in inputs if o == '-i') - 1

        def add_card(png, sec):
            segments.append(('card', add_input(['-loop', '1', '-t', str(sec), '-i', str(png)])))

        title_card(tmp / 'title.png', args.title, args.subtitle, args.footer)
        add_card(tmp / 'title.png', args.title_sec)
        for k, spec in enumerate(args.clips):
            path, start, end, caption = parse_clip(spec)
            if caption:
                caption_card(tmp / f'caption{k}.png', caption)
                add_card(tmp / f'caption{k}.png', args.caption_sec)
            opts = ['-ss', str(start), '-t', str(end - start)] if start is not None else []
            segments.append(('clip', add_input(opts + ['-i', path])))
        ending_card(tmp / 'ending.png', args.points, args.ending_image)
        add_card(tmp / 'ending.png', args.ending_sec)

        norm = (f'scale={W}:{H}:force_original_aspect_ratio=decrease,'
                f'pad={W}:{H}:(ow-iw)/2:(oh-ih)/2,setsar=1,fps=30,format=yuv420p')
        filters = []
        n_clips = sum(kind == 'clip' for kind, _ in segments)
        if args.speed != 1.0:
            badge(tmp / 'badge.png', args.speed)
            b = add_input(['-i', str(tmp / 'badge.png')])
            filters.append(f'[{b}:v]split={n_clips}' + ''.join(f'[b{j}]' for j in range(n_clips)))
        labels, j = [], 0
        for k, (kind, idx) in enumerate(segments):
            if kind == 'card':
                filters.append(f'[{idx}:v]{norm}[s{k}]')
            elif args.speed != 1.0:
                filters.append(f'[{idx}:v]setpts=(PTS-STARTPTS)/{args.speed},{norm}[v{k}]')
                filters.append(f'[v{k}][b{j}]overlay=W-w-20:H-h-100[s{k}]')
                j += 1
            else:
                filters.append(f'[{idx}:v]setpts=PTS-STARTPTS,{norm}[s{k}]')
            labels.append(f'[s{k}]')
        filters.append(''.join(labels) + f'concat=n={len(segments)}:v=1:a=0[out]')
        cmd = ['ffmpeg', '-y', '-loglevel', 'error'] + inputs
        out = Path(args.output)
        out.parent.mkdir(parents=True, exist_ok=True)
        cmd += ['-filter_complex', ';'.join(filters), '-map', '[out]',
                '-c:v', 'libx264', '-preset', 'medium', '-crf', '20', '-pix_fmt', 'yuv420p',
                '-movflags', '+faststart', str(out)]
        subprocess.run(cmd, check=True)

    sec = duration(out)
    print(f'書き出しました: {out}（{sec:.1f} 秒）')
    if not 60 <= sec <= 180:
        print('⚠ 課題の要件（1〜3 分）に収まっていません。--speed やクリップを調整してください。')
    if args.speed != 1.0:
        print(f'※ {args.speed:g} 倍速で再生しています。YouTube の概要欄にも明記してください。')


if __name__ == '__main__':
    main()
