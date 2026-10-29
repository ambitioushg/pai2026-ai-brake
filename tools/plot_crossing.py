#!/usr/bin/env python3
"""横断歩道の場面だけを取り出して、複数の試行を並べて描く（レポートの考察用）。

縦軸: ロボット前端から横断者（の手前側）までの距離。0 を下回ると接触。
赤い帯: 横断者がロボットの進路上にいる時間。

使い方:
  python3 tools/plot_crossing.py "カメラのみ=videos/compare_camera_only.csv" \
      "カメラ + LiDAR=videos/compare_lidar.csv" -o docs/images/crossing.png
"""

import argparse
import csv
from pathlib import Path
import warnings

import matplotlib
matplotlib.use('Agg')
warnings.filterwarnings('ignore', message='Unable to import Axes3D')
from matplotlib import font_manager  # noqa: E402
from matplotlib.patches import Patch  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402

JP_FONT = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
CROSSING_X, HALF_DEPTH, FRONT, PATH_HALF_WIDTH = 16.0, 0.2, 0.25, 0.5  # trial_monitor と同じ値


def main():
    p = argparse.ArgumentParser()
    p.add_argument('trials', nargs='+', help='"表示名=時系列CSV"')
    p.add_argument('-o', '--output', default='docs/images/crossing.png')
    args = p.parse_args()
    if Path(JP_FONT).exists():
        font_manager.fontManager.addfont(JP_FONT)
        plt.rcParams['font.family'] = font_manager.FontProperties(fname=JP_FONT).get_name()

    fig, axes = plt.subplots(len(args.trials), 1, figsize=(8, 2.6 * len(args.trials)),
                             sharex=True, squeeze=False)
    for ax, spec in zip(axes[:, 0], args.trials):
        name, path = spec.split('=', 1)
        rows = [r for r in csv.DictReader(open(path)) if 11.0 <= float(r['x']) <= 17.5]
        t = [float(r['t']) for r in rows]
        gap = [CROSSING_X - HALF_DEPTH - (float(r['x']) + FRONT) for r in rows]
        in_path = [abs(float(r['crossing_person_y']) - float(r['y'])) < PATH_HALF_WIDTH
                   for r in rows]
        lidar = [r['reason'] == 'obstacle' for r in rows]
        for i in range(len(rows) - 1):
            if in_path[i]:
                ax.axvspan(t[i], t[i + 1], color='#e53935', alpha=0.18, lw=0)
            if lidar[i]:
                ax.axvspan(t[i], t[i + 1], ymin=0, ymax=0.06, color='#1e88e5', lw=0)
        ax.plot(t, gap, color='black')
        ax.axhline(0, color='#e53935', lw=1, ls='--')
        ax.set_ylabel('横断者までの\n距離 [m]')
        ax.set_title(name, loc='left', fontsize=11)
        ax.grid(alpha=0.3)
    axes[0, 0].legend(handles=[Patch(color='#e53935', alpha=0.18, label='横断者がロボットの進路上にいる'),
                               Patch(color='#1e88e5', label='LiDAR がブレーキ中')],
                      fontsize=8, loc='upper right')
    axes[-1, 0].set_xlabel('時間 [s]')
    fig.tight_layout()
    Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=150)
    print(f'保存しました: {args.output}')


if __name__ == '__main__':
    main()
