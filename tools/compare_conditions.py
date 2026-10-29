#!/usr/bin/env python3
"""複数の評価条件（scripts/run_trials.sh の結果フォルダ）を並べて比較する。

使い方:
  python3 tools/compare_conditions.py "カメラのみ (yolo11s)=results/camera_s" \
      "カメラ+LiDAR (yolo11s)=results/lidar_s" --out results/comparison
出力: <out>.md（表）と <out>.png（速度別の成功率グラフ）
"""

import argparse
import csv
from pathlib import Path
import warnings

import matplotlib
matplotlib.use('Agg')
warnings.filterwarnings('ignore', message='Unable to import Axes3D')
from matplotlib import font_manager  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402

JP_FONT = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'


def use_japanese_font():
    if Path(JP_FONT).exists():
        font_manager.fontManager.addfont(JP_FONT)
        plt.rcParams['font.family'] = font_manager.FontProperties(fname=JP_FONT).get_name()


def as_float(s):
    return float(s) if s not in ('', 'None') else None


def load(path: Path):
    return list(csv.DictReader(open(path / 'summary.csv')))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('conditions', nargs='+', help='"表示名=結果フォルダ"')
    parser.add_argument('--out', default='results/comparison')
    args = parser.parse_args()
    use_japanese_font()

    conds = []
    for c in args.conditions:
        name, path = c.split('=', 1)
        conds.append((name, load(Path(path))))
    speeds = sorted({float(r['speed']) for _, rows in conds for r in rows})

    header = '| 条件 | ' + ' | '.join(f'{v:.1f} m/s' for v in speeds) + \
             ' | 合計 | 横断者と接触 | 標識で一時停止 | 歩道の人に誤反応 |'
    lines = ['成功した試行の数（各速度で、歩行者と出会うタイミングを変えて実施）', '',
             header, '|' + '---|' * (len(speeds) + 5)]
    rates = {}
    for name, rows in conds:
        cells = []
        rates[name] = []
        for v in speeds:
            rs = [r for r in rows if float(r['speed']) == v]
            ok = sum(r['success'] == 'True' for r in rs)
            cells.append(f'{ok}/{len(rs)}')
            rates[name].append(ok / len(rs) if rs else 0.0)
        ok = sum(r['success'] == 'True' for r in rows)
        hits = sum((as_float(r['crossing_gap']) or 0.0) < 0 for r in rows)
        sign = sum(r['sign_ok'] == 'True' for r in rows)
        false_stop = sum(r['no_false_stop'] != 'True' for r in rows)
        lines.append(f'| {name} | ' + ' | '.join(cells) +
                     f' | **{ok}/{len(rows)}** | {hits} 回 | {sign}/{len(rows)} | {false_stop} 回 |')
    table = '\n'.join(lines)
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.with_suffix('.md').write_text(table + '\n')
    print(table)

    fig, ax = plt.subplots(figsize=(7, 3.8))
    width = 0.8 / len(conds)
    for i, (name, _) in enumerate(conds):
        xs = [k + (i - (len(conds) - 1) / 2) * width for k in range(len(speeds))]
        ax.bar(xs, [r * 100 for r in rates[name]], width, label=name)
    ax.set_xticks(range(len(speeds)), [f'{v:.1f} m/s' for v in speeds])
    ax.set_ylabel('成功率 [%]')
    ax.set_ylim(0, 105)
    ax.set_title('巡航速度ごとの成功率')
    ax.grid(axis='y', alpha=0.3)
    ax.legend(fontsize=8, loc='upper center', bbox_to_anchor=(0.5, -0.12), ncol=len(conds))
    fig.tight_layout()
    fig.savefig(out.with_suffix('.png'), dpi=150)
    print(f'\nグラフを保存: {out.with_suffix(".png")}')


if __name__ == '__main__':
    main()
