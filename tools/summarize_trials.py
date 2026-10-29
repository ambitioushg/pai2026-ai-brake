#!/usr/bin/env python3
"""scripts/run_trials.sh の結果（summary.csv と各試行の時系列 CSV）を集計し、表とグラフを作る。

使い方: python3 tools/summarize_trials.py results/<日時>
出力:   <dir>/summary.md, <dir>/gap_vs_speed.png, <dir>/timeline_<試行名>.png
"""

import csv
from pathlib import Path
import statistics
import sys
import warnings

import matplotlib
matplotlib.use('Agg')
warnings.filterwarnings('ignore', message='Unable to import Axes3D')
from matplotlib import font_manager  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402

JP_FONT = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
if Path(JP_FONT).exists():
    font_manager.fontManager.addfont(JP_FONT)
    plt.rcParams['font.family'] = font_manager.FontProperties(fname=JP_FONT).get_name()

STATE_COLORS = {'CLEAR': '#4caf50', 'SLOW': '#ffb300', 'STOP': '#e53935',
                'SIGN_WAIT': '#8e24aa', 'FAILSAFE': '#546e7a'}


def as_bool(s: str) -> bool:
    return s.strip().lower() == 'true'


def as_float(s: str):
    return float(s) if s not in ('', 'None') else None


def fmt(values, digits=2):
    values = [v for v in values if v is not None]
    if not values:
        return '-'
    if len(values) == 1:
        return f'{values[0]:.{digits}f}'
    return f'{statistics.mean(values):.{digits}f} ± {statistics.stdev(values):.{digits}f}'


def main():
    out = Path(sys.argv[1] if len(sys.argv) > 1 else 'results')
    rows = list(csv.DictReader(open(out / 'summary.csv')))
    speeds = sorted({float(r['speed']) for r in rows})

    lines = ['| 速度 [m/s] | 試行 | 成功 | 標識で一時停止 | 歩道の人に誤反応なし | '
             '横断者との最接近 [m] | レーン上の人との距離 [m] |',
             '|---|---|---|---|---|---|---|']
    for v in speeds:
        rs = [r for r in rows if float(r['speed']) == v]
        n = len(rs)
        lines.append(
            f"| {v:.1f} | {n} | {sum(as_bool(r['success']) for r in rs)}/{n} "
            f"| {sum(as_bool(r['sign_ok']) for r in rs)}/{n} "
            f"| {sum(as_bool(r['no_false_stop']) for r in rs)}/{n} "
            f"| {fmt([as_float(r['crossing_gap']) for r in rs])} "
            f"| {fmt([as_float(r['final_gap']) for r in rs])} |")
    total = sum(as_bool(r['success']) for r in rows)
    lines.append(f'\n全体の成功率: {total}/{len(rows)}')
    lines.append('\n※ 横断者との最接近: 歩行者が進路上にいる間の最小距離（負なら衝突）。'
                 '「-」はその試行で進路上に重ならなかったことを表す。')
    table = '\n'.join(lines)
    (out / 'summary.md').write_text(table + '\n')
    print(table)

    # 速度と停止距離の関係
    fig, ax = plt.subplots(figsize=(6, 4))
    for key, label, marker in [('final_gap', 'レーン上に立つ人', 'o'),
                               ('crossing_gap', '横断歩道の歩行者（最接近）', 's')]:
        xs = [float(r['speed']) for r in rows if as_float(r[key]) is not None]
        ys = [as_float(r[key]) for r in rows if as_float(r[key]) is not None]
        ax.scatter(xs, ys, label=label, marker=marker, alpha=0.8)
    ax.axhline(0.5, color='gray', ls='--', lw=1, label='停止距離の目標 (0.5 m)')
    ax.set_xlabel('巡航速度 [m/s]')
    ax.set_ylabel('ロボット前端と人との距離 [m]')
    ax.set_ylim(bottom=min(0.0, ax.get_ylim()[0]))
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out / 'gap_vs_speed.png', dpi=150)

    # 代表的な 1 試行（速度 0.5 の最初の試行）の時系列
    rep = next((r for r in rows if float(r['speed']) == 0.5), rows[0])
    ts = list(csv.DictReader(open(out / f"{rep['trial']}.csv")))
    t = [float(r['t']) for r in ts]
    fig, ax = plt.subplots(figsize=(9, 3.5))
    ax.plot(t, [float(r['cmd_linear']) for r in ts], color='gray', ls='--', label='操縦者の指令')
    ax.plot(t, [float(r['v']) for r in ts], color='black', label='実際の速度')
    for i in range(len(ts) - 1):
        ax.axvspan(t[i], t[i + 1], color=STATE_COLORS.get(ts[i]['state'], 'white'),
                   alpha=0.25, lw=0)
    for state, color in STATE_COLORS.items():
        ax.axvspan(0, 0, color=color, alpha=0.25, label=state)
    # 停止した区間に理由を書き込む
    names = {('SIGN_WAIT', 'stop sign'): '標識で\n一時停止', ('STOP', 'person'): '人の手前で\n停止',
             ('STOP', 'obstacle'): 'LiDAR で\n停止'}
    start = 0
    for i in range(1, len(ts) + 1):
        if i == len(ts) or (ts[i]['state'], ts[i]['reason']) != (ts[start]['state'],
                                                                  ts[start]['reason']):
            key = (ts[start]['state'], ts[start]['reason'])
            if key in names and t[i - 1] - t[start] > 1.0:
                ax.text((t[start] + t[i - 1]) / 2, 0.18, names[key], ha='center', fontsize=8)
            start = i
    ax.set_xlabel('時間 [s]')
    ax.set_ylabel('速度 [m/s]')
    ax.set_title(f"AI ブレーキの動き（{rep['trial']}）")
    ax.legend(fontsize=7, ncol=7, loc='upper center', bbox_to_anchor=(0.5, -0.2))
    fig.tight_layout()
    fig.savefig(out / f"timeline_{rep['trial']}.png", dpi=150)
    print(f'\nグラフを保存: {out}/gap_vs_speed.png, {out}/timeline_{rep["trial"]}.png')


if __name__ == '__main__':
    main()
