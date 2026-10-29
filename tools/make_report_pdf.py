#!/usr/bin/env python3
"""docs/report.md（レポート）を PDF にする。Google Chrome のヘッドレス印刷を使う。

使い方: python3 tools/make_report_pdf.py --account <omnicampus のアカウント名 or ID>
出力:   説明資料_PAI最終課題_<アカウント名>.pdf（課題で指定されたファイル名）
"""

import argparse
from pathlib import Path
import shutil
import subprocess

import markdown

ROOT = Path(__file__).resolve().parents[1]
CSS = """
@page { size: A4; margin: 15mm 14mm; }
body { font-family: 'Noto Sans CJK JP', 'Noto Sans JP', sans-serif; font-size: 10pt;
       line-height: 1.65; color: #222; }
h1 { font-size: 19pt; margin: 0 0 4px; border-bottom: 3px solid #e53935; padding-bottom: 4px; }
h2 { font-size: 13.5pt; border-left: 6px solid #e53935; padding-left: 8px; margin: 16px 0 6px;
     break-after: avoid; }
h3 { font-size: 11pt; margin: 10px 0 4px; break-after: avoid; }
p, li { margin: 3px 0; }
table { border-collapse: collapse; width: 100%; font-size: 8.8pt; margin: 6px 0;
        break-inside: avoid; }
th, td { border: 1px solid #bbb; padding: 3px 6px; vertical-align: top; }
th { background: #f2f2f2; }
img { max-width: 100%; display: block; margin: 6px auto; break-inside: avoid; }
code { font-family: 'Noto Sans Mono CJK JP', monospace; font-size: 8.8pt; background: #f4f4f4;
       padding: 0 3px; border-radius: 3px; }
em { font-style: normal; color: #555; font-size: 8.8pt; display: block; text-align: center; }
"""


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--account', required=True, help='omnicampus のアカウント名（実名なら ID）')
    p.add_argument('--input', default=str(ROOT / 'docs' / 'report.md'))
    args = p.parse_args()

    src = Path(args.input)
    body = markdown.markdown(src.read_text(), extensions=['tables', 'fenced_code'])
    html = (f"<!DOCTYPE html><html lang='ja'><head><meta charset='utf-8'>"
            f'<style>{CSS}</style></head><body>{body}</body></html>')
    html_path = src.with_suffix('.html')        # 画像の相対パスを保つため docs/ に置く
    html_path.write_text(html)

    chrome = shutil.which('google-chrome') or shutil.which('chromium')
    if chrome is None:
        raise SystemExit('Google Chrome が見つかりません')
    out = ROOT / f'説明資料_PAI最終課題_{args.account}.pdf'
    # ヘッダー・フッター（ファイルのパスや日時）は印刷しない
    subprocess.run([chrome, '--headless=new', '--disable-gpu', '--no-pdf-header-footer',
                    '--virtual-time-budget=5000', f'--print-to-pdf={out}',
                    html_path.resolve().as_uri()],
                   check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    html_path.unlink()
    print(f'書き出しました: {out}')


if __name__ == '__main__':
    main()
