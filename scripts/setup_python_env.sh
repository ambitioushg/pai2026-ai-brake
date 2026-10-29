#!/usr/bin/env bash
# YOLO (ultralytics) 用の Python 仮想環境 .venv を作る。sudo 不要。
# - ROS 2 Jazzy と同じシステムの Python (/usr/bin/python3) を使う
# - --system-site-packages で apt の Python パッケージ (rclpy の依存など) も見えるようにする
set -euo pipefail
cd "$(dirname "$0")/.."

if ! command -v uv >/dev/null 2>&1; then
  echo "uv が見つかりません: https://docs.astral.sh/uv/ からインストールしてください" >&2
  exit 1
fi

uv venv --python /usr/bin/python3 --system-site-packages .venv
touch .venv/COLCON_IGNORE   # colcon が .venv をビルド対象にしないようにする
uv pip install --python .venv/bin/python -r requirements.txt

# YOLO の学習済み重み (COCO) を事前にダウンロードしておく
# 既定は yolo11s（n は近距離の人を見失うことがあったため比較用）
mkdir -p weights
(cd weights && ../.venv/bin/python -c "from ultralytics import YOLO; YOLO('yolo11s.pt'); YOLO('yolo11n.pt')")
echo "完了: weights/yolo11s.pt, weights/yolo11n.pt"
