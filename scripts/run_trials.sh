#!/usr/bin/env bash
# 自動運転の試行を「速度 × 走り出しの待ち時間」の組み合わせで繰り返し、結果を集計する。
# 待ち時間を変えると、横断歩道の歩行者（周期 50 秒で往復）と出会うタイミングが変わる。
# Gazebo は画面なしで動かす。
#
# 使い方: bash scripts/run_trials.sh [速度 ...]
#   例:   bash scripts/run_trials.sh 0.3 0.5 0.8 1.0
# 環境変数で条件を変えられる:
#   MODEL=weights/yolo11n.pt  使う YOLO の重み（既定: weights/yolo11s.pt）
#   USE_LIDAR=false           LiDAR を使わずカメラ（AI）だけで判断する
#   DELAYS="0 12 25 37"       走り出しの待ち時間 [s]
#   OUT=results/xxx           結果の保存先
set -eo pipefail
cd "$(dirname "$0")/.."
source scripts/env.sh
# 他に起動中のシミュレーションと混線しないよう、通信チャンネルを分ける
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-77}
export GZ_PARTITION=${GZ_PARTITION:-ai_brake_trials}

SPEEDS=${*:-0.3 0.5 0.8 1.0}
DELAYS=${DELAYS:-0 12 25 37}
MODEL=${MODEL:-weights/yolo11s.pt}
USE_LIDAR=${USE_LIDAR:-true}
OUT=${OUT:-results/$(date +%Y%m%d_%H%M%S)}
mkdir -p "$OUT"
echo "model=$MODEL use_lidar=$USE_LIDAR speeds=($SPEEDS) delays=($DELAYS) -> $OUT"

for v in $SPEEDS; do
  for d in $DELAYS; do
    name="v${v}_d${d}"
    echo "=== $name"
    timeout 500 ros2 launch ai_brake_bot sim.launch.py mode:=auto headless:=true speed:="$v" \
      start_delay:="$d" use_lidar:="$USE_LIDAR" model:="$PWD/$MODEL" \
      trial_name:="$name" results_dir:="$OUT" > "$OUT/$name.log" 2>&1 \
      || echo "  launch exit $?"
    tail -n 1 "$OUT/summary.csv"
  done
done
python3 tools/summarize_trials.py "$OUT"
