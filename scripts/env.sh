# 使い方: source scripts/env.sh
# ROS 2 Jazzy・このワークスペース・YOLO 用の .venv をまとめて有効にする
_ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
if [ -f "$_ws/install/setup.bash" ]; then
  source "$_ws/install/setup.bash"
else
  echo "まだビルドされていません: cd $_ws && colcon build"
fi
source "$_ws/.venv/bin/activate"
# カメラ画像（約 1.2 MB/枚）を取りこぼさないための DDS 設定（sim.launch.py と同じもの）
export FASTRTPS_DEFAULT_PROFILES_FILE="$_ws/src/ai_brake_bot/config/fastdds_large_shm.xml"
unset _ws
