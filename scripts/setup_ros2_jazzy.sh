#!/usr/bin/env bash
# ROS 2 Jazzy + Gazebo Harmonic (ros_gz) を Ubuntu 24.04 に apt でインストールする。
# 手順は公式ドキュメントに準拠:
#   https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
# sudo を使うため、パスワードを聞かれたら入力してください。
set -euo pipefail

if [ "$(. /etc/os-release && echo "${VERSION_CODENAME}")" != "noble" ]; then
  echo "このスクリプトは Ubuntu 24.04 (noble) 用です" >&2
  exit 1
fi

echo "[1/3] universe リポジトリを有効化"
sudo apt install -y software-properties-common
sudo add-apt-repository -y universe

echo "[2/3] ROS 2 の apt リポジトリを追加 (ros2-apt-source)"
sudo apt update && sudo apt install -y curl
ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo "${UBUNTU_CODENAME:-${VERSION_CODENAME}}")_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

echo "[3/3] ROS 2 Jazzy (desktop) + Gazebo Harmonic 連携 + 開発ツールをインストール"
sudo apt update
sudo apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-ros-gz \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-rqt-image-view \
  ros-dev-tools

echo
echo "完了しました。動作確認: source /opt/ros/jazzy/setup.bash && gz sim --version"
