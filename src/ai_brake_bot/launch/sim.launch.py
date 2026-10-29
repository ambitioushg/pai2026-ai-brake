"""Gazebo + ブリッジ + YOLO 検出 + AI ブレーキを一括起動する。

例:
  ros2 launch ai_brake_bot sim.launch.py                       # キーボード操作モード
  ros2 launch ai_brake_bot sim.launch.py mode:=auto speed:=0.5 # 自動運転モード
  ros2 launch ai_brake_bot sim.launch.py mode:=auto headless:=true trial_name:=v05_1  # 評価
  ros2 launch ai_brake_bot sim.launch.py mode:=auto record:=videos/demo.mp4 trial_name:=demo  # 録画
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, EmitEvent,
                            ExecuteProcess, OpaqueFunction, RegisterEventHandler,
                            SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'ai_brake_bot'


def launch_setup(context):
    share = get_package_share_directory(PKG)
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    world = arg('world')
    if not os.path.isabs(world):
        world = os.path.join(share, 'worlds', world)
    headless = arg('headless').lower() == 'true'
    params = os.path.join(share, 'config', 'params.yaml')
    sim_time = {'use_sim_time': True}

    # ros_gz_sim の gz_sim.launch.py は sh -c 経由で起動するため、終了時に Gazebo 本体へ
    # シグナルが届かずプロセスが残ることがある。ここでは gz を直接起動して確実に止める。
    gz_cmd = ['gz', 'sim', '-r', world]
    if headless:
        gz_cmd[2:2] = ['-s', '--headless-rendering']
    actions = [
        ExecuteProcess(cmd=gz_cmd, name='gazebo', output='screen', sigterm_timeout='20',
                       on_exit=[EmitEvent(event=Shutdown(reason='Gazebo を終了'))]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge',
             parameters=[{'config_file': os.path.join(share, 'config', 'bridge.yaml')}, sim_time],
             output='screen'),
        Node(package=PKG, executable='yolo_detector', output='screen',
             parameters=[params, {'model': arg('model')}, sim_time]),
        Node(package=PKG, executable='safety_brake', output='screen',
             parameters=[params, {'use_lidar': arg('use_lidar').lower() == 'true'}, sim_time]),
    ]
    if not headless and arg('image_view').lower() == 'true':
        actions.append(Node(package='rqt_image_view', executable='rqt_image_view',
                            arguments=['/ai_brake/debug_image'], parameters=[sim_time]))
    if arg('mode') == 'auto':
        actions.append(Node(package=PKG, executable='auto_driver', output='screen',
                            parameters=[params, sim_time, {
                                'speed': float(arg('speed')),
                                'start_delay': float(arg('start_delay'))}]))
    if arg('record'):
        actions.append(Node(package=PKG, executable='video_recorder', output='screen',
                            parameters=[sim_time, {'output': os.path.abspath(arg('record'))}]))
    if arg('trial_name'):
        monitor = Node(package=PKG, executable='trial_monitor', output='screen',
                       parameters=[params, sim_time, {
                           'trial_name': arg('trial_name'),
                           'results_dir': os.path.abspath(arg('results_dir')),
                           'speed': float(arg('speed'))}])
        actions += [monitor, RegisterEventHandler(OnProcessExit(
            target_action=monitor, on_exit=[EmitEvent(event=Shutdown(reason='trial finished'))]))]
    return actions


def generate_launch_description():
    share = get_package_share_directory(PKG)
    # <ワークスペース>/install/ai_brake_bot/share/ai_brake_bot → <ワークスペース>/weights/yolo11s.pt
    # （最小の yolo11n は近くの人を見失うことがあったため、一つ大きい s を使う）
    default_model = str(Path(share).parents[3] / 'weights' / 'yolo11s.pt')
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='town_road.sdf'),
        DeclareLaunchArgument('mode', default_value='teleop',
                              description='teleop: キーボード操作 / auto: 自動運転'),
        DeclareLaunchArgument('speed', default_value='0.5',
                              description='auto モードの巡航速度 [m/s]'),
        DeclareLaunchArgument('start_delay', default_value='0.0',
                              description='auto モードで走り出すまでの待ち時間 [s]'),
        DeclareLaunchArgument('use_lidar', default_value='true',
                              description='false にするとカメラ（AI）だけで判断する（比較用）'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='true なら Gazebo の画面を出さない（評価用）'),
        DeclareLaunchArgument('image_view', default_value='true',
                              description='AI の検出結果画像を rqt_image_view で表示'),
        DeclareLaunchArgument('model', default_value=default_model,
                              description='YOLO の重みファイル'),
        DeclareLaunchArgument('trial_name', default_value='',
                              description='指定すると評価用に 1 試行を記録して終了'),
        DeclareLaunchArgument('results_dir', default_value='results'),
        DeclareLaunchArgument('record', default_value='',
                              description='MP4 のパスを指定すると、デモ動画を録画する'),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(share, 'models')),
        # 1.2 MB/枚のカメラ画像を取りこぼさないよう、DDS の共有メモリ領域を広げる
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE',
                               os.path.join(share, 'config', 'fastdds_large_shm.xml')),
        OpaqueFunction(function=launch_setup),
    ])
