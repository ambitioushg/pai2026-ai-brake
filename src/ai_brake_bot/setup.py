from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'ai_brake_bot'


def tree(src_dir):
    """src_dir 以下の全ファイルを share/<pkg>/ 以下に同じ構成でインストールする。"""
    return [(os.path.join('share', package_name, root), [os.path.join(root, f) for f in files])
            for root, _, files in os.walk(src_dir) if files]


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml') + glob('config/*.xml')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
    ] + tree('models'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pai2026',
    maintainer_email='pai2026@example.com',
    description='AI emergency brake demo with YOLO on ROS 2 Jazzy + Gazebo Harmonic',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'yolo_detector = ai_brake_bot.yolo_detector_node:main',
            'safety_brake = ai_brake_bot.safety_brake_node:main',
            'auto_driver = ai_brake_bot.auto_driver_node:main',
            'trial_monitor = ai_brake_bot.trial_monitor_node:main',
            'video_recorder = ai_brake_bot.video_recorder_node:main',
        ],
    },
)
