from setuptools import setup
import os
from glob import glob

package_name = 'pkg_tf'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 安装launch文件
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.launch.py')),
        # 安装config文件
        (os.path.join('share', package_name, 'config'), 
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jose',
    maintainer_email='your_email@example.com',
    description='TF publisher package for board localization',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 格式: '命令名 = 包名.文件名:函数名'
            'board_tf_publisher = pkg_tf.tf_publisher:main',        # ← 改这里！
            'marker_simulator = pkg_tf.marker_simulator:main',      # ← 这个对的
        ],
    },
)
