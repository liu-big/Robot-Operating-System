#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# setup.py
#
# Copyright (c) 2023. All rights reserved.
#
# This file is used to configure and build the tf Python package.
# It uses distutils for package setup and catkin_pkg for Catkin-specific functionalities.
#

# 导入必要的模块
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# 调用 generate_distutils_setup 函数来生成 distutils setup 的字典。
# 这个函数会处理 Catkin 包的特定设置，例如查找依赖项和设置 Python 路径。
#
# 使用说明:
#   - packages: 指定要安装的 Python 包。这里是 'tf'。
#   - package_dir: 映射包名到实际的源代码目录。这里将根目录映射到 'src'，
#     意味着 'tf' 包的源代码在 'src/tf' 目录下。
#   - requires: 列出此包所需的其他 ROS 消息、服务或动作包。
#     这些依赖项会在 Catkin 编译系统中被检查。
#   - scripts: 指定要安装的可执行脚本。这些脚本会在安装后添加到系统的 PATH 中。
#     这里包含了 tf_remap 和 view_frames 脚本，用于兼容 Groovy 版本。
#
# 详细参数:
#   - genmsg: 用于处理 ROS 消息生成。
#   - genpy: 用于处理 ROS Python 消息的生成。
#   - roslib: ROS 库，提供各种实用功能。
#   - rospkg: ROS 包查找库。
#   - geometry_msgs: 包含 ROS 几何消息类型，如点、姿态等。
#   - sensor_msgs: 包含 ROS 传感器消息类型，如图像、激光扫描等。
#   - std_msgs: 包含 ROS 标准消息类型，如字符串、整数等。
#
d = generate_distutils_setup(
    packages=['tf'],
    package_dir={'': 'src'},
    requires=['genmsg', 'genpy', 'roslib', 'rospkg', 'geometry_msgs', 'sensor_msgs', 'std_msgs'],
    scripts=['scripts/groovy_compatibility/tf_remap',
             'scripts/groovy_compatibility/view_frames']
)

# 调用 setup 函数来执行包的安装。d 字典包含了所有必要的配置信息。
#
# 使用说明:
#   - setup(**d): 将字典 d 中的键值对作为关键字参数传递给 setup 函数。
#     这使得 distutils 能够正确地配置和安装 Python 包。
#
setup(**d)
