#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# setup.py
#
# Copyright (c) 2024, UCAR.AI, Inc. All rights reserved.
#
# 此文件是 tf2_geometry_msgs 包的 setup 脚本，用于定义 Python 包的元数据和安装信息。
# 它使用 catkin_pkg.python_setup 中的 generate_distutils_setup 函数来生成适用于 Catkin 的 distutils 配置。

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# 生成 distutils setup 配置
d = generate_distutils_setup(
    # 定义要包含在包中的 Python 模块
    packages=['tf2_geometry_msgs'],
    # 指定 Python 模块的根目录，这里是 'src' 目录
    package_dir={'': 'src'},
    # 定义包的运行时依赖项
    requires={'rospy','geometry_msgs','tf2_ros','orocos_kdl'}
)

# 调用 setup 函数来执行包的安装配置
setup(**d)

