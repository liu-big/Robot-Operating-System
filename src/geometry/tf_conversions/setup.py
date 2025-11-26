#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# setup.py
#
# Copyright (c) 2023. All rights reserved.
#
# tf_conversions 包的 setup 文件，用于定义包的元数据和安装信息。
# 这是一个标准的 Python setup.py 文件，用于 ROS Python 包的构建系统。

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# generate_distutils_setup 函数用于生成 distutils setup() 函数所需的字典。
# 它会自动处理 catkin 包的特定设置，例如查找包、处理依赖等。
d = generate_distutils_setup(
    # packages 参数指定了要包含在分发包中的 Python 包。
    # 'tf_conversions' 是此 ROS 包的主要 Python 模块。
    packages=['tf_conversions'],
    # package_dir 参数指定了 Python 包的根目录。
    # 这里表示 Python 包的源代码位于当前目录下的 'src' 文件夹中。
    package_dir={'': 'src'},
    # requires 参数指定了此包运行时所需的其他 Python 模块或 ROS 包。
    # 'geometry_msgs': ROS 的标准消息类型，用于几何数据。
    # 'rospy': ROS Python 客户端库，用于编写 ROS 节点。
    # 'tf': ROS 的坐标变换库，用于处理坐标系之间的变换。
    requires=['geometry_msgs', 'rospy', 'tf']
)

# setup 函数是 distutils 的核心函数，用于执行包的安装。
# **d 会将 generate_distutils_setup 返回的字典解包并作为关键字参数传递给 setup 函数。
# 这使得 ROS 包能够正确地被 catkin 构建系统识别和安装。
setup(**d)
