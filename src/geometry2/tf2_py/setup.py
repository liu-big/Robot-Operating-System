#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 从 distutils.core 导入 setup 函数，用于构建和安装 Python 包
from distutils.core import setup
# 从 catkin_pkg.python_setup 导入 generate_distutils_setup 函数，用于生成 Catkin Python 包的 setup 配置
from catkin_pkg.python_setup import generate_distutils_setup

# 调用 generate_distutils_setup 函数，生成 distutils setup 的字典配置
d = generate_distutils_setup(
    # 指定要包含在包中的 Python 模块列表
    packages=['tf2_py'],
    # 指定 Python 模块的根目录，这里表示模块在 'src' 目录下
    package_dir={'': 'src'},
    # 指定包的运行时依赖项列表
    requires=['rospy', 'geometry_msgs', 'tf2_msgs']
)

# 调用 setup 函数，传入生成的配置字典，完成包的设置
setup(**d)
