#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# tf2_kdl 包的 setup 文件
# 该文件用于定义 tf2_kdl Python 包的构建和安装方式。
#

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# 调用 generate_distutils_setup 函数获取 distutils 的设置字典
# 这个函数会根据 package.xml 中的信息自动生成大部分设置
d = generate_distutils_setup(
   ## 如果你希望脚本全局可见，可以取消注释下面这行
   # scripts=['script/test.py'],
   # 定义要包含在包中的 Python 模块
   packages=['tf2_kdl'],
   # 指定 Python 模块的根目录，这里表示模块在 'src' 目录下
   package_dir={'': 'src'}
)

# 调用 setup 函数，传入生成的设置字典，完成包的配置
setup(**d)
