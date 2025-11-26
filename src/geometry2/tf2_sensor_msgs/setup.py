#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 导入distutils的setup函数，用于Python包的安装
from distutils.core import setup
# 导入catkin_pkg的generate_distutils_setup函数，用于生成catkin兼容的setup配置
from catkin_pkg.python_setup import generate_distutils_setup

# 调用generate_distutils_setup函数，生成distutils所需的字典配置
d = generate_distutils_setup(
    # 指定要安装的Python包，这里是'tf2_sensor_msgs'
    packages=['tf2_sensor_msgs'],
    # 指定包的根目录，这里表示包内容在'src'目录下
    package_dir={'': 'src'},
    # 指定Python包的运行时依赖项
    requires={'rospy','sensor_msgs','tf2_ros','orocos_kdl'}
)

# 调用setup函数，传入生成的配置字典，完成包的设置和安装
setup(**d)

