# CMakeLists.txt
#
# Copyright (c) 2023. All rights reserved.
#
# geometry2 元包的 CMake 构建文件。
# 此文件定义了如何构建此 ROS 元包。

cmake_minimum_required(VERSION 2.8.3)
project(geometry2)
# 查找 Catkin 构建系统，REQUIRED 表示如果找不到则报错。
find_package(catkin REQUIRED)
# 声明这是一个 Catkin 元包，它不包含任何代码，只是一组相关包的集合。
catkin_metapackage()
