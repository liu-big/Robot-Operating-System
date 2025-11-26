# CMakeLists.txt
#
# Copyright (c) 2024, UCAR.AI, Inc. All rights reserved.
#
# tf2_bullet 包的 CMakeLists.txt 文件
# 该文件定义了 tf2_bullet 包的构建规则、依赖项和测试。

cmake_minimum_required(VERSION 2.8.3)
project(tf2_bullet)

# 查找 PkgConfig 包，用于查找系统库
find_package(PkgConfig REQUIRED)

# 初始化 bullet_FOUND 变量
set(bullet_FOUND 0)
# 使用 pkg-config 检查 bullet 库
pkg_check_modules(bullet bullet)
# 如果 pkg-config 未找到 bullet，则尝试使用 find_package 查找
if(NOT bullet_FOUND)
    # Windows 构建的 bullet3 默认不带 pkg-config，只带 CMake 配置文件
    # 因此 pkg_check_modules 会失败
    find_package(bullet REQUIRED)
endif()

# 查找 catkin 包，并指定所需的组件：geometry_msgs 和 tf2
find_package(catkin REQUIRED COMPONENTS geometry_msgs tf2)

# 添加包含目录，包括当前项目的 include 目录、bullet 的包含目录和 catkin 的包含目录
include_directories(include ${bullet_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

# 配置 catkin 包，指定包含目录
catkin_package(INCLUDE_DIRS include ${bullet_INCLUDE_DIRS})

# 安装头文件目录
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})


# 如果启用了 CATKIN 测试
if(CATKIN_ENABLE_TESTING)

# 添加 GTest 测试，指定测试名称和源文件
catkin_add_gtest(test_bullet test/test_tf2_bullet.cpp)
# 链接测试可执行文件所需的库
target_link_libraries(test_bullet ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

endif()
