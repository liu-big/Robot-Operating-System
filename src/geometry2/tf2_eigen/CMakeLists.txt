# CMakeLists.txt
#
# Copyright (c) 2024, UCAR.AI, Inc. All rights reserved.
#
# tf2_eigen 包的 CMakeLists.txt 文件
# 该文件定义了 tf2_eigen 包的构建规则、依赖项和测试。

cmake_minimum_required(VERSION 2.8.3)
project(tf2_eigen)

# 查找 catkin 包，并指定所需的组件
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  geometry_msgs
  tf2
)

# 查找 Eigen 库的过程比较复杂，因为需要支持从 saucy 版本开始的 Ubuntu。
# 首先查找 libeigen3-dev 在较新 Ubuntu 上提供的 Eigen3 cmake 模块。
# 如果失败，则回退到 cmake_modules 提供的版本，这是一个替代方案。
find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
endif()

# 注意：Eigen 3.2 (在 Ubuntu Wily 上) 只提供 EIGEN3_INCLUDE_DIR，
# 而不是 EIGEN3_INCLUDE_DIRS，所以我们必须从前者设置后者。
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

# 添加包含目录，包括当前项目的 include 目录、Eigen3 的包含目录和 catkin 的包含目录
include_directories(include
                    ${EIGEN3_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS})

# 配置 catkin 包，指定包含目录和依赖项
catkin_package(
  INCLUDE_DIRS include
  DEPENDS EIGEN3)

# 安装头文件目录
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})


# 如果启用了 CATKIN 测试
if(CATKIN_ENABLE_TESTING)

  # 添加 GTest 测试，指定测试名称和源文件
  catkin_add_gtest(tf2_eigen-test test/tf2_eigen-test.cpp)
  # 链接测试可执行文件所需的库
  target_link_libraries(tf2_eigen-test ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

endif()
