# CMakeLists.txt
#
# Copyright (c) 2023. All rights reserved.
#
# tf_conversions 包的 CMake 构建文件。
# 此文件定义了如何构建、查找依赖项、运行测试和安装此 ROS 包。

cmake_minimum_required(VERSION 2.8.3)
project(tf_conversions)

# 查找所需的 ROS 包和系统库。
# REQUIRED 关键字表示如果找不到这些包，CMake 将会报错。
find_package(orocos_kdl REQUIRED)
find_package(catkin REQUIRED geometry_msgs kdl_conversions tf)
find_package(Eigen3 REQUIRED)

# 启用 Catkin 的 Python 支持，用于处理 Python 模块。
catkin_python_setup()

# 配置 Catkin 包的导出信息，供其他包使用。
catkin_package(
  INCLUDE_DIRS include # 导出头文件目录
  LIBRARIES ${PROJECT_NAME} # 导出库文件
  DEPENDS EIGEN3 orocos_kdl # 运行时依赖项
  CATKIN_DEPENDS geometry_msgs kdl_conversions tf # Catkin 依赖项
)

# 添加包含目录，以便编译器可以找到头文件。
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS})

# 由于 orocos_kdl pkg-config 导出中没有完整的文件名，因此需要此项。
link_directories(${orocos_kdl_LIBRARY_DIRS})

# 添加库，指定源文件。
add_library(${PROJECT_NAME}
  src/tf_kdl.cpp
  src/tf_eigen.cpp
)
# 添加依赖，确保在构建此库之前先构建 Catkin 导出的目标。
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
# 链接库，将所需的库链接到目标。
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

# 测试
if(CATKIN_ENABLE_TESTING)

# 添加 gtest 单元测试。
catkin_add_gtest(test_eigen_tf test/test_eigen_tf.cpp)
# 链接测试可执行文件所需的库。
target_link_libraries(test_eigen_tf ${PROJECT_NAME} ${orocos_kdl_LIBRARIES})

catkin_add_gtest(test_kdl_tf test/test_kdl_tf.cpp)
# 如果编译器是 GCC/G++，则设置编译标志以抑制弃用警告。
if(CMAKE_COMPILER_IS_GNUCXX)
set_target_properties(test_kdl_tf PROPERTIES COMPILE_FLAGS "-Wno-deprecated-declarations")
endif()
# 链接测试可执行文件所需的库。
target_link_libraries(test_kdl_tf ${PROJECT_NAME} ${orocos_kdl_LIBRARIES})

# 添加 nosetests Python 测试。
catkin_add_nosetests(test/posemath.py)

endif()

# 安装规则
# 安装头文件目录。
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# 安装目标库。
install(TARGETS ${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
