# CMakeLists.txt
#
# Copyright (c) 2023. All rights reserved.
#
# test_tf2 包的 CMake 构建文件。
# 此文件定义了包的构建规则、依赖项和测试配置。

# 设置 CMake 的最低版本要求
cmake_minimum_required(VERSION 3.0.2)

# 定义项目名称
project(test_tf2)

# 查找所需的 ROS 包和第三方库
# REQUIRED 表示如果找不到这些包，则停止配置
find_package(catkin REQUIRED COMPONENTS rosconsole roscpp rostest tf tf2 tf2_bullet tf2_ros tf2_geometry_msgs tf2_kdl tf2_msgs)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(orocos_kdl REQUIRED)

# 配置 Catkin 包，导出包的构建信息
catkin_package()

# 如果未启用 Catkin 测试，则直接返回，不编译测试
if(NOT CATKIN_ENABLE_TESTING)
  return()
endif()

# 添加包含目录，以便编译器能够找到头文件
include_directories(${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS})

# 添加链接目录，以便链接器能够找到库文件
link_directories(${orocos_kdl_LIBRARY_DIRS})

# 添加 GTest 单元测试
# buffer_core_test: 测试名称
# test/buffer_core_test.cpp: 测试源文件
catkin_add_gtest(buffer_core_test test/buffer_core_test.cpp)
# 链接测试可执行文件所需的库
target_link_libraries(buffer_core_test ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

# 添加 test_tf2_message_filter GTest 单元测试
catkin_add_gtest(test_tf2_message_filter test/test_message_filter.cpp)
target_link_libraries(test_tf2_message_filter ${Boost_LIBRARIES} ${catkin_LIBRARIES})

# 添加 test_convert GTest 单元测试
catkin_add_gtest(test_convert test/test_convert.cpp)
target_link_libraries(test_convert ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

# 添加 test_utils GTest 单元测试
catkin_add_gtest(test_utils test/test_utils.cpp)
target_link_libraries(test_utils ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

# 添加可执行文件 test_buffer_server，并排除在所有目标之外
add_executable(test_buffer_server EXCLUDE_FROM_ALL test/test_buffer_server.cpp)
# 链接 test_buffer_server 可执行文件所需的库
target_link_libraries(test_buffer_server ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

# 添加可执行文件 test_buffer_client，并排除在所有目标之外
add_executable(test_buffer_client EXCLUDE_FROM_ALL test/test_buffer_client.cpp)
# 链接 test_buffer_client 可执行文件所需的库
target_link_libraries(test_buffer_client ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${GTEST_LIBRARIES} ${orocos_kdl_LIBRARIES})

# 添加 ROS 测试 (rostest)
# test/buffer_client_tester.launch: rostest 启动文件
add_rostest(test/buffer_client_tester.launch)

# 添加可执行文件 test_static_publisher，并排除在所有目标之外
add_executable(test_static_publisher EXCLUDE_FROM_ALL test/test_static_publisher.cpp)
# 链接 test_static_publisher 可执行文件所需的库
target_link_libraries(test_static_publisher ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

# 添加 ROS 测试 (rostest)
# test/static_publisher.launch: rostest 启动文件
add_rostest(test/static_publisher.launch)

# 添加可执行文件 test_tf2_bullet，并排除在所有目标之外
add_executable(test_tf2_bullet EXCLUDE_FROM_ALL test/test_tf2_bullet.cpp)
# 链接 test_tf2_bullet 可执行文件所需的库
target_link_libraries(test_tf2_bullet ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

# 添加 ROS 测试 (rostest)
# ${CMAKE_CURRENT_SOURCE_DIR}/test/test_tf2_bullet.launch: rostest 启动文件
add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_tf2_bullet.launch)

# 如果存在 'tests' 目标，则添加依赖项
if(TARGET tests)
  add_dependencies(tests test_buffer_server test_buffer_client test_static_publisher test_tf2_bullet)
endif()

# 用作测试夹具
if(TARGET tf2_ros_static_transform_publisher)
  add_dependencies(tests tf2_ros_static_transform_publisher test_static_publisher)
endif()
