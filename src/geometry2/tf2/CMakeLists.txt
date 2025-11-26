# CMakeLists.txt
#
# Copyright (c) 2023. All rights reserved.
#
# tf2 包的 CMake 构建文件。
# 此文件定义了包的构建规则、依赖项、库的编译和安装。

# 设置 CMake 的最低版本要求
cmake_minimum_required(VERSION 3.0.2)

# 定义项目名称
project(tf2)

# 查找所需的第三方库和 ROS 包
# console_bridge: 用于日志和调试
# catkin: ROS 构建系统
# Boost: C++ 常用库，这里用于 system 和 thread 组件
find_package(console_bridge REQUIRED)
find_package(catkin REQUIRED COMPONENTS geometry_msgs rostime tf2_msgs)
find_package(Boost REQUIRED COMPONENTS system thread)

# 配置 Catkin 包，导出包的构建信息
# INCLUDE_DIRS: 导出头文件目录
# LIBRARIES: 导出库文件
# DEPENDS: 运行时依赖项
# CATKIN_DEPENDS: Catkin 依赖项
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES tf2
   DEPENDS console_bridge
   CATKIN_DEPENDS geometry_msgs tf2_msgs rostime)

# 添加包含目录，以便编译器能够找到头文件
include_directories(include ${catkin_INCLUDE_DIRS} ${console_bridge_INCLUDE_DIRS})

# 导出用户定义（此行通常用于占位或特定导出需求，此处无实际操作）
# export user definitions

# C++ 库
# 添加库 tf2，由 src/cache.cpp, src/buffer_core.cpp, src/static_cache.cpp 编译生成
add_library(tf2 src/cache.cpp src/buffer_core.cpp src/static_cache.cpp)
# 链接 tf2 库所需的依赖库
target_link_libraries(tf2 ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${console_bridge_LIBRARIES})
# 添加 tf2 库的依赖项，确保在编译 tf2 之前先编译这些依赖项
add_dependencies(tf2 ${catkin_EXPORTED_TARGETS})

# 安装规则
# 安装 tf2 库到指定目录
install(TARGETS tf2
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装头文件目录到指定目录
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# 测试
# 如果启用了 Catkin 测试
if(CATKIN_ENABLE_TESTING)

# 添加 GTest 单元测试 test_cache_unittest
catkin_add_gtest(test_cache_unittest test/cache_unittest.cpp)
# 链接 test_cache_unittest 库所需的依赖库
target_link_libraries(test_cache_unittest tf2  ${console_bridge_LIBRARIES})
# 添加 test_cache_unittest 的依赖项
add_dependencies(test_cache_unittest ${catkin_EXPORTED_TARGETS})

# 添加 GTest 单元测试 test_static_cache_unittest
catkin_add_gtest(test_static_cache_unittest test/static_cache_test.cpp)
# 链接 test_static_cache_unittest 库所需的依赖库
target_link_libraries(test_static_cache_unittest tf2  ${console_bridge_LIBRARIES})
# 添加 test_static_cache_unittest 的依赖项
add_dependencies(test_static_cache_unittest ${catkin_EXPORTED_TARGETS})

# 添加 GTest 单元测试 test_simple
catkin_add_gtest(test_simple test/simple_tf2_core.cpp)
# 链接 test_simple 库所需的依赖库
target_link_libraries(test_simple tf2  ${console_bridge_LIBRARIES})
# 添加 test_simple 的依赖项
add_dependencies(test_simple ${catkin_EXPORTED_TARGETS})

# 添加可执行文件 speed_test，并排除在所有目标之外
add_executable(speed_test EXCLUDE_FROM_ALL test/speed_test.cpp)
# 链接 speed_test 库所需的依赖库
target_link_libraries(speed_test tf2  ${console_bridge_LIBRARIES})
# 添加 tests 目标对 speed_test 的依赖
add_dependencies(tests speed_test)
# 添加 tests 目标对 Catkin 导出目标的依赖
add_dependencies(tests ${catkin_EXPORTED_TARGETS})

endif()
