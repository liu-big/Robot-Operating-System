# 设置 CMake 的最低版本要求
cmake_minimum_required(VERSION 2.8.3)
# 定义项目名称
project(tf2_kdl)

# 查找 Orocos KDL 库
find_package(orocos_kdl)
# 查找 Catkin 包，并指定所需的组件：cmake_modules, tf2, tf2_ros, tf2_msgs
find_package(catkin REQUIRED COMPONENTS cmake_modules tf2 tf2_ros tf2_msgs)

# 查找 Eigen 库的过程有些复杂，因为我们需要支持从 saucy 版本开始的 Ubuntu。
# 首先，我们查找在新版 Ubuntu 上由 libeigen3-dev 提供的 Eigen3 cmake 模块。
# 如果查找失败，则回退到由 cmake_modules 提供的版本，这是一个替代方案。
find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
endif()

# 注意：Eigen 3.2 (在 Ubuntu Wily 上) 只提供 EIGEN3_INCLUDE_DIR，而不是 EIGEN3_INCLUDE_DIRS，
# 所以我们必须从前者设置后者。
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

# 配置 Catkin 包的导出设置
catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS} # 导出的头文件目录，包含 Eigen3
  DEPENDS EIGEN3 orocos_kdl # 运行时依赖项
)

# 设置 Python 模块的 Catkin 构建
catkin_python_setup()
# 添加库文件搜索路径
link_directories(${orocos_kdl_LIBRARY_DIRS})

# 添加头文件搜索路径，包括 Catkin、Eigen3 和 GTest 的头文件
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${GTEST_INCLUDE_DIRS})

# 安装项目头文件目录
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# 安装测试脚本
install(PROGRAMS scripts/test.py
         DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 如果启用了测试，则执行以下操作
if(CATKIN_ENABLE_TESTING)

  # 查找 Catkin 包，用于测试组件
  find_package(catkin REQUIRED COMPONENTS rostest tf2 tf2_ros tf2_msgs)

  # 添加一个可执行测试，用于测试 tf2_kdl
  add_executable(test_kdl EXCLUDE_FROM_ALL test/test_tf2_kdl.cpp)
  # 查找 Threads 库
  find_package(Threads)
  # 为测试目标添加头文件搜索路径
  target_include_directories(test_kdl PUBLIC ${orocos_kdl_INCLUDE_DIRS})
  # 链接测试目标所需的库
  target_link_libraries(test_kdl ${catkin_LIBRARIES} ${GTEST_LIBRARIES} ${orocos_kdl_LIBRARIES} ${GTEST_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

  # 添加 ROS 测试启动文件
  add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test.launch)
  add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_python.launch)

  # 如果存在 'tests' 目标，则添加依赖
  if(TARGET tests)
    add_dependencies(tests test_kdl)
  endif()

endif()
