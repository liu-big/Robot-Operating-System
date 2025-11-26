# 设置所需的CMake最低版本
cmake_minimum_required(VERSION 2.8.3)
# 定义项目名称
project(tf2_sensor_msgs)

# 查找catkin包及其所需的组件
# REQUIRED表示如果找不到这些组件，则停止配置
find_package(catkin REQUIRED COMPONENTS cmake_modules sensor_msgs tf2_ros tf2)
# 查找Boost库的thread组件，REQUIRED表示必须找到
find_package(Boost COMPONENTS thread REQUIRED)

# 查找Eigen库，这里处理了不同Ubuntu版本对Eigen3模块支持的差异
# 首先尝试查找Eigen3的CMake模块（新版本Ubuntu提供）
find_package(Eigen3 QUIET)
# 如果Eigen3未找到，则回退到使用cmake_modules提供的版本
if(NOT EIGEN3_FOUND)
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  # 设置Eigen3的包含目录
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
endif()

# 确保EIGEN3_INCLUDE_DIRS变量被正确设置，以兼容Eigen 3.2版本
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

# 配置catkin包
catkin_package(
   INCLUDE_DIRS include # 指定头文件目录
   CATKIN_DEPENDS sensor_msgs tf2_ros tf2 # 声明catkin依赖项
   DEPENDS EIGEN3 # 声明非catkin依赖项
)


# 添加包含目录，以便编译器能够找到头文件
include_directories(include
   ${catkin_INCLUDE_DIRS}
   ${EIGEN3_INCLUDE_DIRS}
)

# 安装头文件目录
install(DIRECTORY include/${PROJECT_NAME}/ # 指定要安装的目录
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION} # 指定安装目标路径
)

# 启用catkin的Python支持
catkin_python_setup()

# 如果启用了测试，则进行以下配置
if(CATKIN_ENABLE_TESTING)
  # 添加Python nose测试
  catkin_add_nosetests(test/test_tf2_sensor_msgs.py)

  # 查找测试所需的catkin组件
  find_package(catkin REQUIRED COMPONENTS geometry_msgs sensor_msgs rostest tf2_ros tf2)

  # 添加C++测试可执行文件，EXCLUDE_FROM_ALL表示默认不构建
  add_executable(test_tf2_sensor_msgs_cpp EXCLUDE_FROM_ALL test/test_tf2_sensor_msgs.cpp)
  # 链接测试可执行文件所需的库
  target_link_libraries(test_tf2_sensor_msgs_cpp ${catkin_LIBRARIES} ${GTEST_LIBRARIES})


  # 如果存在'tests'目标，则添加依赖
  if(TARGET tests)
    add_dependencies(tests test_tf2_sensor_msgs_cpp)
  endif()
  # 添加ROS测试启动文件
  add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test.launch)

endif()
