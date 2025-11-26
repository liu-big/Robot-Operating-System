# 设置 CMake 的最低版本要求
cmake_minimum_required(VERSION 2.8.3)
# 定义项目名称
project(tf2_geometry_msgs)

# 查找 Orocos KDL 库
find_package(orocos_kdl)
# 查找 Catkin 包，并指定所需的组件：geometry_msgs, tf2_ros, tf2
find_package(catkin REQUIRED COMPONENTS geometry_msgs tf2_ros tf2)
# 查找 Boost 库，并指定所需的组件：thread
find_package(Boost COMPONENTS thread REQUIRED)

# 解决 Issue #53：查找 KDL 库，如果找不到则报错
find_library(KDL_LIBRARY REQUIRED NAMES orocos-kdl HINTS ${orocos_kdl_LIBRARY_DIRS})

# 配置 Catkin 包的导出设置
catkin_package(
   LIBRARIES ${KDL_LIBRARY} # 导出的库
   INCLUDE_DIRS include # 导出的头文件目录
   DEPENDS orocos_kdl # 运行时依赖项
   CATKIN_DEPENDS  geometry_msgs tf2_ros tf2 # Catkin 包依赖项
)

# 添加头文件搜索路径
include_directories(include
                    ${catkin_INCLUDE_DIRS}
)

# 添加库文件搜索路径
link_directories(${orocos_kdl_LIBRARY_DIRS})

# 安装项目头文件目录
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# 设置 Python 模块的 Catkin 构建
catkin_python_setup()

# 如果启用了测试，则执行以下操作
if(CATKIN_ENABLE_TESTING)

# 添加一个 GTest 测试，用于测试 toMsg 和 fromMsg 转换
catkin_add_gtest(test_tomsg_frommsg test/test_tomsg_frommsg.cpp)
# 为测试目标添加头文件搜索路径
target_include_directories(test_tomsg_frommsg PUBLIC ${orocos_kdl_INCLUDE_DIRS})
# 链接测试目标所需的库
target_link_libraries(test_tomsg_frommsg ${catkin_LIBRARIES} ${GTEST_LIBRARIES} ${orocos_kdl_LIBRARIES})

# 再次查找 Catkin 包，用于测试组件
find_package(catkin REQUIRED COMPONENTS geometry_msgs rostest tf2_ros tf2)

# 添加一个可执行测试，用于测试 tf2_geometry_msgs
add_executable(test_geometry_msgs EXCLUDE_FROM_ALL test/test_tf2_geometry_msgs.cpp)
# 为测试目标添加头文件搜索路径
target_include_directories(test_geometry_msgs PUBLIC ${orocos_kdl_INCLUDE_DIRS})
# 链接测试目标所需的库
target_link_libraries(test_geometry_msgs ${catkin_LIBRARIES} ${GTEST_LIBRARIES} ${orocos_kdl_LIBRARIES})
# 添加 ROS 测试启动文件
add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test.launch)
add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_python.launch)

# 如果存在 'tests' 目标，则添加依赖
if(TARGET tests)
  add_dependencies(tests test_geometry_msgs)
endif()


endif()
