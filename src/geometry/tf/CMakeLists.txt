# 设置 CMake 的最低版本要求。
cmake_minimum_required(VERSION 2.8)
# 定义项目名称为 tf。
project(tf)

# 包含用于检查 C++ 编译器标志的模块。
include(CheckCXXCompilerFlag)
# 清除 COMPILER_SUPPORTS_CXX11 缓存变量。
unset(COMPILER_SUPPORTS_CXX11 CACHE)
# 如果是 MSVC 编译器，则不需要额外指定 C++11，因为 C++14 默认启用。
if(MSVC)
  # https://docs.microsoft.com/en-us/cpp/build/reference/std-specify-language-standard-version
  # MSVC 默认启用 c++14，无需指定 c++11
else()
  # 检查编译器是否支持 C++11 标准。
  check_cxx_compiler_flag(-std=c++11 COMPILER_SUPPORTS_CXX11)
  # 如果支持 C++11，则添加相应的编译选项。
  if(COMPILER_SUPPORTS_CXX11)
    add_compile_options(-std=c++11)
  endif()
endif()

# 查找 Catkin 包及其所需的组件。
find_package(catkin REQUIRED COMPONENTS
    angles
    geometry_msgs
    message_filters
    message_generation
    rosconsole
    roscpp
    rostime
    sensor_msgs
    std_msgs
    tf2_ros
)
# 查找 Boost 库及其所需的 thread 和 system 组件。
find_package(Boost REQUIRED COMPONENTS thread system)

# 设置 Catkin 的 Python 相关配置。
catkin_python_setup()

# 添加包含目录，以便编译器查找头文件。
include_directories(
    include
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
)

# 添加消息文件，用于生成 ROS 消息。
add_message_files(DIRECTORY msg FILES tfMessage.msg)
# 添加服务文件，用于生成 ROS 服务。
add_service_files(DIRECTORY srv FILES FrameGraph.srv)

# 生成 ROS 消息，并指定依赖项。
generate_messages(DEPENDENCIES geometry_msgs sensor_msgs std_msgs)

# 配置 Catkin 包的属性。
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS geometry_msgs message_filters message_runtime roscpp sensor_msgs std_msgs tf2_ros rosconsole
)

# 添加库，并指定源文件。
add_library(${PROJECT_NAME}
    src/cache.cpp
    src/tf.cpp
    src/transform_broadcaster.cpp
    src/transform_listener.cpp
)
# 链接库，包括 Catkin 和 Boost 库。
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
# 添加依赖项。
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 调试相关的可执行文件。
# 添加 tf_empty_listener 可执行文件。
add_executable(tf_empty_listener src/empty_listener.cpp)
# 链接 tf_empty_listener 到当前项目库。
target_link_libraries(tf_empty_listener ${PROJECT_NAME})

# 添加 tf_echo 可执行文件。
add_executable(tf_echo src/tf_echo.cpp)
# 链接 tf_echo 到当前项目库。
target_link_libraries(tf_echo ${PROJECT_NAME})

# 添加 tf_change_notifier 可执行文件。
add_executable(tf_change_notifier src/change_notifier.cpp)
# 链接 tf_change_notifier 到当前项目库。
target_link_libraries(tf_change_notifier ${PROJECT_NAME})

# 添加 tf_monitor 可执行文件。
add_executable(tf_monitor src/tf_monitor.cpp)
# 链接 tf_monitor 到当前项目库。
target_link_libraries(tf_monitor ${PROJECT_NAME})

# 添加 static_transform_publisher 可执行文件。
add_executable(static_transform_publisher src/static_transform_publisher.cpp)
# 链接 static_transform_publisher 到当前项目库。
target_link_libraries(static_transform_publisher ${PROJECT_NAME})

# 测试部分。
# 只有在 CATKIN_ENABLE_TESTING 启用且不是 Android 平台时才执行测试。
if(CATKIN_ENABLE_TESTING AND NOT ANDROID)

# 查找 rostest 包。
find_package(rostest REQUIRED)

# 添加 tf_unittest 的 GTest 测试。
catkin_add_gtest(tf_unittest test/tf_unittest.cpp)
# 为 tf_unittest 添加包含目录。
target_include_directories(tf_unittest PRIVATE ${rostest_INCLUDE_DIRS})
# 链接 tf_unittest 到当前项目库。
target_link_libraries(tf_unittest ${PROJECT_NAME})

# 添加 tf_quaternion_unittest 的 GTest 测试。
catkin_add_gtest(tf_quaternion_unittest test/quaternion.cpp)
# 链接 tf_quaternion_unittest 到当前项目库。
target_link_libraries(tf_quaternion_unittest ${PROJECT_NAME})

# 添加 test_transform_datatypes 的 GTest 测试。
catkin_add_gtest(test_transform_datatypes test/test_transform_datatypes.cpp)
# 链接 test_transform_datatypes 到当前项目库和 Boost 库。
target_link_libraries(test_transform_datatypes ${PROJECT_NAME} ${Boost_LIBRARIES})

# 添加 transform_listener_unittest 可执行文件。
add_executable(transform_listener_unittest test/transform_listener_unittest.cpp)
# 链接 transform_listener_unittest 到当前项目库和 GTest 库。
target_link_libraries(transform_listener_unittest ${PROJECT_NAME} ${GTEST_LIBRARIES})
# 添加 rostest 测试。
add_rostest(test/transform_listener_unittest.launch)

# 因 TransformStorage 的更改而禁用。
#catkin_add_gtest_future(tf_unittest_future test/tf_unittest_future.cpp)
#target_link_libraries(tf_unittest_future ${PROJECT_NAME})

# 添加 test_velocity 的 GTest 测试。
catkin_add_gtest(test_velocity test/velocity_test.cpp)
# 链接 test_velocity 到当前项目库。
target_link_libraries(test_velocity ${PROJECT_NAME})

# 添加 test_transform_twist 可执行文件（注释掉）。
#add_executable(test_transform_twist test/transform_twist_test.cpp)
#target_link_libraries(test_transform_twist ${PROJECT_NAME})
#catkin_add_gtest_build_flags(test_transform_twist)
#add_rostest(test/transform_twist_test.launch)

# 添加 cache_unittest 的 GTest 测试。
catkin_add_gtest(cache_unittest test/cache_unittest.cpp)
# 链接 cache_unittest 到当前项目库。
target_link_libraries(cache_unittest ${PROJECT_NAME})

# 添加 test_message_filter 可执行文件。
add_executable(test_message_filter EXCLUDE_FROM_ALL test/test_message_filter.cpp)
# 链接 test_message_filter 到 tf 库、Boost 库和 GTest 库。
target_link_libraries(test_message_filter tf ${Boost_LIBRARIES} ${GTEST_LIBRARIES})
# 添加 rostest 测试。
add_rostest(test/test_message_filter.xml)


### 性能测试
#catkin_add_gtest_future(tf_benchmark test/tf_benchmark.cpp)
#target_link_libraries(tf_benchmark ${PROJECT_NAME})

# 添加 testListener 可执行文件。
add_executable(testListener test/testListener.cpp)
# 链接 testListener 到当前项目库和 GTest 库。
target_link_libraries(testListener ${PROJECT_NAME} ${GTEST_LIBRARIES})
# 添加 rostest 测试。
add_rostest(test/test_broadcaster.launch)

# 添加 testBroadcaster 可执行文件。
add_executable(testBroadcaster test/testBroadcaster.cpp)
# 链接 testBroadcaster 到当前项目库。
target_link_libraries(testBroadcaster ${PROJECT_NAME})

# 添加 Python Nose 测试。
catkin_add_nosetests(test/testPython.py) 
# 添加 tf_speed_test 可执行文件。
add_executable(tf_speed_test EXCLUDE_FROM_ALL test/speed_test.cpp)
# 链接 tf_speed_test 到当前项目库。
target_link_libraries(tf_speed_test ${PROJECT_NAME})

# 如果存在 tests 目标，则添加依赖项。
if(TARGET tests)
    add_dependencies(tests testBroadcaster testListener transform_listener_unittest test_message_filter)
endif()

endif() # CATKIN_ENABLE_TESTING AND NOT ANDROID

# 安装头文件目录。
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# 安装库文件。
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# 安装可执行文件。
install(TARGETS tf_echo tf_empty_listener tf_change_notifier tf_monitor static_transform_publisher
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装可由 rosrun 运行的脚本。
install(PROGRAMS
  scripts/bullet_migration_sed.py
  scripts/tf_remap
  scripts/view_frames
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )

