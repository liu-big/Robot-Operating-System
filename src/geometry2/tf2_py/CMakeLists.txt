# 指定所需的 CMake 最低版本
cmake_minimum_required(VERSION 2.8.3)
# 定义项目名称
project(tf2_py)

## 查找 catkin 宏和库
## 如果使用 COMPONENTS 列表，例如 find_package(catkin REQUIRED COMPONENTS xyz)
## 也会查找其他 catkin 包
find_package(catkin REQUIRED COMPONENTS rospy tf2)

## 系统依赖项使用 CMake 的约定进行查找
# find_package(Boost REQUIRED COMPONENTS system)

# 查找 Python 库，要求版本为 2
find_package(PythonLibs 2 REQUIRED)
# 包含目录，包括 Python 的头文件路径和 catkin 的包含目录
include_directories(${PYTHON_INCLUDE_PATH} ${catkin_INCLUDE_DIRS})

## 如果包有 setup.py，取消注释此行。此宏确保
## 其中声明的模块和全局脚本得到安装
## 参见 http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

#######################################
## 声明 ROS 消息和服务 ##
#######################################

## 在 'msg' 文件夹中生成消息
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## 在 'srv' 文件夹中生成服务
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## 生成添加的消息和服务，以及此处列出的任何依赖项
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # 或包含消息的其他包
# )

###################################
## catkin 特定配置 ##
###################################
## catkin_package 宏为您的包生成 cmake 配置文件
## 声明要传递给依赖项目的内容
## LIBRARIES: 您在此项目中创建的，依赖项目也需要的库
## CATKIN_DEPENDS: 依赖项目也需要的 catkin 包
## DEPENDS: 依赖项目也需要的系统依赖项
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tf2_py
  CATKIN_DEPENDS rospy tf2
#  DEPENDS system_lib
)

###########
## 构建 ##
###########

## 指定头文件的其他位置
## 您的包位置应列在其他位置之前
# include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

## 声明一个 C++ 库
# add_library(tf2_py
#   src/${PROJECT_NAME}/tf2_py.cpp
# )

## 声明一个 C++ 可执行文件
# add_executable(tf2_py_node src/tf2_py_node.cpp)

## 添加可执行文件/库的 cmake 目标依赖项
## 例如，消息头文件可能需要在节点之前生成
# add_dependencies(tf2_py_node tf2_py_generate_messages_cpp)

## 指定库以链接库或可执行目标
# target_link_libraries(tf2_py_node
#   ${catkin_LIBRARIES}
# )


# 检查 SSE
#!!! rosbuild_check_for_sse()

# 与 tf 的动态链接工作正常，除了异常传播，这在单元测试中失败了。
# 因此直接使用对象构建。

link_libraries(${PYTHON_LIBRARIES})
add_library(tf2_py src/tf2_py.cpp)
target_link_libraries(tf2_py ${catkin_LIBRARIES})
add_dependencies(tf2_py ${catkin_EXPORTED_TARGETS})

if(WIN32)
  # 在 Windows 上使用 .pyd 扩展名
  set_target_properties(tf2_py PROPERTIES OUTPUT_NAME tf2 PREFIX "_" SUFFIX ".pyd")
else()
  set_target_properties(tf2_py PROPERTIES COMPILE_FLAGS "-g -Wno-missing-field-initializers")
  set_target_properties(tf2_py PROPERTIES OUTPUT_NAME tf2 PREFIX "_" SUFFIX ".so")
endif()
set_target_properties(tf2_py PROPERTIES
  ARCHIVE_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
  LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
  RUNTIME_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
)
#!! rosbuild_add_compile_flags(tf2_py ${SSE_FLAGS}) #如果可用，有条件地添加 sse 标志


#############
## 安装 ##
#############

# 所有安装目标都应使用 catkin DESTINATION 变量
# 参见 http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## 标记可执行脚本（Python 等）以进行安装
## 与 setup.py 不同，您可以选择目标位置
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## 标记可执行文件和/或库以进行安装
# install(TARGETS tf2_py tf2_py_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## 标记 cpp 头文件以进行安装
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## 标记其他文件以进行安装（例如启动文件和 bag 文件等）
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

if(WIN32)
  install(FILES ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/_tf2.pyd
    DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
  )
else()
  install(FILES ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/_tf2.so
    DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
  )
endif()

#############
## 测试 ##
#############

## 添加基于 gtest 的 cpp 测试目标并链接库
# catkin_add_gtest(${PROJECT_NAME}-test test/test_tf2_py.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## 添加要由 python nosetests 运行的文件夹
# catkin_add_nosetests(test)
