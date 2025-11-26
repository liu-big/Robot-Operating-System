# CMakeLists.txt
#
# @brief ROS 包 `speech_command` 的 CMake 构建配置文件。
#
# 该文件定义了如何使用 CMake 和 Catkin 构建 `speech_command` ROS 包。
# 它包括了编译选项、依赖项查找、库和可执行文件的构建以及安装规则。

# 设置 CMake 最低版本要求
cmake_minimum_required(VERSION 2.8.3)
# 定义项目名称
project(speech_command)

## 编译选项：将 C++ 标准设置为 C++11，ROS Kinetic 及更高版本支持
# add_compile_options(-std=c++11)
add_definitions(-std=c++11)
#SET(CMAKE_C_COMPILER "/usr/bin/gcc") # 可选：指定 C 编译器路径
##set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC -g -D_GLIBCXX_USE_CXX11_ABI=0") # 可选：设置 C 编译标志
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -g -D_GLIBCXX_USE_CXX11_ABI=0") # 可选：设置 C++ 编译标志

## 查找 Catkin 宏和库
## 如果使用 find_package(catkin REQUIRED COMPONENTS xyz) 这样的组件列表，也会查找其他 Catkin 包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  roslib
  serial
)

## 系统依赖项通过 CMake 的约定查找
# find_package(Boost REQUIRED COMPONENTS system) # 可选：查找 Boost 库


## 如果包有 setup.py 文件，取消注释此行。此宏确保
## 其中声明的模块和全局脚本被安装
## 详见 http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup() # 可选：为 Python 包设置 Catkin

################################################
## 声明 ROS 消息、服务和动作 ##
################################################

## 要在此包中声明和构建消息、服务或动作，请遵循以下步骤：
## * 将 MSG_DEP_SET 设置为您的消息/服务/动作中使用的消息类型所属的包集合
##   (例如 std_msgs, actionlib_msgs, ...)。
## * 在 package.xml 文件中：
##   * 添加 "message_generation" 的 build_depend 标签
##   * 为 MSG_DEP_SET 中的每个包添加 build_depend 和 exec_depend 标签
##   * 如果 MSG_DEP_SET 不为空，则以下依赖项已被拉入，但仍可声明以确保：
##     * 添加 "message_runtime" 的 exec_depend 标签
## * 在此文件 (CMakeLists.txt) 中：
##   * 将 "message_generation" 和 MSG_DEP_SET 中的每个包添加到
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * 将 "message_runtime" 和 MSG_DEP_SET 中的每个包添加到
##     catkin_package(CATKIN_DEPENDS ...)
##   * 根据需要取消注释下面的 add_*_files 部分
##     并列出要处理的每个 .msg/.srv/.action 文件
##   * 取消注释下面的 generate_messages 条目
##   * 将 MSG_DEP_SET 中的每个包添加到 generate_messages(DEPENDENCIES ...)

## 在 'msg' 文件夹中生成消息
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# ) # 可选：添加自定义消息文件

## 在 'srv' 文件夹中生成服务
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# ) # 可选：添加自定义服务文件

## 在 'action' 文件夹中生成动作
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# ) # 可选：添加自定义动作文件

## 生成添加的消息和服务以及此处列出的所有依赖项
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# ) # 可选：生成消息

################################################
## 声明 ROS 动态重新配置参数 ##
################################################

## 要在此包中声明和构建动态重新配置参数，请遵循以下步骤：
## * 在 package.xml 文件中：
##   * 添加 "dynamic_reconfigure" 的 build_depend 和 exec_depend 标签
## * 在此文件 (CMakeLists.txt) 中：
##   * 将 "dynamic_reconfigure" 添加到
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * 根据需要取消注释下面的 "generate_dynamic_reconfigure_options" 部分
##     并列出要处理的每个 .cfg 文件

## 在 'cfg' 文件夹中生成动态重新配置参数
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# ) # 可选：生成动态重新配置选项

###################################
## Catkin 特定配置 ##
###################################
## catkin_package 宏为您的包生成 cmake 配置文件
## 声明要传递给依赖项目的内容
## INCLUDE_DIRS: 如果您的包包含头文件，请取消注释此行
## LIBRARIES: 您在此项目中创建的、依赖项目也需要的库
## CATKIN_DEPENDS: 依赖项目也需要的 catkin 包
## DEPENDS: 此项目的系统依赖项，依赖项目也需要
catkin_package(
   INCLUDE_DIRS include # 导出头文件目录
   LIBRARIES speech_command # 导出库
#  CATKIN_DEPENDS roscpp rospy std_msgs # 导出 Catkin 依赖项
#  DEPENDS system_lib # 导出系统依赖项
)

###########
## 构建 ##
###########

## 指定头文件的额外位置
## 您的包位置应列在其他位置之前

include_directories(
  include # 项目内部头文件
  include/jsoncpp # JSONCPP 库头文件
  /usr/include # 系统头文件
  ${catkin_INCLUDE_DIRS} # Catkin 依赖项的头文件
  /usr/lib/libusb/include/libusb-1.0 # libusb 头文件
)

link_directories(
  lib/arm64 # 链接器搜索路径：arm64 架构的库
  /usr/include # 链接器搜索路径：系统库
)

## 声明一个 C++ 库
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/speech_command.cpp
#) # 可选：添加一个库目标

## 添加库的 cmake 目标依赖项
## 例如，代码可能需要在库之前生成
## 来自消息生成或动态重新配置
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}) # 可选：添加库依赖项

## 声明一个 C++ 可执行文件
## 使用 catkin_make，所有包都在单个 CMake 上下文中构建
## 推荐的前缀确保不同包之间的目标名称不会冲突
add_executable(${PROJECT_NAME}_node src/aiuiMain.cpp # 添加可执行文件目标
)

## 重命名 C++ 可执行文件，不带前缀
## 上述推荐的前缀会导致目标名称过长，以下将其重命名回较短的版本，方便用户使用
## 例如 "rosrun someones_pkg node" 而不是 "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "") # 可选：重命名可执行文件

## 添加可执行文件的 cmake 目标依赖项
## 与上述库相同
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}) # 添加可执行文件依赖项

## 指定库以链接库或可执行文件目标
target_link_libraries(${PROJECT_NAME}_node # 链接到主节点可执行文件
${catkin_LIBRARIES} # Catkin 提供的库
AIUITester hid_lib rt dl pthread stdc++ # 其他依赖库
)

# audio_player 库的构建
add_library(audio_player SHARED # 定义名为 audio_player 的共享库
  src/AudioPlayer.cpp # 包含 AudioPlayer.cpp 源文件
)
target_link_libraries(audio_player asound) # 链接到 asound 库（ALSA 音频库）


# audio_recorder 库的构建
add_library(audio_recorder SHARED # 定义名为 audio_recorder 的共享库
  src/AudioRecorder.cpp # 包含 AudioRecorder.cpp 源文件
)
target_link_libraries(audio_recorder hid_lib msc) # 链接到 hid_lib 和 msc 库

# AIUITester 库的构建
add_library(AIUITester SHARED # 定义名为 AIUITester 的共享库
  # src/AudioRecorder.cpp # 可选：如果需要，可以包含 AudioRecorder.cpp
  src/AIUITester.cpp # 包含 AIUITester.cpp 源文件
  src/TestListener.cpp # 包含 TestListener.cpp 源文件
  src/WriteAudioThread.cpp # 包含 WriteAudioThread.cpp 源文件
  src/FileUtil.cpp # 包含 FileUtil.cpp 源文件
)
target_link_libraries(AIUITester aiui audio_recorder audio_player jsoncpp) # 链接到 aiui, audio_recorder, audio_player 和 jsoncpp 库

#############
## 安装 ##
#############

# 所有安装目标都应使用 catkin DESTINATION 变量
# 详见 http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## 标记可执行脚本 (Python 等) 以进行安装
## 与 setup.py 不同，您可以选择目标位置
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# ) # 可选：安装 Python 脚本

## 标记可执行文件以进行安装
## 详见 http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# ) # 可选：安装可执行文件

## 标记库以进行安装
## 详见 http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# ) # 可选：安装库文件

## 标记 C++ 头文件以进行安装
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# ) # 可选：安装头文件

## 标记其他文件以进行安装 (例如 launch 和 bag 文件等)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# ) # 可选：安装其他文件

#############
## 测试 ##
#############

## 添加基于 gtest 的 C++ 测试目标并链接库
# catkin_add_gtest(${PROJECT_NAME}-test test/test_speech_command.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif() # 可选：添加 C++ 测试

## 添加要由 python nosetests 运行的文件夹
# catkin_add_nosetests(test) # 可选：添加 Python 测试
