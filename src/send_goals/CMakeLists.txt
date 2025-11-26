# CMakeLists.txt
#
# @Author: guo
# @Date: 2024-06-20
# @Description: ROS包 `send_goals` 的CMake构建文件。
#               定义了项目的编译规则、依赖项、可执行文件和安装目标。
#               该文件遵循ROS Catkin构建系统的标准。

cmake_minimum_required(VERSION 3.0.2)
project(send_goals)

## 编译选项：将C++标准设置为C++11，ROS Kinetic及更高版本支持
# add_compile_options(-std=c++11)

## 查找Catkin宏和库
## 如果使用 find_package(catkin REQUIRED COMPONENTS xyz) 这样的组件列表，
## 也会查找其他Catkin包。
find_package(catkin REQUIRED COMPONENTS
  actionlib         # 动作库，用于实现ROS动作
  move_base_msgs    # 导航栈的动作消息定义
  roscpp            # ROS C++客户端库
  geometry_msgs     # 几何消息类型，如点、姿态等
  rospy             # ROS Python客户端库
  std_msgs          # 标准消息类型
  tf2               # TF2变换库
  tf2_geometry_msgs # TF2几何消息转换
  tf2_ros           # TF2 ROS接口
)

## 系统依赖项通过CMake的约定进行查找
# find_package(Boost REQUIRED COMPONENTS system)


## 如果包包含 setup.py 文件，取消注释此宏。此宏确保
## 其中声明的模块和全局脚本得到安装。
## 详见 http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## 声明ROS消息、服务和动作 ##
################################################

## 要在此包中声明和构建消息、服务或动作，请遵循以下步骤：
## * 设 MSG_DEP_SET 为您的消息/服务/动作中使用的消息类型所属的包集合
##   （例如 std_msgs, actionlib_msgs, ...）。
## * 在 package.xml 文件中：
##   * 添加一个 build_depend 标签用于 "message_generation"
##   * 为 MSG_DEP_SET 中的每个包添加一个 build_depend 和一个 exec_depend 标签
##   * 如果 MSG_DEP_SET 不为空，则以下依赖项已被引入，但仍可声明以确保：
##     * 添加一个 exec_depend 标签用于 "message_runtime"
## * 在此文件 (CMakeLists.txt) 中：
##   * 将 "message_generation" 和 MSG_DEP_SET 中的每个包添加到
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * 将 "message_runtime" 和 MSG_DEP_SET 中的每个包添加到
##     catkin_package(CATKIN_DEPENDS ...)
##   * 根据需要取消注释下面的 add_*_files 部分，
##     并列出要处理的每个 .msg/.srv/.action 文件
##   * 取消注释下面的 generate_messages 条目
##   * 将 MSG_DEP_SET 中的每个包添加到 generate_messages(DEPENDENCIES ...)

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

## 在 'action' 文件夹中生成动作
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## 生成添加的消息和服务，以及此处列出的所有依赖项
# generate_messages(
#   DEPENDENCIES
#   move_base_msgs
# )

################################################
## 声明ROS动态参数配置 ##
################################################

## 要在此包中声明和构建动态参数配置，请遵循以下步骤：
## * 在 package.xml 文件中：
##   * 添加一个 build_depend 和一个 exec_depend 标签用于 "dynamic_reconfigure"
## * 在此文件 (CMakeLists.txt) 中：
##   * 将 "dynamic_reconfigure" 添加到
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * 取消注释下面的 "generate_dynamic_reconfigure_options" 部分，
##     并列出要处理的每个 .cfg 文件

## 在 'cfg' 文件夹中生成动态参数配置
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## Catkin特定配置 ##
###################################
## catkin_package 宏为您的包生成cmake配置文件
## 声明要传递给依赖项目的内容
## INCLUDE_DIRS: 如果您的包包含头文件，请取消注释此行
## LIBRARIES: 您在此项目中创建的、依赖项目也需要的库
## CATKIN_DEPENDS: 依赖项目也需要的catkin包
## DEPENDS: 此项目的系统依赖项，依赖项目也需要
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES send_goals
#  CATKIN_DEPENDS actionlib move_base_msgs roscpp
#  DEPENDS system_lib
)

###########
## 构建 ##
###########

## 指定头文件的额外位置
## 您的包位置应列在其他位置之前
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## 声明一个C++库
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/send_goals.cpp
# )

## 添加库的cmake目标依赖项
## 例如，代码可能需要在生成库之前生成
## 无论是通过消息生成还是动态参数配置
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## 声明一个C++可执行文件
## 使用 catkin_make，所有包都在单个CMake上下文中构建
## 推荐的前缀确保包之间的目标名称不会冲突
add_executable(send_goal_node src/send_goal_node.cpp)
add_executable(get_nav_poinet src/get_nav_poinet.cpp)
add_executable(tf_stop_pid src/tf_stop_pid.cpp)

## 重命名不带前缀的C++可执行文件
## 上述推荐的前缀会导致目标名称过长，以下将其重命名回较短的版本，方便用户使用
## 例如 "rosrun someones_pkg node" 而不是 "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## 添加可执行文件的cmake目标依赖项
## 与上面的库相同
#  add_dependencies(send_goal_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## 指定库以链接库或可执行目标
target_link_libraries(send_goal_node
  ${catkin_LIBRARIES}
)
target_link_libraries(get_nav_poinet
  ${catkin_LIBRARIES}
)
target_link_libraries(tf_stop_pid
  ${catkin_LIBRARIES}
)
#############
## 安装 ##
#############

# 所有安装目标都应使用 catkin DESTINATION 变量
# 详见 http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## 标记可执行脚本（Python等）以进行安装
## 与 setup.py 不同，您可以选择目标位置
catkin_install_python(PROGRAMS
  scripts/pub_pos.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
catkin_install_python(PROGRAMS
  scripts/camera_tf.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
catkin_install_python(PROGRAMS
  scripts/camera_tf_2.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
## 标记可执行文件以进行安装
## 详见 http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## 标记库以进行安装
## 详见 http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## 标记头文件以进行安装
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## 使用 install(TARGETS... DESTINATION...) 获取更多详细信息

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_send_goals.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
