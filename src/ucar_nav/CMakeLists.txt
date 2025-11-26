# 指定CMake的最低版本要求。
cmake_minimum_required(VERSION 2.8.3)
# 定义ROS包的项目名称。
project(ucar_nav)

## 编译选项：将C++标准设置为C++11，ROS Kinetic及更高版本支持。
# add_compile_options(-std=c++11)

## 查找Catkin宏和库。
## 如果使用了find_package(catkin REQUIRED COMPONENTS xyz)这样的组件列表，也会查找其他Catkin包。
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  std_msgs
)

## 系统依赖项通过CMake的约定来查找。
# find_package(Boost REQUIRED COMPONENTS system)

## 如果包有setup.py文件，取消注释此行。此宏确保其中声明的模块和全局脚本被安装。
## 更多信息请参见: http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## 声明ROS消息、服务和动作（Actions） ##
################################################

## 要在此包中声明和构建消息、服务或动作，请遵循以下步骤:
## * 设MSG_DEP_SET为您在消息/服务/动作中使用的消息类型所属的包集合（例如std_msgs, actionlib_msgs等）。
## * 在package.xml文件中:
##   * 添加一个针对"message_generation"的build_depend标签。
##   * 为MSG_DEP_SET中的每个包添加一个build_depend和exec_depend标签。
##   * 如果MSG_DEP_SET不为空，以下依赖项已被引入，但为确保起见仍可声明:
##     * 添加一个针对"message_runtime"的exec_depend标签。
## * 在此文件（CMakeLists.txt）中:
##   * 将"message_generation"和MSG_DEP_SET中的每个包添加到find_package(catkin REQUIRED COMPONENTS ...)。
##   * 将"message_runtime"和MSG_DEP_SET中的每个包添加到catkin_package(CATKIN_DEPENDS ...)。
##   * 根据需要取消注释下面的add_*_files部分，并列出要处理的每个.msg/.srv/.action文件。
##   * 取消注释下面的generate_messages条目。
##   * 将MSG_DEP_SET中的每个包添加到generate_messages(DEPENDENCIES ...)。

## 在'msg'文件夹中生成消息。
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## 在'srv'文件夹中生成服务。
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## 在'action'文件夹中生成动作。
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## 生成添加的消息和服务，以及此处列出的所有依赖项。
# generate_messages(
#   DEPENDENCIES
#   nav_msgs#   std_msgs
# )

################################################
## 声明ROS动态参数（Dynamic Reconfigure） ##
################################################

## 要在此包中声明和构建动态参数，请遵循以下步骤:
## * 在package.xml文件中:
##   * 添加一个针对"dynamic_reconfigure"的build_depend和exec_depend标签。
## * 在此文件（CMakeLists.txt）中:
##   * 将"dynamic_reconfigure"添加到find_package(catkin REQUIRED COMPONENTS ...)。
##   * 取消注释下面的"generate_dynamic_reconfigure_options"部分，并列出要处理的每个.cfg文件。

## 在'cfg'文件夹中生成动态参数。
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## Catkin特定配置 ##
###################################
## catkin_package宏为您的包生成cmake配置文件。
## 声明要传递给依赖项目的内容。
## INCLUDE_DIRS: 如果您的包包含头文件，请取消注释此行。
## LIBRARIES: 您在此项目中创建的、依赖项目也需要的库。
## CATKIN_DEPENDS: 依赖项目也需要的catkin包。
## DEPENDS: 依赖项目也需要的此项目的系统依赖项。
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ucar_nav
#  CATKIN_DEPENDS nav_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## 构建 ##
###########

## 指定头文件的额外位置。
## 您的包位置应在其他位置之前列出。
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## 声明一个C++库。
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/ucar_nav.cpp
# )

## 添加库的cmake目标依赖项。
## 例如，代码可能需要在生成库之前生成，无论是通过消息生成还是动态参数。
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## 声明一个C++可执行文件。
## 使用catkin_make时，所有包都在一个CMake上下文中构建。
## 推荐的前缀确保不同包之间的目标名称不会冲突。
# add_executable(${PROJECT_NAME}_node src/ucar_nav_node.cpp)

## 重命名C++可执行文件，不带前缀。
## 上述推荐的前缀会导致目标名称过长，以下将目标重命名回较短的版本，方便用户使用。
## 例如: "rosrun someones_pkg node" 而不是 "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## 添加可执行文件的cmake目标依赖项。
## 与上面的库相同。
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## 指定库以链接库或可执行目标。
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## 安装 ##
#############

# 所有安装目标都应使用catkin DESTINATION变量。
# 更多信息请参见: http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## 标记可执行脚本（Python等）以进行安装。
## 与setup.py不同，您可以选择目标位置。
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## 标记可执行文件以进行安装。
## 更多信息请参见: http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## 标记库以进行安装。
## 更多信息请参见: http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## 标记C++头文件以进行安装。
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## 标记其他文件以进行安装（例如launch文件和bag文件等）。
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## 测试 ##
#############

## 添加基于gtest的C++测试目标并链接库。
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ucar_nav.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## 添加要由python nosetests运行的文件夹。
# catkin_add_nosetests(test)
