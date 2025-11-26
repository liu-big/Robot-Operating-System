# 指定CMake的最低版本要求
cmake_minimum_required(VERSION 2.8.3)
# 定义项目名称
project(ucar_map)

## 编译选项：将C++标准设置为C++11，ROS Kinetic及更高版本支持
# add_compile_options(-std=c++11)

## 查找Catkin宏和库
## 如果使用find_package(catkin REQUIRED COMPONENTS xyz)这样的组件列表，也会查找其他Catkin包
find_package(catkin REQUIRED COMPONENTS
  nav_msgs      # 导航消息
  roscpp        # ROS C++客户端库
  rospy         # ROS Python客户端库
  sensor_msgs   # 传感器消息
  std_msgs      # 标准消息
)

## 系统依赖项通过CMake的约定进行查找
# find_package(Boost REQUIRED COMPONENTS system)


## 如果包中包含setup.py文件，取消此行的注释。此宏确保
## 其中声明的模块和全局脚本被安装
## 参见 http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## 声明ROS消息、服务和动作                       ##
################################################

## 要在此包中声明和构建消息、服务或动作，请遵循以下步骤：
## * 设MSG_DEP_SET为您在消息/服务/动作中使用的消息类型所属的包集合
##   （例如 std_msgs, actionlib_msgs, ...）。
## * 在package.xml文件中：
##   * 添加一个build_depend标签，值为"message_generation"
##   * 添加一个build_depend和一个exec_depend标签，值为MSG_DEP_SET中的每个包
##   * 如果MSG_DEP_SET不为空，则以下依赖项已被引入，但仍可声明以确保：
##     * 添加一个exec_depend标签，值为"message_runtime"
## * 在此文件（CMakeLists.txt）中：
##   * 将"message_generation"和MSG_DEP_SET中的每个包添加到
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * 将"message_runtime"和MSG_DEP_SET中的每个包添加到
##     catkin_package(CATKIN_DEPENDS ...)
##   * 根据需要取消下面add_*_files部分的注释，并列出要处理的每个.msg/.srv/.action文件
##   * 取消下面generate_messages条目的注释
##   * 将MSG_DEP_SET中的每个包添加到generate_messages(DEPENDENCIES ...)

## 在'msg'文件夹中生成消息
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## 在'srv'文件夹中生成服务
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## 在'action'文件夹中生成动作
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## 生成添加的消息和服务，以及此处列出的所有依赖项
# generate_messages(
#   DEPENDENCIES
#   nav_msgs#   sendor_msgs#   std_msgs
# )

################################################
## 声明ROS动态参数配置                         ##
################################################

## 要在此包中声明和构建动态参数配置，请遵循以下步骤：
## * 在package.xml文件中：
##   * 添加一个build_depend和exec_depend标签，值为"dynamic_reconfigure"
## * 在此文件（CMakeLists.txt）中：
##   * 将"dynamic_reconfigure"添加到
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * 取消下面"generate_dynamic_reconfigure_options"部分的注释，
##     并列出要处理的每个.cfg文件

## 在'cfg'文件夹中生成动态参数配置
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## Catkin特定配置                  ##
###################################
## catkin_package宏为您的包生成cmake配置文件
## 声明要传递给依赖项目的内容
## INCLUDE_DIRS: 如果您的包包含头文件，请取消此行的注释
## LIBRARIES: 您在此项目中创建的、依赖项目也需要的库
## CATKIN_DEPENDS: 依赖项目也需要的catkin包
## DEPENDS: 此项目的系统依赖项，依赖项目也需要
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ucar_map
#  CATKIN_DEPENDS nav_msgs roscpp rospy sendor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## 构建    ##
###########

## 指定头文件的其他位置
## 您的包位置应列在其他位置之前
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## 声明一个C++库
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/ucar_map.cpp
# )

## 添加库的cmake目标依赖项
## 例如，代码可能需要在生成库之前生成
## 无论是通过消息生成还是动态参数配置
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## 声明一个C++可执行文件
## 使用catkin_make时，所有包都在单个CMake上下文中构建
## 推荐的前缀确保不同包之间的目标名称不冲突
# add_executable(${PROJECT_NAME}_node src/ucar_map_node.cpp)

## 重命名不带前缀的C++可执行文件
## 上述推荐的前缀会导致目标名称过长，以下将目标重命名回较短的版本，以便用户使用
## 例如 "rosrun someones_pkg node" 而不是 "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## 添加可执行文件的cmake目标依赖项
## 与上面的库相同
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## 指定库以链接库或可执行目标
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## 安装    ##
#############

# 所有安装目标都应使用catkin DESTINATION变量
# 参见 http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## 标记可执行脚本（Python等）以进行安装
## 与setup.py不同，您可以选择目标位置
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## 标记可执行文件以进行安装
## 参见 http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## 标记库以进行安装
## 参见 http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## 标记C++头文件以进行安装
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## 标记其他文件以进行安装（例如 launch 和 bag 文件等）
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## 测试    ##
#############

## 添加基于gtest的C++测试代码检测和执行
## 参考：http://ros.org/wiki/catkin/Reference/cmake_results#enable_testing
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ucar_map.cpp)
## 添加要由python nosetests运行的文件夹
## 参考：http://ros.org/wiki/catkin/Reference/cmake_results#add_nosetests
# add_nosetests(test/test_ucar_map.py)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   nav_msgs#   sendor_msgs#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ucar_map
#  CATKIN_DEPENDS nav_msgs roscpp rospy sendor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/ucar_map.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/ucar_map_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ucar_map.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
