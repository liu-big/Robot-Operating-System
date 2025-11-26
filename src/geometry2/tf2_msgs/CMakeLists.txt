# CMakeLists.txt
#
# Copyright (c) 2024, UCAR.AI, Inc. All rights reserved.
#
# tf2_msgs 包的 CMakeLists.txt 文件
# 该文件定义了 tf2_msgs 包的构建规则、消息、服务和动作。

cmake_minimum_required(VERSION 2.8.3)
project(tf2_msgs)

# 查找 catkin 包，并指定所需的组件：消息生成、几何消息和动作库消息
find_package(catkin REQUIRED COMPONENTS message_generation geometry_msgs actionlib_msgs)
# 查找 Boost 库，并指定所需的组件：线程
find_package(Boost COMPONENTS thread REQUIRED)

# 添加消息文件，从 msg 目录中添加 TF2Error.msg 和 TFMessage.msg
add_message_files(DIRECTORY msg FILES TF2Error.msg TFMessage.msg)
# 添加服务文件，从 srv 目录中添加 FrameGraph.srv
add_service_files(DIRECTORY srv FILES FrameGraph.srv)

# 添加动作文件，从 action 目录中添加 LookupTransform.action
add_action_files(DIRECTORY action FILES LookupTransform.action)
# 生成消息，并指定依赖项：actionlib_msgs、std_msgs 和 geometry_msgs
generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs geometry_msgs
)

# 配置 catkin 包，指定包含目录和 catkin 依赖项
catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS message_generation geometry_msgs actionlib_msgs)



