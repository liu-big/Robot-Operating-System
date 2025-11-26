# 设置CMake的最低版本要求
# CMakeLists.txt 是ROS包的构建配置文件，定义了如何编译、链接和安装项目中的可执行文件、库和脚本。

# 最小CMake版本要求
cmake_minimum_required(VERSION 2.8.3)
# 定义项目名称
project(ucar_controller)

## 编译选项：启用C++11标准，ROS Kinetic及更高版本支持
add_compile_options(-std=c++11)

# 查找Eigen3库，REQUIRED表示如果找不到则报错
find_package(Eigen3 REQUIRED)
# 设置Eigen3的头文件目录变量
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
# 查找Catkin依赖包，这些是ROS项目所需的核心组件
find_package(catkin REQUIRED COMPONENTS
  message_generation # 用于生成ROS消息
  genmsg             # 消息生成工具
  geometry_msgs      # 几何消息类型，如点、姿态等
  roscpp             # ROS C++客户端库
  rospy              # ROS Python客户端库
  sensor_msgs        # 传感器消息类型，如激光雷达数据、图像等
  std_msgs           # 标准消息类型，如字符串、数字等
  std_srvs           # 标准服务类型
  tf                 # 坐标变换库
  serial             # 串口通信库
  roslib             # ROS库的通用工具
)

## 消息文件生成：
## 取消注释以下部分并列出.msg文件，即可生成自定义消息
# add_message_files(FILES
#   # Battery_info.msg  # 电池信息消息
#   # Health_info.msg   # 健康信息消息
#   # Robot_health.msg  # 机器人健康状态消息
# )

## 服务文件生成：
## 列出项目中的所有服务文件
add_service_files(FILES
  GetMaxVel.srv    # 获取最大速度服务
  SetMaxVel.srv    # 设置最大速度服务
  GetSensorTF.srv  # 获取传感器坐标变换服务
  SetSensorTF.srv  # 设置传感器坐标变换服务
  GetBatteryInfo.srv # 获取电池信息服务
  SetLEDMode.srv   # 设置LED模式服务
)

## 动作文件生成：
## 取消注释以下部分并列出.action文件，即可生成自定义动作
# add_action_files(
#   FILES
#   Action1.action # 动作文件示例1
#   Action2.action # 动作文件示例2
# )

## 生成消息和服务的依赖项：
## 确保所有依赖的消息和服务的定义都被正确生成
generate_messages(DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

# Catkin包配置：
# 定义当前Catkin包的属性，如依赖项、包含目录和库
catkin_package(
  CATKIN_DEPENDS
  message_runtime # 消息运行时依赖
#  INCLUDE_DIRS include # 如果有头文件需要安装，取消注释此行
#  LIBRARIES ucar_controller # 如果有库需要安装，取消注释此行
#  CATKIN_DEPENDS genmsg geometry_msgs roscpp rospy sensor_msgs std_msgs tf # 其他Catkin包依赖
#  DEPENDS system_lib # 系统库依赖
)

# 包含目录：
# 指定编译时查找头文件的目录
include_directories(
  include # 项目自身的头文件目录
  ${catkin_INCLUDE_DIRS} # Catkin工作空间中的所有头文件目录
  ${Eigen3_INCLUDE_DIRS} # Eigen3库的头文件目录
)

# 再次包含include目录，确保所有头文件路径都被正确添加
include_directories(include)

# 添加可执行文件：
# 定义名为base_driver的可执行文件，源文件为src/base_driver.cpp
add_executable(base_driver  src/base_driver.cpp)
# 添加依赖项，确保在编译base_driver之前生成ROS消息
add_dependencies(base_driver ucar_controller_generate_messages_cpp)
# 链接库：
# 将base_driver可执行文件与所需的库链接
target_link_libraries(base_driver crc_table_fd ${catkin_LIBRARIES} ${Boost_LIBRARIES})

## 添加库：
## 定义名为crc_table_fd的库，源文件为src/crc_table.cpp
add_library(crc_table_fd src/crc_table.cpp)


# 安装Python脚本：
# 将odom_ekf.py脚本安装到Catkin包的bin目录下
catkin_install_python(PROGRAMS
  scripts/odom_ekf.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)