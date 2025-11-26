# 设置CMake的最低版本要求
cmake_minimum_required(VERSION 2.8)

# 定义ROS项目的名称
project(fdilink_ahrs)

# 设置C++编译标准为C++0x (C++11)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")  

# 查找Eigen3库，REQUIRED表示如果找不到则报错
find_package(Eigen3 REQUIRED)
# 设置Eigen3的头文件目录变量
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

# 查找catkin包及其所需的组件
find_package(catkin REQUIRED COMPONENTS
  # message_generation # 消息生成，如果需要自定义消息则取消注释
  # genmsg             # 消息生成工具
  # geometry_msgs      # 几何消息类型，如点、姿态等
  roscpp             # ROS C++客户端库
  rospy              # ROS Python客户端库
  sensor_msgs        # 传感器消息类型，如IMU数据
  std_msgs           # 标准消息类型
  tf                 # 坐标变换库
  # std_srvs           # 标准服务类型
  # nav_msgs           # 导航消息类型
  serial             # 串口通信库
)

# 添加头文件搜索路径
include_directories(
  include            # 项目自身的头文件目录
  src                # 项目的源文件目录，有时也包含头文件
  ${catkin_INCLUDE_DIRS} # catkin工作空间中所有依赖包的头文件目录
  ${Eigen3_INCLUDE_DIRS} # Eigen3库的头文件目录
)

# 配置catkin包的相关信息
catkin_package(
  CATKIN_DEPENDS     # 声明本包的运行时依赖，这些依赖包在编译时也需要
  # message_runtimes # 消息运行时，如果使用了自定义消息则取消注释
#  INCLUDE_DIRS include # 声明本包的公共头文件目录，供其他包使用
#  LIBRARIES aicar_controller # 声明本包生成的库，供其他包链接
#  CATKIN_DEPENDS genmsg geometry_msgs roscpp rospy sensor_msgs std_msgs tf # 详细列出所有catkin依赖
#  DEPENDS system_lib # 声明本包的系统库依赖
)

# 以下是被注释掉的串口库设置，如果需要特定路径的libserial.so，可以取消注释并修改路径
# set(SERIAL_LIB /opt/ros/kinetic/lib/libserial.so)
# set(SERIAL_LIB_PATH /opt/ros/kinetic/lib)
# link_directories(${SERIAL_LIB_PATH})

## 构建ahrs_driver可执行文件
# 从src/ahrs_driver.cpp编译生成ahrs_driver可执行文件
add_executable(ahrs_driver src/ahrs_driver.cpp)
# 链接所需的库：crc_table库和所有catkin依赖库
target_link_libraries(ahrs_driver crc_table ${catkin_LIBRARIES})# ${SERIAL_LIB} # 如果使用特定串口库，取消注释

## 构建crc_table库
# 从src/crc_table.cpp编译生成crc_table库
add_library(crc_table src/crc_table.cpp)