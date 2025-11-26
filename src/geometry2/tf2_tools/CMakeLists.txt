# 设置所需的CMake最低版本
cmake_minimum_required(VERSION 2.8.3)
# 定义项目名称
project(tf2_tools)

# 查找catkin包及其所需的组件
# REQUIRED表示如果找不到这些组件，则停止配置
find_package(catkin REQUIRED COMPONENTS tf2
                                         tf2_msgs
                                         tf2_ros
)

# 配置catkin包
catkin_package(
   CATKIN_DEPENDS tf2 # 声明catkin依赖项
                  tf2_msgs
                  tf2_ros)

# 安装可执行脚本
install(PROGRAMS scripts/view_frames.py scripts/echo.py # 指定要安装的脚本
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} # 指定安装目标路径，通常是ROS的bin目录
)
