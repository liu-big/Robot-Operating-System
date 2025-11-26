# CMakeLists.txt
# @brief YDLIDAR ROS 包的 CMake 构建配置文件。
# 
# 该文件定义了如何使用 CMake 和 Catkin 构建系统来编译 YDLIDAR ROS 包中的源代码，
# 生成可执行文件，并安装相关资源。

# @brief 指定 CMake 的最低版本要求。
cmake_minimum_required(VERSION 2.8.3)

# @brief 定义项目名称。
project(ydlidar)

# @brief 设置 C++ 编译器标志，启用 C++11 标准。
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#add_definitions(-std=c++11) # Use C++11 (另一种启用 C++11 的方式，这里被注释掉了)

# @brief 查找 Catkin 构建系统所需的 ROS 组件。
# REQUIRED 表示如果找不到这些组件，CMake 配置将失败。
find_package(catkin REQUIRED COMPONENTS
  rosconsole  # 用于 ROS 日志输出
  roscpp      # 用于 ROS C++ 客户端库
  sensor_msgs # 用于 ROS 标准传感器消息，如 LaserScan
)

#add_subdirectory(sdk) # 如果 sdk 是一个独立的 CMake 项目，可以使用此命令将其作为子目录添加。
                      # 这里被注释掉了，因为 sdk 的源文件是直接通过 GLOB 方式包含的。

# @brief 定义 YDLIDAR SDK 的路径。
set(SDK_PATH "./sdk/")

# @brief 使用 FILE(GLOB) 命令查找 SDK 目录下的所有 C++ 和 C 源文件。
# 这将把匹配到的文件路径存储在 SDK_SRC 变量中。
FILE(GLOB SDK_SRC 
  "${SDK_PATH}/src/impl/unix/*.cpp" # 查找 Unix 平台特定的 C++ 实现文件
  "${SDK_PATH}/src/*.cpp"          # 查找 SDK 根 src 目录下的所有 C++ 文件
  "${SDK_PATH}/src/*.c"            # 查找 SDK 根 src 目录下的所有 C 文件
)

# @brief 声明这是一个 Catkin 包。
# 这个宏会处理 ROS 包的各种设置，如生成 pkg-config 文件、设置安装路径等。
catkin_package()

# @brief 指定编译时需要包含的头文件目录。
include_directories(
  ${catkin_INCLUDE_DIRS}      # Catkin 提供的所有包含目录
  ${PROJECT_SOURCE_DIR}/src   # 项目自身的 src 目录
  ${PROJECT_SOURCE_DIR}/sdk/include # YDLIDAR SDK 的公共头文件目录
  ${PROJECT_SOURCE_DIR}/sdk/src     # YDLIDAR SDK 的源文件目录（可能包含一些内部头文件）
)

# @brief 添加可执行文件。
# 第一个参数是可执行文件的名称，后面是构成该可执行文件的源文件。
add_executable(ydlidar_node src/ydlidar_node.cpp  ${SDK_SRC}) # 编译生成 ydlidar_node 可执行文件
add_executable(ydlidar_client src/ydlidar_client.cpp) # 编译生成 ydlidar_client 可执行文件

# @brief 链接库到可执行文件。
# 这将确保可执行文件能够找到并使用 Catkin 提供的 ROS 库。
target_link_libraries(ydlidar_node
   ${catkin_LIBRARIES} # 链接所有 Catkin 依赖的库
 )
target_link_libraries(ydlidar_client
   ${catkin_LIBRARIES} # 链接所有 Catkin 依赖的库
 )

# @brief 安装目标文件。
# 定义了编译生成的可执行文件和库文件在安装时的目标路径。
install(TARGETS ydlidar_node ydlidar_client
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} # 静态库安装路径
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} # 动态库安装路径
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} # 可执行文件安装路径
)


# @brief 安装目录。
# 定义了 launch、startup 和 sdk 目录在安装时的目标路径，并保留源文件的权限。
install(DIRECTORY launch startup sdk
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} # 安装到 ROS 包的共享目录
  USE_SOURCE_PERMISSIONS # 保留源目录的权限设置
)


