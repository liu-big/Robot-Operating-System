# 设置CMake的最低版本要求
cmake_minimum_required(VERSION 3.0.2)
# 定义项目名称
project(eigen_conversions)

# 查找所需的依赖包
# REQUIRED表示如果找不到包，则停止配置
find_package(orocos_kdl REQUIRED)
find_package(catkin REQUIRED geometry_msgs std_msgs)
find_package(Eigen3 REQUIRED)

# 添加头文件搜索路径
# SYSTEM表示这些头文件是系统库，编译器将抑制对它们的警告
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})
# 添加项目自身的include目录以及catkin和orocos_kdl的头文件目录
include_directories(include ${catkin_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS})
# 添加库文件搜索路径
link_directories(${catkin_LIBRARY_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS})

# 配置catkin包
catkin_package(
  # 导出的头文件目录，供其他依赖此包的项目使用
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
  # 导出的库文件
  LIBRARIES ${PROJECT_NAME}
  # catkin构建依赖，这些依赖会传递给依赖此包的其他catkin项目
  CATKIN_DEPENDS geometry_msgs std_msgs
  # 非catkin依赖，这些依赖不会传递
  DEPENDS orocos_kdl
)

# 添加库
# ${PROJECT_NAME}是库的名称，这里是eigen_conversions
add_library(${PROJECT_NAME}
  src/eigen_msg.cpp
  src/eigen_kdl.cpp
)
# 添加依赖，确保在构建此库之前，catkin导出的目标已构建完成
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
# 链接库，将依赖的catkin库和orocos_kdl库链接到当前库
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

# 安装头文件
# 将include/${PROJECT_NAME}/目录下的所有内容安装到指定的目标路径
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# 安装目标库
# 将构建好的库安装到指定的目标路径
install(TARGETS ${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
