# 设置 CMake 的最低版本要求。
cmake_minimum_required(VERSION 2.8)

# 定义项目名称。
project(kdl_conversions)

# 查找 Catkin 包，并声明对 geometry_msgs 的依赖。
# REQUIRED 表示如果找不到这些包，则配置失败。
find_package(catkin REQUIRED geometry_msgs)
# 查找 Orocos KDL 库。
find_package(orocos_kdl REQUIRED)

# 配置 Catkin 包的属性。
catkin_package(
  # 导出头文件目录，使得其他依赖此包的包可以找到其头文件。
  INCLUDE_DIRS include
  # 导出库文件，${PROJECT_NAME} 会被替换为 kdl_conversions。
  LIBRARIES ${PROJECT_NAME}
  # 声明 Catkin 依赖项，这些依赖项也会被传递给依赖此包的其他 Catkin 包。
  CATKIN_DEPENDS geometry_msgs
  # 声明非 Catkin 依赖项，这些依赖项不会被传递。
  DEPENDS orocos_kdl
)

# 添加头文件搜索路径。
# include: 当前包的头文件目录。
# ${catkin_INCLUDE_DIRS}: Catkin 依赖包的头文件目录。
# ${orocos_kdl_INCLUDE_DIRS}: Orocos KDL 库的头文件目录。
include_directories(include ${catkin_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS})

# 添加库文件搜索路径。
# ${catkin_LIBRARY_DIRS}: Catkin 依赖包的库文件目录。
link_directories(${catkin_LIBRARY_DIRS})
# ${orocos_kdl_LIBRARY_DIRS}: Orocos KDL 库的库文件目录。
link_directories(${orocos_kdl_LIBRARY_DIRS})

# 添加库文件。
# ${PROJECT_NAME} 会被替换为 kdl_conversions，表示构建一个名为 kdl_conversions 的库。
add_library(${PROJECT_NAME}
  # 指定源文件。
  src/kdl_msg.cpp
)

# 添加构建依赖，确保在构建当前库之前，其依赖的 Catkin 目标已被构建。
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

# 链接库文件。
# ${catkin_LIBRARIES}: Catkin 依赖包的库文件。
# ${orocos_kdl_LIBRARIES}: Orocos KDL 库的库文件。
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

# 安装头文件。
# 将 include/${PROJECT_NAME}/ 目录下的所有内容安装到指定的目标路径。
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# 安装目标库文件。
# 将构建好的库文件安装到指定的目标路径。
install(TARGETS ${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
