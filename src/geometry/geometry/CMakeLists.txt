# 设置 CMake 的最低版本要求。ROS 通常需要较新的 CMake 版本来构建项目。
cmake_minimum_required(VERSION 2.8.3)

# 定义项目名称为 'geometry'。
# 在 ROS 中，这通常与 package.xml 中的包名一致。
project(geometry)

# 查找并加载 Catkin 构建系统所需的包。
# 'REQUIRED' 关键字表示如果找不到 Catkin，则配置失败。
find_package(catkin REQUIRED)

# 声明这是一个 Catkin 元包 (metapackage)。
# 元包本身不包含源代码，而是用于组织和管理一组相关的 ROS 包。
# 它通过 package.xml 文件中的 <metapackage/> 标签和 <run_depend> 来定义其包含的子包。
catkin_metapackage()
