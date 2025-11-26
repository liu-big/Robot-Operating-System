# toplevel CMakeLists.txt for a catkin workspace
# 这是Catkin工作区的顶级CMakeLists.txt文件。
# 它负责在整个工作区中查找和配置Catkin。

# 设置CMake的最低版本要求。
cmake_minimum_required(VERSION 2.8.3)

# 标记当前CMakeLists文件为Catkin的顶级文件。
set(CATKIN_TOPLEVEL TRUE)

# 在工作区内搜索Catkin包。
# _cmd 变量定义了执行 catkin_find_pkg 命令的参数。
set(_cmd "catkin_find_pkg" "catkin" "${CMAKE_SOURCE_DIR}")
# 执行 catkin_find_pkg 命令，并捕获结果、输出和错误。
execute_process(COMMAND ${_cmd}
  RESULT_VARIABLE _res
  OUTPUT_VARIABLE _out
  ERROR_VARIABLE _err
  OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_STRIP_TRAILING_WHITESPACE
)
# 检查命令执行结果，如果不是成功（0）或未找到（2），则报错。
if(NOT _res EQUAL 0 AND NOT _res EQUAL 2)
  # 搜索catkin失败时，替换分号为空格以格式化命令字符串。
  string(REPLACE ";" " " _cmd_str "${_cmd}")
  # 抛出致命错误，指示catkin搜索失败。
  message(FATAL_ERROR "在工作区中搜索'catkin'失败 (${_cmd_str}): ${_err}")
endif()

# 根据catkin是否在工作区中找到来包含它。
if(_res EQUAL 0)
  # 如果找到，设置catkin的额外目录。
  set(catkin_EXTRAS_DIR "${CMAKE_SOURCE_DIR}/${_out}/cmake")
  # 包含all.cmake，使其在当前作用域内操作，不添加子目录。
  include(${catkin_EXTRAS_DIR}/all.cmake NO_POLICY_SCOPE)
  # 添加catkin作为子目录。
  add_subdirectory("${_out}")

else()
  # 如果未在工作区中找到catkin，则尝试通过 find_package() 查找。
  # 检查 CMAKE_PREFIX_PATH 是否已定义，如果未定义，则从环境变量中获取。
  if(NOT DEFINED CMAKE_PREFIX_PATH)
    if(NOT "$ENV{CMAKE_PREFIX_PATH}" STREQUAL "")
      # 根据操作系统类型处理路径分隔符。
      if(NOT WIN32)
        string(REPLACE ":" ";" CMAKE_PREFIX_PATH $ENV{CMAKE_PREFIX_PATH})
      else()
        set(CMAKE_PREFIX_PATH $ENV{CMAKE_PREFIX_PATH})
      endif()
    endif()
  endif()

  # 构建catkin工作区搜索路径列表。
  set(catkin_search_path "")
  foreach(path ${CMAKE_PREFIX_PATH})
    # 检查路径是否存在 .catkin 文件，以确定是否为catkin工作区。
    if(EXISTS "${path}/.catkin")
      # 检查路径是否已在搜索路径列表中。
      list(FIND catkin_search_path ${path} _index)
      if(_index EQUAL -1)
        # 如果不在，则添加到列表中。
        list(APPEND catkin_search_path ${path})
      endif()
    endif()
  endforeach()

  # 在所有工作区中搜索catkin。
  set(CATKIN_TOPLEVEL_FIND_PACKAGE TRUE)
  find_package(catkin QUIET
    NO_POLICY_SCOPE
    PATHS ${catkin_search_path}
    NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
  unset(CATKIN_TOPLEVEL_FIND_PACKAGE)

  # 如果仍然没有找到catkin，则抛出致命错误。
  if(NOT catkin_FOUND)
    message(FATAL_ERROR "find_package(catkin) 失败。在工作区或 CMAKE_PREFIX_PATH 中均未找到 catkin。可能的原因是之前没有 source 任何 ROS setup.sh 文件。")
  endif()
endif()

# 调用catkin_workspace()函数，完成Catkin工作区的配置。
catkin_workspace()
