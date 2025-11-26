# `include` 文件夹使用开发操作手册

## 1. 概述

在 `ucar_controller` ROS 包中，`include` 文件夹用于存放 C++ 头文件（`.h` 或 `.hpp`）。这些头文件包含了类定义、函数声明、常量定义以及数据结构等，它们是实现模块化编程和代码复用的关键。通过将接口与实现分离，`include` 文件夹有助于提高代码的可读性、可维护性和编译效率。

## 2. 文件夹结构

```
src/ucar_controller/include/
└── ucar_controller/
    ├── base_driver.h
    ├── crc_table.h
    ├── data_struct.h
    └── fdilink_data_struct.h
```

- `ucar_controller/`: 通常会有一个与包名相同的子文件夹，用于存放该包特有的头文件，以避免命名冲突。
  - `base_driver.h`: 可能包含机器人底层驱动的类定义和接口声明。
  - `crc_table.h`: 可能包含 CRC 校验相关的函数声明和常量定义。
  - `data_struct.h`: 可能包含通用的数据结构定义。
  - `fdilink_data_struct.h`: 可能包含与特定通信协议（如 FDILink）相关的数据结构定义。

## 3. 主要功能与用途

`include` 文件夹的主要功能是：

- **声明接口**：定义 C++ 类和函数的公共接口，供其他源文件或包使用。
- **代码重用**：将常用的函数、类和数据结构定义在头文件中，方便在多个源文件中包含和使用，避免重复编写代码。
- **编译优化**：通过头文件预编译（Precompiled Headers）等技术，可以加速大型项目的编译过程。
- **模块化**：促进代码的模块化设计，将不同功能的声明分离到不同的头文件中，提高代码的组织性。

## 4. 使用方法

- **头文件创建**：
  在 `src/ucar_controller/include/ucar_controller/` 目录下创建新的 `.h` 或 `.hpp` 文件，并按照 C++ 规范编写类定义、函数声明等。例如：
  ```cpp
  // my_utility.h
  #ifndef MY_UTILITY_H
  #define MY_UTILITY_H

  #include <string>

  namespace ucar_controller
  {
      class MyUtility
      {
      public:
          static std::string greet(const std::string& name);
      };
  }

  #endif // MY_UTILITY_H
  ```

- **在源文件中包含**：
  在需要使用头文件中声明的类或函数的 `.cpp` 源文件中，使用 `#include` 指令包含相应的头文件。例如：
  ```cpp
  // my_node.cpp
  #include <ros/ros.h>
  #include <ucar_controller/my_utility.h> // 包含自定义头文件

  int main(int argc, char** argv)
  {
      ros::init(argc, argv, "my_node");
      ROS_INFO("Greeting: %s", ucar_controller::MyUtility::greet("World").c_str());
      return 0;
  }
  ```

- **`CMakeLists.txt` 配置**：
  为了让编译器能够找到 `include` 文件夹中的头文件，需要在 `ucar_controller` 包的 `CMakeLists.txt` 中配置 `include_directories`。通常，`catkin_package` 宏会自动处理大部分情况，但手动添加可以确保：
  ```cmake
  # 在 find_package(catkin REQUIRED COMPONENTS ...) 之后
  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
  )

  # 在 catkin_package() 中声明头文件目录，以便其他包可以使用
  catkin_package(
    INCLUDE_DIRS include
    # ... 其他参数
  )
  ```

## 5. 项目全局应用

在 `ucar_ws` 项目中，`ucar_controller` 包的 `include` 文件夹是实现代码复用和模块化设计的关键。它定义了机器人控制模块的公共接口和数据结构，使得其他 ROS 包（如导航、规划）能够通过这些接口与 `ucar_controller` 进行高效、稳定的交互。这种清晰的接口定义有助于构建一个可扩展、易于维护的机器人软件架构。

## 6. 维护与更新

- **头文件保护**：所有头文件都应包含 `#ifndef/#define/#endif` 或 `#pragma once` 宏，以防止重复包含。
- **最小化依赖**：头文件应尽量只包含其声明所需的最小依赖，避免不必要的 `#include`。
- **命名空间**：使用命名空间来组织代码，避免全局命名冲突。
- **注释**：为公共接口、类和函数添加详细的 Doxygen 风格注释，方便生成文档。
- **版本控制**：将头文件纳入版本控制，确保团队成员使用最新版本。

## 7. 故障排除

- **头文件找不到**：
  - 检查 `CMakeLists.txt` 中 `include_directories` 是否正确配置，路径是否正确。
  - 确保 `#include` 路径与文件实际路径匹配（例如，`#include <ucar_controller/base_driver.h>` 而不是 `#include <base_driver.h>`）。
  - 编译时查看错误信息，通常会指出找不到的头文件。
- **重复定义错误**：
  - 检查头文件是否包含头文件保护宏（`#ifndef/#define/#endif` 或 `#pragma once`）。
  - 确保没有在 `.cpp` 文件中错误地 `#include` 了 `.cpp` 文件。
- **链接错误**：
  - 如果头文件中声明的函数或类在 `.cpp` 文件中没有实现，或者实现文件没有被编译和链接，会导致链接错误。确保所有相关的 `.cpp` 文件都已添加到 `CMakeLists.txt` 的 `add_library` 或 `add_executable` 中。