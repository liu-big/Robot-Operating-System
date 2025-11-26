# `include` 文件夹使用开发操作手册

## 1. 概述

在 C++ 项目中，`include` 文件夹（或 `inc`）是存放头文件（`.h` 或 `.hpp` 文件）的约定俗成的位置。头文件包含了函数、类、结构体、宏、常量等的声明，供其他源文件（`.cpp` 文件）引用。这种分离声明和定义的方式有助于模块化编程、提高编译效率，并方便代码的重用和管理。

## 2. 文件夹结构

`src/speech_command/include/`

- `src/speech_command/include/speech_command/`: 通常会有一个与包名相同的子文件夹，用于存放该包特有的头文件，避免命名冲突。
  - `src/speech_command/include/speech_command/speech_processor.h`: 声明语音处理相关的类和函数。
  - `src/speech_command/include/speech_command/command_parser.h`: 声明命令解析相关的类和函数。
- ... (可能包含其他头文件)

## 3. 主要功能与用途

`include` 文件夹的主要功能是：

- **声明接口**：定义模块对外提供的接口，包括函数原型、类定义、常量等。
- **代码重用**：允许在多个源文件中包含相同的头文件，从而重用声明的代码。
- **编译优化**：通过头文件，编译器可以预先知道函数和类的结构，从而进行更有效的编译。
- **模块化**：将不同功能的声明分离到不同的头文件中，提高代码的可读性和可维护性。

## 4. 使用方法

- **头文件创建**：在 `include/speech_command/` 目录下创建 `.h` 或 `.hpp` 文件，并在其中声明函数、类等。

  示例 `speech_processor.h`：
  ```cpp
  #ifndef SPEECH_PROCESSOR_H
  #define SPEECH_PROCESSOR_H

  #include <string>
  #include <vector>

  namespace speech_command {

  class SpeechProcessor {
  public:
      SpeechProcessor();
      ~SpeechProcessor();

      bool processAudio(const std::vector<short>& audio_data);
      std::string getRecognizedCommand() const;

  private:
      // Internal members
  };

  } // namespace speech_command

  #endif // SPEECH_PROCESSOR_H
  ```

- **在源文件中包含**：在 `.cpp` 源文件中使用 `#include` 指令包含所需的头文件。

  示例 `speech_processor.cpp`：
  ```cpp
  #include <speech_command/speech_processor.h> // 包含头文件
  #include <iostream>

  namespace speech_command {

  SpeechProcessor::SpeechProcessor() {
      // Constructor implementation
  }

  SpeechProcessor::~SpeechProcessor() {
      // Destructor implementation
  }

  bool SpeechProcessor::processAudio(const std::vector<short>& audio_data) {
      // Audio processing logic
      std::cout << "Processing audio data..." << std::endl;
      return true;
  }

  std::string SpeechProcessor::getRecognizedCommand() const {
      return "hello robot";
  }

  } // namespace speech_command
  ```

- **`CMakeLists.txt` 配置**：在 `CMakeLists.txt` 中，需要告诉编译器在哪里查找头文件。对于 ROS 包，通常使用 `include_directories`。

  示例 `CMakeLists.txt` 片段：
  ```cmake
  # ...
  # 指定头文件目录
  include_directories(
    ${catkin_INCLUDE_DIRS}
    include
  )

  # ...
  # 添加可执行文件或库时，链接到头文件
  add_executable(speech_command_node src/speech_command_node.cpp)
  target_link_libraries(speech_command_node
    ${catkin_LIBRARIES}
  )
  ```

## 5. 项目全局应用

在 `ucar_ws` 项目中，`speech_command` 包的 `include` 文件夹是构建模块化和可维护 C++ 代码的基础。它定义了语音处理、命令解析等核心功能的接口，使得不同的 `.cpp` 文件可以共享这些定义，并确保了代码的一致性和可扩展性。这对于复杂的机器人软件系统至关重要，因为它促进了代码的组织和团队协作。

## 6. 维护与更新

- **头文件保护**：使用 `#ifndef/#define/#endif` 或 `#pragma once` 来防止头文件被重复包含。
- **最小化依赖**：头文件中应只包含必要的声明，避免引入不必要的依赖。
- **命名空间**：使用命名空间来避免全局命名冲突。
- **文档注释**：为头文件中的类、函数和变量添加清晰的文档注释。

## 7. 故障排除

- **头文件找不到**（`No such file or directory`）：
  - 检查 `#include` 路径是否正确。
  - 确认 `CMakeLists.txt` 中 `include_directories` 是否包含了正确的路径。
  - 确保头文件确实存在于指定路径。
- **重复定义错误**（`redefinition`）：
  - 检查头文件是否使用了头文件保护宏（`#ifndef/#define/#endif`）。
  - 确保没有在多个源文件中定义了相同的全局变量或函数（定义应该在 `.cpp` 文件中）。
- **链接错误**（`undefined reference`）：
  - 这通常意味着函数或类的定义（在 `.cpp` 文件中）没有被编译或链接到可执行文件。检查 `CMakeLists.txt` 中是否正确添加了源文件和链接库。