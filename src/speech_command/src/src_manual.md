# `src` 文件夹使用开发操作手册

## 1. 概述

在 ROS (Robot Operating System) 项目中，`src` 文件夹是存放源代码的约定俗成的位置。它包含了实现 ROS 节点、库、服务、消息等核心功能的 C++ 或 Python 源文件。这个文件夹是项目开发的主体，所有的业务逻辑和算法实现都将在这里进行。

## 2. 文件夹结构

`src/speech_command/src/`

- `src/speech_command/src/speech_command_node.cpp`: C++ 实现的 ROS 节点，负责语音命令的核心处理逻辑。
- `src/speech_command/src/voice_play.py`: Python 实现的脚本，可能用于播放语音反馈或处理音频。
- `src/speech_command/src/speech_processor.cpp`: C++ 源文件，包含语音处理算法的实现。
- `src/speech_command/src/command_parser.cpp`: C++ 源文件，包含命令解析逻辑的实现。
- ... (可能包含其他 `.cpp`, `.py` 等格式的源文件)

## 3. 主要功能与用途

`src` 文件夹的主要功能是：

- **ROS 节点实现**：编写 ROS 节点（Publisher, Subscriber, Service Server, Service Client, Action Server, Action Client）的源代码。
- **库的实现**：实现可供其他节点或模块调用的 C++ 类和函数库。
- **复杂逻辑**：包含机器人控制、感知、规划等复杂算法和业务逻辑的实现。
- **硬件交互**：编写与机器人硬件（如麦克风、扬声器）进行交互的代码。

## 4. 使用方法

- **创建源文件**：在 `src` 目录下创建 `.cpp`（C++）或 `.py`（Python）文件，编写您的代码。

  示例 `speech_command_node.cpp` 骨架：
  ```cpp
  #include <ros/ros.h>
  #include <std_msgs/String.h>
  #include <speech_command/speech_processor.h> // 假设有自定义头文件

  int main(int argc, char **argv)
  {
      ros::init(argc, argv, "speech_command_node");
      ros::NodeHandle nh;

      // 创建一个发布者，发布识别到的命令
      ros::Publisher command_pub = nh.advertise<std_msgs::String>("recognized_command", 10);

      // 创建一个订阅者，订阅音频数据
      // ros::Subscriber audio_sub = nh.subscribe("audio_in", 10, audioCallback);

      speech_command::SpeechProcessor processor;

      ros::Rate loop_rate(10); // 10 Hz

      while (ros::ok())
      {
          // 模拟语音处理和命令识别
          // processor.processAudio(audio_data);
          std::string command = processor.getRecognizedCommand();

          if (!command.empty()) {
              std_msgs::String msg;
              msg.data = command;
              command_pub.publish(msg);
              ROS_INFO("Recognized command: %s", msg.data.c_str());
          }

          ros::spinOnce();
          loop_rate.sleep();
      }

      return 0;
  }
  ```

- **`CMakeLists.txt` 配置**：对于 C++ 源文件，需要在 `CMakeLists.txt` 中配置编译规则，将其编译成可执行文件或库。

  示例 `CMakeLists.txt` 片段：
  ```cmake
  # ...
  # 添加可执行文件
  add_executable(speech_command_node src/speech_command_node.cpp)

  # 链接必要的库
  target_link_libraries(speech_command_node
    ${catkin_LIBRARIES}
    # 如果有自定义库，例如 speech_utils
    # speech_utils
  )

  # ...
  ```

- **在 `launch` 文件中启动**：编译后的可执行文件或 Python 脚本通常通过 `launch` 文件启动。

  示例 `launch` 文件片段：
  ```xml
  <launch>
      <node pkg="speech_command" type="speech_command_node" name="my_speech_node" output="screen"/>
      <node pkg="speech_command" type="voice_play.py" name="voice_player" output="screen"/>
  </launch>
  ```

## 5. 项目全局应用

在 `ucar_ws` 项目中，`speech_command` 包的 `src` 文件夹是实现机器人语音交互功能的核心。它包含了实际执行语音识别、命令解析、以及与机器人其他模块（如导航、运动）进行通信的代码。这个文件夹的代码质量和逻辑直接决定了机器人语音控制系统的性能和可靠性，是整个包的“大脑”。

## 6. 维护与更新

- **代码规范**：遵循 C++ 或 Python 的编码规范，保持代码风格一致性。
- **模块化**：将复杂功能拆分为小的、独立的函数或类，提高可读性和可维护性。
- **注释**：为关键代码段、函数和类添加详细的注释。
- **版本控制**：将所有源文件提交到版本控制系统。
- **单元测试**：为核心功能编写单元测试，确保代码的正确性。

## 7. 故障排除

- **编译错误**：
  - 检查代码语法错误。
  - 确认所有头文件都已正确包含，并且 `CMakeLists.txt` 中的 `include_directories` 设置正确。
  - 检查库链接是否正确。
- **运行时错误**：
  - 检查 ROS 节点日志输出（`output="screen"` 或 `roslog`）。
  - 使用调试器（如 GDB）进行调试。
  - 检查话题、服务、参数名称是否匹配。
- **Python 脚本执行问题**：
  - 确保脚本具有执行权限（`chmod +x script.py`）。
  - 检查 Python 依赖是否安装。
  - 检查 `#!/usr/bin/env python` 或 `#!/usr/bin/env python3` 是否正确。