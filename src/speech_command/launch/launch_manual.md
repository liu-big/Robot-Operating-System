# `launch` 文件夹使用开发操作手册

## 1. 概述

在 ROS (Robot Operating System) 项目中，`launch` 文件夹是存放 `.launch` 文件的标准位置。`.launch` 文件是 XML 格式的配置文件，用于定义和启动一个或多个 ROS 节点、设置参数、加载机器人描述、以及执行其他 ROS 相关的操作。它们是组织和管理复杂 ROS 系统启动过程的核心工具。

## 2. 文件夹结构

`src/speech_command/launch/`

- `src/speech_command/launch/speech_command.launch`: 主启动文件，用于启动 `speech_command` 包中的所有相关节点。
- `src/speech_command/launch/test_audio.launch`: 可能用于测试音频输入/输出的独立启动文件。
- `src/speech_command/launch/record_speech.launch`: 可能用于启动语音录制节点的启动文件。
- ... (可能包含其他 `.launch` 文件)

## 3. 主要功能与用途

`launch` 文件夹的主要功能是：

- **节点启动**：同时启动一个或多个 ROS 节点，无需手动逐个运行 `rosrun`。
- **参数设置**：在节点启动时为其设置参数，这些参数会被加载到 ROS 参数服务器。
- **话题重映射**：改变节点发布或订阅的话题名称，以适应不同的系统配置。
- **节点分组**：将相关的节点组织到命名空间中，避免命名冲突。
- **包含其他 launch 文件**：通过 `include` 标签复用其他 `.launch` 文件，构建模块化的启动配置。
- **加载机器人描述**：加载 URDF (Unified Robot Description Format) 或 XACRO (XML Macro) 文件，用于机器人模型的可视化和仿真。
- **条件启动**：根据条件（如参数值）决定是否启动某个节点或加载某个文件。

## 4. 使用方法

- **创建 `.launch` 文件**：使用文本编辑器创建 XML 格式的 `.launch` 文件。

  示例 `speech_command.launch`：
  ```xml
  <launch>
      <!-- 加载语音识别参数 -->
      <rosparam command="load" file="$(find speech_command)/config/speech_recognition.yaml" />

      <!-- 启动音频输入节点 -->
      <node pkg="audio_common" type="audio_capture_node" name="audio_input" output="screen">
          <param name="format" value="wave"/>
          <param name="channels" value="1"/>
          <param name="sample_rate" value="16000"/>
          <param name="device" value="default"/>
          <remap from="audio" to="/audio_in"/>
      </node>

      <!-- 启动语音命令处理节点 -->
      <node pkg="speech_command" type="speech_command_node" name="speech_processor" output="screen">
          <param name="~confidence_threshold" value="0.8"/>
          <remap from="recognized_command" to="/robot_command"/>
      </node>

      <!-- 启动语音反馈节点 -->
      <node pkg="speech_command" type="voice_play.py" name="voice_feedback" output="screen"/>

      <!-- 可选：启动 RViz 进行可视化 -->
      <!-- <node name="rviz" pkg="rviz" type="rviz" args="-d $(find speech_command)/rviz/speech_command.rviz" /> -->
  </launch>
  ```

- **运行 `.launch` 文件**：使用 `roslaunch` 命令来运行 `.launch` 文件。

  ```bash
  roslaunch speech_command speech_command.launch
  ```

## 5. 项目全局应用

在 `ucar_ws` 项目中，`speech_command` 包的 `launch` 文件夹是整个语音控制系统启动和协调的核心。它定义了所有必要的 ROS 节点（如音频捕获、语音处理、语音反馈）如何协同工作，以及它们所需的参数和话题连接。通过 `.launch` 文件，可以一键启动整个语音命令系统，极大地简化了系统的部署和测试过程，并确保了各组件之间的正确交互。

## 6. 维护与更新

- **模块化**：尽量将大型 `.launch` 文件拆分为多个小的、可复用的文件，并通过 `include` 标签组合。
- **注释**：为 `.launch` 文件添加清晰的注释，说明每个节点、参数和重映射的用途。
- **参数化**：使用 `arg` 标签使 `.launch` 文件更具通用性，可以通过命令行传递参数。
- **版本控制**：将 `.launch` 文件提交到版本控制系统。

## 7. 故障排除

- **节点未启动**：
  - 检查 `.launch` 文件中 `node` 标签的 `pkg` 和 `type` 属性是否正确。
  - 确认对应的可执行文件（在 `bin` 或 `scripts` 目录下）是否存在且具有执行权限。
  - 查看 `roslaunch` 的输出，通常会有错误信息提示。
- **参数未生效**：
  - 检查 `param` 或 `rosparam` 标签的名称和值是否正确。
  - 确认参数是否已加载到参数服务器（`rosparam get /param_name`）。
- **话题连接问题**：
  - 使用 `rostopic list` 和 `rostopic info <topic_name>` 检查话题是否正确发布和订阅。
  - 检查 `remap` 标签是否正确配置。
- **XML 格式错误**：
  - 使用 XML 验证工具检查 `.launch` 文件的语法。
  - 仔细检查标签是否闭合，属性是否正确。