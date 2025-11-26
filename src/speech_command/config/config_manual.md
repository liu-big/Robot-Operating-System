# `config` 文件夹使用开发操作手册

## 1. 概述

在 ROS (Robot Operating System) 项目中，`config` 文件夹通常用于存放各种配置文件。这些文件定义了 ROS 节点、算法或整个系统运行所需的参数、映射、模型、规则等。将配置与代码分离是一种良好的实践，它使得系统更易于配置、维护和部署，无需修改和重新编译代码即可调整系统行为。

## 2. 文件夹结构

`src/speech_command/config/`

- `src/speech_command/config/params.yaml`: 存储节点参数，如阈值、增益、设备ID等。
- `src/speech_command/config/speech_recognition.yaml`: 语音识别相关的配置，如模型路径、语言设置、关键词列表等。
- `src/speech_command/config/audio_device.yaml`: 音频设备相关的配置，如采样率、通道数、设备名称等。
- `src/speech_command/config/command_mapping.yaml`: 语音命令到机器人动作的映射配置。
- ... (可能包含其他 `.yaml`, `.xml`, `.json` 等格式的配置文件)

## 3. 主要功能与用途

`config` 文件夹的主要功能是：

- **参数管理**：集中管理 ROS 节点的运行参数，方便在不修改代码的情况下调整系统行为。
- **模块配置**：为特定的功能模块（如语音识别、音频处理）提供详细的配置选项。
- **数据映射**：定义不同数据或命令之间的映射关系，例如语音指令与机器人动作的对应。
- **环境适应**：通过修改配置文件，使系统能够适应不同的硬件环境或应用场景。

## 4. 使用方法

- **创建和编辑配置文件**：使用文本编辑器创建或修改 `.yaml`、`.xml` 等格式的配置文件。YAML 是 ROS 中最常用的参数配置文件格式。

- **在 `launch` 文件中加载**：通常通过 ROS 的 `rosparam` 或 `param` 标签在 `launch` 文件中加载配置文件。

  示例 `launch` 文件片段（加载 `speech_recognition.yaml`）：
  ```xml
  <launch>
      <!-- 加载语音识别参数 -->
      <rosparam command="load" file="$(find speech_command)/config/speech_recognition.yaml" />

      <node pkg="speech_command" type="speech_command_node" name="speech_node" output="screen">
          <!-- 也可以单独加载某个参数 -->
          <param name="audio_topic" value="/audio_in" />
      </node>
  </launch>
  ```

- **在代码中读取参数**：ROS 节点可以通过 `ros::param::get()` (C++) 或 `rospy.get_param()` (Python) 等 API 读取加载的参数。

  示例 Python 代码片段：
  ```python
  import rospy

  def init_node():
      rospy.init_node('my_speech_node')
      # 从参数服务器获取参数
      model_path = rospy.get_param('~speech_model_path', '/default/path/to/model')
      confidence_threshold = rospy.get_param('~confidence_threshold', 0.7)
      rospy.loginfo(f"Loaded speech model from: {model_path}")
      rospy.loginfo(f"Confidence threshold: {confidence_threshold}")
      # ... 使用参数进行初始化

  if __name__ == '__main__'
      init_node()
      rospy.spin()
  ```

## 5. 项目全局应用

在 `ucar_ws` 项目中，`speech_command` 包的 `config` 文件夹是实现其灵活性和可配置性的关键。它允许开发者和用户在不修改和重新编译源代码的情况下，调整语音识别的灵敏度、命令映射、音频处理参数等。这对于快速迭代、适应不同应用场景以及在部署后进行微调至关重要，极大地提高了系统的可用性和适应性。

## 6. 维护与更新

- **版本控制**：`config` 文件夹中的所有配置文件都应该被提交到版本控制系统（如 Git），以便团队成员共享和跟踪配置变更。
- **注释清晰**：在配置文件中添加详细的注释，说明每个参数的含义、取值范围和默认值。
- **参数校验**：在代码中读取参数时，最好进行参数校验，确保参数的有效性。
- **避免硬编码**：尽量将所有可配置的参数都放入配置文件中，避免在代码中硬编码。

## 7. 故障排除

- **参数未加载**：
  - 检查 `launch` 文件中 `rosparam` 或 `param` 标签的 `file` 路径是否正确。
  - 确保 `launch` 文件被正确执行。
  - 检查配置文件本身的语法是否正确（例如 YAML 格式）。
- **节点未读取到参数**：
  - 检查代码中 `get_param` 的参数名是否与配置文件中的键名一致。
  - 确认参数是否已成功加载到 ROS 参数服务器（可以使用 `rosparam list` 和 `rosparam get <param_name>` 命令检查）。
- **配置错误导致系统行为异常**：
  - 仔细检查修改过的配置文件，特别是数值型参数的取值范围。
  - 尝试恢复到上一个已知可用的配置版本进行对比。