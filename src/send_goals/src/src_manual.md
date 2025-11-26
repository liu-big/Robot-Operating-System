# `src` 文件夹使用开发操作手册

## 1. 概述

在 `send_goals` ROS 包中，`src` 文件夹包含了 ROS 节点的 C++ 源代码。这些文件实现了包的核心功能，例如获取导航目标点、发送导航目标以及处理与 PID 控制相关的 TF 变换。作为编译后的可执行文件，它们提供了高性能和稳定的机器人控制和导航接口，是 `send_goals` 包功能实现的基础。

## 2. 文件夹结构

```
src/send_goals/src/
├── get_nav_poinet.cpp
├── send_goal_node.cpp
└── tf_stop_pid.cpp
```

- `get_nav_poinet.cpp`: 这个文件可能实现了获取导航目标点的逻辑，例如通过订阅 RViz 中的 2D Pose Estimate 或 2D Nav Goal 消息，或者通过其他用户界面获取目标点。
- `send_goal_node.cpp`: 这是主要的导航目标发送节点。它会创建一个 ROS 节点，负责接收导航目标，并将其封装成 `move_base_msgs/MoveBaseActionGoal` 消息发送给 ROS 导航栈的 `move_base` 动作服务器。
- `tf_stop_pid.cpp`: 这个文件可能实现了与 PID 控制相关的 TF 变换逻辑，例如在机器人到达目标点后，通过发布特定的 TF 变换来触发或停止 PID 控制，以实现精确停车或姿态保持。

## 3. 主要功能与用途

`src` 文件夹的主要功能是：

- **导航目标获取与发送**：实现从用户或其他模块获取导航目标，并将其发送给 `move_base` 导航栈，驱动机器人自主移动。
- **机器人运动控制**：通过与 PID 控制相关的 TF 变换，实现对机器人运动的精细控制，例如在目标点附近的精确停车。
- **ROS 节点实现**：作为编译后的可执行文件，提供高效、稳定的机器人控制和导航接口。
- **参数配置**：通过 ROS 参数服务器加载和管理节点运行所需的参数。

## 4. 使用方法

- **编译**：
  `src` 文件夹中的 C++ 源文件会通过 `send_goals` 包的 `CMakeLists.txt` 进行编译。在 ROS 工作空间的根目录下执行 `catkin_make` 或 `colcon build` 命令即可编译整个包，包括这些源文件。
  ```bash
  cd ~/ucar_ws
  catkin_make
  # 或者 colcon build
  ```
- **运行节点**：
  编译成功后，可以通过 `rosrun` 命令直接运行节点，或者通过 `roslaunch` 文件来启动节点。
  ```bash
  rosrun send_goals send_goal_node
  # 或者通过 launch 文件启动
  # roslaunch send_goals send_goal_node.launch # 假设存在此 launch 文件
  ```
- **修改代码**：
  如果需要修改导航目标发送、目标点获取或 PID 控制逻辑，可以直接编辑 `get_nav_poinet.cpp`、`send_goal_node.cpp` 或 `tf_stop_pid.cpp`。修改后需要重新编译才能生效。

## 5. 项目全局应用

在 `ucar_ws` 项目中，`send_goals` 包的 `src` 文件夹是实现机器人自主导航和精确控制的核心。它提供了与 ROS 导航栈交互的 C++ 接口，确保了导航指令的及时响应和高效执行。同时，其包含的 PID 控制相关逻辑也为机器人在复杂环境下的精确操作提供了保障，是构建高性能机器人系统的关键组成部分。

## 6. 维护与更新

- **代码审查**：定期审查代码，确保其符合 ROS 最佳实践和编码规范。
- **性能优化**：根据实际需求和机器人硬件，对导航目标处理和控制逻辑进行优化，减少延迟和提高精度。
- **新功能开发**：根据新的导航策略或控制需求，更新或扩展节点的功能。
- **依赖管理**：确保代码所依赖的 ROS 库和消息类型版本兼容。

## 7. 故障排除

- **节点无法启动**：
  - 检查编译是否成功，是否有编译错误或警告。
  - 确认 `CMakeLists.txt` 中是否正确添加了可执行文件和依赖库。
  - 检查 `rosrun` 或 `roslaunch` 命令中的节点名称是否正确。
- **导航目标无法发送**：
  - 确认 `move_base` 导航栈是否正在运行，并且其动作服务器已启动。
  - 检查 `send_goal_node.cpp` 中发送目标的代码逻辑，确保目标消息的字段（如 `target_pose.header.frame_id`）正确。
  - 使用 `rostopic list` 和 `rostopic info /move_base/goal` 检查话题是否正常。
- **PID 控制异常**：
  - 检查 `tf_stop_pid.cpp` 中发布 TF 变换的逻辑是否正确，以及是否被其他控制模块正确订阅。
  - 确认 PID 控制器本身是否正常工作，并接收到正确的输入。