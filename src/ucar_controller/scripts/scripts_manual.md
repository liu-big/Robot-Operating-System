# `scripts` 文件夹使用开发操作手册

## 1. 概述

在 `ucar_controller` ROS 包中，`scripts` 文件夹用于存放可执行的脚本文件，通常是 Python 脚本。这些脚本可以用于实现 ROS 节点、辅助工具、数据处理、测试或自动化任务等。与 C++ 编写的节点相比，Python 脚本在快速原型开发、数据处理和与 ROS 系统的交互方面具有优势。

## 2. 文件夹结构

```
src/ucar_controller/scripts/
├── __init__.py
├── odom_ekf.py
└── sensor_tf_server.py
```

- `__init__.py`: 这是一个空的 Python 文件，它的存在使得 `scripts` 文件夹被 Python 解释器视为一个包（package），从而允许在其他 Python 模块中导入 `scripts` 文件夹内的内容。
- `odom_ekf.py`: 可能实现了一个 ROS 节点，用于执行里程计和扩展卡尔曼滤波（EKF）的融合，以提供更精确的机器人位姿估计。
- `sensor_tf_server.py`: 可能实现了一个 ROS 节点，用于发布传感器相关的 TF（坐标变换）信息，确保机器人各个部件的坐标系正确发布。

## 3. 主要功能与用途

`scripts` 文件夹的主要功能是：

- **ROS 节点实现**：编写 Python 脚本作为 ROS 节点，处理传感器数据、发布控制指令、执行算法等。
- **辅助工具**：提供一些辅助性的脚本，例如数据转换、日志分析、系统监控等。
- **快速原型开发**：利用 Python 的灵活性和丰富的库，快速验证新的算法或功能。
- **自动化任务**：编写脚本自动化一些重复性的任务，如测试、部署或数据收集。
- **与 ROS 系统的交互**：通过 `rospy` 库与 ROS 话题、服务、参数服务器等进行交互。

## 4. 使用方法

- **创建 Python 脚本**：
  在 `scripts` 文件夹中创建 `.py` 文件，并在文件开头添加 shebang（`#!/usr/bin/env python` 或 `#!/usr/bin/env python3`）以指定解释器。
  ```python
  #!/usr/bin/env python
  # -*- coding: utf-8 -*-

  import rospy
  from nav_msgs.msg import Odometry
  from geometry_msgs.msg import TransformStamped
  import tf

  def odom_ekf_node():
      rospy.init_node('odom_ekf_node', anonymous=True)
      # ... 节点逻辑 ...
      rospy.spin()

  if __name__ == '__main__':
      try:
          odom_ekf_node()
      except rospy.ROSInterruptException:
          pass
  ```

- **赋予执行权限**：
  在 Linux/macOS 系统中，需要为脚本文件添加执行权限：
  ```bash
  chmod +x src/ucar_controller/scripts/odom_ekf.py
  ```
  在 Windows 系统中，通常不需要显式设置执行权限，但确保 Python 解释器已正确安装并配置到 PATH 环境变量中。

- **在 `CMakeLists.txt` 中配置**：
  为了让 ROS 能够找到并安装这些脚本，需要在 `ucar_controller` 包的 `CMakeLists.txt` 中添加配置：
  ```cmake
  # ...
  # 安装 Python 脚本
  install(PROGRAMS scripts/odom_ekf.py
                   scripts/sensor_tf_server.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
  # ...
  ```
  `CATKIN_PACKAGE_BIN_DESTINATION` 通常会解析到 `install/ucar_controller/lib/ucar_controller/` 目录下。

- **在 `launch` 文件中启动**：
  在 `launch` 文件夹中的 `.launch` 文件中，可以使用 `<node>` 标签启动 Python 脚本：
  ```xml
  <!-- 启动 odom_ekf.py 节点 -->
  <node pkg="ucar_controller" type="odom_ekf.py" name="odom_ekf_node" output="screen" />

  <!-- 启动 sensor_tf_server.py 节点 -->
  <node pkg="ucar_controller" type="sensor_tf_server.py" name="sensor_tf_server_node" output="screen" />
  ```

## 5. 项目全局应用

在 `ucar_ws` 项目中，`ucar_controller` 包的 `scripts` 文件夹承载了部分核心的 ROS 节点功能，特别是那些需要快速迭代或利用 Python 生态系统优势的模块。例如，`odom_ekf.py` 和 `sensor_tf_server.py` 这样的脚本，通过 Python 的灵活性，能够方便地处理数据流、实现复杂的逻辑并与 ROS 系统无缝集成。这使得 `ucar_controller` 能够更高效地进行开发和调试，同时保持了代码的模块化和可维护性。

## 6. 维护与更新

- **代码规范**：遵循 PEP 8 等 Python 编码规范，保持代码风格一致性。
- **注释和文档**：为复杂的逻辑和函数添加详细的注释和 docstrings。
- **错误处理**：在脚本中加入适当的错误处理机制，提高程序的健壮性。
- **依赖管理**：明确脚本所需的 Python 库依赖，并在 `package.xml` 中声明。
- **版本控制**：所有脚本文件都应该被提交到版本控制系统（如 Git）。
- **测试**：为关键脚本编写单元测试或集成测试。

## 7. 故障排除

- **脚本无法运行**：
  - 检查脚本文件是否具有执行权限（`chmod +x`）。
  - 检查 shebang (`#!/usr/bin/env python`) 是否正确，并且 Python 解释器路径正确。
  - 检查 `CMakeLists.txt` 中 `install(PROGRAMS ...)` 是否正确配置。
  - 确保 `roslaunch` 命令中的 `pkg` 和 `type` 属性正确。
- **Python 依赖问题**：
  - 确保所有必要的 Python 库已安装（`pip install <package_name>`）。
  - 检查 ROS 环境是否正确设置，`rospy` 等 ROS 库是否可用。
- **节点启动后立即退出**：
  - 检查脚本中是否有未捕获的异常，查看 `output="screen"` 的输出信息。
  - 确保 ROS 节点名称唯一，没有冲突。
  - 检查话题订阅和发布是否正确，消息类型是否匹配。