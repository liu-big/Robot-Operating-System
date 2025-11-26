# `log_info` 文件夹使用开发操作手册

## 1. 概述

在 `ucar_controller` ROS 包中，`log_info` 文件夹用于存放与机器人运行相关的日志信息和数据记录。这些文件通常包含里程计数据、传感器读数、控制指令、系统状态等，对于机器人的调试、性能分析、行为回放以及故障诊断至关重要。通过分析这些日志，开发者可以深入了解机器人的运行情况，优化算法和参数。

## 2. 文件夹结构

```
src/ucar_controller/log_info/
├── car_Mileage_info.txt
└── car_Mileage_info.txt.bp
```

- `car_Mileage_info.txt`: 可能包含机器人运行过程中记录的里程计信息，如时间戳、位置、姿态、线速度、角速度等。
- `car_Mileage_info.txt.bp`: 可能是 `car_Mileage_info.txt` 的备份文件或历史记录文件。

这些文件通常以文本格式存储，每行记录一个数据点，字段之间可能用逗号、空格或制表符分隔。

## 3. 主要功能与用途

`log_info` 文件夹的主要功能是：

- **数据记录**：自动记录机器人运行时的关键数据，如里程计、传感器数据、控制输出等。
- **调试辅助**：通过回放和分析历史数据，帮助开发者诊断机器人行为异常或算法问题。
- **性能分析**：评估机器人控制算法的性能，如定位精度、路径跟踪误差等。
- **行为回放**：在仿真环境中重现机器人的历史行为，用于测试和验证。
- **故障诊断**：在机器人出现故障时，通过日志文件追溯问题发生的原因和过程。

## 4. 使用方法

- **查看日志文件**：
  可以直接使用文本编辑器打开 `.txt` 文件查看其内容。对于大型日志文件，可以使用命令行工具（如 `less`、`tail`、`grep`）进行查看和过滤。
  ```bash
  # 查看文件内容
  cat src/ucar_controller/log_info/car_Mileage_info.txt

  # 实时查看文件末尾内容
  tail -f src/ucar_controller/log_info/car_Mileage_info.txt

  # 查找特定信息
  grep "error" src/ucar_controller/log_info/car_Mileage_info.txt
  ```

- **数据分析**：
  可以将日志数据导入到数据分析工具（如 Python 的 Pandas、Matplotlib，MATLAB，Excel）中进行可视化和统计分析。

  Python 示例（读取并绘制里程计数据）：
  ```python
  import pandas as pd
  import matplotlib.pyplot as plt

  # 假设日志文件是 CSV 格式，包含 time, x, y, theta, vx, vy, vtheta
  # 请根据实际文件格式调整分隔符和列名
  try:
      df = pd.read_csv('d:/比赛专用/ucar_ws/src/ucar_controller/log_info/car_Mileage_info.txt', sep=',', header=None,
                       names=['time', 'x', 'y', 'theta', 'vx', 'vy', 'vtheta'])

      plt.figure(figsize=(10, 6))
      plt.plot(df['x'], df['y'], label='Robot Trajectory')
      plt.xlabel('X Position (m)')
      plt.ylabel('Y Position (m)')
      plt.title('Robot Trajectory from Mileage Info')
      plt.grid(True)
      plt.legend()
      plt.show()

      plt.figure(figsize=(10, 6))
      plt.plot(df['time'], df['vx'], label='Linear Velocity X')
      plt.plot(df['time'], df['vtheta'], label='Angular Velocity Theta')
      plt.xlabel('Time (s)')
      plt.ylabel('Velocity')
      plt.title('Robot Velocities from Mileage Info')
      plt.grid(True)
      plt.legend()
      plt.show()

  except FileNotFoundError:
      print("Error: car_Mileage_info.txt not found. Please check the path.")
  except Exception as e:
      print(f"An error occurred: {e}")
  ```

- **在代码中写入日志**：
  在 C++ 或 Python 代码中，可以使用标准的文件操作函数将数据写入这些日志文件。例如：

  Python 示例：
  ```python
  import rospy
  import os

  log_file_path = os.path.join(os.path.dirname(__file__), '..', 'log_info', 'car_Mileage_info.txt')

  def write_mileage_log(x, y, theta, vx, vy, vtheta):
      with open(log_file_path, 'a') as f:
          timestamp = rospy.get_time()
          f.write(f"{timestamp},{x},{y},{theta},{vx},{vy},{vtheta}\n")

  # 示例调用
  # write_mileage_log(1.0, 2.0, 0.5, 0.1, 0.0, 0.05)
  ```

## 5. 项目全局应用

在 `ucar_ws` 项目中，`ucar_controller` 包的 `log_info` 文件夹是机器人运行数据的重要存储库。它为开发者提供了宝贵的历史数据，用于验证控制算法的有效性、诊断潜在问题以及进行性能优化。这些日志文件是机器人系统迭代开发和持续改进的基础，确保了机器人能够不断提升其运动控制和感知能力。

## 6. 维护与更新

- **日志轮转**：对于长时间运行的机器人，应考虑实现日志轮转机制，防止日志文件过大占用过多磁盘空间。
- **数据格式**：保持日志文件的数据格式一致性，方便后续的解析和分析。
- **注释说明**：在代码中记录日志时，添加必要的注释说明每个字段的含义和单位。
- **版本控制**：日志文件通常不直接纳入版本控制，但日志记录的代码逻辑应受版本控制。
- **清理策略**：定期清理旧的或不再需要的日志文件。

## 7. 故障排除

- **日志文件未生成或为空**：
  - 检查代码中日志写入逻辑是否正确，文件路径是否可写。
  - 确保机器人节点已成功启动并正常运行。
  - 检查是否有足够的磁盘空间。
- **日志数据异常**：
  - 检查数据源（如传感器、编码器）是否正常工作。
  - 检查数据处理和记录的代码逻辑是否存在错误。
  - 确认时间戳是否正确，数据是否按预期顺序记录。
- **文件权限问题**：
  - 确保运行 ROS 节点的用户对 `log_info` 文件夹具有写入权限。