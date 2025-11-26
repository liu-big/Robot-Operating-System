# `send_goals` 文件夹使用开发操作手册

## 1. 概述

`send_goals` 文件夹是 `ucar_ws` 工作区中用于处理机器人导航目标发送和相关辅助功能的 ROS 包。它包含了用于发送导航目标、处理相机校正、以及其他与机器人控制和感知相关的脚本和源代码。此包旨在提供一套工具，使得用户能够方便地向机器人发送导航指令，并集成必要的传感器数据处理功能，以支持机器人的自主导航和任务执行。

## 2. 文件夹结构

```
src/send_goals/
├── CMakeLists.txt
├── include/                  # 头文件目录
│   └── send_goals/
├── launch/                   # 启动文件目录
│   ├── camera_correct.launch
│   ├── ost.yaml
│   ├── stop_pid.launch
│   └── usb_cam.launch
├── package.xml
├── scripts/                  # Python 脚本目录
│   ├── camera_tf.py
│   ├── camera_tf_2.py
│   ├── pub_pos.py
│   └── test.py
└── src/                      # C++ 源代码目录
    ├── get_nav_poinet.cpp
    ├── send_goal_node.cpp
    └── tf_stop_pid.cpp
```

- `CMakeLists.txt`: 用于编译 `send_goals` 包的 CMake 构建脚本。
- `include/`: 存放 C++ 头文件，可能包含一些公共的函数或类定义。
- `launch/`: 存放 `.launch` 文件，用于启动 ROS 节点和配置参数，例如相机校正、PID 控制停止等。
- `package.xml`: ROS 包的元数据文件，定义了包的名称、版本、作者、依赖项等信息。
- `scripts/`: 存放 Python 脚本，可能用于相机 TF 变换、位置发布等。
- `src/`: 存放 C++ 源文件，实现了主要的 ROS 节点功能，例如导航目标发送、PID 控制等。

## 3. 主要功能与用途

`send_goals` 文件夹的主要功能是：

- **导航目标发送**：实现向 ROS 导航栈发送 `move_base` 目标的功能，使得机器人能够自主导航到指定位置。
- **相机校正与变换**：提供用于相机图像校正和相机坐标系与机器人坐标系之间 TF 变换的工具和脚本。
- **PID 控制管理**：可能包含用于启动或停止 PID 控制的逻辑，例如在机器人到达目标点后停止运动。
- **USB 摄像头集成**：提供启动 USB 摄像头的 launch 文件，方便获取图像数据。
- **辅助脚本**：包含一些用于测试、数据发布或特定任务的辅助脚本。

## 4. 使用方法

- **编译**：
  在 ROS 工作空间的根目录下执行 `catkin_make` 或 `colcon build` 命令即可编译整个包。
  ```bash
  cd ~/ucar_ws
  catkin_make
  # 或者 colcon build
  ```
- **启动导航目标发送节点**：
  通常通过 launch 文件启动，例如：
  ```bash
  roslaunch send_goals send_goal_node.launch # 假设存在此 launch 文件
  ```
- **启动相机相关功能**：
  ```bash
  roslaunch send_goals usb_cam.launch
  roslaunch send_goals camera_correct.launch
  ```
- **运行脚本**：
  ```bash
  rosrun send_goals camera_tf.py
  ```
- **修改配置**：
  直接编辑 `launch` 文件夹中的 `.launch` 文件或 `scripts` 文件夹中的 `.py` 脚本来修改参数或逻辑。

## 5. 项目全局应用

在 `ucar_ws` 项目中，`send_goals` 包是实现机器人高级自主行为的关键组成部分。它提供了与导航系统交互的接口，使得机器人能够接收并执行导航任务。同时，其包含的相机和控制相关功能也为机器人的感知和精确运动控制提供了支持，是构建完整机器人应用的重要环节。

## 6. 维护与更新

- **功能扩展**：根据新的导航需求或传感器集成，扩展导航目标发送和相机处理功能。
- **参数调优**：根据实际机器人和环境，调整相机校正参数、PID 控制参数等。
- **代码优化**：定期审查代码，提高性能和稳定性。
- **依赖管理**：确保所有依赖的 ROS 包和库都已正确安装和配置。

## 7. 故障排除

- **节点无法启动**：
  - 检查编译是否成功，是否有编译错误。
  - 检查 `launch` 文件中的节点名称和类型是否正确。
  - 检查 ROS 环境变量是否正确设置。
- **导航目标无法发送**：
  - 确认 `move_base` 导航栈是否正在运行。
  - 检查目标话题（通常是 `/move_base_simple/goal`）是否被正确发布和订阅。
  - 检查目标坐标系是否正确。
- **相机图像或 TF 异常**：
  - 检查 USB 摄像头是否正确连接并被系统识别。
  - 检查 `usb_cam.launch` 中的设备路径和参数是否正确。
  - 检查 `camera_tf.py` 或 `camera_tf_2.py` 中的 TF 变换逻辑是否正确，以及是否发布了正确的 TF 消息。
  - 使用 `rqt_image_view` 查看图像话题，使用 `rqt_tf_tree` 查看 TF 树。