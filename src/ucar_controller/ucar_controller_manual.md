# `ucar_controller` 文件夹使用开发操作手册

## 1. 概述

`ucar_controller` 文件夹是 `ucar_ws` 工作区中用于控制 UCAR 机器人运动和行为的核心 ROS 包。它包含了与机器人底层驱动、运动控制算法、传感器数据处理以及与 ROS 系统其他模块进行通信相关的代码和配置文件。此包旨在提供一个稳定、高效的接口，使得机器人能够精确地执行各种运动指令，并与外部环境进行交互。

## 2. 文件夹结构

```
src/ucar_controller/
├── .vscode/             # VS Code 编辑器配置
├── CHANGELOG.md         # 变更日志
├── CMakeLists.txt       # CMake 构建文件
├── README.md            # 项目说明文件
├── config/              # 配置文件，如驱动参数
├── include/             # 头文件，包含接口定义和数据结构
│   └── ucar_controller/
│       ├── base_driver.h
│       ├── crc_table.h
│       ├── data_struct.h
│       └── fdilink_data_struct.h
├── launch/              # ROS launch 文件，用于启动节点
│   ├── base_driver.launch
│   ├── ekf_bringup.launch
│   ├── robot_pose_ekf.launch
│   └── tf_server.launch
├── log_info/            # 日志信息，如里程计数据
│   ├── car_Mileage_info.txt
│   └── car_Mileage_info.txt.bp
├── package.xml          # ROS 包清单文件
├── scripts/             # Python 脚本，如里程计 EKF、性能测试、TF 服务器
│   ├── odom_ekf.py
│   ├── performance_test.py
│   └── sensor_tf_server.py
├── src/                 # C++ 源代码，如底层驱动实现
│   ├── base_driver.cpp
│   ├── base_driver2.cpp
│   └── crc_table.cpp
└── srv/                 # ROS 服务定义文件
    ├── GetBatteryInfo.srv
    ├── GetMaxVel.srv
    ├── GetSensorTF.srv
    ├── SetLEDMode.srv
    ├── SetMaxVel.srv
    └── SetSensorTF.srv
```

## 3. 主要功能与用途

`ucar_controller` 文件夹的主要功能是：

- **底层驱动与通信**：实现与机器人硬件（如电机控制器、传感器）的通信协议和驱动逻辑。
- **运动控制**：根据接收到的指令（如速度、姿态），计算并发送控制命令给机器人执行器。
- **传感器数据处理**：处理来自编码器、IMU 等传感器的数据，进行滤波、融合，生成里程计信息。
- **ROS 接口**：提供标准的 ROS 话题、服务和参数接口，使得其他 ROS 包可以方便地与机器人进行交互。
- **配置管理**：通过 `config` 文件管理机器人不同型号（mini, ucarV2, xiao）的驱动参数。
- **TF 变换管理**：发布和管理机器人各个部件之间的坐标变换（TF）。
- **服务定义**：定义 ROS 服务，用于查询电池信息、设置最大速度、LED 模式等。

## 4. 使用方法

- **编译与安装**：
  在 `ucar_ws` 工作区的根目录下执行 `catkin_make` 命令来编译 `ucar_controller` 包：
  ```bash
  cd ~/ucar_ws
  catkin_make
  source devel/setup.bash
  ```

- **运行控制器节点**：
  通常通过 `launch` 文件来启动 `ucar_controller` 包中的 ROS 节点。例如，启动基础驱动节点：
  ```bash
  roslaunch ucar_controller base_driver.launch
  ```
  （请根据实际的 `launch` 文件名和机器人型号进行调整，例如 `driver_params_mini.yaml`）

- **使用 ROS 服务**：
  启动节点后，可以使用 `rosservice list` 查看可用的服务，并使用 `rosservice call` 调用服务：
  ```bash
  rosservice list | grep ucar_controller
  rosservice call /ucar_controller/get_battery_info
  ```

## 5. 项目全局应用

在 `ucar_ws` 项目中，`ucar_controller` 包是整个机器人系统的“心脏”。它负责将上层导航和规划模块的指令转化为机器人实际的运动，并提供底层的传感器数据。无论是自主导航、远程遥控还是执行特定任务，`ucar_controller` 都扮演着至关重要的角色，确保机器人能够稳定、精确地与物理世界进行交互。它是实现机器人智能行为的基础。

## 6. 维护与更新

- **代码规范**：遵循 ROS 和 C++/Python 的代码编写规范，保持代码的可读性和可维护性。
- **依赖管理**：定期检查 `package.xml` 中的依赖项，确保所有依赖都已正确声明并安装。
- **版本控制**：使用 Git 等版本控制工具管理代码，并进行适当的提交和分支管理。
- **性能优化**：对底层驱动和控制算法进行性能分析和优化，确保实时性和响应速度。
- **参数调优**：根据不同机器人平台和应用场景，调整 `config` 文件夹中的参数，以达到最佳控制效果。
- **文档更新**：随着功能的增加或修改，及时更新相关文档和注释。

## 7. 故障排除

- **节点无法启动**：
  - 检查 `CMakeLists.txt` 和 `package.xml` 是否配置正确。
  - 确保 C++ 可执行文件或 Python 脚本具有执行权限。
  - 检查 ROS 环境是否已正确设置（`source devel/setup.bash`）。
  - 查看终端输出的错误信息，通常会指出编译错误、ROS 依赖问题或运行时异常。
- **机器人无响应或运动异常**：
  - 检查机器人硬件连接是否正常，电源是否充足。
  - 检查底层驱动是否成功启动，并与硬件建立通信。
  - 检查控制指令是否正确发布到相应的话题。
  - 检查 `config` 文件中的参数是否与机器人型号匹配，并进行了正确的调优。
  - 查看 ROS 日志（`roslog`）或 `log_info` 文件夹中的日志文件，查找错误信息。
- **TF 变换问题**：
  - 使用 `rosrun tf tf_echo <source_frame> <target_frame>` 检查 TF 变换是否正确发布。
  - 使用 `rosrun rqt_tf_tree rqt_tf_tree` 可视化 TF 树，检查是否存在断裂或循环。
  - 检查 `launch` 文件中 TF 相关的配置是否正确。