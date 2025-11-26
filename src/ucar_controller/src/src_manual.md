# `src` 文件夹使用开发操作手册

## 1. 概述

在 `ucar_controller` ROS 包中，`src` 文件夹是核心源代码的存放位置，主要包含 C++ 语言编写的 ROS 节点实现。这些节点负责处理机器人的底层控制逻辑、传感器数据处理、里程计计算、以及与硬件的通信等关键功能。它是 `ucar_controller` 包实现其核心控制任务的基础。

## 2. 文件夹结构

```
src/ucar_controller/src/
├── base_driver.cpp
├── odom_ekf.cpp
└── sensor_tf_server.cpp
```

- `base_driver.cpp`: 实现了机器人底层驱动的 ROS 节点，负责与机器人硬件（如电机控制器）进行通信，发送控制指令并接收反馈数据（如编码器读数）。
- `odom_ekf.cpp`: 可能实现了基于扩展卡尔曼滤波（EKF）的里程计融合节点，用于融合来自编码器、IMU 等传感器的数据，提供更精确的机器人位姿估计。
- `sensor_tf_server.cpp`: 实现了 ROS 的 TF（坐标变换）发布节点，负责发布机器人各个传感器（如摄像头、激光雷达）相对于机器人本体的坐标变换关系。

## 3. 主要功能与用途

`src` 文件夹的主要功能是：

- **ROS 节点实现**：包含 `ucar_controller` 包中所有 C++ 编写的 ROS 节点源代码。
- **底层控制逻辑**：实现与机器人硬件交互的驱动程序和控制算法。
- **数据处理与融合**：处理传感器原始数据，并进行数据融合（如里程计融合）以提供更高级的信息。
- **系统集成**：通过 ROS 消息、服务、动作等机制，与其他 ROS 包进行通信和协作。
- **性能优化**：C++ 语言通常用于对性能要求较高的模块，如实时控制和高频数据处理。

## 4. 使用方法

- **创建 C++ 源文件**：
  在 `src` 文件夹中创建 `.cpp` 文件，编写 ROS 节点代码。例如，`base_driver.cpp` 的基本结构：
  ```cpp
  #include "ros/ros.h"
  #include "std_msgs/String.h"
  // ... 其他必要的头文件 ...

  int main(int argc, char **argv)
  {
    ros::init(argc, argv, "base_driver_node");
    ros::NodeHandle nh;

    // ... 节点初始化，如订阅话题、发布话题、加载参数 ...

    ros::Rate loop_rate(10); // 10 Hz

    while (ros::ok())
    {
      // ... 节点逻辑，如读取传感器、计算控制量、发送指令 ...

      ros::spinOnce();
      loop_rate.sleep();
    }

    return 0;
  }
  ```

- **在 `CMakeLists.txt` 中配置**：
  为了编译这些 C++ 源文件并生成可执行的 ROS 节点，需要在 `ucar_controller` 包的 `CMakeLists.txt` 中添加相应的编译规则：
  ```cmake
  # ...
  # 添加可执行文件
  add_executable(base_driver src/base_driver.cpp)
  add_executable(odom_ekf src/odom_ekf.cpp)
  add_executable(sensor_tf_server src/sensor_tf_server.cpp)

  # 链接库
  target_link_libraries(base_driver
    ${catkin_LIBRARIES}
  )
  target_link_libraries(odom_ekf
    ${catkin_LIBRARIES}
  )
  target_link_libraries(sensor_tf_server
    ${catkin_LIBRARIES}
  )

  # 安装可执行文件
  install(TARGETS base_driver odom_ekf sensor_tf_server
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
  # ...
  ```

- **编译**：
  在工作区根目录（`ucar_ws`）下使用 `catkin_make` 或 `catkin build` 命令编译整个工作区：
  ```bash
  cd d:/比赛专用/ucar_ws
  catkin_make
  # 或者
  catkin build
  ```

- **在 `launch` 文件中启动**：
  编译成功后，生成的 ROS 节点可执行文件通常位于 `devel/lib/ucar_controller/` 或 `install/lib/ucar_controller/` 目录下。可以在 `launch` 文件夹中的 `.launch` 文件中启动这些节点：
  ```xml
  <!-- 启动 base_driver 节点 -->
  <node pkg="ucar_controller" type="base_driver" name="base_driver_node" output="screen" />

  <!-- 启动 odom_ekf 节点 -->
  <node pkg="ucar_controller" type="odom_ekf" name="odom_ekf_node" output="screen" />

  <!-- 启动 sensor_tf_server 节点 -->
  <node pkg="ucar_controller" type="sensor_tf_server" name="sensor_tf_server_node" output="screen" />
  ```

## 5. 项目全局应用

在 `ucar_ws` 项目中，`ucar_controller` 包的 `src` 文件夹是实现机器人核心控制逻辑和功能的基石。它包含了所有用 C++ 编写的高性能 ROS 节点，这些节点直接与机器人硬件交互，处理实时数据，并执行复杂的控制算法。`src` 文件夹中的代码是整个机器人系统能够稳定、高效运行的关键，确保了 `ucar_controller` 能够提供精确的运动控制和可靠的感知数据。

## 6. 维护与更新

- **代码规范**：遵循 C++ 编码规范（如 Google C++ Style Guide），保持代码风格一致性。
- **注释和文档**：为函数、类和复杂逻辑添加详细的注释和 Doxygen 风格的文档。
- **错误处理**：在代码中加入健壮的错误处理机制，包括异常捕获和日志记录。
- **内存管理**：注意 C++ 中的内存泄漏问题，合理使用智能指针。
- **版本控制**：所有源文件都应该被提交到版本控制系统（如 Git）。
- **单元测试**：为关键模块编写单元测试，确保代码的正确性。

## 7. 故障排除

- **编译错误**：
  - 检查 `CMakeLists.txt` 配置是否正确，包括头文件路径、库链接等。
  - 检查 C++ 语法错误，使用编译器输出的错误信息进行定位。
  - 确保所有依赖的 ROS 包和第三方库已正确安装。
- **运行时错误**：
  - 检查 ROS 节点的日志输出（`output="screen"` 或 `roslog`）。
  - 使用 `rqt_graph` 查看节点连接图，确认话题、服务连接是否正常。
  - 使用 `rostopic echo`、`rosnode info` 等命令检查节点输入输出。
  - 使用调试器（如 GDB）进行调试。
- **性能问题**：
  - 使用 `rqt_plot` 监控话题数据频率和延迟。
  - 使用 `perf` 或 `valgrind` 等工具进行性能分析和内存泄漏检测。