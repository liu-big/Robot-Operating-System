# `launch` 文件夹使用开发操作手册

## 1. 概述

在 `ucar_controller` ROS 包中，`launch` 文件夹用于存放 `.launch` 文件。这些文件是基于 XML 格式的配置文件，用于定义和启动 ROS 节点、设置参数、加载机器人描述文件以及其他 ROS 相关的配置。它们提供了一种方便的方式来管理和启动复杂的 ROS 系统，特别是 `ucar_controller` 包中的底层驱动、EKF（扩展卡尔曼滤波）和 TF（坐标变换）服务器等核心组件。

## 2. 文件夹结构

```
src/ucar_controller/launch/
├── base_driver.launch
├── ekf_bringup.launch
├── robot_pose_ekf.launch
└── tf_server.launch
```

- `base_driver.launch`: 用于启动机器人底层驱动节点，通常会加载相应的参数文件（如 `config/driver_params_mini.yaml`）。
- `ekf_bringup.launch`: 用于启动扩展卡尔曼滤波节点，可能用于融合里程计和 IMU 数据以提供更精确的机器人位姿估计。
- `robot_pose_ekf.launch`: 启动 ROS 官方的 `robot_pose_ekf` 节点，用于融合多种传感器数据（如里程计、IMU、视觉里程计）来估计机器人位姿。
- `tf_server.launch`: 用于启动 TF 静态或动态发布节点，管理机器人各个部件之间的坐标变换。

## 3. 主要功能与用途

`launch` 文件夹的主要功能是：

- **节点启动**：同时启动一个或多个 `ucar_controller` 包中的 ROS 节点，如 `base_driver`、`odom_ekf`、`sensor_tf_server` 等。
- **参数设置**：在节点启动时为其设置参数，这些参数会被加载到 ROS 参数服务器，例如加载 `config` 文件夹中的驱动参数。
- **话题重映射**：改变节点发布或订阅的话题名称，以适应不同的系统配置。
- **节点分组**：将相关的节点组织到命名空间中，避免命名冲突。
- **包含其他 launch 文件**：通过 `include` 标签复用其他 `.launch` 文件，构建模块化的启动配置。
- **加载机器人描述**：加载 URDF (Unified Robot Description Format) 或 XACRO (XML Macro) 文件，用于机器人模型的可视化和仿真。

## 4. 使用方法

- **创建和编辑 launch 文件**：
  可以使用文本编辑器创建或修改 `.launch` 文件。例如，`base_driver.launch` 可能包含以下内容：
  ```xml
  <!-- base_driver.launch 示例 -->
  <launch>
    <arg name="robot_model" default="mini" />

    <!-- 加载驱动参数 -->
    <rosparam file="$(find ucar_controller)/config/driver_params_$(arg robot_model).yaml" command="load" />

    <!-- 启动底层驱动节点 -->
    <node pkg="ucar_controller" type="base_driver" name="base_driver_node" output="screen" />

    <!-- 启动 TF 服务器 -->
    <include file="$(find ucar_controller)/launch/tf_server.launch" />

    <!-- 启动 EKF 节点 -->
    <include file="$(find ucar_controller)/launch/ekf_bringup.launch" />
  </launch>
  ```

- **运行 launch 文件**：
  在终端中使用 `roslaunch` 命令运行 `.launch` 文件：
  ```bash
  roslaunch ucar_controller base_driver.launch robot_model:=ucarV2
  ```
  这会启动 `base_driver.launch` 文件中定义的所有节点，并加载 `driver_params_ucarV2.yaml` 参数。

## 5. 项目全局应用

在 `ucar_ws` 项目中，`ucar_controller` 包的 `launch` 文件夹是部署和运行其核心功能的入口。它使得开发者能够通过简单的命令启动复杂的机器人控制系统，包括底层驱动、位姿估计和坐标变换服务。这种集中式的启动管理极大地简化了系统的集成和测试过程，确保了 `ucar_controller` 能够与其他 ROS 包协同工作，共同实现机器人的自主运动和感知能力。

## 6. 维护与更新

- **版本控制**：所有 `.launch` 文件都应该被提交到版本控制系统（如 Git），以便团队成员共享和跟踪变更。
- **清晰注释**：在 `.launch` 文件中添加详细的注释，说明每个节点、参数和包含文件的用途。
- **模块化**：将复杂的启动逻辑分解为多个小的 `.launch` 文件，并通过 `include` 标签组合，提高可读性和复用性。
- **参数化**：使用 `<arg>` 标签定义可配置的参数，使得 `.launch` 文件更具灵活性。
- **错误处理**：在 `.launch` 文件中可以设置 `required="true"` 来确保关键节点启动失败时整个 launch 文件停止。

## 7. 故障排除

- **launch 文件无法运行**：
  - 检查 XML 语法是否正确，可以使用 XML 验证工具。
  - 检查 `pkg` 和 `type` 属性是否正确指向 ROS 包和可执行文件/脚本。
  - 确保所有引用的文件（如参数文件、其他 launch 文件）路径正确且存在。
  - 查看终端输出的错误信息，通常会指出问题所在。
- **节点未启动或启动失败**：
  - 检查 `output="screen"` 属性，以便在终端中看到节点的输出信息。
  - 检查节点的可执行文件是否具有执行权限。
  - 检查节点所需的依赖是否已安装。
  - 检查参数是否正确加载，以及是否存在参数冲突。
- **话题或服务问题**：
  - 使用 `rostopic list`、`rostopic echo`、`rosservice list` 等命令检查话题和服务是否按预期工作。
  - 检查 `launch` 文件中是否存在话题重映射，以及重映射是否正确。