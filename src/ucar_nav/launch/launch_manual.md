# `ucar_nav` 包 `launch` 文件夹说明文档

## 1. 概述

在 `ucar_nav` ROS 包中，`launch` 文件夹用于存放 `.launch` 文件。这些文件是基于 XML 格式的配置文件，用于定义和启动与机器人导航相关的 ROS 节点、设置参数、以及集成导航栈的各个组件。它们提供了一种方便的方式来管理和启动复杂的导航功能，例如地图服务器、AMCL 定位、`move_base` 路径规划和运动控制。

## 2. 文件夹结构

```
src/ucar_nav/launch/
├── config/
│   ├── amcl/
│   │   ├── amcl_diff.launch
│   │   ├── amcl_omni.launch
│   │   ├── amcl_omni3.launch
│   │   └── amcl_omni_2.launch
│   ├── move_base copy/
│   ├── move_base copy___能用/
│   ├── move_base copy（last）/
│   ├── move_base(1)/
│   ├── move_base(7_17_ij)/
│   ├── move_base/
│   ├── move_base_4/
│   ├── move_base_8/
│   ├── move_base_end/
│   └── rviz/
│       └── tebrviz.rviz
└── ucar_navigation.launch
```

- `config/`: 包含导航栈各个组件的配置文件，这些文件通常是 YAML 格式，用于设置参数。
  - `amcl/`: 存放 AMCL（Adaptive Monte Carlo Localization）定位算法的 `.launch` 文件，用于启动和配置 AMCL 节点。
  - `move_base*/`: 存放 `move_base` 节点及其相关插件（如全局规划器、局部规划器、代价地图）的参数配置文件。这些文件夹可能包含不同版本或不同配置的参数。
  - `rviz/`: 存放 RViz 相关的配置文件，例如 `tebrviz.rviz` 可能用于可视化 TEB 局部规划器的轨迹。
- `ucar_navigation.launch`: 主导航启动文件，通常会 `include` 其他 `launch` 文件来启动地图服务器、AMCL、`move_base` 等核心导航节点，并设置全局参数。

## 3. 主要功能与用途

`launch` 文件夹的主要功能是：

- **导航系统启动**：集中启动 `map_server`、AMCL、`move_base` 等所有导航所需的 ROS 节点。
- **参数配置**：在节点启动时为其设置参数，这些参数会被加载到 ROS 参数服务器，例如 AMCL 的粒子数量、`move_base` 的规划器类型和代价地图参数。
- **话题重映射**：改变节点发布或订阅的话题名称，以适应不同的系统配置。
- **包含其他 launch 文件**：通过 `include` 标签复用其他 `.launch` 文件，构建模块化的启动配置，例如将 AMCL 的启动配置独立出来。
- **可视化配置**：启动 RViz 并加载预设的配置文件，方便用户监控导航过程。

## 4. 使用方法

- **运行导航系统**：
  通过运行主导航 `launch` 文件来启动整个导航系统：
  ```bash
  roslaunch ucar_nav ucar_navigation.launch
  ```
- **修改配置**：
  要调整导航行为，需要修改 `config/` 文件夹下的 YAML 配置文件。修改后，通常需要重新启动 `ucar_navigation.launch` 文件以使更改生效。
- **创建自定义启动文件**：
  可以根据特定需求创建新的 `.launch` 文件，例如只启动部分导航组件进行测试。

## 5. 项目全局应用

在 `ucar_ws` 项目中，`ucar_nav` 包的 `launch` 文件夹是部署和运行机器人自主导航功能的入口。它使得开发者能够通过简单的命令启动复杂的导航系统，包括地图的加载、机器人的定位以及路径的规划和执行。这种集中式的启动管理极大地简化了系统的集成和测试过程，确保了 `ucar_nav` 能够与其他 ROS 包（如 `ucar_map` 和 `ucar_controller`）协同工作，共同实现机器人的自主导航能力。

## 6. 维护与更新

- **参数调优**：根据机器人平台和环境特点，持续调整 `config` 文件夹中的参数以优化导航性能。
- **依赖管理**：确保 `launch` 文件中引用的节点和话题名称与实际系统配置一致。
- **模块化**：保持 `launch` 文件的模块化，便于管理和复用。
- **版本控制**：将所有 `launch` 文件和配置文件纳入版本控制，方便追踪更改和回溯。

## 7. 故障排除

- **导航系统无法启动**：
  - 检查 `launch` 文件中引用的所有文件路径是否正确。
  - 检查所有 `include` 的 `launch` 文件是否存在且无语法错误。
  - 查看终端输出，查找 ROS 节点启动失败的错误信息。
- **导航行为异常**：
  - 检查 `config` 文件夹中的参数设置是否合理，特别是代价地图、规划器和 AMCL 的参数。
  - 确保 TF 树正确发布，特别是 `map`、`odom` 和 `base_link` 之间的变换。
  - 检查传感器数据（如激光雷达、里程计）是否正常发布且数据质量良好。
- **RViz 显示问题**：
  - 检查 `rviz` 文件夹中的 `.rviz` 配置文件是否正确加载。
  - 确保 RViz 中订阅的话题和显示的坐标系与导航系统一致.