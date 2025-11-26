# `config` 文件夹使用开发操作手册

## 1. 概述

在 `ucar_nav` ROS 包的 `launch` 文件夹中，`config` 文件夹专门用于存放 ROS 导航栈（Navigation Stack）各个组件的配置文件。这些文件通常是 YAML 格式，用于详细配置定位（AMCL）、全局路径规划、局部路径规划以及代价地图等模块的行为和参数。通过修改这些配置文件，可以根据不同的机器人平台和环境需求，精细调整导航系统的性能。

## 2. 文件夹结构

```
src/ucar_nav/launch/config/
├── amcl/
│   ├── amcl_diff.launch
│   ├── amcl_omni.launch
│   ├── amcl_omni3.launch
│   └── amcl_omni_2.launch
├── move_base copy/
│   ├── costmap_common_params.yaml
│   ├── costmap_converter_params.yaml
│   ├── dwa_local_planner_params.yaml
│   ├── ekf_localization.yaml
│   ├── global_costmap_params.yaml
│   ├── global_planner_params.yaml
│   ├── local_costmap_params.yaml
│   ├── move_base_params.yaml
│   ├── my_navigation.yaml
│   ├── teb1.yaml
│   └── teb_local_planner_params.yaml
├── move_base copy___能用/
├── move_base copy（last）/
├── move_base(1)/
├── move_base(7_17_ij)/
├── move_base/
├── move_base_4/
├── move_base_8/
├── move_base_end/
└── rviz/
    └── tebrviz.rviz
```

- `amcl/`: 存放 AMCL（Adaptive Monte Carlo Localization）定位算法的配置文件。这些文件通常是 `.launch` 文件，用于启动和配置 AMCL 节点，例如设置粒子数量、传感器模型、运动模型等。
- `move_base*/`: 包含多个 `move_base` 相关的参数文件夹，这些文件夹可能代表了不同时间点、不同测试目的或不同机器人配置下的参数集。每个文件夹通常包含以下 YAML 文件：
  - `costmap_common_params.yaml`: 代价地图的通用参数，如传感器配置、膨胀半径等。
  - `global_costmap_params.yaml`: 全局代价地图的特定参数。
  - `local_costmap_params.yaml`: 局部代价地图的特定参数。
  - `global_planner_params.yaml`: 全局路径规划器的参数（如 `GlobalPlanner` 或 `A*`）。
  - `local_planner_params.yaml` (或 `dwa_local_planner_params.yaml`, `teb_local_planner_params.yaml`): 局部路径规划器的参数（如 DWA 或 TEB）。
  - `move_base_params.yaml`: `move_base` 节点本身的核心参数。
  - `ekf_localization.yaml`: 可能用于扩展卡尔曼滤波（EKF）的配置，用于融合多种传感器数据进行定位。
  - `costmap_converter_params.yaml`: 代价地图转换器的参数。
  - `my_navigation.yaml`, `teb1.yaml`: 其他自定义或测试用的导航参数文件。
- `rviz/`: 存放 RViz 相关的配置文件，例如 `tebrviz.rviz` 可能用于可视化 TEB 局部规划器的轨迹和相关信息。

## 3. 主要功能与用途

`config` 文件夹的主要功能是：

- **参数集中管理**：将导航栈所有可配置的参数集中存放，便于统一管理和修改。
- **导航行为调优**：通过调整参数，可以精细控制机器人的定位精度、路径规划策略、避障行为和运动平滑度。
- **模块化配置**：每个导航组件（AMCL、全局规划器、局部规划器、代价地图）都有独立的配置文件，提高了系统的模块化和可维护性。
- **多场景适应**：通过切换不同的参数集（例如 `move_base copy` 和 `move_base` 文件夹），可以使导航系统适应不同的环境或机器人模型。
- **故障诊断**：在导航出现问题时，检查和修改这些参数是重要的故障排除步骤。

## 4. 使用方法

- **修改参数**：
  直接编辑相应的 YAML 文件来修改导航参数。例如，要调整 DWA 局部规划器的速度限制，可以修改 `dwa_local_planner_params.yaml`。
- **应用更改**：
  修改配置文件后，需要重新启动 `ucar_nav` 包的主 `launch` 文件（如 `ucar_navigation.launch`），以使新的参数生效。
- **创建新配置**：
  可以复制现有文件夹并修改其中的参数，以创建新的导航配置方案，用于测试或特定任务。
- **在 `launch` 文件中引用**：
  在 `launch` 文件中，通过 `<rosparam command="load" file="$(find ucar_nav)/launch/config/move_base/costmap_common_params.yaml" ns="global_costmap" />` 等方式加载这些参数文件。

## 5. 项目全局应用

在 `ucar_ws` 项目中，`ucar_nav` 包的 `config` 文件夹是导航系统灵活性的关键。它允许开发者根据实际部署环境和机器人特性，对导航行为进行深度定制和优化。这些配置文件是连接理论算法与实际机器人性能的桥梁，确保了 `ucar_nav` 能够高效、安全地完成自主导航任务。

## 6. 维护与更新

- **参数注释**：在配置文件中添加详细注释，说明每个参数的含义和推荐值。
- **版本管理**：将所有配置文件纳入版本控制，并记录每次参数更改的目的和效果。
- **测试与验证**：每次修改参数后，进行充分的测试以验证导航性能和稳定性。
- **清理冗余**：定期清理不再使用或重复的 `move_base` 配置文件夹，保持文件夹结构的清晰。

## 7. 故障排除

- **导航性能不佳**：
  - 检查代价地图参数（膨胀半径、传感器源）是否合理，确保障碍物被正确识别和避开。
  - 调整全局和局部规划器的参数（速度限制、路径平滑度、目标容忍度），以适应机器人运动学和环境特点。
  - 检查 AMCL 参数（粒子数量、运动模型、传感器模型）是否与实际情况匹配，以提高定位精度。
- **参数加载错误**：
  - 检查 YAML 文件语法是否正确，可以使用在线 YAML 验证工具。
  - 确保 `launch` 文件中加载参数的路径和命名空间正确。
  - 查看 ROS 节点的日志输出，查找参数加载相关的错误或警告信息。