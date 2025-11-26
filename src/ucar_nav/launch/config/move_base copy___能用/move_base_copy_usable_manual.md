# `move_base copy___能用` 文件夹使用开发操作手册

## 1. 概述

在 `ucar_nav` ROS 包的 `launch/config` 文件夹中，`move_base copy___能用` 文件夹是 `move_base` 节点配置文件的另一个副本，其命名暗示了这是一个经过验证、可用的配置版本。它包含了与 `move_base` 导航栈相关的各种 YAML 配置文件，用于定义全局路径规划器、局部路径规划器和代价地图的行为和参数。这个文件夹的存在通常是为了明确标记一套在实际测试中表现良好、可以投入使用的导航参数配置，作为开发过程中的一个稳定里程碑。

## 2. 文件夹结构

```
src/ucar_nav/launch/config/move_base copy___能用/
├── costmap_common_params.yaml
├── costmap_converter_params.yaml
├── dwa_local_planner_params.yaml
├── ekf_localization.yaml
├── global_costmap_params.yaml
├── global_planner_params.yaml
├── local_costmap_params.yaml
├── move_base_params.yaml
├── my_navigation.yaml
├── teb1.yaml
└── teb_local_planner_params.yaml
```

- `costmap_common_params.yaml`: 定义了全局和局部代价地图的通用参数。
- `global_costmap_params.yaml`: 定义了全局代价地图的特定参数。
- `local_costmap_params.yaml`: 定义了局部代价地图的特定参数。
- `global_planner_params.yaml`: 定义了全局路径规划器的参数。
- `dwa_local_planner_params.yaml`: 如果使用 DWA 作为局部路径规划器，此文件定义了其参数。
- `teb_local_planner_params.yaml`: 如果使用 TEB 作为局部路径规划器，此文件定义了其参数。
- `move_base_params.yaml`: `move_base` 节点本身的核心参数。
- `ekf_localization.yaml`: 可能用于配置扩展卡尔曼滤波（EKF）或机器人定位的其他参数。
- `costmap_converter_params.yaml`: 代价地图转换器的参数。
- `my_navigation.yaml`, `teb1.yaml`: 其他自定义或测试用的导航参数文件。

这些文件的具体作用和内容与 `move_base` 文件夹中的文件类似，主要区别在于它们被明确标记为“能用”或“可用”的版本，通常代表了经过充分测试和验证的配置。

## 3. 主要功能与用途

`move_base copy___能用` 文件夹的主要功能是：

- **提供稳定配置**：作为一套经过验证、在特定场景下表现良好的 `move_base` 参数配置，可供日常开发和部署使用。
- **参考基准**：为后续的参数调优和新功能开发提供一个可靠的性能基准。
- **快速部署**：允许开发者快速部署一个已知可用的导航系统，而无需从头开始调优。
- **避免回归**：通过保留这个“能用”的版本，可以避免在后续开发中引入导致导航性能下降的回归问题。

## 4. 使用方法

- **加载可用配置**：
  在主导航 `launch` 文件（如 `ucar_navigation.launch`）中，将加载 `move_base` 参数的路径指向 `move_base copy___能用` 文件夹。例如：
  ```xml
  <!-- 示例：加载 move_base copy___能用 中的参数 -->
  <rosparam command="load" file="$(find ucar_nav)/launch/config/move_base copy___能用/costmap_common_params.yaml" ns="global_costmap" />
  <rosparam command="load" file="$(find ucar_nav)/launch/config/move_base copy___能用/global_costmap_params.yaml" ns="global_costmap" />
  <rosparam command="load" file="$(find ucar_nav)/launch/config/move_base copy___能用/local_costmap_params.yaml" ns="local_costmap" />
  <!-- ... 其他参数文件 ... -->
  ```
- **作为开发起点**：
  在进行新的参数调优或功能开发时，可以从这个“能用”的配置开始，在其基础上进行修改和实验。

## 5. 项目全局应用

在 `ucar_ws` 项目中，`ucar_nav` 包的 `move_base copy___能用` 文件夹是确保机器人导航系统稳定性和可靠性的重要组成部分。它提供了一个经过实践检验的导航参数集，使得团队成员可以快速启动一个功能正常的导航系统，并在此基础上进行迭代开发。这对于项目的持续集成和部署流程具有重要意义。

## 6. 维护与更新

- **保持稳定**：这个文件夹的配置应尽量保持稳定，不应频繁修改，除非有充分的理由和验证。
- **同步更新**：如果主 `move_base` 配置经过大量测试后被证明更优，可以考虑将这个“能用”的版本更新为最新的稳定配置。
- **清晰命名**：确保文件夹命名清晰，准确反映其“可用”或“稳定”的状态。
- **版本控制**：确保所有文件都纳入版本控制，并记录每次更新的详细信息。

## 7. 故障排除

- **加载错误**：
  - 检查 `launch` 文件中引用的 `move_base copy___能用` 路径是否正确。
  - 确保文件夹中的 YAML 文件语法正确。
- **导航行为与预期不符**：
  - 尽管标记为“能用”，但在特定新场景下仍可能出现问题。此时应仔细检查参数是否适用于当前环境。
  - 确认当前加载的是这个“能用”版本，而不是其他配置。
  - 查看 ROS 节点的日志输出，确认参数是否被正确加载。