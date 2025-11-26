# `move_base_end` 文件夹使用开发操作手册

## 1. 概述

在 `ucar_nav` ROS 包的 `launch/config` 文件夹中，`move_base_end` 文件夹是 `move_base` 节点配置文件的另一个副本，其命名中的“end”可能表示这是一个最终版本、稳定版本或某个开发阶段的结束版本。它包含了与 `move_base` 导航栈相关的各种 YAML 配置文件，用于定义全局路径规划器、局部路径规划器和代价地图的行为和参数。这个文件夹的存在通常是为了标记一个经过充分测试和验证的配置集，作为部署或后续开发的基础。

## 2. 文件夹结构

```
src/ucar_nav/launch/config/move_base_end/
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

这些文件的具体作用和内容与 `move_base` 文件夹中的文件类似，主要区别在于它们被明确标记为“end”版本，通常代表了某个开发阶段的最终或稳定配置。

## 3. 主要功能与用途

`move_base_end` 文件夹的主要功能是：

- **提供稳定配置**：作为经过验证的、可用于实际部署或作为后续开发基准的稳定参数集。
- **里程碑标记**：标记某个开发阶段的完成，便于项目管理和版本控制。
- **快速部署**：在需要快速部署已知稳定配置时，可以直接引用此文件夹中的参数。

## 4. 使用方法

- **加载稳定配置**：
  在主导航 `launch` 文件（如 `ucar_navigation.launch`）中，将加载 `move_base` 参数的路径指向 `move_base_end` 文件夹。例如：
  ```xml
  <!-- 示例：加载 move_base_end 中的参数 -->
  <rosparam command="load" file="$(find ucar_nav)/launch/config/move_base_end/costmap_common_params.yaml" ns="global_costmap" />
  <rosparam command="load" file="$(find ucar_nav)/launch/config/move_base_end/global_costmap_params.yaml" ns="global_costmap" />
  <rosparam command="load" file="$(find ucar_nav)/launch/config/move_base_end/local_costmap_params.yaml" ns="local_costmap" />
  <!-- ... 其他参数文件 ... -->
  ```
- **修改参数**：
  通常情况下，`move_base_end` 文件夹中的参数不应频繁修改。如果需要修改，应谨慎进行，并在修改后进行充分测试。修改后，需要重新启动导航系统以使更改生效。

## 5. 项目全局应用

在 `ucar_ws` 项目中，`ucar_nav` 包的 `move_base_end` 文件夹是确保导航系统稳定性和可靠性的关键。它提供了一个经过验证的配置基线，减少了在部署或集成过程中出现问题的风险。这种“最终”或“稳定”版本的管理方式对于项目的质量控制和长期维护至关重要。

## 6. 维护与更新

- **谨慎更新**：对 `move_base_end` 文件夹的任何修改都应经过严格的测试和审查。
- **版本控制**：确保所有文件都纳入版本控制，并记录每次更新的详细信息，特别是对“最终”版本的修改。
- **备份**：在进行重大修改前，考虑对整个文件夹进行备份。

## 7. 故障排除

- **加载错误**：
  - 检查 `launch` 文件中引用的 `move_base_end` 路径是否正确。
  - 确保文件夹中的 YAML 文件语法正确。
- **导航行为与预期不符**：
  - 确认当前加载的是 `move_base_end` 中的参数，而不是其他配置。
  - 仔细检查 `move_base_end` 中的参数值，特别是与速度、加速度、代价地图膨胀相关的参数，是否符合预期。
  - 查看 ROS 节点的日志输出，确认参数是否被正确加载。