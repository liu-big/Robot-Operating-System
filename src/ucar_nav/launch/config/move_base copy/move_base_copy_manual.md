# `move_base copy` 文件夹使用开发操作手册

## 1. 概述

在 `ucar_nav` ROS 包的 `launch/config` 文件夹中，`move_base copy` 文件夹是 `move_base` 节点配置文件的副本。它包含了与 `move_base` 导航栈相关的各种 YAML 配置文件，用于定义全局路径规划器、局部路径规划器和代价地图的行为和参数。这个文件夹的存在通常是为了保留一套备用的、测试中的或特定场景下的导航参数配置，以便在不影响主配置的情况下进行实验或快速切换。

## 2. 文件夹结构

```
src/ucar_nav/launch/config/move_base copy/
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

这些文件的具体作用和内容与 `move_base` 文件夹中的文件类似，主要区别在于它们可能包含不同的参数值，以实现不同的导航行为或适应不同的测试场景。

## 3. 主要功能与用途

`move_base copy` 文件夹的主要功能是：

- **参数备份与版本管理**：作为 `move_base` 配置的备份，允许在修改主配置之前进行实验，或保存不同历史版本的参数集。
- **多场景配置**：为特定测试场景、不同机器人模型或不同环境提供一套独立的导航参数配置。
- **快速切换**：允许开发者通过修改 `launch` 文件中引用的路径，快速切换到这套备用配置，而无需修改主配置。
- **实验与调优**：提供一个安全的沙盒环境，用于测试新的参数值或规划器组合，而不会影响正在使用的稳定配置。

## 4. 使用方法

- **加载备用配置**：
  在主导航 `launch` 文件（如 `ucar_navigation.launch`）中，将加载 `move_base` 参数的路径指向 `move_base copy` 文件夹。例如：
  ```xml
  <!-- 示例：加载 move_base copy 中的参数 -->
  <rosparam command="load" file="$(find ucar_nav)/launch/config/move_base copy/costmap_common_params.yaml" ns="global_costmap" />
  <rosparam command="load" file="$(find ucar_nav)/launch/config/move_base copy/global_costmap_params.yaml" ns="global_costmap" />
  <rosparam command="load" file="$(find ucar_nav)/launch/config/move_base copy/local_costmap_params.yaml" ns="local_costmap" />
  <!-- ... 其他参数文件 ... -->
  ```
- **修改参数**：
  直接编辑 `move_base copy` 文件夹中的 YAML 文件来修改参数。修改后，需要重新启动导航系统以使更改生效。
- **与主配置同步**：
  在测试完成后，如果 `move_base copy` 中的参数被验证为更优，可以将其内容同步回主 `move_base` 文件夹，或将其提升为新的主配置。

## 5. 项目全局应用

在 `ucar_ws` 项目中，`ucar_nav` 包的 `move_base copy` 文件夹体现了导航参数配置的灵活性和可管理性。它支持开发者在不中断现有稳定导航功能的前提下，进行参数的迭代优化和新功能的测试。这种多配置管理方式对于复杂机器人系统的开发和维护至关重要，确保了导航系统的持续改进和适应性。

## 6. 维护与更新

- **明确用途**：为每个 `move_base` 的副本文件夹添加清晰的注释或命名，说明其用途（例如 `move_base_test_scenario_A`）。
- **定期清理**：删除不再需要或过时的副本文件夹，避免配置混乱。
- **同步更新**：如果主 `move_base` 配置发生重大变化，考虑是否需要同步更新所有副本，以保持一致性。
- **版本控制**：确保所有副本文件夹及其内容都纳入版本控制。

## 7. 故障排除

- **加载错误**：
  - 检查 `launch` 文件中引用的 `move_base copy` 路径是否正确。
  - 确保 `move_base copy` 文件夹中的 YAML 文件语法正确。
- **导航行为与预期不符**：
  - 确认当前加载的是 `move_base copy` 中的参数，而不是其他配置。
  - 仔细检查 `move_base copy` 中的参数值，特别是与速度、加速度、代价地图膨胀相关的参数，是否符合预期。
  - 查看 ROS 节点的日志输出，确认参数是否被正确加载。