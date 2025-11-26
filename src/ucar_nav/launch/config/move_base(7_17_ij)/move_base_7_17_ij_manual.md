# `move_base(7_17_ij)` 文件夹使用开发操作手册

## 1. 概述

在 `ucar_nav` ROS 包的 `launch/config` 文件夹中，`move_base(7_17_ij)` 文件夹是 `move_base` 节点配置文件的另一个副本，其命名可能包含了日期信息（7月17日）和开发者标识（ij），暗示了这是一个特定时间点由特定开发者创建或修改的配置版本。它包含了与 `move_base` 导航栈相关的各种 YAML 配置文件，用于定义全局路径规划器、局部路径规划器和代价地图的行为和参数。这个文件夹的存在通常是为了记录某个特定开发阶段或特定测试任务的参数配置，便于追踪和回溯。

## 2. 文件夹结构

```
src/ucar_nav/launch/config/move_base(7_17_ij)/
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

这些文件的具体作用和内容与 `move_base` 文件夹中的文件类似，主要区别在于它们被明确标记为特定日期和开发者的版本，通常代表了某个特定开发阶段的配置快照。

## 3. 主要功能与用途

`move_base(7_17_ij)` 文件夹的主要功能是：

- **记录特定开发阶段的配置**：作为某个特定日期或由特定开发者进行的实验的参数快照，便于后续回顾和分析。
- **版本回溯**：在需要回溯到特定历史配置时，提供一个明确的参考点。
- **并行开发支持**：允许不同开发者或在不同时间点进行参数调优时，保留各自的配置，避免相互覆盖。

## 4. 使用方法

- **加载特定版本配置**：
  在主导航 `launch` 文件（如 `ucar_navigation.launch`）中，将加载 `move_base` 参数的路径指向 `move_base(7_17_ij)` 文件夹。例如：
  ```xml
  <!-- 示例：加载 move_base(7_17_ij) 中的参数 -->
  <rosparam command="load" file="$(find ucar_nav)/launch/config/move_base(7_17_ij)/costmap_common_params.yaml" ns="global_costmap" />
  <rosparam command="load" file="$(find ucar_nav)/launch/config/move_base(7_17_ij)/global_costmap_params.yaml" ns="global_costmap" />
  <rosparam command="load" file="$(find ucar_nav)/launch/config/move_base(7_17_ij)/local_costmap_params.yaml" ns="local_costmap" />
  <!-- ... 其他参数文件 ... -->
  ```
- **修改参数**：
  直接编辑 `move_base(7_17_ij)` 文件夹中的 YAML 文件来修改参数。修改后，需要重新启动导航系统以使更改生效。

## 5. 项目全局应用

在 `ucar_ws` 项目中，`ucar_nav` 包的 `move_base(7_17_ij)` 文件夹是开发团队进行协作和版本管理的重要实践。它有助于清晰地记录不同开发阶段的参数配置，减少因参数冲突导致的问题，并为调试和性能分析提供历史数据。这种命名约定和文件夹管理方式对于大型或长期项目尤其有益。

## 6. 维护与更新

- **清晰命名**：继续使用有意义的命名约定，包含日期、开发者或特定实验的标识。
- **定期清理**：删除不再需要或过时的日期/开发者命名文件夹，避免配置混乱。
- **版本控制**：确保所有文件都纳入版本控制，并记录每次更新的详细信息。

## 7. 故障排除

- **加载错误**：
  - 检查 `launch` 文件中引用的 `move_base(7_17_ij)` 路径是否正确。
  - 确保文件夹中的 YAML 文件语法正确。
- **导航行为与预期不符**：
  - 确认当前加载的是 `move_base(7_17_ij)` 中的参数，而不是其他配置。
  - 仔细检查 `move_base(7_17_ij)` 中的参数值，特别是与速度、加速度、代价地图膨胀相关的参数，是否符合预期。
  - 查看 ROS 节点的日志输出，确认参数是否被正确加载。