# `move_base copy（last）` 文件夹使用开发操作手册

## 1. 概述

在 `ucar_nav` ROS 包的 `launch/config` 文件夹中，`move_base copy（last）` 文件夹是 `move_base` 节点配置文件的另一个副本，其命名暗示了这是上一个或最近一次使用的配置版本。它包含了与 `move_base` 导航栈相关的各种 YAML 配置文件，用于定义全局路径规划器、局部路径规划器和代价地图的行为和参数。这个文件夹的存在通常是为了保留一个在开发过程中被认为是“最后一次”或“最近一次”的有效配置，以便在新的更改引入问题时能够快速回溯。

## 2. 文件夹结构

```
src/ucar_nav/launch/config/move_base copy（last）/
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

这些文件的具体作用和内容与 `move_base` 文件夹中的文件类似，主要区别在于它们被明确标记为“last”版本，通常代表了在某个时间点被认为是稳定或最终的配置。

## 3. 主要功能与用途

`move_base copy（last）` 文件夹的主要功能是：

- **快速回溯**：当新的配置或代码更改导致导航问题时，可以迅速切换回这个“last”版本，以恢复导航功能。
- **历史记录**：作为开发过程中一个重要配置节点的快照，记录了某个阶段的参数设置。
- **对比分析**：可以与当前正在使用的配置进行对比，分析参数变化对导航性能的影响。

## 4. 使用方法

- **加载“last”配置**：
  在主导航 `launch` 文件（如 `ucar_navigation.launch`）中，将加载 `move_base` 参数的路径指向 `move_base copy（last）` 文件夹。例如：
  ```xml
  <!-- 示例：加载 move_base copy（last） 中的参数 -->
  <rosparam command="load" file="$(find ucar_nav)/launch/config/move_base copy（last）/costmap_common_params.yaml" ns="global_costmap" />
  <rosparam command="load" file="$(find ucar_nav)/launch/config/move_base copy（last）/global_costmap_params.yaml" ns="global_costmap" />
  <rosparam command="load" file="$(find ucar_nav)/launch/config/move_base copy（last）/local_costmap_params.yaml" ns="local_costmap" />
  <!-- ... 其他参数文件 ... -->
  ```
- **作为回滚点**：
  在进行重大更改或部署新功能之前，可以考虑将当前稳定配置复制到这个文件夹，作为潜在的回滚点。

## 5. 项目全局应用

在 `ucar_ws` 项目中，`ucar_nav` 包的 `move_base copy（last）` 文件夹是开发和调试导航系统时的重要辅助工具。它提供了一个方便的回滚机制，降低了引入新功能或优化参数时的风险。这种版本管理策略有助于保持开发流程的顺畅，并确保在遇到问题时能够快速恢复。

## 6. 维护与更新

- **定期更新**：当有新的稳定配置出现时，可以考虑更新这个“last”版本。
- **清晰命名**：确保文件夹命名清晰，准确反映其“last”的状态。
- **版本控制**：确保所有文件都纳入版本控制，并记录每次更新的详细信息。
- **避免滥用**：不应频繁创建过多的“last”副本，以免造成混乱。

## 7. 故障排除

- **加载错误**：
  - 检查 `launch` 文件中引用的 `move_base copy（last）` 路径是否正确。
  - 确保文件夹中的 YAML 文件语法正确。
- **回滚后问题依旧**：
  - 如果回滚到“last”版本后问题仍然存在，可能意味着问题并非出在参数配置上，需要检查代码或其他系统组件。
  - 确认当前加载的是这个“last”版本，而不是其他配置。
  - 查看 ROS 节点的日志输出，确认参数是否被正确加载。