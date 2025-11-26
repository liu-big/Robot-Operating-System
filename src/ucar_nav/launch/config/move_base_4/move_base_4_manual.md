# `move_base_4` 文件夹使用开发操作手册

## 1. 概述

在 `ucar_nav` ROS 包的 `launch/config` 文件夹中，`move_base_4` 文件夹是 `move_base` 节点配置文件的另一个副本，其命名中的数字“4”可能表示这是一个特定的版本、迭代或针对某种特定场景（例如，第四个测试版本，或针对四轮机器人的配置）。它包含了与 `move_base` 导航栈相关的各种 YAML 配置文件，用于定义全局路径规划器、局部路径规划器和代价地图的行为和参数。这个文件夹的存在是为了方便开发者在不修改主配置的情况下，并行地测试和管理多套导航参数，并明确区分不同的配置集。

## 2. 文件夹结构

```
src/ucar_nav/launch/config/move_base_4/
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

这些文件的具体作用和内容与 `move_base` 文件夹中的文件类似，主要区别在于它们可能包含不同的参数值，以实现不同的导航行为或适应不同的测试场景，并通过数字“4”进行版本或场景的区分。

## 3. 主要功能与用途

`move_base_4` 文件夹的主要功能是：

- **特定版本/场景配置**：用于存储和管理针对特定版本、迭代或应用场景的 `move_base` 参数配置。
- **并行测试与比较**：允许开发者在不干扰其他配置的情况下，对这套参数进行独立的测试和性能比较。
- **配置快照**：作为某个开发阶段或测试结果的配置快照，便于后续回溯和分析。

## 4. 使用方法

- **加载特定版本配置**：
  在主导航 `launch` 文件（如 `ucar_navigation.launch`）中，将加载 `move_base` 参数的路径指向 `move_base_4` 文件夹。例如：
  ```xml
  <!-- 示例：加载 move_base_4 中的参数 -->
  <rosparam command="load" file="$(find ucar_nav)/launch/config/move_base_4/costmap_common_params.yaml" ns="global_costmap" />
  <rosparam command="load" file="$(find ucar_nav)/launch/config/move_base_4/global_costmap_params.yaml" ns="global_costmap" />
  <rosparam command="load" file="$(find ucar_nav)/launch/config/move_base_4/local_costmap_params.yaml" ns="local_costmap" />
  <!-- ... 其他参数文件 ... -->
  ```
- **修改参数**：
  直接编辑 `move_base_4` 文件夹中的 YAML 文件来修改参数。修改后，需要重新启动导航系统以使更改生效。

## 5. 项目全局应用

在 `ucar_ws` 项目中，`ucar_nav` 包的 `move_base_4` 文件夹是实现导航参数灵活管理和迭代优化的重要组成部分。它使得开发者能够并行地进行多项实验，快速验证不同参数组合的效果，从而加速导航系统的开发和性能提升。这种通过数字区分的命名方式有助于清晰地管理和追踪不同配置集。

## 6. 维护与更新

- **清晰命名**：为每个编号的 `move_base` 文件夹添加清晰的注释或命名约定，说明其用途或对应的实验。
- **定期清理**：删除不再需要或过时的编号文件夹，避免配置混乱。
- **版本控制**：确保所有编号文件夹及其内容都纳入版本控制，并记录每次更新的详细信息。

## 7. 故障排除

- **加载错误**：
  - 检查 `launch` 文件中引用的 `move_base_4` 路径是否正确。
  - 确保文件夹中的 YAML 文件语法正确。
- **导航行为与预期不符**：
  - 确认当前加载的是 `move_base_4` 中的参数，而不是其他配置。
  - 仔细检查 `move_base_4` 中的参数值，特别是与速度、加速度、代价地图膨胀相关的参数，是否符合预期。
  - 查看 ROS 节点的日志输出，确认参数是否被正确加载。