# `cfg` 文件夹使用开发操作手册

## 1. 概述

在 `ucar_map` ROS 包中，`cfg` 文件夹用于存放与地图构建和定位相关的配置文件，特别是针对 Cartographer 这样的高级 SLAM 算法。这些配置文件通常采用 Lua 语言编写，用于定义算法的各种参数、传感器配置、优化设置等。它们是调整 SLAM 算法性能和适应不同机器人平台及环境的关键。

## 2. 文件夹结构

```
src/ucar_map/cfg/
├── carto_2d.lua
├── localization_2d.lua
└── test_2d.lua
```

- `carto_2d.lua`: Cartographer 2D SLAM 算法的主配置文件，定义了传感器输入、前端（scan matching）和后端（loop closure、optimization）的参数。
- `localization_2d.lua`: Cartographer 2D 定位模式的配置文件，通常用于在已知地图中进行机器人定位。
- `test_2d.lua`: 可能是一个用于测试或特定场景的 Cartographer 2D 配置。

这些 Lua 文件通常包含一系列 `MAP_BUILDER`、`TRAJECTORY_BUILDER_2D`、`POSE_GRAPH` 等配置块，用于设置 Cartographer 的详细参数。

## 3. 主要功能与用途

`cfg` 文件夹的主要功能是：

- **参数配置**：为 Cartographer 等 SLAM 算法提供详细的参数配置，包括传感器数据源、滤波器设置、扫描匹配算法、回环检测、位姿图优化等。
- **算法调优**：通过修改配置文件中的参数，可以对 SLAM 算法进行精细调优，以适应不同的传感器特性、机器人运动模式和环境复杂度。
- **模式切换**：定义不同运行模式（如建图模式、定位模式）的配置，方便在不同应用场景之间切换。
- **模块化管理**：将复杂的算法参数集中管理，提高配置的可读性和可维护性。

## 4. 使用方法

- **编辑 Lua 配置文件**：
  可以使用文本编辑器打开 `.lua` 文件进行修改。例如，`carto_2d.lua` 中可能包含以下参数：
  ```lua
  -- carto_2d.lua 示例片段
  MAP_BUILDER.use_trajectory_builder_2d = true

  TRAJECTORY_BUILDER_2D.min_range = 0.1
  TRAJECTORY_BUILDER_2D.max_range = 30.0
  TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0
  TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
  TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05

  POSE_GRAPH.constraint_builder.min_score = 0.55
  POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6
  -- ... 更多参数 ...
  ```

- **在 `launch` 文件中加载配置**：
  在 `launch` 文件夹中的 `.launch` 文件中，通过 `configuration_basename` 参数指定要加载的 Lua 配置文件：
  ```xml
  <!-- launch/cartographer_start.launch 示例 -->
  <launch>
    <param name="robot_description" textfile="$(find ucar_description)/urdf/ucar.urdf" />

    <node name="cartographer_node" pkg="cartographer_ros" type="cartographer_node"
          args="-configuration_directory $(find ucar_map)/cfg
                -configuration_basename carto_2d.lua"
          output="screen">
      <remap from="scan" to="/scan" />
      <!-- ... 其他 remap ... -->
    </node>

    <node name="cartographer_occupancy_grid_node" pkg="cartographer_ros" type="cartographer_occupancy_grid_node"
          args="-resolution 0.05" />
  </launch>
  ```

## 5. 项目全局应用

在 `ucar_ws` 项目中，`ucar_map` 包的 `cfg` 文件夹是 Cartographer SLAM 算法的核心配置中心。它使得开发者能够根据不同的机器人传感器配置、环境特点和性能要求，灵活地调整 SLAM 算法的行为。通过这些 Lua 配置文件，`ucar_map` 包能够为整个机器人系统提供高精度、鲁棒的地图构建和定位能力，是实现机器人自主导航的关键支撑。

## 6. 维护与更新

- **版本控制**：所有 `.lua` 配置文件都应该被提交到版本控制系统（如 Git），以便团队成员共享和跟踪变更。
- **清晰注释**：在配置文件中添加详细的注释，说明每个参数的含义、取值范围和对算法性能的影响。
- **参数化**：对于经常需要调整的参数，可以考虑在 `launch` 文件中通过 `<arg>` 标签进行参数化，提高灵活性。
- **测试与验证**：每次修改配置后，都应进行充分的测试和验证，确保算法性能符合预期。

## 7. 故障排除

- **配置加载失败**：
  - 检查 `launch` 文件中 `configuration_directory` 和 `configuration_basename` 参数是否正确指向 `.lua` 文件。
  - 检查 `.lua` 文件是否存在且路径正确。
  - 检查 `.lua` 文件是否存在语法错误。
- **SLAM 性能不佳**：
  - 仔细检查 `.lua` 文件中的传感器参数（如 `min_range`, `max_range`, `voxel_filter_size`）是否与实际传感器匹配。
  - 调整扫描匹配和位姿图优化参数（如 `min_score`, `loop_closure_min_score`）。
  - 检查传感器数据质量，确保没有噪声或数据丢失。
  - 查看 Cartographer 节点的日志输出，查找警告或错误信息。