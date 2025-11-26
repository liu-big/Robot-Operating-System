# `maps` 文件夹使用开发操作手册

## 1. 概述

在 `ucar_map` ROS 包中，`maps` 文件夹专门用于存放机器人导航和定位所需的地图文件。这些地图通常以图像文件（`.pgm`）和元数据文件（`.yaml`）的形式成对出现。`.pgm` 文件表示环境的占用栅格图，而 `.yaml` 文件则提供了地图的分辨率、原点、图像路径等关键信息。这个文件夹是机器人进行自主导航的基础，因为它提供了机器人操作环境的静态表示。

## 2. 文件夹结构

```
src/ucar_map/maps/
├── 123.pgm
├── 123.yaml
├── map(1).pgm
├── mapname.pgm
├── mapname.yaml
├── ucar_map_001.pgm
└── ucar_map_001.yaml
```

- `*.pgm`: 图像文件，通常是灰度图，表示环境的占用栅格地图。白色像素代表空闲空间，黑色像素代表障碍物，灰色像素代表未知区域。
- `*.yaml`: YAML 格式的配置文件，与同名的 `.pgm` 文件配对使用，描述了地图的元数据，包括：
  - `image`: `.pgm` 文件的相对路径。
  - `resolution`: 地图分辨率，每个像素代表的米数。
  - `origin`: 地图原点在世界坐标系中的 (x, y, yaw) 坐标。yaw 是地图的旋转角度。
  - `negate`: 一个布尔值，表示是否反转占用值。
  - `occupied_thresh`: 大于此值的像素被认为是占用的。
  - `free_thresh`: 小于此值的像素被认为是空闲的。

## 3. 主要功能与用途

`maps` 文件夹的主要功能是：

- **地图存储**：集中存储机器人操作环境的各种地图，方便管理和切换。
- **导航基础**：为 ROS 导航栈（如 `move_base`）提供环境信息，用于路径规划和避障。
- **定位参考**：为定位算法（如 AMCL）提供参考地图，帮助机器人在环境中确定自身位置。
- **多环境支持**：允许存储多个不同环境的地图，使机器人能够在不同场景下工作。

## 4. 使用方法

- **生成地图**：
  地图通常通过 SLAM（Simultaneous Localization and Mapping）算法（如 GMapping, Cartographer）在机器人探索环境时自动生成。例如，运行 SLAM 节点后，可以使用 `map_saver` 工具保存地图：
  ```bash
  rosrun map_server map_saver -f src/ucar_map/maps/new_map_name
  ```
  这会在 `src/ucar_map/maps/` 目录下生成 `new_map_name.pgm` 和 `new_map_name.yaml` 文件。

- **加载地图**：
  在 `launch` 文件中，可以使用 `map_server` 节点加载 `maps` 文件夹中的地图。例如：
  ```xml
  <!-- launch/load_my_map.launch 示例 -->
  <launch>
    <arg name="map_file" default="$(find ucar_map)/maps/ucar_map_001.yaml"/>
    <node name="map_server" pkg="map_server" type="map_server" args="$(arg map_file)" />
  </launch>
  ```
  运行：`roslaunch ucar_map load_my_map.launch`

- **在 RViz 中查看地图**：
  启动 `map_server` 后，可以在 RViz 中添加 `Map` 显示类型，并订阅 `/map` 话题来可视化加载的地图。

## 5. 项目全局应用

在 `ucar_ws` 项目中，`ucar_map` 包的 `maps` 文件夹是机器人自主导航系统的核心数据源。它提供了机器人所处环境的精确表示，使得机器人能够进行有效的路径规划、避障和自身定位。通过存储和管理多张地图，`maps` 文件夹支持机器人在不同工作环境中灵活切换，是构建智能移动机器人不可或缺的一部分。

## 6. 维护与更新

- **命名规范**：为地图文件使用清晰、有意义的命名，例如包含地图的地点或版本信息。
- **备份**：定期备份重要的地图文件。
- **更新地图**：当环境发生变化时（如添加或移除障碍物），需要更新或重新生成地图。
- **分辨率选择**：根据导航精度和计算资源的需求，选择合适的地图分辨率。
- **版本控制**：虽然 `.pgm` 文件通常较大不适合直接版本控制，但 `.yaml` 文件和地图生成脚本应纳入版本控制。

## 7. 故障排除

- **地图加载失败**：
  - 检查 `.pgm` 和 `.yaml` 文件是否存在于指定路径。
  - 检查 `.yaml` 文件中的 `image` 字段是否正确指向 `.pgm` 文件。
  - 检查 `.yaml` 文件语法是否正确。
  - 确保 `map_server` 节点已启动。
- **地图显示不正确**：
  - 在 RViz 中检查 `Map` 显示的 `Topic` 是否设置为 `/map`。
  - 检查 RViz 的 `Fixed Frame` 是否设置为 `map`。
  - 检查 `.yaml` 文件中的 `resolution` 和 `origin` 参数是否正确。
  - 确保机器人坐标系（如 `base_link`）与地图坐标系（`map`）之间的 TF 变换正确。
- **地图与实际环境不符**：
  - 重新进行 SLAM 建图，确保环境变化已反映在地图中。
  - 检查传感器数据质量，确保建图时没有异常数据。