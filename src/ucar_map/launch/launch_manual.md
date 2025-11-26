# `launch` 文件夹使用开发操作手册

## 1. 概述

在 `ucar_map` ROS 包中，`launch` 文件夹用于存放 `.launch` 文件。这些文件是基于 XML 格式的配置文件，用于定义和启动与地图相关的 ROS 节点、设置参数、以及集成地图构建和定位系统。它们提供了一种方便的方式来管理和启动复杂的地图功能，例如地图服务器、GMapping 或 Cartographer SLAM 算法。

## 2. 文件夹结构

```
src/ucar_map/launch/
├── cartographer_start.launch
├── gmapping.launch
├── gmapping_demo.launch
├── ucar_gmapping.launch
└── ucar_mapping.launch
```

- `cartographer_start.launch`: 用于启动 Cartographer SLAM 算法的 ROS 节点，通常会加载 `cfg` 文件夹中的 Lua 配置文件。
- `gmapping.launch`: 可能是一个通用的 GMapping SLAM 启动文件。
- `gmapping_demo.launch`: 可能是一个用于演示 GMapping SLAM 的启动文件。
- `ucar_gmapping.launch`: 针对 UCAR 机器人定制的 GMapping SLAM 启动文件，可能包含特定的参数或传感器配置。
- `ucar_mapping.launch`: 可能是一个用于启动地图服务器或通用地图相关功能的启动文件。

## 3. 主要功能与用途

`launch` 文件夹的主要功能是：

- **地图服务器启动**：启动 `map_server` 节点，加载预先构建的地图（`.pgm` 和 `.yaml` 文件），并将其发布到 ROS 话题。
- **SLAM 算法启动**：启动 GMapping 或 Cartographer 等 SLAM 算法的 ROS 节点，用于实时构建环境地图。
- **参数配置**：在节点启动时为其设置参数，这些参数会被加载到 ROS 参数服务器，例如 SLAM 算法的各种调优参数。
- **话题重映射**：改变节点发布或订阅的话题名称，以适应不同的系统配置。
- **包含其他 launch 文件**：通过 `include` 标签复用其他 `.launch` 文件，构建模块化的启动配置。

## 4. 使用方法

- **创建和编辑 launch 文件**：
  可以使用文本编辑器创建或修改 `.launch` 文件。例如，`ucar_mapping.launch` 可能包含以下内容：
  ```xml
  <!-- ucar_mapping.launch 示例 -->
  <launch>
    <arg name="map_file" default="$(find ucar_map)/maps/ucar_map_001.yaml"/>

    <!-- 启动地图服务器 -->
    <node name="map_server" pkg="map_server" type="map_server" args="$(arg map_file)" />

    <!-- 启动 AMCL 定位节点（如果需要） -->
    <!-- <include file="$(find amcl)/launch/amcl.launch" /> -->
  </launch>
  ```

- **运行 launch 文件**：
  在终端中使用 `roslaunch` 命令运行 `.launch` 文件：
  ```bash
  roslaunch ucar_map ucar_mapping.launch
  ```
  这会启动 `ucar_mapping.launch` 文件中定义的所有节点，例如加载并发布 `ucar_map_001.yaml` 地图。

## 5. 项目全局应用

在 `ucar_ws` 项目中，`ucar_map` 包的 `launch` 文件夹是部署和运行其地图相关功能的入口。它使得开发者能够通过简单的命令启动复杂的地图系统，包括地图的加载、发布以及实时的 SLAM 算法。这种集中式的启动管理极大地简化了系统的集成和测试过程，确保了 `ucar_map` 能够与其他 ROS 包（如导航包）协同工作，共同实现机器人的自主导航能力。

## 6. 维护与更新

- **版本控制**：所有 `.launch` 文件都应该被提交到版本控制系统（如 Git），以便团队成员共享和跟踪变更。
- **清晰注释**：在 `.launch` 文件中添加详细的注释，说明每个节点、参数和包含文件的用途。
- **模块化**：将复杂的启动逻辑分解为多个小的 `.launch` 文件，并通过 `include` 标签组合，提高可读性和复用性。
- **参数化**：使用 `<arg>` 标签定义可配置的参数，使得 `.launch` 文件更具灵活性。
- **错误处理**：在 `.launch` 文件中可以设置 `required="true"` 来确保关键节点启动失败时整个 launch 文件停止。

## 7. 故障排除

- **launch 文件无法运行**：
  - 检查 XML 语法是否正确，可以使用 XML 验证工具。
  - 检查 `pkg` 和 `type` 属性是否正确指向 ROS 包和可执行文件/脚本。
  - 确保所有引用的文件（如地图文件、其他 launch 文件）路径正确且存在。
  - 查看终端输出的错误信息，通常会指出问题所在。
- **节点未启动或启动失败**：
  - 检查 `output="screen"` 属性，以便在终端中看到节点的输出信息。
  - 检查节点的可执行文件是否具有执行权限。
  - 检查节点所需的依赖是否已安装。
  - 检查参数是否正确加载，以及是否存在参数冲突。
- **地图未发布或发布异常**：
  - 使用 `rostopic list` 检查 `/map` 话题是否正在发布。
  - 使用 `rostopic echo /map` 检查地图数据内容。
  - 在 RViz 中检查 `map` 话题的订阅和显示设置。