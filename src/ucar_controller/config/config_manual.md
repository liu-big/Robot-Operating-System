# `config` 文件夹使用开发操作手册

## 1. 概述

在 `ucar_controller` ROS 包中，`config` 文件夹用于存放各种配置文件，特别是机器人驱动和控制相关的参数文件。这些文件通常采用 YAML 格式，用于定义不同型号机器人（如 mini, ucarV2, xiao）的特定参数、传感器校准数据、PID 控制器增益等。通过集中管理这些配置，可以方便地调整机器人行为，适应不同的硬件平台和应用场景。

## 2. 文件夹结构

```
src/ucar_controller/config/
├── driver_params_mini.yaml
├── driver_params_ucarV2.yaml
└── driver_params_xiao.yaml
```

- `driver_params_mini.yaml`: 针对 mini 型号机器人的驱动参数配置文件。
- `driver_params_ucarV2.yaml`: 针对 ucarV2 型号机器人的驱动参数配置文件。
- `driver_params_xiao.yaml`: 针对 xiao 型号机器人的驱动参数配置文件。

这些 YAML 文件通常包含键值对，用于定义各种参数，例如：
```yaml
# 示例：driver_params_mini.yaml
wheel_radius: 0.0325
wheel_separation: 0.16
max_linear_velocity: 0.5
max_angular_velocity: 1.0
pid_gains:
  kp: 1.0
  ki: 0.1
  kd: 0.01
```

## 3. 主要功能与用途

`config` 文件夹的主要功能是：

- **参数管理**：集中管理机器人驱动和控制相关的可调参数，便于修改和维护。
- **多平台适应**：为不同型号的机器人提供独立的配置文件，实现参数的定制化，提高代码的复用性。
- **行为调优**：通过修改参数文件，可以方便地调整机器人的运动特性、响应速度和稳定性，而无需修改和重新编译代码。
- **数据映射**：定义传感器数据的转换系数、坐标系偏移等。

## 4. 使用方法

- **创建和编辑配置文件**：
  可以直接使用文本编辑器打开 `.yaml` 文件进行编辑。在修改参数时，请确保遵循 YAML 语法，并注意参数的含义和取值范围。

- **在 `launch` 文件中加载**：
  通常在 ROS 的 `.launch` 文件中加载这些参数文件，以便 ROS 节点在启动时读取所需的配置。例如：
  ```xml
  <launch>
    <arg name="robot_model" default="mini" />
    <param name="robot_description" command="$(find xacro)/xacro $(find ucar_description)/urdf/$(arg robot_model).urdf.xacro" />

    <node pkg="ucar_controller" type="base_driver" name="base_driver_node" output="screen">
      <rosparam file="$(find ucar_controller)/config/driver_params_$(arg robot_model).yaml" command="load" />
    </node>
  </launch>
  ```
  上述示例中，`$(arg robot_model)` 允许根据 `robot_model` 参数动态加载不同的配置文件。

- **在代码中读取参数**：
  ROS 节点可以通过 `rospy.get_param()` (Python) 或 `ros::NodeHandle::param()` (C++) 等函数从 ROS 参数服务器中读取这些已加载的参数。

  Python 示例：
  ```python
  import rospy

  rospy.init_node('my_node')
  wheel_radius = rospy.get_param('~wheel_radius', 0.0)
  rospy.loginfo("Wheel radius: %f" % wheel_radius)
  ```

  C++ 示例：
  ```cpp
  #include <ros/ros.h>

  int main(int argc, char** argv)
  {
    ros::init(argc, argv, "my_node");
    ros::NodeHandle nh("~"); // Private NodeHandle to access private parameters

    double wheel_radius;
    nh.param<double>("wheel_radius", wheel_radius, 0.0); // Parameter name, variable, default value
    ROS_INFO("Wheel radius: %f", wheel_radius);

    return 0;
  }
  ```

## 5. 项目全局应用

在 `ucar_ws` 项目中，`ucar_controller` 包的 `config` 文件夹是实现机器人多型号兼容性和灵活配置的关键。它使得开发者能够为不同硬件配置的机器人提供定制化的控制参数，极大地简化了新机器人平台的集成和现有机器人性能的调优过程。这种参数化的管理方式提高了系统的可配置性和可维护性，是构建适应性强、可扩展的机器人系统的基础。

## 6. 维护与更新

- **版本控制**：所有配置文件都应该被提交到版本控制系统（如 Git），以便团队成员共享和跟踪变更。
- **清晰注释**：在 YAML 文件中添加详细的注释，说明每个参数的含义、单位和推荐值。
- **参数验证**：在代码中对读取的参数进行有效性检查，避免因错误配置导致的问题。
- **命名规范**：遵循一致的参数命名规范，提高可读性。
- **避免硬编码**：尽量将可配置的参数放入配置文件，而不是硬编码在代码中。

## 7. 故障排除

- **参数未加载**：
  - 检查 `launch` 文件中 `rosparam` 标签的 `file` 路径是否正确。
  - 确保 `command="load"` 属性存在。
  - 检查 YAML 文件是否存在语法错误（可以使用在线 YAML 验证工具）。
  - 确保 `launch` 文件被正确执行。
- **代码读取参数失败**：
  - 检查代码中 `get_param` 或 `param` 函数的参数名称是否与 YAML 文件中的键名完全匹配。
  - 检查节点句柄的命名空间是否正确（例如，私有参数需要使用私有节点句柄 `ros::NodeHandle nh("~");`）。
  - 确保参数在节点启动时已成功加载到 ROS 参数服务器（可以使用 `rosparam list` 和 `rosparam get <param_name>` 进行检查）。
- **参数值不正确**：
  - 仔细检查 YAML 文件中的参数值是否符合预期。
  - 确认没有其他地方覆盖了该参数（ROS 参数服务器允许参数被覆盖）。