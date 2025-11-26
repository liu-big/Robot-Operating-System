# `geometry2` 文件夹使用开发操作手册

## 1. 概述

`geometry2` 是 ROS 中用于处理坐标系变换的第二代库 `tf2` 的元包。它提供了一套强大的工具，用于跟踪多个坐标系随时间的变化，并在这些坐标系之间转换点、向量、姿态等数据。`tf2` 旨在取代旧的 `tf` 库，提供更高效、更灵活的 API。

`geometry2` 文件夹包含了 `tf2` 核心库及其各种数据类型转换包，例如 `tf2_bullet`、`tf2_eigen`、`tf2_geometry_msgs`、`tf2_kdl`、`tf2_msgs`、`tf2_py`、`tf2_ros`、`tf2_sensor_msgs` 和 `tf2_tools`。

## 2. 文件夹结构

`./`

- `src/geometry2/`: `geometry2` 元包的定义，主要包含 `package.xml`。
- `src/geometry2/test_tf2/`: `tf2` 的测试相关文件。
- `src/geometry2/tf2/`: `tf2` 核心库，提供坐标系变换的核心功能。
  - `src/geometry2/tf2/include/tf2/`: `tf2` 的头文件。
  - `src/geometry2/tf2/src/`: `tf2` 的源文件。
  - `src/geometry2/tf2/mainpage.dox`: `tf2` 的 Doxygen 文档主页。
  - `src/geometry2/tf2/package.xml`: `tf2` 包的定义文件。
- `src/geometry2/tf2_bullet/`: 提供 `tf2` 与 Bullet 物理引擎数据类型之间的转换。
- `src/geometry2/tf2_eigen/`: 提供 `tf2` 与 Eigen 线性代数库数据类型之间的转换。
- `src/geometry2/tf2_geometry_msgs/`: 提供 `tf2` 与 `geometry_msgs` 消息类型之间的转换。
- `src/geometry2/tf2_kdl/`: 提供 `tf2` 与 KDL (Kinematics and Dynamics Library) 数据类型之间的转换。
  - `src/geometry2/tf2_kdl/mainpage.dox`: `tf2_kdl` 的 Doxygen 文档主页。
  - `src/geometry2/tf2_kdl/package.xml`: `tf2_kdl` 包的定义文件。
- `src/geometry2/tf2_msgs/`: 定义 `tf2` 相关的 ROS 消息和服务类型。
- `src/geometry2/tf2_py/`: `tf2` 的 Python 绑定。
- `src/geometry2/tf2_ros/`: `tf2` 的 ROS 接口，提供 ROS 环境下的 `tf2` 功能。
- `src/geometry2/tf2_sensor_msgs/`: 提供 `tf2` 与 `sensor_msgs` 消息类型之间的转换。
- `src/geometry2/tf2_tools/`: 包含 `tf2` 的一些实用工具，如 `view_frames.py`。

## 3. 主要功能与用途

`geometry2` 及其包含的 `tf2` 库主要用于解决机器人应用中不同坐标系之间的数据转换问题。在机器人系统中，通常会有多个传感器（如摄像头、激光雷达、IMU）和执行器，它们各自在不同的坐标系中工作。`tf2` 允许开发者：

- **跟踪坐标系关系**：维护一个随时间变化的坐标系树，表示机器人内部各个部件以及机器人与外部环境之间的相对姿态。
- **数据转换**：将点、向量、姿态等数据从一个坐标系转换到另一个坐标系。
- **时间缓冲**：支持在过去任意时间点进行坐标系转换，这对于处理传感器数据的时间同步非常有用。

### 3.1 `tf2` 核心库

`tf2` 核心库提供了坐标系变换的基础功能。其主要接口是 `tf2::BufferCore`，它负责存储和管理坐标系之间的变换关系。开发者可以通过 `tf2::BufferCore` 查询任意两个坐标系在特定时间点的变换。

### 3.2 `tf2_kdl`

`tf2_kdl` 包提供了 `geometry_msgs` 消息类型与 KDL (Kinematics and Dynamics Library) 数据类型之间的转换函数。KDL 是一个用于机器人运动学和动力学计算的库，`tf2_kdl` 使得在 `tf2` 和 KDL 之间传递数据变得更加方便。

### 3.3 其他转换包

- `tf2_bullet`：用于 `tf2` 与 Bullet 物理引擎数据类型之间的转换。
- `tf2_eigen`：用于 `tf2` 与 Eigen 线性代数库数据类型之间的转换。
- `tf2_geometry_msgs`：用于 `tf2` 与 ROS 标准 `geometry_msgs` 消息类型（如 `PoseStamped`、`PointStamped`、`Vector3Stamped` 等）之间的转换。
- `tf2_sensor_msgs`：用于 `tf2` 与 ROS 标准 `sensor_msgs` 消息类型（如 `PointCloud2`）之间的转换。

## 4. 使用方法

### 4.1 安装依赖

`geometry2` 通常作为 ROS 的一部分安装。如果您的 ROS 环境已正确配置，则无需额外安装。如果需要手动安装，可以通过 ROS 的包管理工具进行安装：

```bash
sudo apt-get install ros-<ros_distro>-geometry2
```

### 4.2 发布坐标系变换 (C++ 示例)

使用 `tf2_ros::TransformBroadcaster` 发布坐标系变换：

```cpp
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf2_broadcaster");
  ros::NodeHandle node;

  tf2_ros::TransformBroadcaster tfb;
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.frame_id = "world";
  transformStamped.child_frame_id = "turtle1";
  transformStamped.transform.translation.x = 0.0;
  transformStamped.transform.translation.y = 0.0;
  transformStamped.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, 0);
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();

  ros::Rate rate(10.0);
  while (node.ok()){
    transformStamped.header.stamp = ros::Time::now();
    tfb.sendTransform(transformStamped);
    rate.sleep();
  }
  return 0;
};
```

### 4.3 监听并转换数据 (C++ 示例)

使用 `tf2_ros::Buffer` 和 `tf2_ros::TransformListener` 监听并转换数据：

```cpp
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf2_listener");
  ros::NodeHandle node;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Rate rate(10.0);
  while (node.ok()){
    geometry_msgs::PointStamped laser_point;
    laser_point.header.frame_id = "laser";
    laser_point.header.stamp = ros::Time(0); // Use latest available transform
    laser_point.point.x = 1.0;
    laser_point.point.y = 2.0;
    laser_point.point.z = 0.0;

    try{
      geometry_msgs::PointStamped base_point;
      tfBuffer.transform(laser_point, base_point, "base_link");
      ROS_INFO("Point in base_link: (%.2f, %.2f, %.2f)",
               base_point.point.x, base_point.point.y, base_point.point.z);
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    rate.sleep();
  }
  return 0;
};
```

### 4.4 `tf2_kdl` 转换示例

```cpp
#include <tf2_kdl/tf2_kdl.h>
#include <geometry_msgs/TransformStamped.h>
#include <kdl/frames.hpp>

int main(int argc, char** argv)
{
  // geometry_msgs::TransformStamped to KDL::Frame
  geometry_msgs::TransformStamped transform_stamped;
  // ... populate transform_stamped ...
  KDL::Frame kdl_frame;
  tf2::fromMsg(transform_stamped, kdl_frame);

  // KDL::Frame to geometry_msgs::TransformStamped
  KDL::Frame another_kdl_frame;
  // ... populate another_kdl_frame ...
  geometry_msgs::TransformStamped another_transform_stamped = tf2::toMsg(another_kdl_frame);

  return 0;
}
```

## 5. 项目全局应用

在 `ucar_ws` 项目中，`geometry2` 文件夹中的 `tf2` 库是构建机器人感知、导航和控制系统的核心组件。它被广泛应用于：

- **传感器数据融合**：将来自不同传感器（如激光雷达、摄像头、IMU）的数据转换到统一的坐标系中进行处理。
- **机器人运动学**：计算机器人关节和末端执行器在不同坐标系中的位置和姿态。
- **导航**：在地图坐标系、机器人自身坐标系和传感器坐标系之间进行转换，以实现定位和路径规划。
- **可视化**：在 RViz 等工具中显示不同坐标系及其之间的变换关系，帮助开发者理解机器人状态。

例如，在 `ucar_ws` 中，如果存在一个激光雷达驱动包，它可能会发布激光扫描数据在 `laser_frame` 坐标系中。而导航堆栈可能期望在 `base_link` 坐标系中接收数据。此时，`tf2` 就会被用来将 `laser_frame` 中的数据转换到 `base_link` 中。

## 6. 维护与更新

- **版本兼容性**：`tf2` 是 ROS 的核心库，其 API 相对稳定。但在升级 ROS 版本时，仍需注意 `geometry2` 包的版本兼容性。
- **性能优化**：对于需要大量坐标系变换的实时应用，应关注 `tf2` 的性能，例如避免不必要的查询，或者使用 `tf2::MessageFilter` 来优化传感器数据的处理。
- **坐标系树管理**：确保所有重要的坐标系都已发布，并且坐标系树是完整且无环的。可以使用 `rosrun tf2 tf2_monitor` 或 `rosrun tf2_tools view_frames.py` 来检查坐标系树。

## 7. 故障排除

- **`TransformException`**：当 `tf2` 无法找到请求的变换时，会抛出 `tf2::TransformException`。常见原因包括：
  - **坐标系未发布**：确保所有涉及的坐标系都已通过 `TransformBroadcaster` 发布。
  - **时间不同步**：请求的变换时间点在 `tf2` 的缓冲区之外。可以尝试使用 `ros::Time(0)` 来获取最新变换，或者调整 `tf2` 缓冲区的长度。
  - **坐标系名称拼写错误**：检查 `frame_id` 和 `child_frame_id` 是否正确。
- **循环变换**：坐标系树中存在循环会导致 `tf2` 无法正确计算变换。使用 `view_frames.py` 工具可以帮助检测循环。
- **性能问题**：如果 `tf2` 转换导致 CPU 占用过高，可以检查是否在循环中频繁创建 `TransformListener` 对象（应只创建一个），或者是否进行了大量不必要的历史变换查询。