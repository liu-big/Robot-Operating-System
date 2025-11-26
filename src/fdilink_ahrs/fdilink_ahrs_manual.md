# FDILink AHRS 驱动使用开发操作手册

## 1. 概述

`fdilink_ahrs` 文件夹包含了用于 FDILink IMU 设备的 ROS 驱动程序。该驱动能够读取 IMU 数据，并将其发布为 ROS 消息，以便在 ROS 生态系统中进行进一步处理和应用。它支持不同类型的 IMU 数据（IMU、AHRS、INSGPS）以及多种设备坐标系。

## 2. 文件夹结构

`src/fdilink_ahrs/` 文件夹的主要内容如下：

- `CMakeLists.txt`: ROS 包的 CMake 构建文件。
- `package.xml`: ROS 包的元数据文件。
- `README.md`: 驱动的简要说明和使用指南。
- `include/`:
  - `ahrs_driver.h`: 驱动程序的核心头文件，定义了 `ahrsBringup` 类，负责串口通信、数据解析和 ROS 消息发布。
  - `crc_table.h`: CRC 校验相关的头文件。
  - `fdilink_data_struct.h`: 定义了 FDILink IMU 数据的各种结构体，包括原始 IMU 数据、AHRS 数据和 INSGPS 数据。
- `launch/`:
  - `ahrs_driver.launch`: ROS 启动文件，用于配置和启动 `ahrs_driver` 节点。
- `src/`:
  - `ahrs_driver.cpp`: 驱动程序的实现文件，包含数据处理逻辑和 ROS 接口。
  - `crc_table.cpp`: CRC 校验的实现文件。

## 3. 功能描述

`fdilink_ahrs` 驱动主要实现以下功能：

- **串口通信**: 通过 `serial` ROS 包与 FDILink IMU 设备进行串口通信。
- **数据解析**: 解析从 IMU 设备接收到的原始数据帧，支持 IMU、AHRS 和 INSGPS 三种数据类型。
- **数据校验**: 对接收到的数据进行 CRC 校验，确保数据完整性。
- **ROS 消息发布**: 将解析后的 IMU 数据发布为标准的 `sensor_msgs/Imu` 消息和 `geometry_msgs/Pose2D` 消息。
- **坐标系转换**: 支持根据 `device_type` 参数将 IMU 数据转换为不同的坐标系，以适应不同的应用场景（例如，Deta-10 原始坐标系、单 IMU 坐标系、ucar ROS 坐标系等）。
- **参数配置**: 允许通过 ROS 参数配置串口、波特率、话题名称、帧 ID 和设备类型等。

## 4. 使用说明

### 4.1 依赖安装

在使用 `fdilink_ahrs` 驱动之前，需要安装 ROS 的 `serial` 包：

```bash
sudo apt install ros-melodic-serial
```

### 4.2 启动驱动

通过 `ahrs_driver.launch` 文件启动 `fdilink_ahrs` 驱动节点：

```bash
roslaunch fdilink_ahrs ahrs_driver.launch
```

### 4.3 `ahrs_driver.launch` 配置

`ahrs_driver.launch` 文件允许配置以下参数：

- `debug` (bool): 是否输出调试信息。默认为 `false`。
- `port` (string): 串口设备路径。例如，`/dev/ttyUSB0`。可以通过 `rules.d` 配置固定串口设备。
- `baud` (int): 串口波特率。默认为 `921600`。
- `imu_topic` (string): 发布的 IMU 话题名称。默认为 `/imu`。
- `imu_frame` (string): 发布的 IMU 话题中的 `frame_id`。默认为 `imu`。
- `mag_pose_2d_topic` (string): 发布的二维指北角话题名称。默认为 `/mag_pose_2d`。
- `device_type` (int): 发布的数据基于不同设备有不同的坐标系。
  - `0`: Deta-10 的原始坐标系模式。
  - `1`: 单独 IMU 的坐标系模式（适用于 ROS 中的单 IMU 或 ucar）。
  - `2`: 适用于 ROS 中的 Xiao 设备。

**示例配置:**

```xml
<launch>
  <node pkg="fdilink_ahrs" name="ahrs_driver" type="ahrs_driver" output="screen" >
    <param name="debug"  value="false"/>
    <param name="port"  value="/dev/ttyUSB0"/>
    <param name="baud"  value="921600"/>
    <param name="imu_topic"  value="/imu"/>
    <param name="imu_frame"  value="imu"/>
    <param name="mag_pose_2d_topic"  value="/mag_pose_2d"/>
    <param name="device_type"  value="1"/>
  </node>
</launch>
```

### 4.4 发布的 ROS 话题

`ahrs_driver` 节点会发布以下 ROS 话题：

- `/imu` (sensor_msgs/Imu):
  - 包含 IMU 的加速度、角速度和姿态（四元数）信息。
  - `orientation_covariance`、`angular_velocity_covariance` 和 `linear_acceleration_covariance` 字段通常用于表示数据的协方差，如果设备不提供，可能为零矩阵或固定值。

  ```
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  geometry_msgs/Quaternion orientation
    float64 x
    float64 y
    float64 z
    float64 w
  float64[9] orientation_covariance
  geometry_msgs/Vector3 angular_velocity
    float64 x
    float64 y
    float64 z
  float64[9] angular_velocity_covariance
  geometry_msgs/Vector3 linear_acceleration
    float64 x
    float64 y
    float64 z
  float64[9] linear_acceleration_covariance
  ```

- `/mag_pose_2d` (geometry_msgs/Pose2D):
  - 包含二维指北角信息。
  - `theta` 字段表示指北角，北为 0，逆时针增加，取值范围 0~2π。

  ```
  float64 x
  float64 y
  float64 theta  # 指北角
  ```

## 5. 项目全局应用

`fdilink_ahrs` 驱动在 ROS 项目中扮演着重要的角色，特别是在需要精确姿态、运动和导航信息的应用中。以下是一些常见的应用场景：

- **机器人导航与定位**: IMU 数据是机器人导航和定位系统的关键输入。通过融合 IMU 数据与其他传感器（如 GPS、里程计）的数据，可以实现更精确的机器人位姿估计。
- **姿态控制**: 机器人的姿态（俯仰、横滚、偏航）信息可以直接用于机器人控制系统，实现姿态稳定和运动控制。
- **运动分析**: IMU 提供的加速度和角速度数据可用于分析机器人的运动状态，例如速度、加速度和角速度的变化。
- **传感器融合**: `fdilink_ahrs` 发布的 IMU 消息可以方便地与其他 ROS 传感器数据进行融合，例如在 `robot_localization` 包中使用 IMU 数据进行扩展卡尔曼滤波 (EKF) 或无迹卡尔曼滤波 (UKF)。
- **数据记录与回放**: 可以使用 ROS 的 `rosbag` 工具记录 IMU 数据，以便后续分析、调试或离线算法开发。

## 6. 维护与更新

- **2020-1-15**: 维护了文件注释。
- **2020-10-20**: 添加了 `device_type` 参数，可以在 `ahrs_driver.launch` 文件中指定设备类型，根据不同设备类型以不同的坐标系发布 ROS 的 IMU 数据。

## 7. 故障排除

- **串口通信问题**: 检查串口设备路径 (`port` 参数) 是否正确，串口权限是否足够。可以使用 `ls /dev/tty*` 查看可用的串口设备，并使用 `sudo chmod 666 /dev/ttyUSB0` (替换为实际串口) 临时修改权限。
- **数据接收异常**: 检查波特率 (`baud` 参数) 是否与 IMU 设备匹配。如果数据帧校验失败，可能是波特率不匹配或数据线连接问题。
- **话题未发布**: 检查 `ahrs_driver` 节点是否成功启动，是否有错误信息输出。可以使用 `rostopic list` 查看当前发布的 ROS 话题。
- **坐标系问题**: 如果发布的 IMU 数据姿态不正确，请检查 `device_type` 参数是否与您的 IMU 设备和应用场景匹配。

通过本手册，开发者可以更好地理解和使用 `fdilink_ahrs` 驱动，并将其集成到自己的 ROS 项目中。