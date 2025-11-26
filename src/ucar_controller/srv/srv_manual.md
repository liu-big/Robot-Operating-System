# `srv` 文件夹使用开发操作手册

## 1. 概述

在 `ucar_controller` ROS 包中，`srv` 文件夹用于定义 ROS 服务（Service）的消息类型。ROS 服务是一种请求/响应通信机制，允许一个节点（客户端）向另一个节点（服务器）发送请求，并等待服务器返回响应。这种机制适用于需要即时响应的、一对一的通信模式，例如请求机器人执行某个动作并等待结果，或者查询某个状态信息。

## 2. 文件夹结构

```
src/ucar_controller/srv/
├── SetPose.srv
└── SetSpeed.srv
```

- `SetPose.srv`: 定义了一个服务消息，可能用于设置机器人的目标位姿（位置和方向）。
- `SetSpeed.srv`: 定义了一个服务消息，可能用于设置机器人的线速度和角速度。

每个 `.srv` 文件都包含请求（Request）和响应（Response）两部分，由 `---` 分隔。例如，`SetPose.srv` 可能包含：

```
# 请求部分
geometry_msgs/Pose2D pose
---
# 响应部分
bool success
string message
```

## 3. 主要功能与用途

`srv` 文件夹的主要功能是：

- **定义服务接口**：明确客户端可以向服务器发送什么数据（请求），以及服务器会返回什么数据（响应）。
- **实现请求/响应通信**：为 `ucar_controller` 包中的节点提供一种同步的、可靠的通信方式。
- **模块化功能**：将特定的功能封装为服务，方便其他节点调用。
- **简化交互**：客户端无需了解服务器的内部实现细节，只需按照服务接口进行调用。

## 4. 使用方法

- **定义 `.srv` 文件**：
  在 `srv` 文件夹中创建 `.srv` 文件，按照 ROS 服务消息的语法定义请求和响应字段。例如 `SetSpeed.srv`：
  ```
  # 请求部分：设置线速度和角速度
  float32 linear_x
  float32 angular_z
  ---
  # 响应部分：指示操作是否成功
  bool success
  string message
  ```

- **在 `CMakeLists.txt` 中配置**：
  为了让 ROS 能够生成 `.srv` 文件对应的 C++、Python 等语言的头文件或类，需要在 `ucar_controller` 包的 `CMakeLists.txt` 中添加配置：
  ```cmake
  # ...
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation # 添加这一行以生成消息和服务
    geometry_msgs      # 如果服务中使用了 geometry_msgs/Pose2D
  )

  # ...

  # 添加服务文件
  add_service_files(
    FILES
    SetPose.srv
    SetSpeed.srv
  )

  # ...

  # 确保在生成消息和服务的依赖之后再添加可执行文件和库
  add_dependencies(${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_generate_messages_py)
  # ...
  ```

- **在 `package.xml` 中配置**：
  在 `package.xml` 中添加 `message_generation` 和 `message_runtime` 的依赖：
  ```xml
  <!-- ... -->
  <build_depend>message_generation</build_depend>
  <exec_depend>message_runtime</exec_depend>
  <!-- ... -->
  ```

- **编译**：
  在工作区根目录（`ucar_ws`）下使用 `catkin_make` 或 `catkin build` 命令编译整个工作区。编译后，ROS 会自动生成对应的 C++ 头文件（在 `devel/include/ucar_controller/` 或 `install/include/ucar_controller/` 下）和 Python 模块。

- **在代码中使用服务**：
  **C++ 服务服务器示例**：
  ```cpp
  #include "ros/ros.h"
  #include "ucar_controller/SetSpeed.h"

  bool handle_set_speed(ucar_controller::SetSpeed::Request &req,
                        ucar_controller::SetSpeed::Response &res)
  {
    ROS_INFO("Received SetSpeed request: linear_x=%.2f, angular_z=%.2f", req.linear_x, req.angular_z);
    // 在这里实现设置机器人速度的逻辑
    // 例如，将速度值发送给底层驱动

    res.success = true;
    res.message = "Speed set successfully";
    return true;
  }

  int main(int argc, char **argv)
  {
    ros::init(argc, argv, "speed_server");
    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("set_robot_speed", handle_set_speed);
    ROS_INFO("Ready to set robot speed.");
    ros::spin();

    return 0;
  }
  ```

  **C++ 服务客户端示例**：
  ```cpp
  #include "ros/ros.h"
  #include "ucar_controller/SetSpeed.h"

  int main(int argc, char **argv)
  {
    ros::init(argc, argv, "speed_client");
    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<ucar_controller::SetSpeed>("set_robot_speed");
    ucar_controller::SetSpeed srv;

    srv.request.linear_x = 0.5; // 设置请求的线速度
    srv.request.angular_z = 0.1; // 设置请求的角速度

    if (client.call(srv))
    {
      ROS_INFO("Service call successful: %s", srv.response.message.c_str());
    }
    else
    {
      ROS_ERROR("Failed to call service set_robot_speed");
      return 1;
    }

    return 0;
  }
  ```

## 5. 项目全局应用

在 `ucar_ws` 项目中，`ucar_controller` 包的 `srv` 文件夹定义了机器人控制的关键服务接口。这些服务允许其他 ROS 节点或外部应用程序以同步的方式请求 `ucar_controller` 执行特定的控制任务，例如设置机器人的目标位姿或运动速度。这种服务机制确保了不同模块之间可以进行可靠的、请求-响应式的通信，是构建模块化和可扩展机器人控制系统的基础。

## 6. 维护与更新

- **清晰命名**：服务名称和字段名称应具有描述性，清晰表达其功能和含义。
- **版本控制**：`.srv` 文件应纳入版本控制，以便跟踪接口变更。
- **兼容性**：修改服务定义时，应考虑对现有客户端和服务器的影响，尽量保持向后兼容性。
- **文档**：为每个 `.srv` 文件添加注释，说明其用途、请求和响应字段的含义。

## 7. 故障排除

- **服务文件未生成**：
  - 检查 `CMakeLists.txt` 和 `package.xml` 中 `message_generation` 和 `add_service_files` 的配置是否正确。
  - 确保在编译前执行了 `catkin_make clean` 或 `catkin clean`，然后重新编译。
- **服务调用失败**：
  - 确保服务服务器节点已启动并正在提供服务（使用 `rosservice list` 和 `rosservice info <service_name>` 检查）。
  - 检查客户端和服务端的话题名称是否一致。
  - 检查请求和响应消息类型是否匹配。
  - 检查网络连接和 ROS Master 是否正常。
- **服务响应异常**：
  - 检查服务服务器的逻辑，确保其正确处理请求并返回预期响应。
  - 检查客户端处理响应的逻辑。