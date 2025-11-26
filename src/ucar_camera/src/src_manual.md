# `src` 文件夹使用开发操作手册

## 1. 概述

在 `ucar_camera` ROS 包中，`src` 文件夹是存放主要源代码的地方。它包含了实现摄像头功能、图像处理逻辑以及与 ROS 系统交互的 Python 脚本。这些脚本是 `ucar_camera` 包的核心，负责数据的采集、处理和发布，使得机器人能够利用视觉信息进行感知和决策。

## 2. 文件夹结构

```
src/ucar_camera/src/
└── ucar_camera.py
```

- `ucar_camera.py`: 这是一个 Python 脚本，很可能包含了 `ucar_camera` 包的主要 ROS 节点实现。它可能负责：
  - 初始化 ROS 节点。
  - 订阅或发布 ROS 话题（例如，发布摄像头图像数据）。
  - 实现图像处理算法或视觉功能。
  - 与其他 ROS 节点进行通信。

## 3. 主要功能与用途

`src` 文件夹的主要功能是：

- **ROS 节点实现**：包含 `ucar_camera` 包的核心 ROS 节点，负责摄像头数据的采集、处理和发布。
- **图像处理逻辑**：实现各种图像处理算法，如图像校正、滤波、特征提取等。
- **视觉功能开发**：开发基于视觉的感知功能，如目标检测、跟踪、识别等。
- **与 ROS 系统的交互**：通过 ROS 话题、服务、参数等机制与其他 ROS 包进行数据交换和功能协作。

## 4. 使用方法

- **创建源文件**：在 `src/ucar_camera/src/` 目录下创建新的 Python 脚本文件（例如 `my_vision_node.py`）。

  示例 `my_vision_node.py`：
  ```python
  #!/usr/bin/env python
  import rospy
  from sensor_msgs.msg import Image
  from cv_bridge import CvBridge
  import cv2

  def image_callback(msg):
      rospy.loginfo("Received an image!")
      # Add your image processing logic here
      # For example, convert ROS Image to OpenCV image
      # bridge = CvBridge()
      # cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
      # cv2.imshow("Image window", cv_image)
      # cv2.waitKey(1)

  def vision_node():
      rospy.init_node('my_vision_node', anonymous=True)
      rospy.Subscriber("/ucar_camera/image_raw", Image, image_callback)
      rospy.spin()

  if __name__ == '__main__':
      try:
          vision_node()
      except rospy.ROSInterruptException:
          pass
  ```

- **`CMakeLists.txt` 配置**：
  如果添加了新的可执行 Python 脚本，确保在 `ucar_camera` 包的 `CMakeLists.txt` 中将其标记为可执行程序，以便 `catkin_make` 能够正确处理。例如，如果 `ucar_camera.py` 是一个 ROS 节点，通常会在 `CMakeLists.txt` 中有类似以下行的配置（如果它是 Python 脚本，通常不需要在 `CMakeLists.txt` 中显式添加 `add_executable`，但需要确保其有执行权限并在 `package.xml` 中声明了 `rospy` 依赖）：

  ```cmake
  # Mark executable scripts (Python etc.) for installation
  install(PROGRAMS
    src/ucar_camera.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
  ```
  **注意**：对于 Python 脚本，更常见的是通过 `install(PROGRAMS ...)` 将其安装到 `bin` 目录，并在 `launch` 文件中直接引用。确保脚本文件具有执行权限：
  ```bash
  chmod +x src/ucar_camera/src/ucar_camera.py
  ```

- **在 `launch` 文件中启动**：
  通常通过 `.launch` 文件来启动 `src` 文件夹中的 ROS 节点。例如：
  ```xml
  <launch>
    <node pkg="ucar_camera" type="ucar_camera.py" name="camera_node" output="screen" />
  </launch>
  ```

## 5. 项目全局应用

在 `ucar_ws` 项目中，`ucar_camera` 包的 `src` 文件夹是实现机器人视觉感知和处理的核心。它包含了直接与摄像头硬件交互、处理图像数据、并将其转化为 ROS 消息的关键逻辑。这些源代码使得机器人能够“看”到并理解其环境，为导航、目标识别、避障等高级功能提供基础数据。它是连接硬件和上层应用的关键桥梁。

## 6. 维护与更新

- **代码注释**：为复杂的逻辑和功能添加详细的注释，提高代码可读性。
- **模块化**：将不同的功能模块化，便于测试、维护和复用。
- **错误处理**：在代码中加入健壮的错误处理机制，提高程序的稳定性。
- **性能优化**：对于图像处理等计算密集型任务，考虑性能优化，如使用 OpenCV 的高效函数、多线程等。
- **依赖管理**：确保代码中使用的所有库和 ROS 依赖都在 `package.xml` 中正确声明。

## 7. 故障排除

- **脚本无法执行**：
  - 检查脚本是否具有执行权限（`chmod +x`）。
  - 检查脚本的 shebang（`#!/usr/bin/env python`）是否正确。
  - 确保 ROS 环境已正确设置（`source devel/setup.bash`）。
- **ROS 节点未启动或报错**：
  - 检查 `launch` 文件中的 `pkg` 和 `type` 属性是否正确指向 `ucar_camera` 包和 `ucar_camera.py` 脚本。
  - 查看终端输出的错误信息，通常会指出 Python 语法错误、ROS 依赖问题或运行时异常。
  - 使用 `rosnode info <node_name>` 查看节点状态。
- **话题未发布或订阅失败**：
  - 使用 `rostopic list` 和 `rostopic echo <topic_name>` 检查话题是否存在和数据流。
  - 检查代码中话题名称是否拼写正确。
  - 确保发布者和订阅者的数据类型匹配。