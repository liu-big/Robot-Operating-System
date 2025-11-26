# `qingzhou.sh` 脚本使用开发操作手册

## 1. 概述

`qingzhou.sh` 是一个用于启动机器人系统（特别是基于 ROS 的 UCAR 机器人）的 Shell 脚本。它通过 `gnome-terminal` 命令在多个终端标签页中并行启动不同的组件，包括 YOLOv8 目标检测、主处理脚本、机器人底层驱动和 ROS 导航栈。这个脚本旨在简化机器人系统的启动流程，确保所有必要的组件都以正确的顺序和配置运行。

## 2. 脚本结构

```
src/
├── qingzhou.sh               # 主脚本文件
└── qingzhou_manual.md        # 本文档
```

### 2.1 脚本内容解析

`qingzhou.sh` 脚本使用 `gnome-terminal --window -e 'bash -c "exec bash"'` 命令来创建一个新的 GNOME 终端窗口，并在其中打开多个标签页（`--tab`），每个标签页执行一个特定的命令。`sleep` 命令用于在启动下一个组件之前等待一段时间，以确保前一个组件有足够的时间初始化。

- **第一个标签页**：`gnome-terminal --window -e 'bash -c "exec bash"'`
  - 启动一个空的终端标签页，用于后续的命令。

- **第二个标签页**：`--tab -e 'bash -c "sleep 5; cd src/rknn-multi-threaded-yolov8; echo iflytek | sudo -S sh performance.sh; exec bash"'`
  - 等待 5 秒。
  - 进入 `src/rknn-multi-threaded-yolov8` 目录。
  - 使用 `sudo -S sh performance.sh` 运行 `performance.sh` 脚本。`echo iflytek | sudo -S` 用于以非交互方式提供 `sudo` 密码。
  - `exec bash` 保持终端打开。

- **第三个标签页**：`--tab -e 'bash -c "sleep 5; cd src/rknn-multi-threaded-yolov8; echo iflytek | sudo -S -k && conda activate yolov8 && python3 main.py; exec bash"'`
  - 等待 5 秒。
  - 进入 `src/rknn-multi-threaded-yolov8` 目录。
  - 使用 `sudo -S -k` 运行 `sudo` 命令，并清除之前的 `sudo` 缓存。
  - 激活 `yolov8` Conda 环境。
  - 运行 `main.py` Python 脚本，这通常是 YOLOv8 目标检测的主程序。
  - `exec bash` 保持终端打开。

- **第四个标签页**：`--tab -e 'bash -c "sleep 5; cd src; python3 process.py; exec bash"'`
  - 等待 5 秒。
  - 进入 `src` 目录。
  - 运行 `process.py` Python 脚本，这可能是机器人主逻辑处理脚本。
  - `exec bash` 保持终端打开。

- **第五个标签页**：`--tab -e 'bash -c "sleep 1; source ~/ucar_ws/devel/setup.bash; roslaunch ucar_controller base_driver.launch; exec bash"'`
  - 等待 1 秒。
  - `source ~/ucar_ws/devel/setup.bash`：设置 ROS 环境变量，使其能够找到 ROS 包。
  - `roslaunch ucar_controller base_driver.launch`：启动 `ucar_controller` 包中的 `base_driver.launch` 文件，这通常是机器人底层驱动。
  - `exec bash` 保持终端打开。

- **第六个标签页**：`--tab -e 'bash -c "sleep 5; source ~/ucar_ws/devel/setup.bash; roslaunch ucar_nav ucar_navigation.launch; exec bash"'`
  - 等待 5 秒。
  - `source ~/ucar_ws/devel/setup.bash`：设置 ROS 环境变量。
  - `roslaunch ucar_nav ucar_navigation.launch`：启动 `ucar_nav` 包中的 `ucar_navigation.launch` 文件，这通常是 ROS 导航栈。
  - `exec bash` 保持终端打开。

## 3. 主要功能与用途

- **一键启动**：通过一个脚本启动机器人系统的所有核心组件，简化操作。
- **模块化启动**：每个组件在独立的终端标签页中运行，便于监控和调试。
- **环境配置**：自动设置 ROS 环境变量和激活 Conda 环境。
- **系统集成**：协调 YOLO 目标检测、主控制逻辑、底层驱动和导航系统之间的启动顺序。

## 4. 使用方法

### 4.1 依赖

- **GNOME 终端**：脚本依赖于 `gnome-terminal` 命令。如果您使用的是其他桌面环境，可能需要修改脚本以适应相应的终端模拟器（例如 `xterm` 或 `konsole`）。
- **ROS 环境**：确保您的 ROS 环境已正确安装和配置，并且 `ucar_ws` 工作空间已编译 (`catkin_make`)。
- **Conda 环境**：如果使用 Conda 管理 Python 环境，请确保 `yolov8` 环境已创建并包含所有必要的依赖。
- **sudo 权限**：`performance.sh` 脚本可能需要 `sudo` 权限来执行某些操作。请确保当前用户具有 `sudo` 权限，并且 `iflytek` 是正确的 `sudo` 密码。

### 4.2 运行脚本

1. **打开终端**：打开一个新的终端窗口。
2. **执行脚本**：
   ```bash
   bash qingzhou.sh
   ```
   或者，如果脚本有执行权限：
   ```bash
   ./qingzhou.sh
   ```
   （如果脚本没有执行权限，请先运行 `chmod +x qingzhou.sh`）

## 5. 项目全局应用

在 `ucar_ws` 项目中，`qingzhou.sh` 脚本是机器人系统启动的核心入口，它确保了：

- **快速部署**：方便在机器人上快速部署和启动整个系统。
- **开发调试**：为开发者提供独立的终端窗口，便于查看每个组件的日志输出和进行调试。
- **自动化测试**：可以集成到自动化测试流程中，用于启动机器人系统进行功能测试。

## 6. 维护与更新

- **组件路径更新**：如果任何组件（如 `rknn-multi-threaded-yolov8`、`process.py` 等）的路径发生变化，需要相应地更新脚本中的 `cd` 命令。
- **启动顺序调整**：根据组件之间的依赖关系，调整 `sleep` 时间或启动顺序。
- **新组件集成**：如果需要添加新的 ROS 节点或程序，可以在脚本中添加新的 `--tab` 命令。
- **密码管理**：硬编码 `sudo` 密码 (`echo iflytek`) 存在安全风险。在生产环境中，应考虑使用更安全的 `sudo` 权限管理方式，例如配置 `sudoers` 文件，允许特定命令无需密码执行。
- **日志重定向**：为了更好地管理日志，可以将每个命令的输出重定向到文件中，而不是直接显示在终端中。

## 7. 故障排除

- **终端未打开或命令未执行**：
  - **原因**：`gnome-terminal` 命令不存在或路径错误，或者脚本语法错误。
  - **解决方案**：
    - 检查 `gnome-terminal` 是否已安装。
    - 检查脚本中的命令是否正确。
- **组件启动失败**：
  - **原因**：ROS 环境变量未设置，ROS 包未找到，依赖项缺失，或者程序内部错误。
  - **解决方案**：
    - 检查每个终端标签页的输出，查找错误信息。
    - 确保 `source ~/ucar_ws/devel/setup.bash` 已正确执行。
    - 检查 ROS 包和 Conda 环境的依赖项是否已安装。
- **`sudo` 密码问题**：
  - **原因**：`iflytek` 密码不正确，或者 `sudo` 配置不允许非交互式密码输入。
  - **解决方案**：
    - 确认 `sudo` 密码是否正确。
    - 考虑修改 `sudoers` 文件，允许特定命令无需密码执行。
- **`conda` 环境问题**：
  - **原因**：`yolov8` Conda 环境不存在或未正确激活。
  - **解决方案**：
    - 检查 Conda 环境是否已创建。
    - 尝试手动激活环境并运行 `python3 main.py`，以调试问题。