# `rknn-multi-threaded-yolov5` 文件夹使用开发操作手册

## 1. 概述

`rknn-multi-threaded-yolov5` 文件夹是 `ucar_ws` 工作区中用于实现基于 RKNN 平台的多线程 YOLOv5 目标检测的 ROS 包。它集成了 YOLOv5 模型在瑞芯微 NPU 上的部署，并通过多线程优化，旨在提供高效、实时的目标检测能力。此包通常用于机器人视觉感知任务，例如目标识别、跟踪和定位，为机器人的自主导航和交互提供视觉信息。

## 2. 文件夹结构

```
src/rknn-multi-threaded-yolov5/
├── __pycache__/              # Python 缓存文件目录
├── __pycache___manual.md     # __pycache__ 目录说明文档
├── camera.py                 # 摄像头相关脚本
├── detectline.py             # 检测线相关脚本
├── func.py                   # 辅助函数库
├── main.py                   # 主程序入口
├── newcamera.py              # 新摄像头相关脚本
├── performance.sh            # 性能测试脚本
├── photo/                    # 图片存储目录
│   ├── 0.jpg
│   ├── 1.jpg
│   ├── 2.jpg
│   ├── 3.jpg
│   ├── 4.jpg
│   ├── 5.jpg
│   ├── lines.avi
│   ├── photo_manual.md       # 图片目录说明文档
│   └── qipan.png
├── rkcat.sh                  # RKNN 相关脚本
├── rknnModel/                # RKNN 模型文件目录
│   ├── iflytek.rknn
│   ├── iflytek2.rknn
│   ├── iflytek3.rknn
│   └── rknnModel_manual.md   # RKNN 模型目录说明文档
└── rknnpool.py               # RKNN 线程池管理脚本
```

- `__pycache__/`: 存放 Python 解释器生成的字节码缓存文件。
- `__pycache___manual.md`: `__pycache__` 文件夹的说明文档。
- `camera.py`: 可能包含用于处理摄像头数据流、图像捕获或视频录制的代码。
- `detectline.py`: 可能用于实现特定于线条检测或处理的逻辑。
- `func.py`: 包含项目通用的辅助函数、工具类或常用操作。
- `main.py`: 项目的主入口文件，负责初始化、调用核心功能和协调各个模块。
- `newcamera.py`: 可能是 `camera.py` 的更新版本或替代方案，用于新的摄像头接口或功能。
- `performance.sh`: 用于测试和评估系统性能的 shell 脚本，可能包括帧率、推理时间等指标。
- `photo/`: 存储测试图片、视频或其他图像资源。
- `rkcat.sh`: 可能是一个用于 RKNN 相关操作的 shell 脚本，例如模型转换、部署或测试。
- `rknnModel/`: 存放转换后的 RKNN 模型文件，这些模型是针对瑞芯微 NPU 优化的。
- `rknnModel_manual.md`: `rknnModel` 文件夹的说明文档。
- `rknnpool.py`: 实现 RKNN 推理的多线程管理，通过线程池提高并发处理能力和效率。

## 3. 主要功能与用途

`rknn-multi-threaded-yolov5` 文件夹的主要功能是：

- **RKNN 平台上的 YOLOv5 部署**：将 YOLOv5 目标检测模型转换为 RKNN 格式，并在瑞芯微 NPU 上高效运行。
- **多线程推理**：通过 `rknnpool.py` 实现多线程管理，充分利用 NPU 资源，提高目标检测的实时性和吞吐量。
- **实时视频流处理**：通过 `camera.py` 或 `newcamera.py` 集成摄像头，实现对实时视频流的目标检测。
- **性能优化与测试**：提供 `performance.sh` 脚本，用于评估和优化模型在 NPU 上的运行性能。
- **辅助功能与工具**：包含 `func.py` 等辅助脚本，提供数据处理、图像操作等通用功能。
- **模型管理**：`rknnModel/` 目录集中管理不同版本或类型的 RKNN 模型文件。

## 4. 使用方法

- **环境配置**：
  确保已安装瑞芯微 RKNN 工具链和相关驱动，以及所有 Python 依赖库。
- **模型准备**：
  将训练好的 YOLOv5 模型转换为 RKNN 格式，并放置在 `rknnModel/` 目录下。
- **运行主程序**：
  在终端中执行 `main.py` 来启动目标检测程序：
  ```bash
  python main.py
  ```
- **性能测试**：
  运行 `performance.sh` 脚本来测试模型的推理性能：
  ```bash
  ./performance.sh
  ```
- **修改配置**：
  直接编辑 `.py` 脚本文件来修改摄像头参数、模型路径或检测逻辑。

## 5. 项目全局应用

在 `ucar_ws` 项目中，`rknn-multi-threaded-yolov5` 包是机器人视觉感知系统的核心组件。它为机器人提供了强大的目标检测能力，能够识别环境中的物体，为导航、抓取、避障和人机交互等高级功能提供关键的视觉信息。其多线程和 NPU 优化特性确保了在嵌入式平台上实现高性能的实时感知。

## 6. 维护与更新

- **模型更新**：根据新的数据集或任务需求，更新和优化 YOLOv5 模型，并重新转换为 RKNN 格式。
- **RKNN 工具链升级**：及时更新瑞芯微 RKNN 工具链，以获取性能提升和新功能。
- **代码优化**：定期审查代码，提高多线程处理效率和资源利用率。
- **依赖管理**：确保所有 Python 库和系统依赖都已正确安装和配置。
- **性能调优**：根据实际部署环境，调整模型参数和推理配置，以达到最佳性能。

## 7. 故障排除

- **模型加载失败**：
  - 检查 RKNN 模型文件路径是否正确。
  - 确保 RKNN 模型与当前 NPU 硬件兼容。
  - 检查 RKNN 运行时环境是否正确配置。
- **推理速度慢**：
  - 检查是否充分利用了多线程。
  - 确认模型已正确加载到 NPU 而非 CPU 上运行。
  - 检查 NPU 负载和温度，避免过热降频。
- **摄像头无法打开**：
  - 检查摄像头设备是否正确连接。
  - 确认摄像头驱动已安装并正常工作。
  - 检查 `camera.py` 或 `newcamera.py` 中配置的设备路径是否正确。
- **检测精度低**：
  - 检查 RKNN 模型是否经过充分训练和量化。
  - 确认输入图像预处理与模型训练时一致。
  - 检查模型版本与代码是否匹配。