# `rknnModel` 文件夹使用开发操作手册

## 1. 概述

`rknnModel` 文件夹在 `rknn-multi-threaded-yolov8` 项目中扮演着核心角色，它主要用于存放与 Rockchip NPU (Neural Processing Unit) 相关的模型文件。这些模型通常是经过 RKNN Toolkit 转换和优化的深度学习模型，可以直接在瑞芯微的硬件平台上高效运行。对于 YOLOv8 项目，这里存放的将是用于目标检测的 `.rknn` 格式模型文件。

## 2. 文件夹结构

`src/rknn-multi-threaded-yolov8/rknnModel/`

- `src/rknn-multi-threaded-yolov8/rknnModel/yolov8s.rknn`: 经过 RKNN Toolkit 转换的 YOLOv8s 模型文件。
- `src/rknn-multi-threaded-yolov8/rknnModel/yolov8n.rknn`: 经过 RKNN Toolkit 转换的 YOLOv8n 模型文件。
- `src/rknn-multi-threaded-yolov8/rknnModel/yolov8_custom.rknn`: 用户自定义训练的 YOLOv8 模型文件。
- `src/rknn-multi-threaded-yolov8/rknnModel/labels.txt`: 模型对应的类别标签文件。
- `src/rknn-multi-threaded-yolov8/rknnModel/config.yaml`: 可能包含模型加载或推理相关的配置文件。
- ... (可能包含其他模型版本或相关辅助文件)

## 3. 主要功能与用途

`rknnModel` 文件夹的主要功能是：

- **模型存储**：集中存放所有用于 NPU 推理的 `.rknn` 模型文件。
- **硬件加速**：这些模型经过优化，旨在利用 Rockchip NPU 的计算能力，实现高效、低功耗的深度学习推理。
- **版本管理**：方便管理不同版本或不同大小（如 `yolov8n`, `yolov8s`, `yolov8m` 等）的 YOLOv8 模型。
- **配置管理**：可能包含模型推理所需的配置信息，如输入输出尺寸、预处理参数等。

## 4. 使用方法

- **模型转换**：原始的 PyTorch, TensorFlow, ONNX 等格式的模型需要通过 RKNN Toolkit 工具链转换为 `.rknn` 格式，然后放置到此文件夹。
  示例转换命令（伪代码，具体请参考 RKNN Toolkit 文档）：
  ```bash
  # 假设你已经安装了rknn-toolkit2
  # python convert_model.py --model_path yolov8s.pt --output_path yolov8s.rknn
  ```
- **模型加载**：项目中的 Python 或 C++ 代码会从该文件夹加载 `.rknn` 模型进行推理。
  示例代码片段（Python，使用 `rknnpool.py` 中的逻辑）：
  ```python
  import os
  from rknnpool import rknnPoolExecutor

  model_path = os.path.join('src/rknn-multi-threaded-yolov8/rknnModel', 'yolov8s.rknn')
  # 假设 rknnPoolExecutor 能够加载 .rknn 模型
  # rknn_executor = rknnPoolExecutor(model=model_path, ...)
  # rknn_executor.load_rknn()
  ```
- **标签文件**：`labels.txt` 文件通常包含模型能够识别的所有对象类别的名称，每行一个，与模型输出的类别索引对应。

## 5. 项目全局应用

在 `ucar_ws` 项目中，`rknn-multi-threaded-yolov8` 模块通过 `rknnModel` 文件夹实现了与瑞芯微 NPU 硬件的深度集成。这使得整个系统能够利用硬件加速进行实时目标检测，对于需要高性能和低延迟的机器人应用（如自动驾驶、智能监控）至关重要。它确保了模型部署的便捷性和运行的高效性。

## 6. 维护与更新

- **模型版本管理**：当有新的模型版本或经过重新训练的模型时，应更新此文件夹中的 `.rknn` 文件。建议使用清晰的命名约定（如 `yolov8s_v1.0.rknn`）来区分不同版本。
- **标签同步**：如果模型识别的类别发生变化，务必同步更新 `labels.txt` 文件，以确保推理结果的正确解释。
- **性能优化**：定期评估模型在 NPU 上的性能，并根据需要重新量化或优化模型，然后更新此文件夹中的模型文件。

## 7. 故障排除

- **模型加载失败**：
  - 检查 `.rknn` 文件路径是否正确。
  - 确认模型文件没有损坏。
  - 确保 RKNN Runtime 环境已正确安装且版本兼容。
  - 检查 NPU 设备是否正常连接和初始化。
- **推理结果不准确**：
  - 检查 `labels.txt` 文件是否与模型输出的类别顺序一致。
  - 确认模型输入预处理（如图像尺寸、归一化）与模型训练时一致。
  - 尝试使用 RKNN Toolkit 提供的工具对模型进行离线推理，以排除硬件或运行时环境问题。
- **NPU 占用率高或内存不足**：
  - 考虑使用更小尺寸的模型（如 `yolov8n`）。
  - 优化模型输入批处理大小。
  - 检查是否有其他进程占用了 NPU 资源。