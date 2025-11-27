# UCAR - 智能自动驾驶小车项目
![224916c2f67abcd6e364ff63b6bb821a](https://github.com/user-attachments/assets/3288de70-d7fe-4324-a156-97002796f058)
<img width="702" height="978" alt="efe56dca-5913-4ed1-bec4-9086785f2f56" src="https://github.com/user-attachments/assets/d348f4d4-687f-43ac-8b75-9b964098db02" />


UCAR（智能自动驾驶小车）是一个基于ROS（Robot Operating System）平台开发的自动驾驶机器人项目。该项目整合了多种传感器和算法，实现了自主导航、语音控制、计算机视觉等功能，可用于教学、研究和开发目的。

## 📋 项目概述

本项目是一个完整的机器人系统，具有以下核心功能：

- **自主导航**: 使用激光雷达和SLAM技术进行地图构建和路径规划
- **语音控制**: 支持语音识别和语音指令响应
- **视觉识别**: 基于YOLOv5/v8的目标检测和识别
- **惯性导航**: 使用IMU进行姿态估计和方向校正
- **运动控制**: 精确的底盘驱动和运动控制

## 🏗️ 项目架构

```
ucar_ws/
├── src/
│   ├── fdilink_ahrs/         # 惯性测量单元(IMU)驱动
│   ├── ydlidar/              # 激光雷达驱动
│   ├── ucar_controller/      # 机器人底盘控制器
│   ├── ucar_map/             # 地图数据和配置
│   ├── ucar_nav/             # 导航和路径规划
│   ├── send_goals/           # 导航目标点管理
│   ├── speech_command/       # 语音识别和指令处理
│   ├── rknn-multi-threaded-yolov5/  # 基于RKNN的YOLOv5目标检测
│   ├── rknn-multi-threaded-yolov8/  # 基于RKNN的YOLOv8目标检测
│   ├── ucar_camera/          # 摄像头驱动
│   ├── process0.py           # 主控制脚本
│   └── ...                   # 其他辅助文件和脚本
├── qingzhou.sh               # 系统一键启动脚本
└── README.md                 # 项目说明文档
```

## 🚀 核心功能模块

### 1. 自主导航系统 (ucar_nav)
基于ROS的move_base导航栈，结合激光雷达数据实现SLAM建图和路径规划。

### 2. 语音控制系统 (speech_command)
集成语音识别技术，支持自然语言指令控制机器人行为。

### 3. 视觉识别系统 (rknn-multi-threaded-yolov5/yolov8)
利用瑞芯微NPU加速的YOLO目标检测算法，实现实时物体识别。

### 4. 惯性导航系统 (fdilink_ahrs)
通过IMU传感器提供精确的姿态和方向信息。

### 5. 运动控制系统 (ucar_controller)
负责底层电机控制和机器人运动管理。

## 🎯 使用方法

### 系统启动

```bash
cd ~/ucar_ws
./qingzhou.sh
```

该脚本将自动打开多个终端标签页，分别启动：
1. YOLOv8性能优化脚本
2. YOLOv8主程序
3. 数据处理脚本
4. ROS底盘驱动
5. ROS导航系统

### 手动启动各模块

#### 启动底盘驱动
```bash
source ~/ucar_ws/devel/setup.bash
roslaunch ucar_controller base_driver.launch
```

#### 启动导航系统
```bash
source ~/ucar_ws/devel/setup.bash
roslaunch ucar_nav ucar_navigation.launch
```

#### 启动语音控制
```bash
roslaunch speech_command speech_recognition.launch
```

#### 启动视觉识别
```bash
cd src/rknn-multi-threaded-yolov8
conda activate yolov8
python main.py
```

## ⚙️ 硬件要求

- 瑞芯微RK3399开发板
- YDLIDAR X2/X4激光雷达
- FDILINK AHRS IMU模块
- USB摄像头
- 直流电机和编码器
- 电源管理系统

## 🛠️ 开发环境

- Ubuntu 18.04 LTS
- ROS Melodic
- Python 3.6+
- Conda环境 (用于深度学习模型)
- RKNN Toolkit (用于神经网络推理)

## 📁 主要脚本说明

| 文件 | 功能 |
|------|------|
| [process0.py](file:///c%3A/Users/liu%27jin/xwechat_files/wxid_gf63e4ybenh522_9079/msg/file/2025-06/ucar_ws/ucar_ws/src/process0.py) | 主控制逻辑，整合各模块功能 |
| [qingzhou.sh](file:///c%3A/Users/liu%27jin/xwechat_files/wxid_gf63e4ybenh522_9079/msg/file/2025-06/ucar_ws/ucar_ws/src/qingzhou.sh) | 一键启动所有系统组件 |
| [goal_test1.py](file:///c%3A/Users/liu%27jin/xwechat_files/wxid_gf63e4ybenh522_9079/msg/file/2025-06/ucar_ws/ucar_ws/src/goal_test1.py) | 导航目标点测试脚本 |


## 📄 许可证

本项目采用BSD许可证，具体请参考各功能包中的LICENSE文件。
