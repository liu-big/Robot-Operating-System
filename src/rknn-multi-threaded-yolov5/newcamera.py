# -*- coding: utf-8 -*-
#
# Copyright (c) 2024, UCAR.AI, All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ============================================================================== 

"""
文件: newcamera.py
描述: 该文件用于从摄像头捕获视频流，进行畸变校正、水平翻转，并将处理后的视频保存为MP4文件，同时实时显示视频。
作者: UCAR.AI
创建日期: 2024-01-01
修订历史:
    2024-01-01: 初始版本
"""

import cv2 as cv
import numpy as np

# 定义相机内参矩阵
# camera_matrix: 相机的内参矩阵，用于图像畸变校正。
# 包含焦距(fx, fy)和光学中心(cx, cy)等参数。
camera_matrix = np.array([[408.12625, 0, 319.03663],
                          [0, 408.87682, 235.3358],
                          [0, 0, 1]], dtype=np.float32)

# 定义相机畸变参数
# dist_coeffs: 相机的畸变系数，用于图像畸变校正。
# 包含径向畸变(k1, k2, k3)和切向畸变(p1, p2)等参数。
dist_coeffs = np.array([-0.325698, 0.111425, 0.002560, 0.000459, 0.000000], dtype=np.float32)

# 打开摄像头
# cap: 视频捕获对象，参数0表示打开默认摄像头。
cap = cv.VideoCapture(0)

# 定义视频编码器
# fourcc: 视频编解码器，*'mp4v'表示使用MP4格式。
fourcc = cv.VideoWriter_fourcc(*'mp4v')

# 创建视频写入对象
# lz: 视频写入对象，用于将处理后的帧写入MP4文件。
# 参数:
#   'a8f1.mp4': 输出文件名。
#   fourcc: 视频编码器。
#   30: 帧率（每秒帧数）。
#   (640,480): 视频分辨率（宽度，高度）。
lz = cv.VideoWriter('a8f1.mp4',fourcc,30,(640,480))

# 循环读取摄像头帧并进行处理
while cap.isOpened():
    # 读取一帧图像
    # ret: 布尔值，表示是否成功读取帧。
    # frame: 读取到的图像帧。
    ret,frame = cap.read()
    
    # 如果没有成功读取帧，则退出循环
    if not ret:
        break
    
    # 水平翻转图像
    frame = cv.flip(frame, 1)
    # 对图像进行畸变校正
    frame = cv.undistort(frame, camera_matrix, dist_coeffs)
    
    # 将处理后的帧写入视频文件
    lz.write(frame)
    # 在窗口中显示实时视频流
    cv.imshow('frame',frame)

    # 检测按键，如果按下'q'键则退出循环
    if cv.waitKey(1) == ord('q'):
        break

# 释放摄像头资源
cap.release()
# 释放视频写入对象资源
lz.release()
# 销毁所有OpenCV窗口
cv.destroyAllWindows()

