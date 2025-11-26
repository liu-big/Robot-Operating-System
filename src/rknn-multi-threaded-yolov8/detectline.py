# -*- coding: utf-8 -*-
# @Author: guo
# @Date: 2024-06-20
# @Description: 该脚本用于实时视频流中进行图像二值化处理，并提供滑动条动态调整二值化阈值。
#               它会打开一个摄像头，对视频帧进行畸变校正、灰度转换，
#               然后根据用户设定的阈值进行二值化，并显示结果。

import cv2
import numpy as np

# 定义相机内参矩阵
# camera_matrix: 相机的内参矩阵，用于将三维空间点投影到二维图像平面。
# 包含焦距(fx, fy)和光学中心(cx, cy)。
camera_matrix = np.array([[408.12625, 0, 319.03663],
                          [0, 408.87682, 235.3358],
                          [0, 0, 1]], dtype=np.float32)

# 定义相机畸变参数
# dist_coeffs: 相机的畸变系数，用于校正图像中的径向和切向畸变。
# 包含k1, k2, p1, p2, k3等参数。
dist_coeffs = np.array([-0.325698, 0.111425, 0.002560, 0.000459, 0.000000], dtype=np.float32)

# 初始参数定义
# threshold_initial: 初始二值化阈值，用于将灰度图像转换为黑白图像。
threshold_initial = 200  # 初始二值化阈值

# 创建窗口和滑动条
# cv2.namedWindow: 创建一个窗口，用于显示二值化图像。
# cv2.createTrackbar: 创建滑动条，允许用户在运行时调整二值化阈值。
cv2.namedWindow('Binary Threshold Adjustment')
cv2.createTrackbar('Threshold', 'Binary Threshold Adjustment', threshold_initial, 255, lambda x: None)

# 打开视频文件
# cv2.VideoCapture(0): 打开默认摄像头。如果需要打开视频文件，将参数改为视频文件路径。
cap = cv2.VideoCapture(0)  # 替换为你的视频路径

# 循环读取视频帧并进行处理
while cap.isOpened():
    # 读取一帧
    ret, frame = cap.read()
    # 如果帧读取失败，则退出循环
    if not ret:
        break

    # 应用水平镜像
    # cv2.flip: 对图像进行翻转，参数1表示水平翻转。
    frame = cv2.flip(frame, 1)

    # 畸变矫正
    # cv2.undistort: 使用预定义的相机内参和畸变系数对图像进行校正。
    frame = cv2.undistort(frame, camera_matrix, dist_coeffs)

    # 将图像转换为灰度图
    # cv2.cvtColor: 将BGR格式的彩色图像转换为灰度图像。
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 读取滑动条的当前值
    # cv2.getTrackbarPos: 获取指定滑动条的当前位置（值）。
    threshold = cv2.getTrackbarPos('Threshold', 'Binary Threshold Adjustment')

    # 应用二值化
    # cv2.threshold: 将灰度图像转换为二值图像，像素值大于阈值的设为255，否则设为0。
    _, binary = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY)

    # 显示二值化后的图像
    cv2.imshow('Binary Threshold Adjustment', binary)

    # 等待按键，如果按下 'q' 键则退出循环
    # cv2.waitKey(1): 等待1毫秒，返回按下的键的ASCII码。
    # 0xFF: 掩码，用于只保留按键的低8位。
    # ord('q'): 获取字符 'q' 的ASCII码。
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 清理资源
# cap.release(): 释放摄像头资源。
# cv2.destroyAllWindows(): 关闭所有OpenCV窗口。
cap.release()
cv2.destroyAllWindows()
