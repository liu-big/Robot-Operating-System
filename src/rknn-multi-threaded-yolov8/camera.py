# -*- coding: utf-8 -*-
# @Author: guo
# @Date: 2024-06-20
# @Description: 该脚本用于实时视频流中进行霍夫直线检测，并可视化检测结果。
#               它会打开一个摄像头，对视频帧进行畸变校正、灰度转换、二值化，
#               然后使用霍夫变换检测直线，并将直线绘制到原始帧上。
#               同时，提供了滑动条用于动态调整霍夫变换的参数。

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
# threshold_initial: 霍夫变换的累加器阈值，低于此值的直线不会被检测到。
# minLineLength_initial: 最小直线长度，低于此长度的线段不会被检测到。
# maxLineGap_initial: 最大允许的线段之间的间隙，用于将共线的线段连接起来。
threshold_initial = 50
minLineLength_initial = 100
maxLineGap_initial = 10

# 创建窗口和滑动条
# cv2.namedWindow: 创建一个窗口，用于显示视频帧和二值化图像。
# cv2.createTrackbar: 创建滑动条，允许用户在运行时调整霍夫变换的参数。
cv2.namedWindow('Hough Lines Visualization')
cv2.createTrackbar('Threshold', 'Hough Lines Visualization', threshold_initial, 500, lambda x: None)
cv2.createTrackbar('Min Line Length', 'Hough Lines Visualization', minLineLength_initial, 500, lambda x: None)
cv2.createTrackbar('Max Line Gap', 'Hough Lines Visualization', maxLineGap_initial, 200, lambda x: None)

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

    # 裁剪图像，只保留感兴趣区域
    # 这里裁剪了图像的下半部分，具体裁剪范围可根据实际需求调整。
    frame = frame[220:480,0:640]

    # 将图像转换为灰度图
    # cv2.cvtColor: 将BGR格式的彩色图像转换为灰度图像。
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 对灰度图进行二值化处理
    # cv2.threshold: 将灰度图像转换为二值图像，像素值大于200的设为255（白色），否则设为0（黑色）。
    _, binary = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)

    # 读取滑动条的当前值
    # cv2.getTrackbarPos: 获取指定滑动条的当前位置（值）。
    threshold = cv2.getTrackbarPos('Threshold', 'Hough Lines Visualization')
    minLineLength = cv2.getTrackbarPos('Min Line Length', 'Hough Lines Visualization')
    maxLineGap = cv2.getTrackbarPos('Max Line Gap', 'Hough Lines Visualization')

    # 使用概率霍夫变换检测直线
    # cv2.HoughLinesP: 检测图像中的直线段。参数包括：
    #   - binary: 输入的二值图像。
    #   - 1: 累加器的距离分辨率（以像素为单位）。
    #   - np.pi / 180: 累加器的角度分辨率（以弧度为单位）。
    #   - threshold: 累加器阈值。
    #   - minLineLength: 最小直线长度。
    #   - maxLineGap: 最大允许的线段之间的间隙。
    lines = cv2.HoughLinesP(binary, 1, np.pi / 180, threshold=threshold, minLineLength=minLineLength, maxLineGap=maxLineGap)

    # 绘制检测到的直线到原始帧上
    if lines is not None:
        for line in lines:
            # line[0] 包含线段的起始和结束坐标 (x1, y1, x2, y2)
            x1, y1, x2, y2 = line[0]
            # cv2.line: 在图像上绘制一条线段，颜色为绿色(0, 255, 0)，线宽为2。
            cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

    # 显示二值化图像和绘制了直线的原始帧
    cv2.imshow('Hough Lines Visualization', binary)
    cv2.imshow('Hough Lines Visualization', frame)

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
