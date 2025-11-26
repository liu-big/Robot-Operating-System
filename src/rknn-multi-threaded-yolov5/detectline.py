# -*- coding: utf-8 -*-
# Copyright (c) 2024, UCAR
# All rights reserved.
#
# 文件名: detectline.py
# 文件描述: 该脚本用于实现基于OpenCV的巡线功能，通过图像处理识别赛道，并计算车辆的偏差和角度。
#           同时，它集成了ROS发布功能，将巡线结果发布到ROS话题。
#
# 作者: UCAR开发团队
# 创建日期: 2024年5月15日
# 版本: 1.0
#
# 修订历史:
# 2024年5月15日 - 初始版本创建。
#
import cv2 as cv
import numpy as np
import rospy
from std_msgs.msg import Int32MultiArray
# from rknnlite.api import RKNNLite

def mid(follow, mask):
    """
    计算赛道中线偏差、角度和停车标志。

    该函数通过分析二值化图像（mask）中的白色像素分布，来确定赛道的中线位置。
    它从图像底部向上逐行扫描，计算左右边界的平均位置，并以此确定赛道中点。
    同时，根据白色像素面积判断是否达到停车线。

    Args:
        follow (numpy.ndarray): 用于绘制中线和圆点的图像副本，通常是原始图像的ROI。
        mask (numpy.ndarray): 经过二值化处理的图像，白色像素代表赛道。

    Returns:
        tuple: 包含以下三个值的元组：
            - error (int): 赛道中点与图像中心线的水平偏差。正数表示赛道中点在图像中心线右侧，负数表示在左侧。
            - mid_angle (int): 车辆相对于赛道方向的偏差角度（度）。正数表示需要右转，负数表示需要左转。
            - stop_flag (int): 停车标志。1表示检测到停车线，0表示未检测到。
    """
    height = follow.shape[0]
    width = follow.shape[1]

    half_width = width // 2 
    half = half_width  # 从下往上扫描赛道,最下端取图片中线为分割线
    mid_output = 0 # 初始化赛道中值
    mid_angle = 0 # 初始化偏差角度
    stop_flag = 0 # 停止标志符
    # 计算图像下半部分（从140行开始）的白色像素数量，用于判断停车线。
    white_pixels = np.count_nonzero(mask[140:] == 255)
    print(white_pixels)

    # 从图像底部向上遍历每一行。
    for y in range(height - 1, -1, -1):
        # 检查分割线左侧区域是否完全没有赛道（白色像素）。
        if (mask[y][max(0, half - half_width):half] == np.zeros_like(
                mask[y][max(0, half - half_width):half])).all():  # 分割线左端无赛道
            left = max(0, half - half_width)  # 取图片左边界
            cv.circle(follow, (int(left), y), 1, (255, 255, 255), -1)
        else:
            # 计算分割线左侧赛道（白色像素）的平均水平位置。
            left = np.average(np.where(mask[y][0:half] == 255))  # 计算分割线左端平均位置
            cv.circle(follow, (int(left), y), 1, (255, 255, 255), -1)

        # 检查分割线右侧区域是否完全没有赛道（白色像素）。
        if (mask[y][half:min(width, half + half_width)] == np.zeros_like(
                mask[y][half:min(width, half + half_width)])).all():  # 分割线右端无赛道
            right = min(width, half + half_width)  # 取图片右边界
            cv.circle(follow, (int(right), y), 1, (255, 255, 255), -1)
        else:
            # 计算分割线右侧赛道（白色像素）的平均水平位置。
            right = np.average(np.where(mask[y][half:width] == 255)) + half  # 计算分割线右端平均位置
            cv.circle(follow, (int(right), y), 1, (255, 255, 255), -1)

        # 计算当前行的赛道中点。
        mid = int((left + right) // 2) # 计算拟合中点
        
           

        half = mid  # 递归,从下往上确定分割线
        cv.circle(follow, (mid, y), 1, (255, 255, 255), -1)  # mid_line

        # 在指定的前瞻点位置（y=170）记录赛道中点，用于计算偏差。
        if y == 170:  # 设置前瞻点位置
            mid_output = int(mid)
            cv.circle(follow, (mid_output, 170), 5, (255, 255, 255), -1)  # opencv为(x,y),画出指定提取中点

        # 计算赛道中点与图像中心线的水平偏差。
        error = follow.shape[1] // 2 - mid_output  # 计算图片中点与指定提取中点的误差
    # 判断白色像素面积是否超过阈值，以确定是否检测到停车线。
    if white_pixels > stop_line_area_pixels:  # 如果实心矩形穿过220行的直线的白色像素面积大于阈值
       error = 0
       stop_flag = 1       
       print("stop")
    else:
       stop_flag = 0
       print("go")
    # 计算偏差角度。
    mid_angle = np.arctan(error / height) * 180 / np.pi
    mid_angle = int(mid_angle)      


    return error, mid_angle, stop_flag  # error为正数右转,为负数左转；角度同理


if __name__ == "__main__":
    # 定义相机内参矩阵，用于图像畸变矫正。
    camera_matrix = np.array([[408.12625, 0, 319.03663],
                            [0, 408.87682, 235.3358],
                            [0, 0, 1]], dtype=np.float32)

    # 定义相机畸变参数，用于图像畸变矫正。
    dist_coeffs = np.array([-0.325698, 0.111425, 0.002560, 0.000459, 0.000000], dtype=np.float32)
    
    # 定义停车线区域的白色像素阈值，用于判断是否停车。
    stop_line_area_pixels = 6000
    # 获取视频流，0表示默认摄像头。
    cap = cv.VideoCapture(0)  # 0为默认摄像头，也可以传入视频文件路径
    # 定义视频编码器，用于保存视频文件。
    fourcc = cv.VideoWriter_fourcc(*'mp4v')
    # 创建视频写入对象，将处理后的视频保存到文件。
    video = cv.VideoWriter('./2024_datasets.mp4',fourcc,30,(640,480))   
    # 初始化消息数组，用于存储发布到ROS的数据。
    message_array = []
    # 初始化ROS节点，节点名为"linedetect"。
    rospy.init_node("linedetect") 
    # 创建ROS发布器，发布Int32MultiArray类型的消息到"lines"话题。
    error_pub = rospy.Publisher("lines",Int32MultiArray,queue_size=7)
    # 创建Int32MultiArray消息对象。
    error_msg = Int32MultiArray()
    # 主循环，持续读取视频帧并进行处理。
    while True:
        start = cv.getTickCount()  # 获取开始时间，用于计算帧率。
        ret, frame = cap.read()  # 读取视频流的帧。
        if not ret:
            # 如果无法读取帧，则退出循环。
            break

        # 对图像进行畸变矫正。
        frame = cv.undistort(frame, camera_matrix, dist_coeffs)
        # 将矫正后的帧写入视频文件。
        video.write(frame)
        # 设定感兴趣区域（ROI），这里选取图像的下半部分，并进行水平镜像。
        roi_frame = frame[220:480, 0:640]
        frame = cv.flip(roi_frame, 1)
        
        # 将ROI转换为灰度图像。
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        # 进行高斯滤波，平滑图像，减少噪声。
        gauss = cv.GaussianBlur(gray, (3, 3), 0)

        # 使用固定阈值进行二值化处理，将赛道区域变为白色。
        # #地面不反光使用
        _, mask = cv.threshold(gauss, 200, 255, cv.THRESH_BINARY)
        
        # 地面反光使用（注释掉的代码，备用）
        #mask = cv.adaptiveThreshold(gauss, 255, cv.ADAPTIVE_THRESH_MEAN_C, cv.THRESH_BINARY_INV, 7, 7)
        # 使用闭运算和膨胀操作，连接断开的线条（注释掉的代码，备用）
        #kernel = np.ones((3, 3), np.uint8)
        #mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)
        #kernel = np.ones((3, 3), np.uint8)
        #mask = cv.dilate(mask, kernel, iterations=3)

        # 复制mask图像，用于绘制中线。
        follow = mask.copy()
        # 调用mid函数计算偏差、角度和停车标志。
        error, mid_angle, stop_flag = mid(follow, mask)

        # 计算帧率。
        fps = cv.getTickFrequency() / (cv.getTickCount() - start)
        # 在图像上显示帧率。
        cv.putText(frame, f'FPS: {int(fps)}', (10, 30), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv.LINE_AA)
        
        # 将计算结果存储到消息数组中。
        message_array = error, mid_angle, stop_flag
        # 将消息数组赋值给ROS消息对象的数据字段。
        error_msg.data = message_array
        # 发布ROS消息。
        error_pub.publish(error_msg)
        
        # 打印偏差和角度信息。
        print(f"error:{error}")
        print(f"angle:{mid_angle}")

        # 显示处理后的图像。
        cv.imshow("frame", frame)
        cv.imshow("mask", mask)
        cv.imshow("follow", follow)
        # 退出键，按下'q'键退出循环。
        if cv.waitKey(1) & 0xFF == ord('q'):
            break
        # 释放资源（此处的释放操作在循环内部，实际应在循环外部）
    cap.release()
    video.release()
    cv.destroyAllWindows()
