import cv2
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
文件: main.py
描述: 该文件是YOLOv5多线程推理的主程序，集成了目标检测和车道线检测功能。
      它通过ROS订阅任务标志，根据任务类型切换识别或巡线模式，并发布视觉消息。
作者: UCAR.AI
创建日期: 2024-01-01
修订历史:
    2024-01-01: 初始版本
"""

import cv2
import time
import numpy as np
from func import myFunc
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32

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

# 巡线数据提取函数
# mid: 根据给定的图像和掩码，计算赛道中点、偏差角度和停止标志。
# 参数:
#   follow: 用于绘制赛道中线的图像。
#   mask: 经过处理的二值化图像掩码，其中白色像素代表赛道。
# 返回值:
#   error: 图像中点与指定提取中点的误差，正数表示右转，负数表示左转。
#   mid_angle: 偏差角度，与error方向一致。
#   stop_flag: 停止标志，1表示检测到停止线，0表示未检测到。
def mid(follow, mask):
    height = follow.shape[0]  # 获取图像高度
    width = follow.shape[1]   # 获取图像宽度

    half_width = width // 2 
    half = half_width  # 从下往上扫描赛道，最下端取图片中线为分割线
    mid_output = 0     # 初始化赛道中值
    mid_angle = 0      # 初始化偏差角度
    stop_flag = 0      # 停止标志符
    # 计算图像下半部分（从140行开始）的白色像素数量，用于判断是否检测到停止线
    white_pixels = np.count_nonzero(mask[140:] == 255)
    print(white_pixels)

    # 从图像底部向上遍历每一行，寻找赛道边界
    for y in range(height - 1, -1, -1):
        # 检查分割线左端是否有赛道
        if (mask[y][max(0, half - half_width):half] == np.zeros_like(
                mask[y][max(0, half - half_width):half])).all():  
            left = max(0, half - half_width)  # 如果无赛道，则取图片左边界
            cv2.circle(follow, (int(left), y), 1, (255, 255, 255), -1) # 绘制左边界点
        else:
            left = np.average(np.where(mask[y][0:half] == 255))  # 计算分割线左端白色像素的平均位置
            cv2.circle(follow, (int(left), y), 1, (255, 255, 255), -1) # 绘制左边界点

        # 检查分割线右端是否有赛道
        if (mask[y][half:min(width, half + half_width)] == np.zeros_like(
                mask[y][half:min(width, half + half_width)])).all():  
            right = min(width, half + half_width)  # 如果无赛道，则取图片右边界
            cv2.circle(follow, (int(right), y), 1, (255, 255, 255), -1) # 绘制右边界点
        else:
            right = np.average(np.where(mask[y][half:width] == 255)) + half  # 计算分割线右端白色像素的平均位置
            cv2.circle(follow, (int(right), y), 1, (255, 255, 255), -1) # 绘制右边界点

        mid = int((left + right) // 2) # 计算当前行的拟合中点
        
        half = mid  # 递归，将当前中点作为下一行扫描的中心，从下往上确定分割线
        cv2.circle(follow, (mid, y), 1, (255, 255, 255), -1)  # 绘制中点

        # 在指定前瞻点位置（y=170）记录赛道中值
        if y == 170:  
            mid_output = int(mid)
            cv2.circle(follow, (mid_output, 170), 5, (255, 255, 255), -1)  # 绘制指定提取中点

        # 计算图片中点与指定提取中点的误差
        error = follow.shape[1] // 2 - mid_output  
    
    # 判断是否检测到停止线
    if white_pixels > stop_line_area_pixels:  # 如果实心矩形穿过220行的直线的白色像素面积大于阈值
       error = 0.0       # 检测到停止线，误差设为0
       stop_flag = 1.0   # 设置停止标志为1
       print("stop")
    else:
       stop_flag = 0.0   # 未检测到停止线，停止标志为0
       print("go")
    
    # 计算偏差角度
    mid_angle = np.arctan(error / height) * 180 / np.pi
    mid_angle = mid_angle  

    return error, mid_angle, stop_flag  # 返回误差、角度和停止标志

# detect_line: ROS订阅回调函数，用于更新任务标志。
# 参数:
#   msg: 接收到的ROS消息，包含任务标志数据。
# 使用说明:
#   当ROS节点接收到'/mission_msg'话题的消息时，此函数会被调用，
#   并将消息中的数据赋值给全局变量mission_flag，从而控制程序的任务模式切换。
def detect_line(msg):
    global mission_flag
    mission_flag = msg.data

# 主程序入口
if __name__ == "__main__":
    # 打开摄像头，参数0表示默认摄像头
    cap = cv2.VideoCapture(0)
    # 初始化 ROS 节点，节点名为"detect"
    rospy.init_node("detect")
    # 导入rknn-lite包，由于与ROS有冲突，必须在ROS节点初始化后导入
    from rknnpool import rknnPoolExecutor
    # 定义ROS消息发布器，发布类型为Float32MultiArray，话题名为"vision_msg_list"
    vision_pub = rospy.Publisher("vision_msg_list", Float32MultiArray, queue_size=10)
    vision_msg = Float32MultiArray() # 创建Float32MultiArray消息实例
    # 定义ROS消息订阅器，订阅类型为Int32，话题名为'/mission_msg'，回调函数为detect_line
    vision_sub = rospy.Subscriber('/mission_msg',Int32,detect_line,queue_size=10) 
    # 初始化任务标志，0表示识别任务，1表示巡线任务
    mission_flag = 0
    
    # 停车线白色像素面积阈值（暂定），用于判断是否停车
    stop_line_area_pixels = 6000
    # 视觉消息列表，用于存储发布的数据
    vision_msg_list = []
    # 导入RKNN模型路径
    modelPath = "./rknnModel/iflytek_2.rknn"
    # 设置RKNN推理线程数，增大可提高帧率
    TPEs = 3
    # 初始化RKNN线程池，加载模型并指定后处理函数
    pool = rknnPoolExecutor(
        rknnModel=modelPath,
        TPEs=TPEs,
        func=myFunc)
    
    # 初始化异步推理所需的图像帧，预填充RKNN池
    if (cap.isOpened()):
        for i in range(TPEs + 1):
            ret, frame = cap.read() # 读取摄像头帧
            if not ret:
                cap.release()
                del pool
                exit(-1)
            pool.put(frame) # 将帧放入RKNN池
    
    # 初始化帧计数器和时间戳
    frames, loopTime, initTime = 0, time.time(), time.time()
    
    # 主循环，持续从摄像头获取图像并进行处理
    while (cap.isOpened()):
        frames += 1 # 帧计数加1
        ret, frame = cap.read() # 读取摄像头帧
        if not ret:
            break # 读取失败则退出循环
        
        # 应用水平镜像，使图像左右翻转
        frame = cv2.flip(frame, 1)
        # 对图像进行畸变矫正
        frame = cv2.undistort(frame, camera_matrix, dist_coeffs)
        
        # 根据任务标志执行不同的任务
        if mission_flag == 0: # 识别任务
            pool.put(frame) # 将当前帧放入RKNN池进行推理
            frame, flag = pool.get() # 获取推理结果
            
            if flag: # 如果推理成功
                img, class_array, center_array = frame # 解包推理结果
                img = np.asarray(frame[0]) # 将图像转换为NumPy数组
                # 组合视觉消息列表，包含类别信息和中心点坐标
                vision_msg_list = class_array + center_array[0] + center_array[1] + center_array[2]
            else:
                pass # 推理失败则跳过
            
            # 计算帧率
            fps = frames / (time.time() - initTime)
            
            # 在图像上叠加帧率信息
            cv2.putText(img, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow('test', img) # 显示图像
            # 按'q'键退出
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        elif mission_flag == 1: # 巡线任务
            cv2.destroyWindow('test') # 销毁识别任务的显示窗口
            
            # 确保frame是有效的图像数据
            if frame is not None and len(frame) == 3:
                frame, class_array, center_array = frame
                frame = np.asarray(frame) # 将图像列表转换为NumPy数组
            else:
                print("Frame is None or does not contain the expected lists")
                continue # 跳过当前帧，继续下一帧
            
            # 设定感兴趣区域（ROI），用于车道线检测
            roi_frame = frame[220:480, 0:640]
            # 将ROI转换为灰度图
            gray = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2GRAY)
            
            # 进行高斯滤波，平滑图像
            gauss = cv2.GaussianBlur(gray, (3, 3), 0)
            # 使用阈值处理将图像二值化，提取赛道（地面不反光使用）
            _, mask = cv2.threshold(gauss, 200, 255, cv2.THRESH_BINARY)
            
            # 复制掩码图像用于绘制中线
            follow = mask.copy()
            # 调用mid函数计算误差、角度和停止标志
            error, mid_angle, stop_flag = mid(follow, mask)
            # 组合视觉消息列表，包含误差、角度和停止标志
            vision_msg_list_lines =  [error, mid_angle, stop_flag]
            vision_msg_list = vision_msg_list_lines
            
            # 计算帧率
            fps = frames / (time.time() - initTime)
            print(f"error:{error}")
            print(f"angle:{mid_angle}")
            
            # 显示图像
            cv2.putText(frame, f"FPS: {fps:.2f}", (230, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow("frame", roi_frame) # 显示ROI图像
            cv2.imshow("mask", mask)       # 显示二值化掩码图像
            cv2.imshow("follow", follow)   # 显示绘制了中线的图像
            
            # 按'q'键退出
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            pass # 其他任务标志则跳过

        # 发布视觉消息
        vision_msg.data = vision_msg_list
        vision_pub.publish(vision_msg)
        print(mission_flag)

    # 释放摄像头资源和销毁所有OpenCV窗口
    cap.release()
    cv2.destroyAllWindows()
    # 释放RKNN线程池资源
    pool.release()
