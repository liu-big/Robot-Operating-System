# -*- coding: utf-8 -*-
# Copyright (c) 2024, UCAR
# All rights reserved.
#
# 文件名: camera.py
# 文件描述: 该脚本用于通过摄像头捕获视频流，并支持实时显示、拍照和退出功能。
#           它利用OpenCV库进行视频处理。
#
# 作者: UCAR开发团队
# 创建日期: 2024年5月15日
# 版本: 1.0
#
# 修订历史:
# 2024年5月15日 - 初始版本创建。
#
import cv2
import numpy as np

# 初始化摄像头
# cv2.VideoCapture(0) 表示打开默认摄像头（通常是内置摄像头或第一个USB摄像头）。
# 如果有多个摄像头，可以通过更改数字来选择，例如 1, 2 等。
cap = cv2.VideoCapture(0)

# 设置等待按键的标志位，用于控制cv2.waitKey的延时。
# flag = 1 表示每帧等待1毫秒，实现视频流畅播放。
flag = 1

# 初始化照片计数器，用于保存照片时命名文件。
count = 0

# 循环读取摄像头帧，直到摄像头关闭或用户退出。
# cap.isOpened() 检查摄像头是否成功打开。
while cap.isOpened():
    # ret: 布尔值，表示是否成功读取帧。
    # frame: 读取到的视频帧，一个NumPy数组。
    ret, frame = cap.read()

    # 检查帧是否成功读取，如果未成功读取则退出循环。
    if not ret:
        print("无法读取摄像头帧，请检查摄像头连接或权限。")
        break

    # 水平翻转图像。
    # cv2.flip(frame, 1) 中的 1 表示水平翻转（沿Y轴翻转）。
    # 0 表示垂直翻转，-1 表示水平垂直翻转。
    frame = cv2.flip(frame, 1)

    # 显示视频帧。
    # 'img' 是显示窗口的名称。
    cv2.imshow('img', frame)

    # 处理按键事件。
    # cv2.waitKey(flag) 等待按键输入，参数为等待时间（毫秒）。
    # 如果在指定时间内没有按键按下，则返回 -1。
    # ord(' ') 获取空格键的ASCII值。
    if cv2.waitKey(flag) == ord(' '):
        # 将计数器转换为字符串，用于文件名。
        c = str(count)
        # 保存当前帧为JPG图片。
        # 文件名格式为 'count.jpg'，例如 '0.jpg', '1.jpg' 等。
        cv2.imwrite(c + '.jpg', frame)
        print(f"已保存照片: {c}.jpg")
        # 增加计数器。
        count = count + 1

    # 如果按下 'q' 键，则退出循环。
    # ord('q') 获取 'q' 键的ASCII值。
    if cv2.waitKey(flag) == ord('q'):
        print("检测到 'q' 键，程序退出。")
        break

# 销毁所有OpenCV创建的窗口。
cv2.destroyAllWindows()

# 释放摄像头资源。
# 这是一个重要的步骤，确保摄像头被正确关闭，以便其他应用程序可以使用。
cap.release()

print("程序已结束。")