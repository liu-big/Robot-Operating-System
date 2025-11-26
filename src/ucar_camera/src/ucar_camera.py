#!/usr/bin/python3
# -*- coding: UTF-8 -*-

# @file ucar_camera.py
# @brief ROS 摄像头节点，用于捕获图像并发布为 ROS 消息。
#
# 该脚本初始化一个 ROS 节点，通过 OpenCV 从摄像头捕获视频流，
# 并将图像数据以 `sensor_msgs/Image` 消息的形式发布到 ROS topic。
# 支持配置图像分辨率、topic 名称和发布频率。

import os
import numpy as np
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Image
import cv2
import time
import threading


class UcarCamera:
    """
    @class UcarCamera
    @brief Ucar 摄像头类，负责从摄像头捕获图像并发布到 ROS。

    该类封装了摄像头的初始化、图像捕获、图像格式转换以及 ROS 消息发布等功能。
    """
    def __init__(self):
        """
        @brief 构造函数，初始化 UcarCamera 节点。

        初始化 ROS 节点，读取参数服务器上的配置，设置摄像头参数，
        并创建图像发布器。
        """
        rospy.init_node("ucar_camera", anonymous=True)
        
        # 从参数服务器获取图像宽度，默认为 640
        self.img_width=int(rospy.get_param('~image_width',default=640))
        # 从参数服务器获取图像高度，默认为 480
        self.img_height=int(rospy.get_param('~image_height',default=480))

        # 从参数服务器获取摄像头 topic 名称，默认为 "/ucar_camera/image_raw"
        self.camera_topic_name=rospy.get_param('~cam_topic_name',default="/ucar_camera/image_raw")
        # 定义 ROS 图像发布器，发布 `sensor_msgs/Image` 消息
        self.cam_pub=rospy.Publisher(self.camera_topic_name,Image,queue_size=1)          
        
        # 创建一个 ROS 的用于发布图片的 Image() 消息对象
        self.image_temp=Image()                    
        # 定义图片 header 里的 frame_id
        self.image_temp.header.frame_id = 'opencv' 
        # 定义图片高度
        self.image_temp.height=self.img_height        
        # 定义图片宽度
        self.image_temp.width=self.img_width          
        # 图片格式，rgb8 表示 8 位 RGB 格式
        self.image_temp.encoding='rgb8'            
        # 图片是否为大端字节序
        self.image_temp.is_bigendian=True
        # 告诉 ROS 图片每行的大小 (宽度 * 3 字节/像素)
        self.image_temp.step=self.img_width*3         
        
        ## 设置摄像头相关信息
        # 从参数服务器获取摄像头设备路径，默认为 "/dev/video0"
        device_path=rospy.get_param('device_path',default="/dev/video0")
        # 初始化 OpenCV 视频捕获对象
        self.cap = cv2.VideoCapture(device_path)
        # 设置捕获图像的宽度
        self.cap.set(3, self.img_width) 
        # 设置捕获图像的高度
        self.cap.set(4, self.img_height)
        # codec = cv2.cv.CV_FOURCC(*'XVID') # 旧版 OpenCV 的 FOURCC 设置方式
        # 设置视频编码格式为 I420
        codec = cv2.VideoWriter_fourcc('I', '4', '2', '0')
        # codec = cv2.VideoWriter.fourcc('M', 'J', 'P', 'G') # 另一种常见的 MJPG 编码格式
        # 应用编码格式设置
        self.cap.set(cv2.CAP_PROP_FOURCC, codec)
        # 从参数服务器获取摄像头发布频率，默认为 15 Hz
        self.cam_pub_rate=int(rospy.get_param('~rate',default=15))
        
        # 循环捕获和发布图像，直到 ROS 节点关闭
        while not rospy.is_shutdown():
            # 初始化发布频率对象
            ros_rate = rospy.Rate(self.cam_pub_rate)  
            # 从摄像头读取一帧图像，ret 为布尔值表示是否成功，frame_1 为捕获的帧
            ret,frame_1 = self.cap.read()   
            # 图像水平翻转
            frame_1 = cv2.flip(frame_1,1) 
            # 将 OpenCV 默认的 BGR 格式转换为 RGB 格式
            self.frame = cv2.cvtColor(frame_1, cv2.COLOR_BGR2RGB) 
            # 定义图片 header 的时间戳为当前 ROS 时间
            self.image_temp.header = Header(stamp=rospy.Time.now())   
            # 将图像内容转换为字符串（字节流）
            self.image_temp.data=np.array(self.frame).tostring()   
            # 发布图像消息
            self.cam_pub.publish(self.image_temp)
            # 根据设定的频率休眠，控制发布速率
            ros_rate.sleep()
       
if __name__ == '__main__':
    """
    @brief 主函数，程序入口。

    当脚本作为主程序运行时，创建 `UcarCamera` 类的实例，启动摄像头图像发布。
    """
    ucar_camera =  UcarCamera()


