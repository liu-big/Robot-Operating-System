#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 本项目主要功能是实现机器人按照预设导航点进行移动，并在距离目标点一定范围内进行图像抓拍，输出拍照结果。
# 脚本中包含两种拍照模式：①到达一个点拍一张；②持续拍照但只保留一张。

# 导入必要的库
import argparse # 用于解析命令行参数
import tf # ROS的坐标变换库，用于处理坐标变换
import threading # 用于多线程操作，例如同时进行导航和距离测量
import rospy # ROS Python客户端库，用于ROS节点开发
import sys # 系统相关功能，例如退出程序
import math # 数学函数，用于距离计算等
from sensor_msgs.msg import LaserScan # 激光雷达数据消息类型，虽然在此脚本中未直接使用，但通常用于导航
import actionlib # ROS动作库，用于与move_base等动作服务器通信
from actionlib_msgs.msg import * # 导入所有动作消息类型
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist # 几何消息类型，用于定义姿态、点、四元数、速度等
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal # move_base动作消息类型，用于向move_base发送导航目标
import roslib # ROS库加载器，用于加载ROS包
import tf2_ros # ROS的tf2坐标变换库，用于更现代的坐标变换操作
import numpy as np # 数值计算库，用于数组操作和数学计算
from std_msgs.msg import String # 标准消息类型，用于字符串数据
import os # 操作系统功能，例如文件路径操作
import ctypes # 外部函数库接口，用于调用C库函数
import cv2 # OpenCV库，用于图像处理和摄像头操作
from ctypes.util import find_library # 用于查找共享库文件

# 全局变量定义
# 有1个回家的点
key = 0 # 未知用途的标志位
circle_ktd = 0 # 未知用途的标志位，可能与旋转行为有关
distance = False # 距离标志，用于判断是否到达目标点附近
gg = 0 # 未知用途的标志位
cam = 0 # 摄像头状态标志，0表示关闭，1表示打开
latest_frame = None # 存储最新图像帧
crop = 0 # 未知用途的标志位，可能与图像裁剪有关
camera = 0 # 摄像头状态标志，与cam类似，可能表示摄像头设备ID
destroy = 0 # 未知用途的标志位，可能与销毁窗口或资源有关
F_list = [] # 未知用途的列表

# 预设的导航点列表，每个子列表包含 [X坐标, Y坐标, 偏航角(弧度)]
# 这些点是机器人将依次前往的目标位置。
xy = [[2.9631063000692928, -0.19845071289273267, -1.6994449527149291], 
      [2.9699946355935354, -4.079984961831021, 1.6275405602410764], 
      [4.820265552933081, -0.9930045808959614, -1.7169176160854547], 
      [4.7943819152339024, -3.627640521846614, 1.9699315126897938], 
      [0.900197608047419, -0.20358226304161914, -1.5778584895002257]]

# 导航点的数量
pose_num= len(xy)
qtn_list = [] # 存储导航点对应的四元数列表
voice_play = "/home/ucar/Desktop/broadcast_ws/src//" # 语音播放文件路径前缀
connect_crop = [] # 未知用途的列表
processing_lock = threading.Lock()  # 线程锁，用于同步图像处理过程
stop_processing = False  # 停止图像处理的标志位
lib_x11 = ctypes.CDLL(find_library('X11')) # 加载X11库
lib_x11.XInitThreads() # 初始化X11线程支持
car_to_goal_x=np.float64(3.14) # 机器人当前X坐标（初始化值）
car_to_goal_y=np.float64(3.14)  # 机器人当前Y坐标（初始化值）

def dis(x, y):
    """
    测量机器人当前位置到目标点的距离。

    :param x: 目标点的X坐标。
    :type x: float
    :param y: 目标点的Y坐标。
    :type y: float
    """
    global car_to_goal_x
    global car_to_goal_y
    listener = tf.TransformListener() # 创建tf监听器，用于获取坐标变换
    
    while not rospy.is_shutdown(): # 循环直到ROS节点关闭
        global distance
        global destroy 
        rospy.sleep(0.1) # 短暂休眠，避免CPU占用过高
        try:
            # 获取从 "map" 坐标系到 "base_link" 坐标系的变换
            (trans, rot) = listener.lookupTransform("map", "base_link", rospy.Time(0))
            car_to_goal_x=trans[0] # 更新机器人当前X坐标
            car_to_goal_y=trans[1] # 更新机器人当前Y坐标
            # 计算小车到目标点的欧几里得距离
            distance = pow(pow( car_to_goal_x- x, 2) + pow(car_to_goal_y - y, 2), 0.5)
            # if distance > 0.3:
            print(distance) # 打印当前距离
            if distance<0.05: # 如果距离小于0.05米，认为到达目标点，设置distance为0并退出循环
                distance=0
                break 
        except Exception as e:
            rospy.logwarn("警告:%s", e) # 捕获并警告tf变换查找失败的异常
        

def goals(x, y, i,count):
    """
    发布导航目标点，并处理到达目标点附近的拍照逻辑。

    :param x: 目标点的X坐标。
    :type x: float
    :param y: 目标点的Y坐标。
    :type y: float
    :param i: 目标点在列表中的索引。
    :type i: int
    :param count: 计数器，可能用于照片命名。
    :type count: int
    """
    
    global qtn_list
    global distance
    global gg
    global destroy
    global crop
    global cam
    global latest_frame
    global stop_processing
    
    distance=0 # 重置距离标志
# 发布目标点
    target = Pose(Point(x, y, 0), Quaternion(qtn_list[i][0], qtn_list[i][1], qtn_list[i][2], qtn_list[i][3])) # 创建目标姿态
    goal = MoveBaseGoal() # 创建MoveBaseGoal对象
    goal.target_pose.pose = target # 设置目标姿态
    goal.target_pose.header.frame_id = 'map' # 设置目标姿态的坐标系
    goal.target_pose.header.stamp = rospy.Time.now() # 设置时间戳
    rospy.loginfo("Going to: " + str(target)) # 记录日志，显示前往的目标点
    new_thread1 = threading.Thread(target = dis, args=(x,y), name="T2") # 创建新线程测量距离
    new_thread1.start() # 启动距离测量线程
    rospy.sleep(0.1) # 短暂休眠
    move_base.send_goal(goal) # 向move_base发送导航目标
    while not distance: # 等待直到距离测量线程将distance设置为0（表示到达）
        rospy.sleep(1) # 休眠1秒
        print("wwwwwwwwwwwww") # 调试信息
    new_thread1.join() # 等待距离测量线程结束
    #保存图片 (以下是被注释掉的拍照逻辑，可能用于调试或备用)
    # if_ret=0
    # while distance>0.3:	
    #     rospy.sleep(0.1)
    #     if distance<=1.5 :
    #         rospy.sleep(0.2)
    #         # 读取图像帧
    #         ret, frame = cap.read()
    #         if_ret=1
    # # 检查图像帧是否成功读取
    # if ifret:
    #     print("无法获取图像帧")
        
    # 保存图像
    
    # save_path=str(count)+":"+str(distance) + ".jpg"
    # cv2.imwrite(save_path, frame)
    # print("已保存照片：", save_path)
    
    # flag = 0
    # 距离小于2.0时打开摄像头，识别到了结束该函数
    # while 1:
    #     if distance < 8:
    #         if crop == 1:
    #             gg = 1
    #             crop = 0
    #             cam=0
    #             stop_processing = True
    #             cap.release()
    #             return
    # 让小车转圈
            # if distance < 0.1 and flag == 0 and crop == 0:
            #     circle_ktd = 1
            #     destroy = 1
            #     rospy.sleep(1)
            #     new_thread = threading.Thread(target=job1, name="T1")
            #     new_thread.start()
            #     flag = 1
def go_home(x, y, i):
    """
    导航机器人回到安全区（通常是最后一个导航点）。

    :param x: 目标点的X坐标。
    :type x: float
    :param y: 目标点的Y坐标。
    :type y: float
    :param i: 目标点在列表中的索引。
    :type i: int
    """
    global car_to_goal_x
    global car_to_goal_y    
    print('------------>go_home<--------------') # 打印调试信息
    
    if i  == pose_num-1: # 如果是最后一个导航点（回家的点）
        #导航到安全区
        target = Pose(Point(x, y, 0), Quaternion(qtn_list[i][0], qtn_list[i][1], qtn_list[i][2], qtn_list[i][3])) # 创建目标姿态
        goal = MoveBaseGoal() # 创建MoveBaseGoal对象
        goal.target_pose.pose = target # 设置目标姿态
        goal.target_pose.header.frame_id = 'map' # 设置坐标系
        goal.target_pose.header.stamp = rospy.Time.now() # 设置时间戳
        rospy.loginfo("Going to: " + str(target)) # 记录日志
        new_thread1 = threading.Thread(target = dis, args=(x,y), name="T2") # 创建新线程测量距离
        new_thread1.start() # 启动距离测量线程
        rospy.sleep(0.1) # 短暂休眠
        move_base.send_goal(goal) # 向move_base发送导航目标
        while not distance: # 等待直到距离测量线程将distance设置为0
            rospy.sleep(1) # 休眠1秒
            print("wwwwwwwwwwwww") # 调试信息
        new_thread1.join() # 等待距离测量线程结束
        
    # elif i==pose_num-1:
    #     print('------------------run_last----------------------')
    #     pub = rospy.Publisher("/cmd_vel",Twist,queue_size=1000)
    #     while not rospy.is_shutdown():
    #         rospy.sleep(0.1)
    #         try:
    #             #def lookup_transform(self, target_frame, source_frame, time, timeout=rospy.Duration(0.0)): 其实是获取tf树中两个坐标系之间的变换
    #             rospy.loginfo("相对坐标:(%.2f,%.2f)",
    #                         car_to_goal_x,
    #                         car_to_goal_y,
    #                         )
    #             print(distance)   
    #             twist = Twist()
    #             twist.linear.x = 1 * math.sqrt(math.pow(car_to_goal_x,2) + math.pow( car_to_goal_y ,2))
    #             twist.angular.z = 2 * math.atan2(car_to_goal_y , car_to_goal_x)
    #             pub.publish(twist)
    #             if  distance<=0.08:
    #                 break
    #         except Exception as e:
    #             rospy.logwarn("警告:%s",e)



if __name__ == '__main__':
    # 主程序入口
    # 节点初始化
    rospy.init_node('move_test', anonymous=True) # 初始化ROS节点，命名为 'move_test'
    # 转换导航点：将欧拉角偏航转换为四元数，并存储在qtn_list中
    for i in range(pose_num):
        qtn = tf.transformations.quaternion_from_euler(0,0,xy[i][2]) # 将欧拉角 (roll=0, pitch=0, yaw=xy[i][2]) 转换为四元数
        qtn_list.append(qtn) # 将生成的四元数添加到列表中

    # 订阅move_base服务器的消息
    move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server...")
    #等待连接服务器，5s等待时间限制
    while move_base.wait_for_server(rospy.Duration(5.0)) == 0:
        rospy.loginfo("Request to connect to move_base server")
    rospy.loginfo("Be connected successfully")
    #打开摄像头
    # cap = cv2.VideoCapture(0)
    # if not cap.isOpened():
    #     print("cannot open camera")
    # else:
    #     print("open camera success")
    count=1
    # 开跑
    '''
    while 1:
        for i in range(pose_num):
            goals(xy[i][0], xy[i][1], i,count) 
        a=input()
    # go_home(xy[pose_num-1][0], xy[pose_num-1][1],pose_num-1);
    '''
    
    #测点
    listener = tf.TransformListener() #开始监听tf
    pos = []
    while 1:
        a = input()
        if a == 's':
            try:
                (trans,rot) = listener.lookupTransform('map', 'base_link', rospy.Time(0)) #获取'base_link'相对于'map'的坐标变换，返回两个列表
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
               continue
            print('orient',rot)
            #获取yaw角信息
            euler = tf.transformations.euler_from_quaternion(rot)
            yaw = euler[2]  # 第三个元素是yaw角
            trans[2]= yaw
            pos.append(trans)
            print('position',trans)
            rospy.sleep(0.5)
        if a == 'q':
           break

    

    
    print(pos)
    print("  ________             __                ________ ")
    print(" /_  __/ /_  ___  ____/ /___  _________/_  __/ /_")
    print("  / / / __ \/ _ \/ __  / __ \/ ___/ ___// / / __/")
    print(" / / / / / /  __/ /_/ / /_/ / /  / /__ / / / /_  ")
    print("/_/ /_/ /_/\___/\__,_/\____/_/   \___//_/  \__/  ")
    print()
