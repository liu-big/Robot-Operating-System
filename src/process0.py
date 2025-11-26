#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 导入必要的库
import argparse # 用于解析命令行参数
import tf # 用于处理ROS中的坐标变换
import threading # 用于多线程编程
import rospy # ROS Python客户端库
import sys # 提供对Python解释器相关变量和函数的访问
import math # 提供数学函数
from sensor_msgs.msg import LaserScan # 激光雷达数据消息类型
import actionlib # ROS动作库，用于与动作服务器通信
from actionlib_msgs.msg import * # 导入所有动作消息类型
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist # 几何消息类型，用于表示位姿、点、四元数、速度等
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal # 导航动作消息类型
import roslib # ROS库，用于查找包路径等
import tf2_ros # ROS2的坐标变换库，兼容ROS1
from geometry_msgs.msg import TransformStamped # 坐标变换消息类型
import numpy as np # 用于数值计算，特别是数组操作
from std_msgs.msg import String, Float32MultiArray , Int32MultiArray # 标准消息类型，用于字符串、浮点数数组、整数数组
import ctypes # 用于调用动态链接库函数
from ctypes.util import find_library # 用于查找库文件
import os # 提供与操作系统交互的函数
from std_msgs.msg import Int32 # 整数消息类型
from collections import deque # 双端队列，用于高效地从两端添加和删除元素
from pydub import AudioSegment # 用于处理音频文件
from pydub.playback import play # 用于播放音频
from playsound import playsound # 用于播放声音文件
from geometry_msgs.msg import TransformStamped, PoseStamped # 坐标变换和位姿消息类型
from tf2_geometry_msgs import do_transform_pose # 用于对位姿进行坐标变换
import numpy as np # 再次导入numpy，可能因为之前有其他用途
from pykalman import KalmanFilter # 导入卡尔曼滤波器库

# 定义全局变量
global task_xy # 任务点坐标列表
global qtn_list_xy # 四元数列表
global qtn_list_task_xy # 任务点四元数列表
global pose_num_xy # 位姿数量
global pose_num_task_xy # 任务点位姿数量
global yaw # 偏航角
global now_pose_X # 当前X坐标
global now_pose_Y # 当前Y坐标
global mission_flag # 任务标志


# 预设任务点坐标列表，每个子列表包含 [x, y, yaw] 或 [x, y, z] (根据实际使用情况)
task_xy =[[0.326577, 0.014266, 3.061457],[-1.819254, 0.021767, -3.138521],[-2.369234, -0.140197, 1.137543],
          [-2.369234, -0.140197, -0.613619],[-2.684323, -1.903119,-1.555473],[-1.286645, -1.929026, 1.832317],
          [-1.929975, -1.293006, 1.569783],[-2.341881, -0.005103 , 0.501970]]

global count # 全局计数器
count = 0 

qtn_list_xy = []           # 存储四元数的列表
qtn_list_task_xy = []      # 存储任务点四元数的列表

pose_num_task_xy=len(task_xy) # 任务点位姿的数量
yaw = 0 # 初始化偏航角
global move_base # 全局导航客户端对象
now_pose_X=0 # 初始化当前X坐标
now_pose_Y=0 # 初始化当前Y坐标
mission_flag = 1 # 任务标志，初始化为1
error = 0.0 # 误差变量，初始化为0.0
stop_flag = 0 # 停止标志，初始化为0



class PointProcessor:
    """
    PointProcessor 类用于处理姿态点，包括异常值移除和卡尔曼滤波，以获取更平滑和可靠的姿态数据。
    """
    def __init__(self):
        """
        构造函数，初始化点列表和上一个最优姿态点及自信度。
        """
        self.points = [] # 存储接收到的位姿点
        self.last_optimal_point = [0, 0, 0]  # 上一个最优位姿点，初始化为 [0, 0, 0]
        self.last_confidence = 0  # 上一个自信度，初始化为 0

    # 方法：add_point
    # 功能：向处理器添加新的位姿点，并返回经过处理后的最优位姿点和自信度。
    # 参数：
    #   x: 机器人的X坐标。
    #   y: 机器人的Y坐标。
    #   yaw: 机器人的偏航角。
    # 返回：
    #   optimal_point: 经过滤波和处理后的最优位姿点 [x, y, yaw]。
    #   confidence: 当前最优位姿点的自信度。
    def add_point(self, x, y, yaw):
        """
        添加新的姿态点并进行处理。

        参数:
            x (float): 机器人的X坐标。
            y (float): 机器人的Y坐标。
            yaw (float): 机器人的偏航角。

        返回:
            tuple: 经过处理后的最优姿态点和对应的自信度。
        """
        # 忽略无效点 (0, 0, 0)，直接返回上一个最优位姿点和自信度
        if x == 0 and y == 0 and yaw == 0:
            return self.last_optimal_point, self.last_confidence

        # 添加新点到列表中
        self.points.append([x, y, yaw])
        
        # 移除异常值
        filtered_points = self.remove_outliers(self.points)
        
        # 应用卡尔曼滤波
        smoothed_points = self.kalman_filter(filtered_points)
        
        # 计算当前的最优点（均值）
        if len(smoothed_points) > 0:
            optimal_point = np.mean(smoothed_points, axis=0) # 计算平滑后点的均值作为最优位姿点
            self.last_optimal_point = optimal_point # 更新上一个最优位姿点
            self.last_confidence = self.calculate_confidence(smoothed_points) # 计算并更新自信度
        else:
            optimal_point = self.last_optimal_point # 如果没有平滑点，则使用上一个最优位姿点
        
        return optimal_point, self.last_confidence # 返回最优位姿点和自信度

    # 方法：kalman_filter
    # 功能：对输入的位姿点列表应用卡尔曼滤波，以平滑数据。
    # 参数：
    #   points: 待滤波的位姿点列表。
    # 返回：
    #   smoothed_points: 经过卡尔曼滤波后的平滑位姿点列表。
    def kalman_filter(self, points):
        """
        对姿态点应用卡尔曼滤波。

        参数:
            points (list): 姿态点列表。

        返回:
            list: 经过卡尔曼滤波平滑后的姿态点列表。
        """
        # 将点列表转换为numpy数组
        points_array = np.array(points)
        
        # 如果点数组为空，返回空列表
        if points_array.size == 0:
            return []

        # 定义卡尔曼滤波器
        kf = KalmanFilter(initial_state_mean=points_array[0],  # 初始状态均值设为第一个点
                          n_dim_obs=3)  # 观测维度为3（x, y, yaw）
        
        # 对数据应用卡尔曼滤波器
        smoothed_points, _ = kf.smooth(points_array) # 使用smooth方法进行平滑
        
        return smoothed_points.tolist() # 返回平滑后的点列表

    # 方法：remove_outliers
    # 功能：从位姿点列表中移除异常值。
    # 参数：
    #   points: 待处理的位姿点列表。
    #   threshold: 异常值判断阈值，默认为1.5。
    # 返回：
    #   filtered_points: 移除异常值后的位姿点列表。
    def remove_outliers(self, points, threshold=1.5):
        """
        移除姿态点列表中的异常值。

        参数:
            points (list): 姿态点列表。
            threshold (float): 异常值判断阈值。

        返回:
            list: 移除异常值后的姿态点列表。
        """
        # 将点列表转换为numpy数组
        points_array = np.array(points)
        
        # 如果点数组为空，返回空列表
        if points_array.size == 0:
            return []

        # 计算均值和标准差
        mean = np.mean(points_array, axis=0) # 计算每个维度的均值
        std = np.std(points_array, axis=0) # 计算每个维度的标准差
        
        # 计算每个点到均值的距离
        distances = np.linalg.norm(points_array - mean, axis=1) # 计算欧几里得距离
        
        # 移除距离均值太远的点（异常值）
        filtered_points = points_array[distances < threshold * np.mean(distances)] # 筛选出非异常值点
        
        return filtered_points.tolist() # 返回过滤后的点列表

    # 方法：calculate_confidence
    # 功能：计算当前位姿点集合的自信度。
    # 参数：
    #   points: 位姿点列表。
    # 返回：
    #   confidence: 计算出的自信度值。
    def calculate_confidence(self, points):
        """
        计算姿态点的自信度。

        参数:
            points (list): 姿态点列表。

        返回:
            float: 计算出的自信度值。
        """
        # 将点列表转换为numpy数组
        points_array = np.array(points)
        
        # 计算标准差作为自信度指标，标准差越小，自信度越高
        std_dev = np.std(points_array, axis=0) # 计算每个维度的标准差
        
        # 使用指数函数归一化自信度，确保范围在 (0, 1]
        confidence = np.exp(-np.mean(std_dev)) # 标准差的均值越大，自信度越低
        
        return confidence # 返回自信度


# 以下是被注释掉的PointProcessor类的另一个版本，可能用于调试或备用
# class PointProcessor:
#     def __init__(self, max_points=100, confidence_threshold=0.9):
#         self.points = []
#         self.last_optimal_point = [0, 0, 0]  # 初始化为 [0, 0, 0]
#         self.last_confidence = 0  # 初始化自信度为 0
#         self.fixed = False  # 标识是否固定最优点
#         self.max_points = max_points  # 最大存储点数量
#         self.confidence_threshold = confidence_threshold  # 自信度阈值

#     def add_point(self, x, y, yaw):
        """
        添加新的姿态点并进行处理。

        参数:
            x (float): 机器人的X坐标。
            y (float): 机器人的Y坐标。
            yaw (float): 机器人的偏航角。

        返回:
            tuple: 经过处理后的最优姿态点和对应的自信度。
        """
#         # 如果最优点已固定，直接返回固定的最优点和自信度
#         if self.fixed:
#             return self.last_optimal_point, self.last_confidence

#         # 忽略无效点 (0, 0, 0)
#         if x == 0 and y == 0 and yaw == 0:
#             return self.last_optimal_point, self.last_confidence

#         # 添加新点到列表中
#         self.points.append([x, y, yaw])
        
#         # 移除异常值
#         filtered_points = self.remove_outliers(self.points)
        
#         # 应用卡尔曼滤波
#         smoothed_points = self.kalman_filter(filtered_points)
        
#         # 计算当前的最优点（均值）
#         if len(smoothed_points) > 0:
#             optimal_point = np.mean(smoothed_points, axis=0)
#             self.last_optimal_point = list(optimal_point)  # 确保为列表
#             self.last_confidence = self.calculate_confidence(smoothed_points)

#             # 如果达到最大点数或自信度超过阈值，固定最优点
#             if len(self.points) >= self.max_points or self.last_confidence >= self.confidence_threshold:
#                 self.fixed = True
#         else:
#             optimal_point = self.last_optimal_point
        
#         return optimal_point, self.last_confidence

#     def kalman_filter(self, points):
        """
        对姿态点应用卡尔曼滤波。

        参数:
            points (list): 姿态点列表。

        返回:
            list: 经过卡尔曼滤波平滑后的姿态点列表。
        """
#         # 将点列表转换为numpy数组
#         points_array = np.array(points)
        
#         # 如果点数组为空，返回空列表
#         if points_array.size == 0:
#             return []

#         # 定义卡尔曼滤波器
#         kf = KalmanFilter(initial_state_mean=points_array[0],  # 初始状态均值设为第一个点
#                           n_dim_obs=3)  # 观测维度为3（x, y, yaw）
        
#         # 对数据应用卡尔曼滤波器
#         smoothed_points, _ = kf.smooth(points_array)
        
#         return smoothed_points.tolist()

#     def remove_outliers(self, points, threshold=1.5):
        """
        移除姿态点列表中的异常值。

        参数:
            points (list): 姿态点列表。
            threshold (float): 异常值判断阈值。

        返回:
            list: 移除异常值后的姿态点列表。
        """
#         # 将点列表转换为numpy数组
#         points_array = np.array(points)
        
#         # 如果点数组为空，返回空列表
#         if points_array.size == 0:
#             return []

#         # 计算均值和标准差
#         mean = np.mean(points_array, axis=0)
#         std = np.std(points_array, axis=0)
        
#         # 计算每个点到均值的距离
#         distances = np.linalg.norm(points_array - mean, axis=1)
        
#         # 移除距离均值太远的点（异常值）
#         filtered_points = points_array[distances < threshold * np.mean(distances)]
        
#         return filtered_points.tolist()

#     def calculate_confidence(self, points):
        """
        计算姿态点的自信度。

        参数:
            points (list): 姿态点列表。

        返回:
            float: 计算出的自信度值。
        """
#         # 将点列表转换为numpy数组
#         points_array = np.array(points)
        
#         # 计算标准差作为自信度指标，标准差越小，自信度越高
#         std_dev = np.std(points_array, axis=0)
        
#         # 使用指数函数归一化自信度，确保范围在 (0, 1]
#         confidence = np.exp(-np.mean(std_dev))
        
#         return confidence



#####################################限制函数####################################

def limt(limt,target):
    if limt>target:
        limt=target
    if limt<-target:
        limt=-target
    return limt

##############################################语音播放#################################################

def play_voice(number):
    audio = AudioSegment.from_file(str(int(number))+".mp3")
    play(audio)

#########################################墙pid参数及其控制函数##########################################
global w_kp 
global w_ki
global w_kd
global w_target
global w_e_all
global w_last_e

w_kp = 0.001 #2
w_ki = 0.000
w_kd = 0.005 #0
w_e_all = 0
w_last_e = 0

global x_f,x_b,y_l,y_r
x_f=0.0
x_b=0.0
y_l=0.0
y_r=0.0

def w_pid_cal(pid_target,dis):
    global w_kp 
    global w_ki
    global w_kd
    global w_e_all
    global w_last_e
    e = dis -pid_target
    #if e>-25 and e<25:
      #e = 0
    w_e_all = w_e_all+e
    pid = w_kp*e+w_ki*w_e_all+w_kd*(e-w_last_e)
    w_last_e = e
    return pid
#######################################姿态pid参数及其控制函数############################################
global p_kp 
global p_ki
global p_kd
global p_e_all
global p_last_e
global p_pid
p_kp = -1.2
p_ki = 0
p_kd = -0.00
p_e_all = 0
p_last_e = 0
p_pid = 0

def p_pid_cal(pid_target,pose):
    global p_kp 
    global p_ki
    global p_kd
    global p_e_all
    global p_last_e
    ture_pose = (pose/3.14159265359*180.0+180.0)%360
    
    if pid_target==0:
        if ture_pose>0 and ture_pose<180:
            pid_target=0
        if ture_pose>180 and ture_pose<360:
            pid_target=360   
            
    e = ture_pose -pid_target
    # print(e) 
    p_e_all = p_e_all+e
    pid = p_kp*e+p_ki*p_e_all+p_kd*(e-p_last_e)
    #rospy.loginfo("e %f",e)	
    p_last_e = e
    return pid



global point_kp 
global point_ki
global point_kd
global point_e_all
global point_last_e
global point_pid
point_kp = -3
point_ki = 0
point_kd = 0
point_e_all = 0
point_last_e = 0
point_pid = 0

def point_pid(pid_target_x,ture):
    global point_kp 
    global point_ki
    global point_kd
    global point_e_all
    global point_last_e
    e = ture -pid_target_x
    point_e_all = point_e_all+e
    pid = point_kp*e+point_ki*point_e_all+point_kd*(e-point_last_e)	
    point_last_e = e
    return pid


def pid_stop(target_x,target_y,target_yaw):
    global point_kp ,now_pose_X,now_pose_Y
    pid_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=7)
    rate_loop_pid=rospy.Rate(7.2)
    speed = Twist()
    while not rospy.is_shutdown():
        map_pid_x = point_pid(target_x,now_pose_X)
        map_pid_y = point_pid(target_y,now_pose_Y)
        p_pid = p_pid_cal(target_yaw,yaw)

        speed.linear.x = map_pid_x
        speed.linear.y = map_pid_y
        speed.angular.z = p_pid/180.0*3.14159265359
        print("__________________________________________________________________")
        print(map_pid_x,map_pid_y,p_pid/180.0*3.14159265359)
        pid_vel_pub.publish(speed)   
        
        if abs(target_x-now_pose_X) and abs(target_y-now_pose_Y)<=0.1:
            speed.linear.x = 0
            speed.linear.y = 0
            speed.angular.z = 0
            pid_vel_pub.publish(speed)   
            break
        rate_loop_pid.sleep()

# def pid_stop2(target_x,target_y,target_yaw):
#     global w_kp,w_ki,w_kd,w_e_all
#     w_kp = 0.5 #1.5
#     w_ki = 0 #0.005
#     w_kd = 0 #0.01
#     w_e_all=0
#     count =0
#     pid_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=7)
#     rate_loop_pid=rospy.Rate(7)
#     speed = Twist()
#     wich_x = 0
#     wich_y = 0
#     time = 0
#     while not rospy.is_shutdown():
#         rate_loop_pid.sleep()
#         if target_x>0:
#             pid_x = w_pid_cal(target_x,x_f)
#             wich_x=x_f
#         if target_x<0:
#             pid_x = w_pid_cal(target_x,-x_b)
#             wich_x=-x_b
#         if target_y>0:
#             pid_y = w_pid_cal(target_y,y_l)
#             wich_y = y_l
#         if target_y<0:
#             pid_y = w_pid_cal(target_y,-y_r)
#             wich_y = -y_r
#         p_pid = p_pid_cal(target_yaw,yaw)
#         speed.linear.y = 0
#         speed.linear.x = pid_x
#         speed.angular.z = p_pid/180.0*3.14159265359
#         w_e_all=limt(w_e_all,5)
#         if abs(wich_x-target_x)<=0.05 and abs(target_yaw-(yaw/3.1415926*180+180))<=5:
#             speed.linear.x = 0
#             speed.linear.y = 0
#             speed.linear.z = 0
#             pid_vel_pub.publish(speed) 
#             #rospy.sleep(0.5)
#             w_e_all=0
#             break
#         pid_vel_pub.publish(speed) 

###################################################去识别恐怖分子的动作函数(包括去、掉头、回动作)#########################################
def pid_go(target_x,target_y,target_yaw):
    global w_kp,w_ki,w_kd,w_e_all,count
    print("--------开始pid_go--------")
    w_kp = 1.0 #2
    w_ki = 0
    w_kd = 0.01 #0
    w_e_all=0
    pid_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=7)
    rate_loop_pid=rospy.Rate(7)
    speed = Twist()
    wich_x = 0
    wich_y = 0
    key = 0
    while not rospy.is_shutdown():
        print("x_f",x_f)
        print("y_l",y_l)
        
        if target_x>0:
            pid_x = w_pid_cal(target_x,x_f)
            wich_x=x_f
        if target_x<0:
            pid_x = w_pid_cal(target_x,-x_b)
            wich_x=-x_b
        if target_y>0:
            pid_y = w_pid_cal(target_y,y_l)
            wich_y = y_l
        if target_y<0:
            pid_y = w_pid_cal(target_y,-y_r)
            wich_y = -y_r
        p_pid = p_pid_cal(target_yaw,yaw)
        
        if target_x == 0.7:
            speed.linear.y = pid_y
            if abs(target_x-wich_x)<0.2 and abs(target_y-wich_y)<0.2:
                w_e_all=0
                speed.linear.x = 0
                speed.linear.y = 0
                break #################################################################这个后面条件是task这里是debug
        else :
            if abs(wich_x)>0.25:
                speed.linear.y = 0
                #speed.linear.y = 0.05*pid_y
            else:
                speed.linear.y = pid_y
                key = 1

        if wich_y<0.9 and key == 1:
            print("@@@@@@@@@@@@@@@@@@@@@@@@@")
            target_x = 0.7

        speed.linear.x = pid_x
        speed.angular.z = p_pid/180.0*3.14159265359

        pid_vel_pub.publish(speed)       
        rate_loop_pid.sleep()
    
        #这个后面移植标定
        # if task != 0:
        #     w_e_all=0
        #     # thread_dis.join()
        #     break
    print("--------结束pid_go--------")


def pid_back(target_x,target_y,target_yaw):
    global w_kp,w_ki,w_kd,w_e_all
    print("--------开始pid_back--------")
    w_kp = 1.0 #2
    w_ki = 0
    w_kd = 0.005 #0
    count =0
    w_e_all=0
    pid_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=7)
    rate_loop_pid=rospy.Rate(10)
    speed = Twist()
    wich_x = 0
    wich_y = 0
    while not rospy.is_shutdown():
        if target_x>0:
            pid_x = w_pid_cal(target_x,x_f)
            wich_x=x_f
        if target_x<0:
            pid_x = w_pid_cal(target_x,-x_b)
            wich_x=-x_b
        if target_y>0:
            pid_y = w_pid_cal(target_y,y_l)
            wich_y = y_l
        if target_y<0:
            pid_y = w_pid_cal(target_y,-y_r)
            wich_y = -y_r
        p_pid = p_pid_cal(target_yaw,yaw)
        
        if  abs(target_y-wich_y)<0.2:
            print("pid_back结束")
            speed.linear.x = 0
            speed.linear.y = 0
            break #################################################################这个后面条件是task这里是debug
        else:
            if abs(wich_x)>0.25:
                speed.linear.y = 0.1*pid_y
            else:
                speed.linear.y = pid_y
            if abs(wich_y)>1.4:
                speed.linear.x=0
            else :
                speed.linear.x = pid_x
            speed.angular.z = p_pid/180.0*3.14159265359

        pid_vel_pub.publish(speed)       
        rate_loop_pid.sleep()
    
        # if task != 0:
        #     w_e_all=0
        #     # thread_dis.join()
        #     break

    print("--------结束pid_back--------")

def pid_turn(target_x,target_y,target_yaw):
    global point_kp 
    count = 0
    pid_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=7)
    rate_loop_pid=rospy.Rate(10)
    speed = Twist()
    while not rospy.is_shutdown():
        p_pid = p_pid_cal(target_yaw,yaw)
        speed.linear.x = 0
        speed.linear.y = 0
        speed.angular.z = p_pid/180.0*3.14159265359

        pid_vel_pub.publish(speed)   
        rate_loop_pid.sleep()
        print("EEEEEEEEEEE:",abs(target_yaw-(yaw/3.1415926*180+180)))
        if abs(target_yaw-(yaw/3.1415926*180+180)%360)<10 or abs(target_yaw-(yaw/3.1415926*180+180)%360)>350:
            count+=1
            print("************************count",count)
        if count>=5:
            print("QQQQQQQQQQQQQQQQQQQQQQQQQ")
            break



global Flag_find_bit
Flag_find_bit = 0
global Flag_find_sure
Flag_find_sure = 0
global Flag_turn_bit
Flag_turn_bit = 0
global target_point_find
target_point_find = 0
def Flag_find():
    global vision_data,confidence,Flag_find_bit,Flag_turn_bit,target_point_find
    rate_loop=rospy.Rate(10)
    while not rospy.is_shutdown():
        if confidence > 0.8:
            Flag_find_bit+=1

        if Flag_find_bit > 10:
            Flag_find_sure = 1
            q = tf.transformations.quaternion_from_euler(0,0,optimal_point[2])
            target_point_find = Pose(Point(optimal_point[0]+1*math.cos(yaw+3.14) ,optimal_point[1]+1*math.sin(yaw+3.14) , 0), Quaternion(q[0], q[1], q[2], q[3]))
            break
        else :
            if confidence > 0.6:
                Flag_turn_bit += 1
            Flag_find_bit = 0
        rate_loop.sleep()


###########################################################导航子函数##########################################################
# 控制导航点发布
global count_dis_times
count_dis_times=0
def dis_func(x,y,min_dis,i):
    global dis_trun_off,now_pose_X,now_pose_Y,distance,count_dis_times
    print("dis_func函数已启动:"+str(count_dis_times)+"次")
    count_dis_times+=1
    # lock = threading.RLock()
    dis_fun_rate=rospy.Rate(10)
    while not rospy.is_shutdown():
        dis_fun_rate.sleep()
        car_to_map_x=now_pose_X
        car_to_map_y=now_pose_Y
        # 计算小车到目标点的距离
        distance = pow(pow( car_to_map_x - x, 2) + pow(car_to_map_y - y, 2), 0.5)
        print("distance:"+str(distance))
        print("i",i)
        if distance<min_dis :
 
            if i == 666:
                pid_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=7)
                speed = Twist()
                speed.linear.x = 0
                speed.linear.y = 0
                pid_vel_pub.publish(speed)
                play_voice(vision_data+3)   
                print("next goal")    
            
            if i < 20 :       
                #goals(x, y, i)
                if i == 4:
                    print("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^")
                    pid_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=7)
                    speed = Twist()
                    speed.linear.x = 0
                    speed.linear.y = 0
                    pid_vel_pub.publish(speed)
                    play_voice(7)
                break

            # move_base.send_goal(goal)
            break

def goals(x, y, i):
    print("******************************************发布导航点*********************************************")
    global qtn_list_task_xy,move_base,now_pose_X,Flag_turn_bit,optimal_point
    print("optimal_point::::::::::::::::::::",optimal_point)    
    if Flag_turn_bit >= 10:
        if optimal_point[2]!= None:
            q = tf.transformations.quaternion_from_euler(0,0,optimal_point[2])
            target_point = Pose(Point(x, y, 0), Quaternion(q[0], q[1], q[2], q[3]))
        else:
            target_point = Pose(Point(x, y, 0), Quaternion(qtn_list_task_xy[i][0], qtn_list_task_xy[i][1], qtn_list_task_xy[i][2], qtn_list_task_xy[i][3]))
    else :
        target_point = Pose(Point(x, y, 0), Quaternion(qtn_list_task_xy[i][0], qtn_list_task_xy[i][1], qtn_list_task_xy[i][2], qtn_list_task_xy[i][3]))
    goal = MoveBaseGoal()
    goal.target_pose.pose = target_point
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    rospy.loginfo("Going to: " + str(target_point))
    print("goal")
    move_base.send_goal(goal)


#########################################################视觉处理#####################################################

#标定参数
cx = 319.03663
cy = 235.3358
fx = 408.12625
fy = 408.87682

#全局变量
global u 
global v
global X
global Z
global sign
global task

#雷达数据转换变量
u = 0.0
v = 0.0
X = 0.0
Z = 0.0
sign = 0

#任务识别变量
task = 0.0
task_list = [0,0,0]
vision_data = 0.0

# 创建一个双向队列，用于存储历史数据
data_window = deque(maxlen=10)
average_x = 0.0
average_z = 0.0

# 设置指数加权移动平均滤波器的权重参数
alpha = 0.2  

def vision_callback(msg):
    global u, v, vision_data, sign,task
    global target, error, mid_angle, stop_flag


    if sign == 0:
        sign=1
    #判断数据包格式是否正确，进行数据记录
    if len(msg.data) >= 13:
        task_assigned = False
        data_list = list(msg.data)
        if task_assigned == False:
            if data_list[0]+data_list[1]+data_list[2] != 0:
                for i in range(3):
                    if data_list[i] != 0.0:
                        task = data_list[i]
                        task_assigned = True
        #print("task:",task)

        # 恐怖分子1：寻找道具：警棍
        if task == 1.0:
            if data_list[3] == 4.0:
                if data_list[7] != 0.0 and data_list[8] != 0.0:
                    u = data_list[7]
                    v = data_list[8]
                    vision_data = 1.0
                else:
                    u = 0.0
                    v = 0.0
            else:
                u = 0.0
                v = 0.0
                vision_data = 0.0

        # 恐怖分子2：寻找道具：防弹衣
        if task == 2.0:
            if data_list[4] == 5.0:
                if data_list[9] != 0.0 and data_list[10] != 0.0:
                    u = data_list[9]
                    v = data_list[10]
                    vision_data = 2.0
                else:
                    u = 0.0
                    v = 0.0
            else:
                u = 0.0
                v = 0.0
                vision_data = 0.0

        # 恐怖分子3：寻找道具：催泪瓦斯
        if task == 3.0:
            if data_list[5] == 6.0:
                if data_list[11] != 0.0 and data_list[12] != 0.0:
                    u = data_list[11]
                    v = data_list[12]
                    vision_data = 3.0
                else:
                    u = 0.0
                    v = 0.0
            else:
                u = 0.0
                v = 0.0
                vision_data = 0.0

        #print(vision_data)

    else:
        #rospy.logwarn("Received an empty Float32MultiArray")
        # 此时进行巡线任务 vision_data = []的数据为error,mid_angle,stop_flag
        error = msg.data[0]
        mid_angle = msg.data[1]
        stop_flag = msg.data[2]

        
def vision_listen():
    rospy.Subscriber('/vision_msg_list',Float32MultiArray,vision_callback,queue_size=10) #订阅视觉话题
    rospy.spin()


global target_in_map
target_in_map = PoseStamped()
global optimal_point
global confidence
confidence = 0
optimal_point=[]
processor = PointProcessor()

def transform_to_map():
    global target_in_map,optimal_point,vision_data,yaw,atan_x,confidence,goal_sign
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rate_loop=rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            camera_to_map = tf_buffer.lookup_transform("map", "camera_frame", rospy.Time(0))
            target_in_camera = PoseStamped()
            target_in_camera.header.frame_id = "camera_frame"
            target_in_camera.pose.position.x = average_z
            target_in_camera.pose.position.y = -average_x
            target_in_camera.pose.position.z = 0
            qtn = tf.transformations.quaternion_from_euler(0, 0, 0)
            target_in_camera.pose.orientation.x = qtn[0]
            target_in_camera.pose.orientation.y = qtn[1]
            target_in_camera.pose.orientation.z = qtn[2]
            target_in_camera.pose.orientation.w = qtn[3]
            if task!=0 and vision_data!=0 and goal_sign>1 and speed_data.angular.z < 0.3:
                target_in_map = do_transform_pose(target_in_camera, camera_to_map)
                optimal_point, confidence = processor.add_point(target_in_map.pose.position.x, target_in_map.pose.position.y , yaw-atan_x)
                print(f"当前的最优点: (x: {optimal_point[0]}, y: {optimal_point[1]}, yaw: {optimal_point[2]} , confidence: {confidence})")
            # print(type(target_in_map))
            # rospy.loginfo("Target position in map: (x: %f, y: %f, z: %f)", 
            #               , 
            #               target_in_map.pose.position.y, 
            #               target_in_map.pose.position.z)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Failed to get transform")
            rospy.sleep(1)
            continue
        rate_loop.sleep()

#------------------- 雷达话题订阅线程 ---------------- 
global scan_data
scan_data=[]
def get_laserscan(scan):	
    global x_f,x_b,y_l,y_r,yaw,scan_data
    scan_data=scan.ranges

    total=0
    f_count=0
  
    if scan.ranges[453]!=float('inf') or scan.ranges[454]!=float('inf') or scan.ranges[455]!=float('inf'): 
        total=0
        f_count=0
        
        if scan.ranges[452]!=float('inf'):
            total+=scan.ranges[452]
            f_count+=1
        if scan.ranges[453]!=float('inf'):
            total+=scan.ranges[453]
            f_count+=1
        if scan.ranges[454]!=float('inf'):
            total+=scan.ranges[454]
            f_count+=1   
        if f_count:  
            x_f = total/f_count
      

    if scan.ranges[0]!=float('inf') or scan.ranges[1]!=float('inf') or scan.ranges[908]!=float('inf'): 
        total=0
        f_count=0
        if scan.ranges[0]!=float('inf'):
            total+=scan.ranges[0]
            f_count+=1
        if scan.ranges[1]!=float('inf'):
            total+=scan.ranges[1]
            f_count+=1
        if scan.ranges[908]!=float('inf'):
            total+=scan.ranges[908]
            f_count+=1    
        if f_count:  
            x_b = total/f_count


    if scan.ranges[680]!=float('inf') or scan.ranges[681]!=float('inf') or scan.ranges[682]!=float('inf'): 
        total=0
        f_count=0
        if scan.ranges[680]!=float('inf'):
            total+=scan.ranges[680]
            f_count+=1
        if scan.ranges[681]!=float('inf'):
            total+=scan.ranges[681]
            f_count+=1
        if scan.ranges[682]!=float('inf'):
            total+=scan.ranges[682]
            f_count+=1   
        if f_count:                     
            y_l = total/f_count 

    if scan.ranges[227]!=float('inf') or scan.ranges[228]!=float('inf') or scan.ranges[229]!=float('inf'): 
        total=0
        f_count=0
        if scan.ranges[227]!=float('inf'):
            total+=scan.ranges[227]
            f_count+=1
        if scan.ranges[228]!=float('inf'):
            total+=scan.ranges[228]
            f_count+=1
        if scan.ranges[229]!=float('inf'):
            total+=scan.ranges[229]
            f_count+=1           
        if f_count:            
            y_r = total/f_count 


def laser_listen():
    rospy.Subscriber('/scan', LaserScan,get_laserscan,queue_size=7)
    rospy.spin()



global atan_x
atan_x = 0
def calibration_tf():
    global average_x, average_z,sign, vision_data,task,scan_data,atan_x
    global u,v
    depth = 0
    rate_loop=rospy.Rate(10)
    print(")))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))00")
    #broadcaster = tf2_ros.TransformBroadcaster()  # 创建广播器

    broadcaster = tf2_ros.TransformBroadcaster()

    while not rospy.is_shutdown():
        if u != 0:
            tanx = (u - cx)/fx
            atan_x = math.atan(tanx)
            angle_x = math.degrees(atan_x)

            if u and v != 0.0:
                    depth = scan_data[440-int(angle_x/0.5)]
                    if depth >= 5:
                        vision_data = 0
            else:
                depth = 0.0

            # if depth == float('inf'):
            #     return 0
                    
            # else :
            #     rospy.loginfo("scan: %f",depth)

            X = (u - cx) * depth / fx
            Z = depth
            #print("depth:",depth)
            #如果msg.data[x]不为x.0，则清空TF坐标
            if (vision_data != task):
                sign = 0
                tfs = TransformStamped()
                tfs.header.frame_id = "camera_frame"
                tfs.header.stamp = rospy.Time.now()
                tfs.child_frame_id = "target_frame"
                tfs.transform.translation.x = 0
                tfs.transform.translation.y = 0
                tfs.transform.translation.z = 0
                tfs.transform.rotation.x = 0
                tfs.transform.rotation.y = 0
                tfs.transform.rotation.z = 0
                tfs.transform.rotation.w = 1
                broadcaster.sendTransform(tfs)
            #     #return



            if depth != float('inf') and X != 0.0 and Z!= 0.0:
                # 计算 (x, z) 数据并存入队列
                data_window.append((X, Z))
                # 更新平均值
                sum_x = sum(x for x, _ in data_window)
                sum_z = sum(z for _, z in data_window)
                average_x = sum_x / len(data_window)
                average_x = alpha * X + (1 - alpha) * average_x
                average_z = sum_z / len(data_window)
                average_z = alpha * Z + (1 - alpha) * average_z

                    # 获取 base_link 到 map 的变换

            # tfs = TransformStamped()

            # tfs.header.frame_id = "map"
            # tfs.header.stamp = rospy.Time.now()
            # tfs.child_frame_id = "target_frame"

            # # 结合相机到目标物体的tf变换
            # if average_x!=0:
            #     tfs.transform.translation.x = now_pose_X + pow(pow(average_x,2) + pow(average_z,2),0.5)*math.cos(math.atan(average_z/(average_x))-abs(yaw))
            #     tfs.transform.translation.y = now_pose_Y + pow(pow(average_x,2) + pow(average_z,2),0.5)*math.sin(math.atan(average_z/(average_x))-abs(yaw))
            #     tfs.transform.translation.z = 0
            #     print("angle",math.atan(-average_z/(average_x))-yaw)
            # qtn = tf.transformations.quaternion_from_euler(0, 0, 0)
            # tfs.transform.rotation.x = 0
            # tfs.transform.rotation.y = 0
            # tfs.transform.rotation.z = 0
            # tfs.transform.rotation.w = 1

            # broadcaster.sendTransform(tfs)



            #4-2.创建 广播的数据(通过 pose 设置)
            tfs = TransformStamped()

            tfs.header.frame_id = "camera_frame"
            tfs.header.stamp = rospy.Time.now()
            tfs.child_frame_id = "target_frame"

            tfs.transform.translation.x = average_z
            tfs.transform.translation.y = -average_x
            tfs.transform.translation.z = 0

            qtn = tf.transformations.quaternion_from_euler(0,0,0)
            tfs.transform.rotation.x = qtn[0]
            tfs.transform.rotation.y = qtn[1]
            tfs.transform.rotation.z = qtn[2]
            tfs.transform.rotation.w = qtn[3]

            #4-3.广播器发布数据
            #broadcaster.sendTransform(tfs)
            if sign == 1:
                broadcaster.sendTransform(tfs)

            else:
                tfs.transform.translation.x = 0
                tfs.transform.translation.y = 0
                tfs.transform.translation.z = 0
                qtn = tf.transformations.quaternion_from_euler(0, 0, 0)
                tfs.transform.rotation.x = qtn[0]
                tfs.transform.rotation.y = qtn[1]
                tfs.transform.rotation.z = qtn[2]
                tfs.transform.rotation.w = qtn[3]
                broadcaster.sendTransform(tfs)
        rate_loop.sleep()

# -----------------------pid回家--------------------------
def pid_line():
        print("---------启动回家--------")
        global yaw
        global w_kp,w_ki,w_kd,w_e_all
        global error , mid_angle , stop_flag
        w_e_all=0
        w_kp = 0.01#0.012 linearx=0.5
        w_ki = 0
        w_kd = 0.01 #0.02
        pid_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=7)
        rate_loop_pid=rospy.Rate(10)
        speed = Twist()
        while not rospy.is_shutdown():
            z_pid=w_pid_cal(0,error)
            z_pid=limt(z_pid,2)
            print(error)
            speed.linear.x = 0.4
            speed.angular.z = z_pid
            if stop_flag == 1:
                print('####################################################################0')
                speed.linear=0
                speed.angular=0
                pid_vel_pub.publish(speed) 
            else:    
                pid_vel_pub.publish(speed)   
            rate_loop_pid.sleep()
            # if abs(x_f-0.22)<0.05 and abs(y_l-0.25)<0.05 and abs(0-(yaw/3.1415926*180+180))<=3:
            #     speed.linear.x = 0
            #     speed.linear.y = 0
            #     pid_vel_pub.publish(speed)
            #     break   
        print("---------结束回家--------")





    
###########################################task函数############################################


def task1_find_terrorist():
    global task
    #去识别框识别恐怖分子
    pid_go(0.15,0.30,180)
    if task !=0:
        play_voice(task)
    pid_turn(0,0,0)
    pid_back(0.15,-1.8,0)

global goal_sign
goal_sign = 0

def task2_find_tool():
    global vision_data,now_pose_X,now_pose_Y,average_x,average_,target_in_map,optimal_point,yaw,confidence,Flag_find_sure,Flag_turn_bit,target_point_find,goal_sign
    q = 0
    for i, j in enumerate(task_xy):
            goal_sign = i
            print("################################") 
            goals(task_xy[i][0],task_xy[i][1],i)
            dis_func(task_xy[i][0],task_xy[i][1],0.2,i)
            print("Flag_find_sure",Flag_find_sure)
            #rospy.sleep(2)
            if i >=4 :
                if Flag_find_sure :
                    print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@",vision_data)
                    Flag_find_sure = 0
                    # q = tf.transformations.quaternion_from_euler(0,0,optimal_point[2])
                    # target_point = Pose(Point(optimal_point[0]+1*math.cos(yaw+3.14) ,optimal_point[1]+1*math.sin(yaw+3.14) , 0), Quaternion(q[0], q[1], q[2], q[3]))
                    # print("now_pose_X:",now_pose_X)
                    # print("now_pose_Y:",now_pose_Y)
                    # print("average_z:",average_z)
                    # print("average_x:",average_x)
                    # print("cos:",1*math.cos(yaw))
                    # print("sin:",1*math.sin(yaw))            
                    goal = MoveBaseGoal()
                    goal.target_pose.pose = target_point_find
                    goal.target_pose.header.frame_id = 'map'
                    goal.target_pose.header.stamp = rospy.Time.now()
                    rospy.loginfo("Going to: " + str(target_point_find))
                    print("goal")
                    print("confidence",confidence)
                    move_base.send_goal(goal)
                    
                    # if confidence>0.9:
                    #     now_goals = [(optimal_point[0]+1*math.cos(yaw+3.14))*math.cos(yaw) ,(optimal_point[1]+1*math.sin(yaw+3.14))*math.sin(yaw) , optimal_point[2]] 
                    #     break
                    # rate_loop.sleep()
                    dis_func(target_point_find.position.x,target_point_find.position.y,0.2,555)
                    print("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
                    #pid_stop2(0.1,0.1,optimal_point[2])
                    pid_stop(now_goals[0]-0.5*math.cos(yaw+3.14),now_goals[1]-0.5*math.sin(yaw+3.14), (optimal_point[2]/3.14159265359*180.0+180.0)%360)
    



    # while not rospy.is_shutdown():
    #         if vision_data!=0 or confidence>0.85:
    #             print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@",vision_data)
    #             q = tf.transformations.quaternion_from_euler(0,0,optimal_point[2])
    #             target_point = Pose(Point(optimal_point[0]+1.4*math.cos(yaw+3.14) ,optimal_point[1]+1.4*math.sin(yaw+3.14) , 0), Quaternion(q[0], q[1], q[2], q[3]))
    #             # print("now_pose_X:",now_pose_X)
    #             # print("now_pose_Y:",now_pose_Y)
    #             # print("average_z:",average_z)
    #             # print("average_x:",average_x)
    #             # print("cos:",1*math.cos(yaw))
    #             # print("sin:",1*math.sin(yaw))            
    #             goal = MoveBaseGoal()
    #             goal.target_pose.pose = target_point
    #             goal.target_pose.header.frame_id = 'map'
    #             goal.target_pose.header.stamp = rospy.Time.now()
    #             rospy.loginfo("Going to: " + str(target_point))
    #             print("goal")
    #             print("confidence",confidence)
    #             move_base.send_goal(goal)
    #             if confidence>0.9:
    #                 now_goals = [(optimal_point[0]+1*math.cos(yaw+3.14))*math.cos(yaw) ,(optimal_point[1]+1*math.sin(yaw+3.14))*math.sin(yaw) , optimal_point[2]] 
    #                 break
    #             rate_loop.sleep()
    # dis_func(now_goals[0],now_goals[1],0.2,555)
    # print("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
    # #pid_stop2(0.1,0.1,optimal_point[2])
    # pid_stop(now_goals[0]-0.5*math.cos(yaw+3.14),now_goals[1]-0.5*math.sin(yaw+3.14), (optimal_point[2]/3.14159265359*180.0+180.0)%360)

    
                    
                

def speed_get():
    rospy.Subscriber('/cmd_vel',Twist,cmd_vel_callback,queue_size=10) #订阅视觉话题
    rospy.spin()


global speed_data
speed_data = Twist()

def cmd_vel_callback(speed):
    speed_data = speed



# ----------------- init ---------------------
# 1.处理点列表的四元数并放在新的列表
# 2.连接move_base

# --------------
def now_pose_xy():
    global now_pose_X,now_pose_Y,yaw
    now_pose=rospy.Rate(10)
    listener = tf.TransformListener()
    while not rospy.is_shutdown():
        now_pose.sleep()
        try:
            (trans, rot) = listener.lookupTransform("map", "base_link", rospy.Time(0))
            # 小车坐标
            now_pose_X=trans[0]
            now_pose_Y=trans[1]
            euler = tf.transformations.euler_from_quaternion(rot)
            yaw = euler[2]   # 第三个元素是yaw角
            #print(yaw)
            #p_pid_cal(0,yaw)
        except Exception as e:
            print("连接tf中.......")

# --------------


# -----------------语音启动-----------
global voice_flag
voice_flag=0
def speech():
    rospy.Subscriber('/anwser',String,speech_going,queue_size=10)
    rospy.spin()

def speech_going(msg):
    global voice_flag
    # while not rospy.is_shutdown():
    #     if msg.data =="\u9ED8\u8BA4\u56DE\u7B54,\u6536\u5230":
    voice_flag=1

def init_fun():
    #转换点
    global qtn_list_xy,qtn_list_task_xy,pose_num_xy,pose_num_task_xy,move_base
    for i in range(pose_num_task_xy):
        qtn = tf.transformations.quaternion_from_euler(0,0,task_xy[i][2])
        qtn_list_task_xy.append(qtn)


    #连接move_base—actition
    move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server...")
    while move_base.wait_for_server(rospy.Duration(5.0)) == 0:
        rospy.loginfo("Request to connect to move_base server")
    rospy.loginfo("Be connected successfully")
    
    

    thread_lidar = threading.Thread(target=laser_listen)                         
    thread_lidar.start() 

    thread_vision = threading.Thread(target=vision_listen)                         
    thread_vision.start() 
     
    thread_now_pose = threading.Thread(target=now_pose_xy)                         
    thread_now_pose.start()

    transform = threading.Thread(target=transform_to_map)                         
    transform.start()

    calibration = threading.Thread(target=calibration_tf)                         
    calibration.start()

    Flag_bit = threading.Thread(target=Flag_find)                         
    Flag_bit.start()

    speed_state = threading.Thread(target=speed_get)                         
    speed_state.start()
    # speech_go = threading.Thread(target=speech)                         
    # speech_go.start()

# ----------------- init ---------------------





if __name__ == '__main__':
    # 初始化节点
    rospy.init_node('move_test', anonymous=True)
    init_fun()
    # while not voice_flag:
    #     print("等待中。")
    a = input()
    begin_time=rospy.Time.now()
    task1_find_terrorist()
    task2_find_tool()

    #pid_go(0.15,0.30,180)
    # pid_turn(0,0,0)
    # pid_back(0.15,-1.8,0)
    #pid_loop()
print(" ________             __                ________ ")
print(" /_  __/ /_  ___  ____/ /___  _________/_  __/ /_")
print("  / / / __ \/ _ \/ __  / __ \/ ___/ ___// / / __/")
print(" / / / / / /  __/ /_/ / /_/ / /  / /__ / / / /_  ")
print("/_/ /_/ /_/\___/\__,_/\____/_/   \___//_/  \__/  ")
print()
finish_time=rospy.Time.now()
print("时间-------->%.2f",(finish_time-begin_time).to_sec())




# import actionlib
# from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# # 创建一个Action客户端
# client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

# client.wait_for_server()

# goal = MoveBaseGoal()
# goal.target_pose.header.frame_id = "map"
# goal.target_pose.pose.position.x = 1.0  # 设置目标点的x坐标
# goal.target_pose.pose.position.y = 2.0  # 设置目标点的y坐标

# client.send_goal(goal)

# client.wait_for_result()
# result = client.get_result()

# if result and result.status == 3:  # status 3 表示导航成功完成
#     print("导航已完成")
# else:
#     print("导航未成功完成")
# 0：PENDING（等待） - 导航目标正在等待处理。

# 1：ACTIVE（激活） - 导航目标正在被执行。

# 2：PREEMPTED（中断） - 导航目标被中断。

# 3：SUCCEEDED（成功完成） - 导航目标成功完成。

# 4：ABORTED（中止） - 导航目标被中止。

# 5：REJECTED（拒绝） - 导航目标被拒绝。

# 6：PREEMPTING（正在中断） - 导航目标正在被中断。

# 7：RECALLING（正在召回） - 导航目标正在被召回。

# 8：RECALLED（已召回） - 导航目标已经被召回。

# 9：LOST（丢失） - 导航目标丢失。
