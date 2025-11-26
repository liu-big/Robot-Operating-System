#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time
import argparse
import tf
import threading
import rospy
import sys
import math
from sensor_msgs.msg import LaserScan
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import roslib
import tf2_ros
from geometry_msgs.msg import TransformStamped
import numpy as np
from std_msgs.msg import String, Float32MultiArray , Int32MultiArray
import ctypes
from ctypes.util import find_library
import os
from std_msgs.msg import Int32
from collections import deque
from pydub import AudioSegment
from pydub.playback import play
from playsound import playsound
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_geometry_msgs import do_transform_pose
import numpy as np
from pykalman import KalmanFilter
from actionlib_msgs.msg import GoalStatusArray


global task_xy
global qtn_list_xy
global qtn_list_task_xy
global pose_num_xy
global pose_num_task_xy
global yaw
global now_pose_X
global now_pose_Y
global mission_flag





# task_xy = [[0.936528, 1.250201 , 0],[0, 0 , 3.1415926], [-1.7, 0 , 3.1415926],#前三个点
#             [-2.212225, -2.000118, -1.521018],#急救包
#             [-1.858646, -1.757976, 0.2436],[-1.462825, -0.811791,2.549783],
#             [-1.759769, 0.372717 , 1.155107],[-1.747787, 0.584893,2.524426],
#             [-2.224554, 1.015460,-0.304820],[-2.224554, 0.021226, 0.051258],[-0.10, -0.05 , 0.00001]]#回家导航点
# 
task_xy = [[0.956528, 1.250201 , 0], [0.000258, -0.050239 , 3.1415926],[-1.7, 0 , 3.1415926],#前三个点
            [-2.412225, -2.000118, -1.521018],#急救包
            [-1.729975, -1.3593006, 1.569783],[-1.9,-0.25, 0],
            [-1.547787, 1.104893, 2.524426],[-2.024554, 1.255460,-0.304820],
            [-2.224554, 0.051226, 0.001258],[0.0 , -0.05 , 0.0]]#回家导航点

end_point = [2.00 , 0.0 , 0]


global count
count = 0 

qtn_list_xy = []           #四元数列表
qtn_list_task_xy = []      #任务点四元数列表

pose_num_task_xy=len(task_xy)
yaw = 0
global move_base
now_pose_X=0
now_pose_Y=0
mission_flag = 1
error = 0.0
stop_flag = 0


# class StablePointProcessor:
#     def __init__(self, position_threshold=0.01, window_size=10):
#         self.position_threshold = position_threshold  # 稳定性阈值
#         self.window_size = window_size  # 滑动窗口大小
#         self.points = []  # 存储最近的点

#     def process_point(self, x, y, yaw):
#         # 添加新点到列表中
#         self.points.append([x, y, yaw])
        
#         # 确保列表中只保留 window_size 个点
#         if len(self.points) > self.window_size:
#             self.points.pop(0)
        
#         # 去除异常值
#         self.remove_outliers()
        
#         # 检查点是否稳定
#         if self.is_stable():
#             stable_point = np.mean(self.points, axis=0)
#             return stable_point.tolist()
#         else:
#             return None

#     def is_stable(self):
#         if len(self.points) < self.window_size:
#             return False
        
#         points_array = np.array(self.points)[:, :2]  # 只考虑 x 和 y 位置
#         max_variation = np.ptp(points_array, axis=0)  # 计算每个维度的最大范围 (峰值-谷值)
        
#         # 检查 x 和 y 的最大范围是否都在稳定性阈值内
#         return np.all(max_variation < self.position_threshold)

#     def remove_outliers(self):
#         if len(self.points) < self.window_size:
#             return
        
#         points_array = np.array(self.points)[:, :2]  # 只考虑 x 和 y 位置
        
#         # 计算均值和标准差
#         mean = np.mean(points_array, axis=0)
#         std = np.std(points_array, axis=0)
        
#         # 计算每个点到均值的距离
#         distances = np.linalg.norm(points_array - mean, axis=1)
        
#         # 移除距离均值太远的点（异常值），这里用3倍标准差作为阈值
#         filtered_points = np.array(self.points)[distances < 3 * np.mean(distances)]
        
#         self.points = filtered_points.tolist()

# class StablePointProcessor:
#     def __init__(self, position_threshold=0.01, yaw_threshold=5.0, window_size=5):
        """
        初始化 StablePointProcessor。

        Args:
            position_threshold (float): 位置稳定性阈值，用于判断 x 和 y 坐标的稳定性。
            yaw_threshold (float): 姿态稳定性阈值，用于判断 yaw 角的稳定性。
            window_size (int): 滑动窗口的大小，用于存储最近的点。
        """
#         self.position_threshold = position_threshold  # 位置稳定性阈值
#         self.yaw_threshold = yaw_threshold  # 姿态稳定性阈值
#         self.window_size = window_size  # 滑动窗口大小
#         self.points = []  # 存储最近的点

#     def process_point(self, x, y, yaw):
#         # 添加新点到列表中
#         self.points.append([x, y, yaw])
#         print("points",self.points)
#         # 确保列表中只保留 window_size 个点
#         if len(self.points) > self.window_size:
#             self.points.pop(0)
        
#         # 去除异常值
#         self.remove_outliers()
        
#         # 检查点是否稳定
#         if self.is_stable():
#             stable_point = np.mean(self.points, axis=0)
#             return stable_point.tolist()
#         else:
#             return None

#     def is_stable(self):
#         if len(self.points) < self.window_size:
#             return False
        
#         points_array = np.array(self.points)
#         position_points = points_array[:, :2]  # 只考虑 x 和 y 位置
#         yaw_points = points_array[:, 2]  # 只考虑 yaw

#         max_variation_position = np.ptp(position_points, axis=0)  # 计算每个维度的最大范围 (峰值-谷值)
#         max_variation_yaw = np.ptp(yaw_points)  # 计算 yaw 的最大范围 (峰值-谷值)

#         # 检查 x 和 y 的最大范围是否都在稳定性阈值内，以及 yaw 的最大范围是否在阈值内
#         return np.all(max_variation_position < self.position_threshold) and max_variation_yaw < self.yaw_threshold

#     def remove_outliers(self):
#         if len(self.points) < self.window_size:
#             return
        
#         points_array = np.array(self.points)
#         position_points = points_array[:, :2]  # 只考虑 x 和 y 位置
#         yaw_points = points_array[:, 2]  # 只考虑 yaw
        
#         # 计算均值和标准差
#         mean_position = np.mean(position_points, axis=0)
#         std_position = np.std(position_points, axis=0)
        
#         mean_yaw = np.mean(yaw_points)
#         std_yaw = np.std(yaw_points)
        
#         # 计算每个点到均值的距离
#         distances_position = np.linalg.norm(position_points - mean_position, axis=1)
        
#         # 移除距离均值太远的点（异常值），这里用3倍标准差作为阈值
#         filtered_points = points_array[distances_position < 3 * np.mean(distances_position)]
        
#         # 移除 yaw 偏差过大的点（异常值）
#         filtered_points = filtered_points[np.abs(filtered_points[:, 2] - mean_yaw) < 3 * std_yaw]
        
#         self.points = filtered_points.tolist()


class StablePointProcessor:
    """
    一个用于处理和稳定三维点（x, y, yaw）的类，通过滑动窗口和异常值移除来确保点的稳定性。
    """
    def __init__(self, position_threshold=0.01, yaw_threshold=5.0, window_size=5):
        """
        初始化 StablePointProcessor。

        Args:
            position_threshold (float): 位置稳定性阈值，用于判断 x 和 y 坐标的稳定性。
            yaw_threshold (float): 姿态稳定性阈值，用于判断 yaw 角的稳定性。
            window_size (int): 滑动窗口的大小，用于存储最近的点。
        """
        self.position_threshold = position_threshold  # 位置稳定性阈值
        self.yaw_threshold = yaw_threshold  # 姿态稳定性阈值
        self.window_size = window_size  # 滑动窗口大小
        self.points = []  # 存储最近的点

    def process_point(self, x, y, yaw):
        """
        处理输入的点，并判断是否稳定。

        Args:
            x (float): 点的 x 坐标。
            y (float): 点的 y 坐标。
            yaw (float): 点的偏航角。

        Returns:
            list: 如果点稳定，返回稳定的点 [x, y, yaw]；否则返回 None。
        """
        # 检查是否有 NaN 值
        if any(np.isnan([x, y, yaw])):
            return None
        
        # 添加新点到列表中
        self.points.append([x, y, yaw])
        
        # 确保列表中只保留 window_size 个点
        if len(self.points) > self.window_size:
            self.points.pop(0)
        
        # 去除异常值
        self.remove_outliers()
        
        # 检查点是否稳定
        if self.is_stable():
            stable_point = np.mean(self.points, axis=0)
            return stable_point.tolist()
        else:
            return None

    def is_stable(self):
        """
        检查当前存储的点是否稳定。

        Returns:
            bool: 如果点稳定，返回 True；否则返回 False。
        """
        if len(self.points) < self.window_size:
            return False
        
        points_array = np.array(self.points)
        position_points = points_array[:, :2]  # 只考虑 x 和 y 位置
        yaw_points = points_array[:, 2]  # 只考虑 yaw

        max_variation_position = np.ptp(position_points, axis=0)  # 计算每个维度的最大范围 (峰值-谷值)
        max_variation_yaw = np.ptp(yaw_points)  # 计算 yaw 的最大范围 (峰值-谷值)

        # 检查 x 和 y 的最大范围是否都在稳定性阈值内，以及 yaw 的最大范围是否在阈值内
        return np.all(max_variation_position < self.position_threshold) and max_variation_yaw < self.yaw_threshold

    def remove_outliers(self):
        """
        从存储的点列表中移除异常值。
        """
        if len(self.points) < self.window_size:
            return
        
        points_array = np.array(self.points)
        position_points = points_array[:, :2]  # 只考虑 x 和 y 位置
        yaw_points = points_array[:, 2]  # 只考虑 yaw
        
        # 计算均值和标准差
        mean_position = np.mean(position_points, axis=0)
        std_position = np.std(position_points, axis=0)
        
        mean_yaw = np.mean(yaw_points)
        std_yaw = np.std(yaw_points)
        
        # 计算每个点到均值的距离
        distances_position = np.linalg.norm(position_points - mean_position, axis=1)
        
        # 移除距离均值太远的点（异常值），这里用3倍标准差作为阈值
        filtered_points = points_array[distances_position < 3 * np.mean(distances_position)]
        
        # 移除 yaw 偏差过大的点（异常值）
        filtered_points = filtered_points[np.abs(filtered_points[:, 2] - mean_yaw) < 3 * std_yaw]
        
        self.points = filtered_points.tolist()

global processor_2
processor_2 = StablePointProcessor(position_threshold=0.1, yaw_threshold=5.0,window_size=25)



#####################################限制函数####################################

def limt(value, target):
    """
    限制一个值的范围，使其不超过目标值的正负范围。

    Args:
        value (float): 需要限制的值。
        target (float): 目标限制值，用于确定正负范围。

    Returns:
        float: 限制后的值。
    """
    if value > target:
        value = target
    if value < -target:
        value = -target
    return value

##############################################语音播放#################################################

def play_voice(number):
    """
    播放指定数字对应的 MP3 音频文件。

    Args:
        number (int): 音频文件的数字标识（例如，1 对应 1.mp3）。
    """
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

def w_pid_cal(pid_target, dis):
    """
    计算墙体 PID 控制器的输出。

    Args:
        pid_target (float): 目标距离。
        dis (float): 当前距离。

    Returns:
        float: PID 控制器的输出值。
    """
    global w_kp 
    global w_ki
    global w_kd
    global w_e_all
    global w_last_e
    e = dis - pid_target
    #if e>-25 and e<25:
      #e = 0
    w_e_all = w_e_all + e
    pid = w_kp * e + w_ki * w_e_all + w_kd * (e - w_last_e)
    w_last_e = e
    return pid
#######################################姿态参数及其控制函数############################################
global p_kp 
global p_ki
global p_kd
global p_e_all
global p_last_e
global p_pid
p_kp = -2.0
p_ki = 0
p_kd = -0.00
p_e_all = 0
p_last_e = 0
p_pid = 0

def p_pid_cal(pid_target, pose):
    """
    计算姿态 PID 控制器的输出。

    Args:
        pid_target (float): 目标姿态（角度）。
        pose (float): 当前姿态（弧度）。

    Returns:
        float: PID 控制器的输出值。
    """
    global p_kp 
    global p_ki
    global p_kd
    global p_e_all
    global p_last_e
    ture_pose = (pose / 3.14159265359 * 180.0 + 180.0) % 360


    if pid_target == 0:
        if ture_pose > 0 and ture_pose < 180:
            pid_target = 0
        if ture_pose > 180 and ture_pose < 360:
            pid_target = 360   
            
    e = ture_pose - pid_target
    # print(e) 
    p_e_all = p_e_all + e
    pid = p_kp * e + p_ki * p_e_all + p_kd * (e - p_last_e)
    #rospy.loginfo("e %f",e)	
    p_last_e = e
    return pid



global point_kp 
global point_ki
global point_kd
global point_e_all
global point_last_edef
global point_pid
point_kp = -5
point_ki = 0
point_kd = 0
point_e_all = 0
point_last_e = 0
point_pid = 0

def point_pid(pid_target_x, ture):
    """
    计算点位 PID 控制器的输出。

    Args:
        pid_target_x (float): 目标 x 坐标。
        ture (float): 当前 x 坐标。

    Returns:
        float: PID 控制器的输出值。
    """
    global point_kp 
    global point_ki
    global point_kd
    global point_e_all
    global point_last_e
    e = ture - pid_target_x
    point_e_all = point_e_all + e
    pid = point_kp * e + point_ki * point_e_all + point_kd * (e - point_last_e)	
    point_last_e = e
    return pid


# def pid_stop(target_x,target_y,target_yaw):
#     global point_kp ,now_pose_X,now_pose_Y
#     pid_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=7)
#     rate_loop_pid=rospy.Rate(7.2)
#     speed = Twist()
#     while not rospy.is_shutdown():
#         map_pid_x = point_pid(target_x,now_pose_X)
#         map_pid_y = point_pid(target_y,now_pose_Y)
#         p_pid = p_pid_cal(target_yaw,yaw)

#         speed.linear.x = map_pid_x
#         speed.linear.y = map_pid_y
#         speed.angular.z = p_pid/180.0*3.14159265359
#         print("__________________________________________________________________")
#         print(map_pid_x,map_pid_y,p_pid/180.0*3.14159265359)
#         pid_vel_pub.publish(speed)   
        
#         if abs(target_x-now_pose_X) and abs(target_y-now_pose_Y)<=0.1:
#             speed.linear.x = 0
#             speed.linear.y = 0
#             speed.angular.z = 0
#             pid_vel_pub.publish(speed)   
#             break
#         rate_loop_pid.sleep()

def pid_stop2(target_x, target_y, target_yaw):
    """
    使用 PID 控制器使机器人停止在指定的目标位置和姿态。

    Args:
        target_x (float): 目标 x 坐标。
        target_y (float): 目标 y 坐标。
        target_yaw (float): 目标偏航角（角度）。
    """
    global w_kp, w_ki, w_kd, w_e_all
    w_kp = 1.0  # 1.5
    w_ki = 0.001  # 0.005
    w_kd = 0  # 0.01
    w_e_all = 0
    count = 0
    pid_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=7)
    rate_loop_pid = rospy.Rate(7)
    speed = Twist()
    wich_x = 0
    wich_y = 0
    time = 0
    while not rospy.is_shutdown():
        print("#############pid中##################")
        rate_loop_pid.sleep()
        if target_x > 0:
            pid_x = w_pid_cal(target_x, x_f)
            wich_x = x_f
        if target_x < 0:
            pid_x = w_pid_cal(target_x, -x_b)
            wich_x = -x_b
        if target_y > 0:
            pid_y = w_pid_cal(target_y, y_l)
            wich_y = y_l
        if target_y < 0:
            pid_y = w_pid_cal(target_y, -y_r)
            wich_y = -y_r
        p_pid = p_pid_cal(target_yaw, yaw)
        speed.linear.y = pid_y
        speed.linear.x = pid_x
        speed.angular.z = p_pid / 180.0 * 3.14159265359
        w_e_all = limt(w_e_all, 5)
        if abs(wich_x - target_x) <= 0.15 and abs(wich_y - target_y) <= 0.15 and abs(target_yaw - (yaw / 3.1415926 * 180 + 180)) <= 10:
            speed.linear.x = 0
            speed.linear.y = 0
            speed.linear.z = 0
            pid_vel_pub.publish(speed) 
            #rospy.sleep(0.5)
            w_e_all = 0
            break
        pid_vel_pub.publish(speed) 

###################################################去识别恐怖分子的动作函数(包括去、掉头、回动作)#########################################
def pid_go(target_x, target_y, target_yaw):
    """
    使用 PID 控制器使机器人向指定目标位置移动。

    Args:
        target_x (float): 目标 x 坐标。
        target_y (float): 目标 y 坐标。
        target_yaw (float): 目标偏航角（角度）。
    """
    global w_kp, w_ki, w_kd, w_e_all, count, y_l
    print("--------开始pid_go--------")
    w_kp = 0.7  # 2
    w_ki = 0
    w_kd = 0.01
    w_e_all = 0
    pid_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=7)
    rate_loop_pid = rospy.Rate(7)
    speed = Twist()
    wich_x = 0
    wich_y = 0
    key = 0
    while not rospy.is_shutdown():
        print("x_f", x_f)
        print("y_l", y_l)
        
        if target_x > 0:
            pid_x = w_pid_cal(target_x, x_f)
            wich_x = x_f
        if target_x < 0:
            pid_x = w_pid_cal(target_x, -x_b)
            wich_x = -x_b
        if target_y > 0:
            pid_y = w_pid_cal(target_y, y_l)
            wich_y = y_l
        if target_y < 0:
            pid_y = w_pid_cal(target_y, -y_r)
            wich_y = -y_r
        p_pid = p_pid_cal(target_yaw, yaw)
        
        #if target_x == 0.7:
            #speed.linear.y = pid_y
        if abs(target_x - wich_x) < 0.1 and abs(target_y - wich_y) < 0.1:
            w_e_all = 0
            speed.linear.x = 0
            speed.linear.y = 0
            break  #################################################################这个后面条件是task这里是debug
        else:
            if abs(wich_x) > 0.25 and key == 0:
                speed.linear.y = 0
            #     #speed.linear.y = 0.05*pid_y
            else:
                speed.linear.y = pid_y
            if abs(wich_y) < 0.5:
                key = 1
        if abs(wich_y) < 0.2:
                key = 2
                #key = 1

        # if wich_y<0.9 and key == 1:
        #     print("@@@@@@@@@@@@@@@@@@@@@@@@@")
        #     target_x = 0.7
        if key == 0 or key == 2:
            speed.linear.x = pid_x
        speed.angular.z = p_pid / 180.0 * 3.14159265359

        pid_vel_pub.publish(speed)       
        rate_loop_pid.sleep()
    
        #这个后面移植标定
        # if task != 0:
        #     w_e_all=0
        #     # thread_dis.join()
        #     break
    print("--------结束pid_go--------")


def pid_back(target_x, target_y, target_yaw):
    """
    使用 PID 控制器使机器人向指定目标位置后退。

    Args:
        target_x (float): 目标 x 坐标。
        target_y (float): 目标 y 坐标。
        target_yaw (float): 目标偏航角（角度）。
    """
    global w_kp, w_ki, w_kd, w_e_all, y_l
    print("--------开始pid_back--------")
    w_kp = 0.7  # 2
    w_ki = 0
    w_kd = 0.005  # 0
    count = 0
    w_e_all = 0
    pid_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=7)
    rate_loop_pid = rospy.Rate(10)
    speed = Twist()
    wich_x = 0
    wich_y = 0
    while not rospy.is_shutdown():
        if target_x > 0:
            pid_x = w_pid_cal(target_x, x_f)
            wich_x = x_f
        if target_x < 0:
            pid_x = w_pid_cal(target_x, -x_b)
            wich_x = -x_b
        if target_y > 0:
            pid_y = w_pid_cal(target_y, y_l)
            wich_y = y_l
        if target_y < 0:
            pid_y = w_pid_cal(target_y, -y_r)
            wich_y = -y_r
        p_pid = p_pid_cal(target_yaw, yaw)
        
        if abs(target_y - wich_y) < 0.2:
            print("pid_back结束")
            speed.linear.x = 0
            speed.linear.y = 0
            break  #################################################################这个后面条件是task这里是debug
        else:
            if abs(wich_x) > 0.25:
                speed.linear.y = 0.1 * pid_y
            else:
                speed.linear.y = pid_y
            if abs(wich_y) > 1.4:
                speed.linear.x = 0
            else:
                speed.linear.x = pid_x
            speed.angular.z = p_pid / 180.0 * 3.14159265359

        pid_vel_pub.publish(speed)       
        rate_loop_pid.sleep()
    
        # if task != 0:
        #     w_e_all=0
        #     # thread_dis.join()
        #     break

    print("--------结束pid_back--------")

def pid_turn(target_x, target_y, target_yaw):
    """
    使用 PID 控制器使机器人原地转向指定角度。

    Args:
        target_x (float): 目标 x 坐标（当前未使用）。
        target_y (float): 目标 y 坐标（当前未使用）。
        target_yaw (float): 目标偏航角（角度）。
    """
    global point_kp, p_kp
    p_kp = -1.5
    count = 0
    pid_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=7)
    rate_loop_pid = rospy.Rate(10)
    speed = Twist()
    while not rospy.is_shutdown():
        p_pid = p_pid_cal(target_yaw, yaw)
        speed.linear.x = 0
        speed.linear.y = 0
        speed.angular.z = p_pid / 180.0 * 3.14159265359
        pid_vel_pub.publish(speed)   
        rate_loop_pid.sleep()
        # print("EEEEEEEEEEE:",abs(target_yaw-(yaw/3.1415926*180+180)))
        # if abs(target_yaw-(yaw/3.1415926*180+180)%360)<5 or abs(target_yaw-(yaw/3.1415926*180+180)%360)>355:
        #     count+=1
        #     print("************************count",count)

        if abs(target_yaw - (yaw / 3.1415926 * 180 + 180) % 360) < 5 or abs(target_yaw - (yaw / 3.1415926 * 180 + 180) % 360) > 355:
            count += 1
            print("************************count", count)
        if count >= 10:
            print("QQQQQQQQQQQQQQQQQQQQQQQQQ")
            break






def pid_turn_line(target_x, target_y, target_yaw):
    """
    使用 PID 控制器使机器人进行带直线运动的转向。

    Args:
        target_x (float): 目标 x 坐标（当前未使用）。
        target_y (float): 目标 y 坐标（当前未使用）。
        target_yaw (float): 目标偏航角（角度）。
    """
    global point_kp, p_kp, yaw
    p_kp = -1.5
    count = 0
    pid_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=7)
    rate_loop_pid = rospy.Rate(10)
    speed = Twist()
    speed.linear.y = 0.3
    pid_vel_pub.publish(speed)   
    rospy.sleep(0.5)
    speed.linear.y = 0
    pid_vel_pub.publish(speed)
    rospy.sleep(0.5)  
    while not rospy.is_shutdown():
        p_pid = p_pid_cal(target_yaw, yaw)
        speed.linear.x = 0
        speed.linear.y = 0
        speed.angular.z = p_pid / 180.0 * 3.14159265359
        pid_vel_pub.publish(speed)   
        rate_loop_pid.sleep()
        # print("EEEEEEEEEEE:",abs(target_yaw-(yaw/3.1415926*180+180)))
        # if abs(target_yaw-(yaw/3.1415926*180+180)%360)<5 or abs(target_yaw-(yaw/3.1415926*180+180)%360)>355:
        #     count+=1
        #     print("************************count",count)
        print("yaw", yaw)
        if abs(target_yaw - (yaw / 3.1415926 * 180 + 180) % 360) < 5 or abs(target_yaw - (yaw / 3.1415926 * 180 + 180) % 360) > 355:
            count += 1
            print("************************count", count)
        if count >= 10:
            print("QQQQQQQQQQQQQQQQQQQQQQQQQ")
            break

            

def pid_turn_find(u):
    """
    使用 PID 控制器使机器人转向以找到目标。

    Args:
        u (float): 目标的中心点。
    """
    global cx
    pid_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=7)
    rate_loop_pid = rospy.Rate(10)
    speed = Twist()
    while not rospy.is_shutdown():
        speed.angular.z = 0.003 * (u - cx) 
        if abs(u - cx) <= 50:
            speed.angular.z = 0
            pid_vel_pub.publish(speed)   
            break
        pid_vel_pub.publish(speed)
        rate_loop_pid.sleep()   

global ture_angle_sure
ture_angle_sure = 0

global Flag_find_bit
Flag_find_bit = 0
global Flag_find_sure
Flag_find_sure = 0
global Flag_turn_bit
Flag_turn_bit = 0
global target_point_find
target_point_find = 0
def Flag_find():
    global vision_data,Flag_find_bit,Flag_turn_bit,target_point_find,Flag_find_sure,ture_angle_sure,speed_data,result,ture_angle_rad
    rate_loop=rospy.Rate(10)
    while not rospy.is_shutdown():
        if result is not None:
            print("result",result)

            q = tf.transformations.quaternion_from_euler(0,0,yaw-ture_angle_rad)
            print("target_point_find",result[0],result[1])
            # target_point_find = Pose(Point(optimal_point[0]+1*math.cos(ture_angle_sure+3.14) ,optimal_point[1]-1*math.sin(ture_angle_sure+3.14) , 0), Quaternion(q[0], q[1], q[2], q[3]))
            # target_point_find = Pose(Point(result[0]-0.2,result[1]-0.4, 0), Quaternion(q[0], q[1], q[2], q[3]))
            # target_point_find = Pose(Point(result[0]+0.6*math.sin(ture_angle_rad),result[1]-0.6*math.cos(ture_angle_rad), 0), Quaternion(q[0], q[1], q[2], q[3]))
            print("ture_angle_rad",ture_angle_rad)
            print("yaw",yaw)
            target_point_find = Pose(Point(result[0]-0.3*math.cos(-ture_angle_rad+yaw),result[1]-0.3*math.sin(-ture_angle_rad+yaw), 0), Quaternion(q[0], q[1], q[2], q[3]))
            Flag_find_sure = 1
            print("迁移量",0.4*math.cos(ture_angle_rad+yaw),0.4*math.sin(ture_angle_rad+yaw))
            print("迁移度",-ture_angle_rad+yaw)

            #print("target_point_find",optimal_point[0],optimal_point[1])
            break
        rate_loop.sleep()


###########################################################导航子函数##########################################################
# 控制导航点发布
        


global status
status = 0

def move_base_state_get():
    status_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, status_callback)
    rospy.spin()

def status_callback(msg):
    global status
    if msg.status_list:
        latest_status = msg.status_list[-1].status
        if latest_status == 3:
            status = 2
        if latest_status == 4:  # ABORTED
            status = 1
        else:
            status = 0






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
vision_i = 0
# 创建一个双向队列，用于存储历史数据
data_window = deque(maxlen=10)
average_x = 0.0
average_z = 0.0

# 设置指数加权移动平均滤波器的权重参数
alpha = 0.2  

def vision_callback(msg):
    global u, v, vision_data, sign,task
    global target, error, mid_angle, stop_flag,vision_i


    if sign == 0:
        sign=1
    #判断数据包格式是否正确，进行数据记录
    if len(msg.data) >= 13:
        data_list = list(msg.data)
        if task == 0:
            if data_list[0]+data_list[1]+data_list[2] != 0:
                vision_i += 1
                for i in range(3):
                    if data_list[i] != 0.0 and vision_i == 3:
                        task = data_list[i]

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

global stop_sign
stop_sign = 0

def dis_func(x,y,min_dis):
    global dis_trun_off,now_pose_X,now_pose_Y,distance,count_dis_times ,vision_data,stop_sign,status,Z
    pid_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=7)
    #print("dis_func函数已启动:"+str(count_dis_times)+"次")
    # lock = threading.RLock()
    dis_fun_rate=rospy.Rate(10)
    pid_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=7)
    speed = Twist()
    goal = MoveBaseGoal()
    tasl_see = 0
    t = time.time()
    while not rospy.is_shutdown():
        dis_fun_rate.sleep()
        car_to_map_x=now_pose_X
        car_to_map_y=now_pose_Y
        # 计算小车到目标点的距离
        distance = pow(pow( car_to_map_x - x, 2) + pow(car_to_map_y - y, 2), 0.5)

        if vision_data == task and Z <= 2.5:
            tasl_see+=1
            print("看到目标================================",tasl_see)
        if min_dis != 0.09: # 0.09
            if tasl_see >= 10 :
                move_base.send_goal(goal)
                speed.linear.x = 0
                speed.linear.y = 0
                pid_vel_pub.publish(speed)
                print("+++++++++++++++++++++++++++++++++++++++++++++++一眼丁真")
                stop_sign = 1
                #rospy.sleep(5)
                break
        if status == 1:
                print("这个点是无效点====================================")
                move_base.send_goal(goal)
                speed.linear.x = 0
                speed.linear.y = 0
                pid_vel_pub.publish(speed)
                break

        if distance < min_dis :
            goal = MoveBaseGoal()
            move_base.send_goal(goal)
            pid_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=7)
            speed = Twist()
            speed.linear.x = 0
            speed.linear.y = 0
            pid_vel_pub.publish(speed)

            rospy.sleep(0.5)
            break
        if time.time() - t > 15:
            print("@@@@@@@@@@@@@@@@@@@@@@@超时@@@@@@@@@@@@@@@@@@@@@@@@@")
            break

def turn_find(angle_speed):
    global stop_sign,ture_angle_rad,u
    tasl_see=0
    pid_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=7)
    speed = Twist()
    speed.angular.z = angle_speed
    t = 0
    a = 0
    t_pid = 0
    b = 0
    U = 0
    rate_loop=rospy.Rate(10)
    while not rospy.is_shutdown():
        pid_vel_pub.publish(speed)
        if vision_data == task:
            tasl_see+=1
            print("看到目标================================",tasl_see)
        if tasl_see >= 2:
            speed.angular.z = 0.0
            pid_vel_pub.publish(speed)
            rospy.sleep(0.02)
            if vision_data == task and Z<2.0:
                print("+++++++++++++++++++++++++++++++++++++++++++++++一眼丁真")
                while not rospy.is_shutdown():
                    speed.angular.z = -0.002*(u - cx)
                    print("^^^^^^^^^^^^^^^^^^^^^^^u-cx:",u-cx)
                    if t_pid > 50:
                        print("##################转移中心点超时##################")
                        b = 1
                        break
                    t_pid+=1
                    if abs(u - cx) <= 50:
                        speed.angular.z = 0
                        pid_vel_pub.publish(speed)
                        break
                    
                    pid_vel_pub.publish(speed)
                    rate_loop.sleep()  
                if not b:
                    stop_sign = 1
                else:
                    stop_sign = 0
                #rospy.sleep(4)
                while not Flag_find_sure:
                    if b:
                        print("##################转移中心点超时##################") 
                        break
                    rospy.sleep(0.02)
                    print("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&等待标定&&&&&&&&&&&&&&&&&&&&&&&")
                    a = 1
            if a:
                break
        t+=1
        if t >= abs(2*np.pi/angle_speed*10):
            print("*******************************自转超时******************************")
            break

        rate_loop.sleep()

def turn_find_wait(angle_speed):
    global stop_sign,ture_angle_rad,u
    tasl_see=0
    pid_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=7)
    speed = Twist()
    speed.angular.z = angle_speed
    t = 0
    a = 0
    U = 0
    rate_loop=rospy.Rate(10)
    while not rospy.is_shutdown():
        pid_vel_pub.publish(speed)
        if vision_data == task and Z<= 2.0:
            tasl_see+=1
            print("看到目标================================",tasl_see)
        if tasl_see >= 5:
            speed.angular.z = 0.0
            pid_vel_pub.publish(speed)
            print("+++++++++++++++++++++++++++++++++++++++++++++++一眼丁真")
            rospy.sleep(0.1)
            if vision_data == task:
                U = u
                while not rospy.is_shutdown():
                    speed.angular.z = -0.002*(u - cx) 
                    if abs(u - cx) <= 50:
                        speed.angular.z = 0
                        pid_vel_pub.publish(speed)   
                        break
                    pid_vel_pub.publish(speed)
                    rate_loop.sleep()  
                stop_sign = 1
                #rospy.sleep(4)
                while not Flag_find_sure:
                    rospy.sleep(0.02)
                    print("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&等待标定&&&&&&&&&&&&&&&&&&&&&&&")
                    a = 1
            if a:
                break
        t+=1
        if t >= abs(1.5*np.pi/angle_speed*10):
            print("*******************************自转超时******************************")
            break

        rate_loop.sleep()



def goals(x, y, i):
    print("******************************************发布导航点*********************************************")
    global qtn_list_task_xy,move_base,now_pose_X,Flag_turn_bit,optimal_point,stop_sign
    stop_sign = 0
    # if Flag_turn_bit >= 10:
    #     if optimal_point[2]!= None:
    #         q = tf.transformations.quaternion_from_euler(0,0,optimal_point[2])
    #         target_point = Pose(Point(x, y, 0), Quaternion(q[0], q[1], q[2], q[3]))
    #     else:
    #         target_point = Pose(Point(x, y, 0), Quaternion(qtn_list_task_xy[i][0], qtn_list_task_xy[i][1], qtn_list_task_xy[i][2], qtn_list_task_xy[i][3]))
    # else :
    target_point = Pose(Point(x, y, 0), Quaternion(qtn_list_task_xy[i][0], qtn_list_task_xy[i][1], qtn_list_task_xy[i][2], qtn_list_task_xy[i][3]))
    goal = MoveBaseGoal()
    goal.target_pose.pose = target_point
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    rospy.loginfo("Going to: " + str(target_point))
    print("goal")
    move_base.send_goal(goal)


def goals_end(x, y):
    print("******************************************发布导航点*********************************************")
    global qtn_list_task_xy,move_base,now_pose_X,Flag_turn_bit,optimal_point,stop_sign
    stop_sign = 0
    # if Flag_turn_bit >= 10:
    #     if optimal_point[2]!= None:
    #         q = tf.transformations.quaternion_from_euler(0,0,optimal_point[2])
    #         target_point = Pose(Point(x, y, 0), Quaternion(q[0], q[1], q[2], q[3]))
    #     else:
    #         target_point = Pose(Point(x, y, 0), Quaternion(qtn_list_task_xy[i][0], qtn_list_task_xy[i][1], qtn_list_task_xy[i][2], qtn_list_task_xy[i][3]))
    # else :
    target_point = Pose(Point(x, y, 0), Quaternion(0, 0, 0, 1))
    goal = MoveBaseGoal()
    goal.target_pose.pose = target_point
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    rospy.loginfo("Going to: " + str(target_point))
    print("goal")
    move_base.send_goal(goal)




global target_in_map
target_in_map = PoseStamped()
global result
result = None 


def transform_to_map():
    global target_in_map,vision_data,yaw,atan_x,goal_sign,ture_angle_sure,X,Z,average_x,average_z,speed_data,processor_2,result,stop_sign
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
            
            #print("===================这个进程======================")
            if  vision_data!=0 and goal_sign == 1 and speed_data.angular.z == 0 and stop_sign:
                #rospy.sleep(1.5)
                target_in_map = do_transform_pose(target_in_camera, camera_to_map)
                result = processor_2.process_point(target_in_map.pose.position.x, target_in_map.pose.position.y, yaw+ture_angle_rad)

                print("target_in_map",target_in_map.pose.position.x,target_in_map.pose.position.y)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Failed to get transform")
            rospy.sleep(0.1)
            continue
        rate_loop.sleep()

#------------------- 雷达话题订阅线程 ---------------- 
global scan_data
scan_data=[]
def get_laserscan(scan):	
    global x_f,x_b,y_l,y_r,yaw,scan_data
    scan_data=scan.ranges

    if scan.ranges[453]!=float('inf') or scan.ranges[454]!=float('inf') or scan.ranges[455]!=float('inf'): 
        total=0.0
        f_count=0.0
        if scan.ranges[452]!=float('inf') and scan.ranges[452]!=0.0:
            total+=scan.ranges[452]
            f_count+=1
        if scan.ranges[453]!=float('inf') and scan.ranges[453]!=0.0:
            total+=scan.ranges[453]
            f_count+=1
        if scan.ranges[454]!=float('inf') and scan.ranges[454]!=0.0:
            total+=scan.ranges[454]
            f_count+=1   
        if f_count:  
            x_f = total/f_count
      

    if scan.ranges[0]!=float('inf') or scan.ranges[1]!=float('inf') or scan.ranges[908]!=float('inf'): 
        total=0.0
        f_count=0.0
        if scan.ranges[0]!=float('inf') and scan.ranges[0]!=0.0:
            total+=scan.ranges[0]
            f_count+=1
        if scan.ranges[1]!=float('inf') and scan.ranges[1]!=0.0:
            total+=scan.ranges[1]
            f_count+=1
        if scan.ranges[908]!=float('inf') and scan.ranges[908]!=0.0:
            total+=scan.ranges[908]
            f_count+=1    
        if f_count:  
            x_b = total/f_count


    if scan.ranges[680]!=float('inf') or scan.ranges[681]!=float('inf') or scan.ranges[682]!=float('inf') : 
        total=0.0
        f_count=0.0
        if scan.ranges[680]!=float('inf') and scan.ranges[680]!=0.0:
            total+=scan.ranges[680]
            f_count+=1
        if scan.ranges[681]!=float('inf') and scan.ranges[681]!=0.0:
            total+=scan.ranges[681]
            f_count+=1
        if scan.ranges[682]!=float('inf') and scan.ranges[682]!=0.0:
            total+=scan.ranges[682]
            f_count+=1   
        if f_count:
            # print("f_count",f_count)
            # print("lida",scan.ranges[680],scan.ranges[681],scan.ranges[682])
            # print("total",total)  
            y_l = total/f_count 

    if scan.ranges[227]!=float('inf') or scan.ranges[228]!=float('inf') or scan.ranges[229]!=float('inf'): 
        total=0.0
        f_count=0.0
        if scan.ranges[227]!=float('inf') and scan.ranges[227]!=0.0:
            total+=scan.ranges[227]
            f_count+=1
        if scan.ranges[228]!=float('inf') and scan.ranges[228]!=0.0:
            total+=scan.ranges[228]
            f_count+=1
        if scan.ranges[229]!=float('inf') and scan.ranges[229]!=0.0:
            total+=scan.ranges[229]
            f_count+=1           
        if f_count:            
            y_r = total/f_count 


def laser_listen():
    rospy.Subscriber('/scan', LaserScan,get_laserscan,queue_size=7)
    rospy.spin()




def calculate_pose_angle(laser1_length, laser2_length, angle_between_lasers):
    # 将夹角从度数转换为弧度
    angle_between_lasers_rad = math.radians(angle_between_lasers)
    
    # 使用余弦定理计算对边的长度
    c = math.sqrt(laser1_length**2 + laser2_length**2 - 2 * laser1_length * laser2_length * math.cos(angle_between_lasers_rad))
    
    # 计算sin值并确保在[-1, 1]范围内
    sin_value_laser1 = laser1_length * math.sin(angle_between_lasers_rad) / c
    sin_value_laser2 = laser2_length * math.sin(angle_between_lasers_rad) / c
    
    sin_value_laser1 = max(min(sin_value_laser1, 1), -1)
    sin_value_laser2 = max(min(sin_value_laser2, 1), -1)
    
    # 使用正弦定理计算每条激光对应的角度
    angle_laser1_rad = math.asin(sin_value_laser1)
    angle_laser2_rad = math.asin(sin_value_laser2)
    
    # 将角度从弧度转换为度数
    angle_laser1_deg = math.degrees(angle_laser1_rad)
    angle_laser2_deg = math.degrees(angle_laser2_rad)
    
    # 计算板子相对于雷达的姿态角
    pose_angle = 90 - (angle_laser1_deg + angle_laser2_deg) / 2
    
    return pose_angle



global atan_x
atan_x = 0

def is_valid_quaternion(quaternion):
    return not any(math.isnan(x) for x in quaternion)


global ture_angle_rad
ture_angle_rad = 0


def calibration_tf():
    global average_x, average_z,sign, vision_data,task,scan_data,atan_x,ture_angle,ture_angle_sure,X,Z,ture_angle_rad
    global u,v
    depth = 0
    ture_angle = 0
    rate_loop=rospy.Rate(5)
    print(")))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))00")
    #broadcaster = tf2_ros.TransformBroadcaster()  # 创建广播器
    broadcaster = tf2_ros.TransformBroadcaster()
    while not rospy.is_shutdown():
        if u != 0:
            tanx = (u - cx)/fx
            atan_x = math.atan(tanx)
            angle_x = math.degrees(atan_x)
            #print("angle_x",angle_x)
            if u and v != 0.0:
                    depth = scan_data[440-int(angle_x/0.396)]
                    depth_2 = scan_data[440-int(angle_x/0.396)+1]
                    depth_3 = scan_data[440-int(angle_x/0.396)-1]
                    #ture_angle = calculate_pose_angle(depth,depth_2,0.3960396)
                    if depth >= 5:
                        vision_data = 0
                    
            else:
                depth = 0.0

            # if depth == float('inf'):
            #     return 0
                    
            X = (u - cx) * depth / fx
            Z = depth

            try:
                ture_angle = calculate_pose_angle(depth_2, depth_3,1.19)
                print("Calculated ture_angle:", ture_angle)
                if depth_2 < depth_3:
                    ture_angle = -ture_angle
                # 将 ture_angle 转换为弧度
                ture_angle_rad = ture_angle / 180.0 * np.pi

                # 计算四元数
                q = tf.transformations.quaternion_from_euler(0, 0, ture_angle_rad)
                #print("Calculated quaternion:", q)
            except Exception as e:
                print("An error occurred:",e)
                continue


            #print("ture_angle",ture_angle)
            #print("depth:",depth)
            #如果msg.data[x]不为x.0，则清空TF坐标
            if (vision_data != task):
                sign = 0


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

            #4-2.创建 广播的数据(通过 pose 设置)
            tfs = TransformStamped()

            tfs.header.frame_id = "camera_frame"
            tfs.header.stamp = rospy.Time.now()
            tfs.child_frame_id = "target_frame"

            tfs.transform.translation.x = Z
            tfs.transform.translation.y = -X
            tfs.transform.translation.z = 0

            qtn = tf.transformations.quaternion_from_euler(0,0,0)
            if is_valid_quaternion(q):
                tfs.transform.rotation.x = q[0]
                tfs.transform.rotation.y = q[1]
                tfs.transform.rotation.z = q[2]
                tfs.transform.rotation.w = q[3]
                broadcaster.sendTransform(tfs)
            else:
                rospy.logwarn("Invalid quaternion:")

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
# def pid_line():
#         print("---------启动回家--------")
#         global yaw
#         global w_kp,w_ki,w_kd,w_e_all
#         global error , mid_angle , stop_flag
#         w_e_all=0
#         w_kp = 0.006#0.012 linearx=0.5
#         w_ki = 0
#         w_kd = 0.006 #0.02
#         pid_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=7)
#         rate_loop_pid=rospy.Rate(10)
#         speed = Twist()
#         count = 0
#         while not rospy.is_shutdown():
#             z_pid=w_pid_cal(0,error)
#             z_pid=limt(z_pid,2)
#             print(error)
#             speed.linear.x = 0.2 - abs(error)*0.001
#             speed.angular.z = z_pid
#             if stop_flag == 1:
#                 print('####################################################################')
#                 speed.linear.x=0
#                 speed.angular.y=0
#                 pid_vel_pub.publish(speed) 
#                 count+=1
#             else:    
#                 pid_vel_pub.publish(speed)  
#                 count = 0
#             rate_loop_pid.sleep()
#             if count>=5:
#                 break
#         print("---------结束回家--------")


def pid_line():
        print("---------启动回家--------")
        global yaw
        global w_kp,w_ki,w_kd,w_e_all
        global error , mid_angle , stop_flag
        turn_sign = 0
        w_e_all=0
        w_kp = 0.006#0.012 linearx=0.5
        w_ki = 0
        w_kd = 0.006 #0.02
        pid_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=7)
        rate_loop_pid=rospy.Rate(10)
        speed = Twist()
        count = 0
        while not rospy.is_shutdown():
            z_pid=w_pid_cal(0,error)
            z_pid=limt(z_pid,2)
            #print(error)
            print("-------------------------------------turn_sign",turn_sign)
            speed.linear.x = 0.2 - abs(error)*0.001
            speed.angular.z = z_pid
            if stop_flag == 1:
                print('####################################################################')
                # speed.linear.x=0
                # speed.angular.y=0
                # pid_vel_pub.publish(speed) 
                count+=1
            else:    
                pid_vel_pub.publish(speed)  
                count = 0
            rate_loop_pid.sleep()
            if count>=5:
                turn_sign+=1
            if turn_sign == 1:
                print("============================================================")
                pid_turn_line(0,0,180)  #0和360连续在车的正后方 90为车的正右边 270为车的正左方，然后正前方为180
                rospy.sleep(2)
                stop_flag = 0
                turn_sign = 2
                count = 0
            if turn_sign == 3:
                pid_turn_line(0,0,270)  #0和360连续在车的正后方 90为车的正右边 270为车的正左方，然后正前方为180
                rospy.sleep(2)
                stop_flag = 0
                turn_sign = 4
                count = 0
            if turn_sign == 5:
                speed.linear.x=0
                speed.angular.y=0
                pid_vel_pub.publish(speed) 
                break
        print("---------结束回家--------")







###########################################task函数############################################


# def task1_find_terrorist():
#     global task
#     #去识别框识别恐怖分子
#     pid_go(0.15,0.30,180)
#     if task !=0:
#         play_voice(task)
#     pid_turn(0,0,0)
#     pid_back(0.15,-1.8,0)


# 改变过桥后的参数
# def change(inflation_radius,cost_scaling_factor):
#     rospy.set_param("/move_base/local_costmap/inflation_layer/inflation_radius",inflation_radius)
#     rospy.set_param("/move_base/local_costmap/inflation_layer/cost_scaling_factor",cost_scaling_factor)
#     rospy.set_param("/move_base/global_costmap/inflation_layer/inflation_radius",inflation_radius)
#     rospy.set_param("/move_base/global_costmap/inflation_layer/cost_scaling_factor",cost_scaling_factor)

def check_goal_status():
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    while not rospy.is_shutdown():
        state = client.get_state()
        print("state",state)
        if state == GoalStatus.SUCCEEDED:
            break
        elif state in [GoalStatus.ABORTED, GoalStatus.REJECTED]:
            return False



def task1_find_terrorist():
    global task,status
    #去识别框识别恐怖分子
    #pid_go(0.15,0.30,180)
    goals(task_xy[0][0],task_xy[0][1],0)
    while not task:
        print("识别恐怖分子")
    rospy.sleep(0.5)
    play_voice(task)
    goals(task_xy[1][0],task_xy[1][1],1)

    dis_func(task_xy[1][0],task_xy[1][1],0.05)
    rospy.sleep(0.5)
    goals(task_xy[2][0],task_xy[2][1],2)
    dis_func(task_xy[2][0],task_xy[2][1],0.09)

    # rospy.set_param("/move_base/local_costmap/inflation_layer/inflation_radius",0.25)
    # rospy.set_param("/move_base/local_costmap/inflation_layer/cost_scaling_factor",3)
    # rospy.set_param("/move_base/global_costmap/inflation_layer/inflation_radius",0.25)
    # rospy.set_param("/move_base/global_costmap/inflation_layer/cost_scaling_factor",3)
    print("___________________task1结束_____________________")
    #pid_turn(0,0,0)
    #pid_back(0.15,-1.8,0)



global goal_sign
goal_sign = 0

def task2_find_tool():
    global vision_data,now_pose_X,now_pose_Y,yaw,Flag_find_sure,Flag_turn_bit,target_point_find,goal_sign,stop_sign,status
    # goals(task_xy[3][0],task_xy[3][1],3)
    # dis_func(task_xy[3][0],task_xy[3][1],0.1)
    goal_sign = 1
    turn_find(-0.50)
    print("________________________________________task2________________________________________")
    for i in range(3, len(task_xy)-2):
        print("Flag_find_sure",Flag_find_sure)
        if Flag_find_sure :
            print("status::::::::::::::::::::::::::::::::::",status)
            print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@",vision_data)
            Flag_find_sure = 0

            goal = MoveBaseGoal()
            goal.target_pose.pose = target_point_find
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()
            rospy.loginfo("Going to: " + str(target_point_find))
            move_base.send_goal(goal)
    
            dis_func(target_point_find.position.x,target_point_find.position.y,0.09)
            print("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
            play_voice(task+3)
            rospy.sleep(1)
            goal2 = MoveBaseGoal()
            move_base.send_goal(goal2)
            if i==3:
                goals(task_xy[i][0],task_xy[i][1],i)
                dis_func(task_xy[i][0],task_xy[i][1],0.15)
                rospy.sleep(0.5)
                pid_stop2(0.2,-0.2,90)
                play_voice(7)
                turn_find_wait(0.6)
            break

        print("################################")
        #if stop_sign!=1:
        print("这是第几个点",i)
        goals(task_xy[i][0],task_xy[i][1],i)
        dis_func(task_xy[i][0],task_xy[i][1],0.2)
        if i>3:
            turn_find(0.50)
            rospy.sleep(0.5)
        if i==3:
            rospy.sleep(0.5)
            pid_stop2(0.2,-0.2,90)
            play_voice(7)
            turn_find_wait(0.6)
        print("***********************************发下个导航点************************")
            # else :
            #     print("看到了sleep3秒")
            #     rospy.sleep(1)

    for i in range(len(task_xy) - 2, len(task_xy)):
        # rospy.set_param("/move_base/local_costmap/inflation_layer/inflation_radius",0.1)
        # rospy.set_param("/move_base/local_costmap/inflation_layer/cost_scaling_factor",100)
        # rospy.set_param("/move_base/global_costmap/inflation_layer/inflation_radius",0.1)
        # rospy.set_param("/move_base/global_costmap/inflation_layer/cost_scaling_factor",100)
        print(f"发布最后几个点: {i}")
        goals(task_xy[i][0], task_xy[i][1], i)
        dis_func(task_xy[i][0], task_xy[i][1], 0.09)
        rospy.sleep(0.5)
    goal2 = MoveBaseGoal()
    move_base.send_goal(goal2)
    goals(task_xy[9][0], task_xy[9][1],9)
    dis_func(task_xy[9][0], task_xy[9][1], 0.09)
    goal2 = MoveBaseGoal()
    move_base.send_goal(goal2)
    #pid_turn(0,0,90)

    # rospy.set_param("/move_base/local_costmap/inflation_layer/inflation_radius",0.24)
    # rospy.set_param("/move_base/local_costmap/inflation_layer/cost_scaling_factor",10)
    # rospy.set_param("/move_base/global_costmap/inflation_layer/inflation_radius",0.24)
    # rospy.set_param("/move_base/global_costmap/inflation_layer/cost_scaling_factor",10)
    # goals(task_xy[len(task_xy)-3][0],task_xy[len(task_xy)-3][1],len(task_xy)-3)
    # dis_func(task_xy[len(task_xy)-3][0],task_xy[len(task_xy)-3][1],0.09)
    # goals(task_xy[len(task_xy)-2][0],task_xy[len(task_xy)-2][1],len(task_xy)-2)
    # dis_func(task_xy[len(task_xy)-2][0],task_xy[len(task_xy)-2][1],0.09)
    # goals(task_xy[len(task_xy)-1][0],task_xy[len(task_xy)-1][1],len(task_xy)-1)
    # dis_func(task_xy[len(task_xy)-1][0],task_xy[len(task_xy)-1][1],0.09)


    print("_________________________________task2结束_____________________________________")


#-------------------------------------------------

def task3_find_line():
    pid_turn(0,0,90)
    find_line = rospy.Publisher("mission_msg",Int32,queue_size=10)
    mission_msg = Int32()
    mission_msg.data = 1
    for i in range(20):
        find_line.publish(mission_msg)
        rospy.sleep(0.1)
    rospy.sleep(1)
    pid_line()

def task3_slam():
    goals_end(2.0,0)
    dis_func(2.0,0.0, 0.05)


def speed_get():
    rospy.Subscriber('/cmd_vel',Twist,cmd_vel_callback,queue_size=10) #订阅视觉话题
    rospy.spin()


global speed_data
speed_data = Twist()

def cmd_vel_callback(speed):
    speed_data = speed




def task4_slam():
    global task
    #去识别框识别恐怖分子
    pid_go(0.15,-0.30,180)
    #pid_turn(0,0,0)
    #pid_back(0.15,-1.8,0)


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
    rospy.Subscriber('/answer',String,speech_going,queue_size=10)
    # status_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, status_callback)

    rospy.spin()

def speech_going(msg3):
    global voice_flag
    # while not rospy.is_shutdown():
    if msg3.data =="\u9ED8\u8BA4\u56DE\u7B54,\u6536\u5230":
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
    
    speech_go = threading.Thread(target=speech)                         
    speech_go.start()
    
    move_base_state = threading.Thread(target=move_base_state_get)                         
    move_base_state.start()

# ----------------- init ---------------------



if __name__ == '__main__':
    # 初始化节点
    rospy.init_node('move_test', anonymous=True)
    init_fun()
    while not voice_flag:
        print("等待中。")
    #a = input()
    begin_time=rospy.Time.now()
    #task4_slam()
    task1_find_terrorist()
    task2_find_tool()
    #task3_find_line()
    task3_slam()
    play_voice(8)
    #pid_go(0.15,0.30,180)
    #pid_turn(0,0,90)
    # pid_back(0.15,-1.8,0)
    #pid_loop()
print(" ________             __                ________ ")
print(" /_  __/ /_  ___  ____/ /___  _________/_  __/ /_")
print("  / / / __ \/ _ \/ __  / __ \/ ___/ ___// / / __/")
print(" / / / / / /  __/ /_/ / /_/ / /  / /__ / / / /_  ")
print("/_/ /_/ /_/\___/\__,_/\____/_/   \___//_/  \__/  ")
finish_time=rospy.Time.now()
print("时间-------->%.2f",(finish_time-begin_time).to_sec())





# [-2.3356403296190886, -1.1735821861239812, -0.4774447805625433]
# [-1.9740078712319697, -0.6642019582022957, -0.20748321454176047]
# [-2.0385920827003536, 0.6556602582025248, 2.7975111684590117]
# [-1.7627215891745593, 1.008736971752894, -0.10746479461809638]
# [0.04330322505603886, -0.0006240285601783518, 0.08651591807722625]