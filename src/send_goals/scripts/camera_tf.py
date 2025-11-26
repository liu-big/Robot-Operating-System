#!/usr/bin/env python
#coding=utf-8

# 文件: camera_tf.py
# 作者: Guo
# 日期: 2023-10-26
# 功能: 该脚本订阅视觉检测结果和激光雷达数据，并根据视觉信息计算目标在相机坐标系下的位置，
#       然后通过 TF 广播器将目标位置转换为机器人坐标系下的变换，以便导航系统使用。
#       脚本中包含了相机标定参数、全局变量定义、数据处理函数以及 TF 广播逻辑。

import rospy
import time
from std_msgs.msg import Float32MultiArray 
from sensor_msgs.msg import LaserScan
import math
import tf2_ros
import tf
from geometry_msgs.msg import TransformStamped
from multiprocessing import Process, Lock
from collections import deque


# 创建进程锁，用于多进程间共享数据时的同步
lock = Lock()

# 相机标定参数
cx = 319.03663  # 图像主点 x 坐标
cy = 235.3358   # 图像主点 y 坐标
fx = 408.12625  # 焦距 x 分量
fy = 408.87682  # 焦距 y 分量

# 全局变量声明
global u         # 视觉检测到的目标在图像中的像素 x 坐标
global v         # 视觉检测到的目标在图像中的像素 y 坐标
global X         # 目标在相机坐标系下的 x 坐标
global Z         # 目标在相机坐标系下的 z 坐标（深度）
global sign      # 标志位，用于控制 TF 广播
global broadcaster # TF 广播器对象
global task      # 当前任务ID

# 雷达数据转换相关变量初始化
u = 0.0
v = 0.0
X = 0.0
Z = 0.0
sign = 0

# 任务识别相关变量初始化
task = 0.0
task_list = [0,0,0] # 任务列表，目前未使用
vision_data = 0.0   # 视觉数据标识，表示当前检测到的物体类型

# 创建一个双向队列，用于存储历史 (X, Z) 数据，用于平滑处理
data_window = deque(maxlen=10) # 队列最大长度为 10
average_x = 0.0 # 平滑后的 X 坐标
average_z = 0.0 # 平滑后的 Z 坐标

# 设置指数加权移动平均滤波器的权重参数
alpha = 0.2  # 权重参数，取值范围通常为 (0, 1)，值越大，当前数据的影响越大

# 视觉数据包格式说明：
# [terrorist1, terrorist2, terrorist3, spontoon, teargas, bulltproo_vest,
# first_aid_kit,  teargas[(x, y)], spontoon[(x, y)], bulltproo_vest[(x, y)]]
# 这是一个示例格式，实际使用时需要根据 msg.data 的具体内容进行解析。



def doMsg(msg):
    """
    回调函数，用于处理订阅到的 Float32MultiArray 类型的视觉消息。
    该函数会根据接收到的数据更新全局变量 u, v, vision_data, sign 和 task。
    参数:
        msg (Float32MultiArray): 包含视觉检测结果的消息。
    """
    with lock:
        # 在回调函数中处理接收到的消息
        global u, v, vision_data
        global sign,task
        # 如果 sign 为 0，则将其设置为 1，表示已接收到第一帧数据
        if sign == 0:
            sign=1
        # 判断数据包格式是否正确，并进行数据记录
        if len(msg.data) >= 13:
            data_list = list(msg.data)
            # 如果前三个元素（恐怖分子ID）不全为 0，则更新当前任务ID
            if data_list[0]+data_list[1]+data_list[2] != 0:
                for i in range(3):
                    if data_list[i] != 0.0:
                        task = data_list[i]
            print(f"当前任务ID: {task}")

            # 根据任务ID和检测到的道具ID更新 u, v 和 vision_data
            # 恐怖分子1：寻找道具：警棍 (ID: 4.0)
            if task == 1.0:
                if data_list[3] == 4.0: # 检查是否检测到警棍
                    if data_list[7] != 0.0 and data_list[8] != 0.0: # 检查警棍的 (x, y) 坐标是否有效
                        u = data_list[7]
                        v = data_list[8]
                        vision_data = 1.0 # 视觉数据标识为 1.0 (警棍)
                    else:
                        u = 0.0
                        v = 0.0
                else:
                    u = 0.0
                    v = 0.0
                    vision_data = 0.0 # 未检测到警棍，视觉数据标识为 0.0

            # 恐怖分子2：寻找道具：防弹衣 (ID: 5.0)
            if task == 2.0:
                if data_list[4] == 5.0: # 检查是否检测到防弹衣
                    if data_list[9] != 0.0 and data_list[10] != 0.0: # 检查防弹衣的 (x, y) 坐标是否有效
                        u = data_list[9]
                        v = data_list[10]
                        vision_data = 2.0 # 视觉数据标识为 2.0 (防弹衣)
                    else:
                        u = 0.0
                        v = 0.0
                else:
                    u = 0.0
                    v = 0.0
                    vision_data = 0.0 # 未检测到防弹衣，视觉数据标识为 0.0

            # 恐怖分子3：寻找道具：催泪瓦斯 (ID: 6.0)
            if task == 3.0:
                if data_list[5] == 6.0: # 检查是否检测到催泪瓦斯
                    if data_list[11] != 0.0 and data_list[12] != 0.0: # 检查催泪瓦斯的 (x, y) 坐标是否有效
                        u = data_list[11]
                        v = data_list[12]
                        vision_data = 3.0 # 视觉数据标识为 3.0 (催泪瓦斯)
                    else:
                        u = 0.0
                        v = 0.0
                else:
                    u = 0.0
                    v = 0.0
                    vision_data = 0.0 # 未检测到催泪瓦斯，视觉数据标识为 0.0

            print(f"视觉数据标识: {vision_data}")

        else:
            rospy.logwarn("接收到空的 Float32MultiArray 消息，或数据长度不足。")  



def get_laserscan(scan):
    """
    回调函数，用于处理订阅到的 LaserScan 类型的激光雷达消息。
    该函数根据视觉检测到的目标像素坐标 (u, v) 和激光雷达深度数据，
    计算目标在相机坐标系下的三维位置 (X, Z)，并将其转换为 TF 变换进行广播。
    参数:
        scan (LaserScan): 包含激光雷达扫描数据的消息。
    """
    global average_x, average_z,sign, vision_data, broadcaster, task
    with lock:
        global u,v
        # 根据相机内参和目标像素坐标计算目标在相机坐标系下的水平角度
        tanx = (u - cx)/fx
        atan_x = math.atan(tanx)
        angle_x = math.degrees(atan_x)

        # 如果视觉检测到有效的目标 (u, v 不为 0)
        if u and v != 0.0:
            # 根据水平角度从激光雷达数据中获取深度值
            # 440 是激光雷达数据的中心索引，0.5 是角度分辨率
            depth = scan.ranges[440-int(angle_x/0.5)]
            # 如果深度值过大（表示目标可能太远或不存在），则清空视觉数据标识
            if depth >= 2.5:
                vision_data = 0
        else:
            depth = 0.0 # 如果没有视觉数据，则深度为 0

        # 如果深度为无穷大，则返回 0 (无效数据)
        if depth == float('inf'):
            return 0
            
        else : 
            rospy.loginfo("激光雷达深度: %f",depth)

        # 根据相机内参和深度计算目标在相机坐标系下的 X 和 Z 坐标
        X = (u - cx) * depth / fx
        Z = depth

        # 如果视觉数据标识与当前任务不匹配，则清空 TF 坐标并广播一个零变换
        if (vision_data != task):
            sign = 0 # 重置 sign 标志位
            tfs = TransformStamped() # 创建一个空的 TF 变换消息
            tfs.header.frame_id = "camera_frame" # 设置父坐标系
            tfs.header.stamp = rospy.Time.now() # 设置时间戳
            tfs.child_frame_id = "target_frame" # 设置子坐标系
            # 设置平移和旋转为零
            tfs.transform.translation.x = 0
            tfs.transform.translation.y = 0
            tfs.transform.translation.z = 0
            tfs.transform.rotation.x = 0
            tfs.transform.rotation.y = 0
            tfs.transform.rotation.z = 0
            tfs.transform.rotation.w = 1
            broadcaster.sendTransform(tfs) # 广播零变换
            return # 提前返回，不进行后续处理
        
        # 如果深度和计算出的 X, Z 坐标有效
        if depth != float('inf') and X != 0.0 and Z!= 0.0:
            # 将当前 (X, Z) 数据添加到数据窗口中，用于平滑处理
            data_window.append((X, Z))
            # 更新指数加权移动平均值
            # average_x = alpha * X + (1 - alpha) * average_x
            # average_z = alpha * Z + (1 - alpha) * average_z
            # 简单平均（如果需要指数加权移动平均，请取消注释上面两行）
            sum_x = sum(x for x, _ in data_window)
            sum_z = sum(z for _, z in data_window)
            average_x = sum_x / len(data_window)
            average_z = sum_z / len(data_window)

        # 创建并设置 TF 广播数据 (通过 pose 设置)
        tfs = TransformStamped()

        tfs.header.frame_id = "camera_frame" # 设置父坐标系为相机坐标系
        tfs.header.stamp = rospy.Time.now() # 设置当前时间戳
        tfs.child_frame_id = "target_frame" # 设置子坐标系为目标坐标系

        # 设置平移变换：相机坐标系下的 Z 轴对应机器人坐标系下的 X 轴，相机坐标系下的 -X 轴对应机器人坐标系下的 Y 轴
        tfs.transform.translation.x = average_z
        tfs.transform.translation.y = -average_x
        tfs.transform.translation.z = 0 # Z 轴通常为 0，因为是 2D 导航

        # 设置旋转变换（四元数表示），这里设置为无旋转
        qtn = tf.transformations.quaternion_from_euler(0,0,0)
        tfs.transform.rotation.x = qtn[0]
        tfs.transform.rotation.y = qtn[1]
        tfs.transform.rotation.z = qtn[2]
        tfs.transform.rotation.w = qtn[3]
    
        # 广播器发布数据
        if sign == 1: # 如果 sign 标志位为 1，表示有有效视觉数据，则广播变换
            broadcaster.sendTransform(tfs)
        else: # 否则，广播一个零变换
            tfs.transform.translation.x = 0
            tfs.transform.translation.y = 0
            tfs.transform.translation.z = 0
            qtn = tf.transformations.quaternion_from_euler(0, 0, 0)
            tfs.transform.rotation.x = qtn[0]
            tfs.transform.rotation.y = qtn[1]
            tfs.transform.rotation.z = qtn[2]
            tfs.transform.rotation.w = qtn[3]
            broadcaster.sendTransform(tfs)


if __name__ == '__main__':
    # 初始化 ROS 节点，命名为 'camera_tf'
    rospy.init_node('camera_tf')

    # 创建一个 TF2 广播器实例，用于发布坐标变换
    broadcaster = tf2_ros.TransformBroadcaster()

    # 订阅 '/objects' 话题，接收 Float32MultiArray 类型的视觉检测数据
    # 当接收到消息时，调用 doMsg 回调函数处理
    rospy.Subscriber("/objects",Float32MultiArray,doMsg,queue_size=10)

    # 订阅 '/scan' 话题，接收 LaserScan 类型的激光雷达数据
    # 当接收到消息时，调用 get_laserscan 回调函数处理
    rospy.Subscriber("/scan",LaserScan,get_laserscan,queue_size=10)

    # 保持节点运行，直到被关闭 (例如，通过 Ctrl+C)
    rospy.spin()