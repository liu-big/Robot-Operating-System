#!/usr/bin/env python
#coding=utf-8

# 文件: camera_tf_2.py
# 作者: Guo
# 日期: 2023-10-26
# 功能: 该脚本订阅视觉检测结果和激光雷达数据，并根据视觉信息计算目标在相机坐标系下的位置，
#       然后通过 TF 广播器将目标位置转换为机器人坐标系下的变换，以便导航系统使用。
#       此版本可能包含与 camera_tf.py 不同的逻辑或参数。

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
global Y         # 目标在相机坐标系下的 y 坐标 (此脚本中未使用)
global Z         # 目标在相机坐标系下的 z 坐标（深度）
global sign      # 标志位，用于控制 TF 广播
global broadcaster # TF 广播器对象

# 变量初始化
u = 0.0
v = 0.0
X = 0.0
Y = 0.0
Z = 0.0
sign = 0

# 创建一个双向队列，用于存储历史 (X, Z) 数据，用于平滑处理
data_window = deque(maxlen=10) # 队列最大长度为 10
average_x = 0.0 # 平滑后的 X 坐标
average_z = 0.0 # 平滑后的 Z 坐标

# 设置指数加权移动平均滤波器的权重参数
alpha = 0.2  # 权重参数，取值范围通常为 (0, 1)，值越大，当前数据的影响越大
vision_data = 0.0 # 视觉数据标识，表示当前检测到的物体类型

# 视觉数据包格式说明：
# [terrorist1, terrorist2, terrorist3, teargas, spontoon, 
# first_aid_kit, bulltproo_vest, teargas[(x, y)], spontoon[(x, y)], bulltproo_vest[(x, y)]]
# 这是一个示例格式，实际使用时需要根据 msg.data 的具体内容进行解析。

global task,task_list
task = 8.0 # 当前任务ID，初始值为 8.0
task_list=[4,6,3,0,0,0,0,0,0] # 任务列表，可能用于存储不同任务对应的道具ID

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
        global sign,task,task_list

        # 如果 sign 为 0，则将其设置为 1，表示已接收到第一帧数据
        if sign == 0:
            sign=1
        # 判断数据包格式是否正确，并进行数据记录
        if len(msg.data) >= 13:
            data_list = list(msg.data)
            # print(type(msg.data))

            # 遍历前三个元素，更新当前任务ID
            for i in range(3):
                print(data_list[i])
                if data_list[i] != 8.0: # 8.0 可能表示无效或默认任务ID
                    print("#######################################################################")
                    task = data_list[i]

            print(f"当前任务ID: {task}")

            # 根据任务ID和检测到的道具ID更新 u, v 和 vision_data
            # 恐怖分子1：寻找道具：警棍 (task == 0.0, 道具ID: 4.0)
            if task == 0.0:
                if data_list[4] == 4.0: # 检查是否检测到警棍
                    if data_list[9] != 8.0 and data_list[10] != 8.0: # 检查警棍的 (x, y) 坐标是否有效
                        u = data_list[9]
                        v = data_list[10]
                    else:
                        u = 0.0
                        v = 0.0
                    vision_data = data_list[4] # 视觉数据标识为警棍的ID
                else:
                    u = 0.0
                    v = 0.0
                    vision_data = 0.0 # 未检测到警棍，视觉数据标识为 0.0

            # 恐怖分子2：寻找道具：防弹衣 (task == 1.0, 道具ID: 6.0)
            if task == 1.0:
                if data_list[6] == 6.0: # 检查是否检测到防弹衣
                    if data_list[11] != 8.0 and data_list[12] != 8.0: # 检查防弹衣的 (x, y) 坐标是否有效
                        u = data_list[11]
                        v = data_list[12]
                    else:
                        u = 0.0
                        v = 0.0
                    vision_data = data_list[6] # 视觉数据标识为防弹衣的ID
                else:
                    u = 0.0
                    v = 0.0
                    vision_data = 0.0 # 未检测到防弹衣，视觉数据标识为 0.0

            # 恐怖分子3：寻找道具：催泪瓦斯 (task == 2.0, 道具ID: 3.0)
            if task == 2.0:
                if data_list[3] == 3.0: # 检查是否检测到催泪瓦斯
                    if data_list[7] != 8.0 and data_list[8] != 8.0: # 检查催泪瓦斯的 (x, y) 坐标是否有效
                        u = data_list[7]
                        v = data_list[8]
                    else:
                        u = 0.0
                        v = 0.0
                    vision_data = data_list[3] # 视觉数据标识为催泪瓦斯的ID
                else:
                    u = 0.0
                    v = 0.0
                    vision_data = 0.0 # 未检测到催泪瓦斯，视觉数据标识为 0.0

            # 更新 msg.data (此行注释掉，因为没有实际更新 msg.data 的操作)

            # 调试信息
            # rospy.loginfo("Received and processed data: %s", str(data_list))
        else:
            rospy.logwarn("接收到空的 Float32MultiArray 消息，或数据长度不足。")  



def get_laserscan(scan):
    """
    回调函数，用于处理订阅到的 LaserScan 类型的激光雷达消息。
    该函数结合视觉数据和激光雷达深度信息，计算目标在机器人坐标系下的三维位置，
    并广播从机器人基坐标系到目标坐标系的 TF 变换。
    参数:
        scan (LaserScan): 包含激光雷达扫描数据消息。
    """
    global average_x, average_z,sign, vision_data, broadcaster
    with lock:
        global u,v
        
        # 计算像素点对应的水平角度
        tanx = (u - cx)/fx
        atan_x = math.atan(tanx)
        angle_x = math.degrees(atan_x)

        # 根据视觉检测到的 u, v 坐标和激光雷达数据获取深度信息
        if u and v != 0.0:
            # 根据角度计算激光雷达数据索引，并获取深度
            # 454 是激光雷达数据中心点索引，0.396 是每度对应的索引步长
            depth = scan.ranges[454-int(angle_x/0.396)]
            # 如果深度大于等于 3.0 米，则认为目标过远或无效，重置视觉数据标识
            if depth >= 3.0:
                vision_data = 0
            else:
                pass # 深度有效，继续处理
        else:
            depth = 0.0 # 如果 u 或 v 为 0，表示未检测到目标，深度为 0

        # 检查深度是否为无穷大，如果是则返回 0，不进行后续处理
        if depth == float('inf'):
            return 0
            
        else :
            rospy.loginfo("激光雷达深度: %f",depth)

        # 计算目标在相机坐标系下的 X 和 Z 坐标
        X = (u - cx) * depth / fx
        Z = depth

        # print(vision_data)
        
        # 根据任务ID和视觉数据判断是否需要清空 TF 坐标
        # 如果当前视觉数据与任务列表中的预期道具ID不匹配，则清空 TF 坐标并广播零变换
        if (vision_data != task_list[int(task)] )   :  # 如果msg.data[x]不为x.0，则清空TF坐标
            sign = 0 # 重置标志位
            # 创建并广播一个零变换，表示目标消失或无效
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
            return
        

        # 如果深度有效且 X, Z 不为 0，则将 (X, Z) 数据存入队列，用于平滑处理
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

        #4-2.创建 广播的数据(通过 pose 设置)
        tfs = TransformStamped()

        tfs.header.frame_id = "camera_frame"
        tfs.header.stamp = rospy.Time.now()
        tfs.child_frame_id = "target_frame"
        #print(average_z,-average_x)
        tfs.transform.translation.x = average_z
        tfs.transform.translation.y = -average_x
        tfs.transform.translation.z = 0


        qtn = tf.transformations.quaternion_from_euler(0,0,0)
        tfs.transform.rotation.x = qtn[0]
        tfs.transform.rotation.y = qtn[1]
        tfs.transform.rotation.z = qtn[2]
        tfs.transform.rotation.w = qtn[3]
        
        # tfs.transform.translation.x = -X
        # tfs.transform.translation.y = -Y+0.5
        # tfs.transform.translation.z = Z
        broadcaster.sendTransform(tfs)
        
        #4-3.广播器发布数据
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



if __name__ == '__main__':
    # 初始化 ROS 节点
    rospy.init_node('camera_tf_broadcaster', anonymous=True)

    # 创建 TF 广播器和 TF 监听器
    br = tf2_ros.TransformBroadcaster() # 用于广播 TF 变换
    tf_buffer = tf2_ros.Buffer()       # 用于存储 TF 变换数据
    listener = tf2_ros.TransformListener(tf_buffer) # 用于监听 TF 变换

    # 订阅视觉检测结果话题
    # 当接收到 /objects 话题的消息时，调用 doMsg 函数进行处理
    rospy.Subscriber('/objects', Float32MultiArray, doMsg)

    # 订阅激光雷达数据话题
    # 当接收到 /scan 话题的消息时，调用 get_laserscan 函数进行处理
    rospy.Subscriber('/scan', LaserScan, get_laserscan)

    # 保持节点运行，直到被关闭
    rospy.spin()