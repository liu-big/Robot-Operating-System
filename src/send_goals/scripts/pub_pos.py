#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 文件: pub_pos.py
# 作者: Guo
# 日期: 2023-10-26
# 功能: 该脚本用于向 ROS 导航栈发布预设的目标点。
#       它会依次发布多个 PoseStamped 消息，每个消息代表一个导航目标。
#       同时，它订阅 '/objects' 话题，但目前仅接收消息，未进行处理。

import rospy
import time
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray 

def doMsg(msg):
    """
    回调函数，用于处理订阅到的 '/objects' 话题消息。
    目前仅将消息赋值给变量 'a'，未进行实际处理。
    参数:
        msg (Float32MultiArray): 从 '/objects' 话题接收到的消息。
    """
    a = msg



if __name__ == '__main__':
    # 初始化 ROS 节点，命名为 'pubpose'
    rospy.init_node('pubpose')

    # 创建一个 Publisher，用于向 'move_base_simple/goal' 话题发布 PoseStamped 消息
    # queue_size=2 表示消息队列的大小，防止消息堆积
    turtle_vel_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=2)

    # 创建一个 Subscriber，用于订阅 '/objects' 话题的 Float32MultiArray 消息
    # 接收到的消息将由 doMsg 回调函数处理
    sub = rospy.Subscriber("/objects",Float32MultiArray,doMsg,queue_size=10)

    # 创建一个 PoseStamped 消息对象
    mypose=PoseStamped()
    # 首次发布一个空的 PoseStamped 消息，作为对导航系统的“试探”
    # 有时第一个消息可能会丢失，此举可提高后续消息的可靠性
    turtle_vel_pub.publish(mypose)
    # 等待 5 秒，给导航系统留出处理时间
    time.sleep(5)
    
    # 重新创建一个 PoseStamped 消息对象，准备发送第一个实际目标点
    mypose=PoseStamped()
    # 设置目标点的坐标系为 'map'，这是 ROS 导航中常用的全局坐标系
    mypose.header.frame_id='map'
    # 设置目标点的 x 坐标
    mypose.pose.position.x=3.924010
    # 设置目标点的 y 坐标
    mypose.pose.position.y=-1.403889
    # 设置目标点的 z 坐标（通常在2D导航中为0）
    mypose.pose.position.z=0
    # 设置目标点的姿态（四元数表示），这里表示没有旋转
    mypose.pose.orientation.x=0
    mypose.pose.orientation.y=0
    mypose.pose.orientation.z=1
    mypose.pose.orientation.w=0
    
    # 发布第一个设置好的目标点
    turtle_vel_pub.publish(mypose)
    # 等待 10 秒，等待机器人到达或处理此目标
    time.sleep(10)

    # 设置第二个目标点的坐标
    mypose.pose.position.x=5.716710
    mypose.pose.position.y=-2.020534
    mypose.pose.position.z=-0.000007

    # 发布第二个目标点
    turtle_vel_pub.publish(mypose)

    # 等待 10 秒
    time.sleep(10)

    # 设置第三个目标点的坐标
    mypose.pose.position.x=0.133
    mypose.pose.position.y=-5.19
    mypose.pose.position.z=-0.000007

    # 发布第三个目标点
    turtle_vel_pub.publish(mypose)

    # 等待 10 秒
    time.sleep(10)





    
