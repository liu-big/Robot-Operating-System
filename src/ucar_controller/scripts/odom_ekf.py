#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 导入rospy库，用于ROS节点编程
import rospy
# 导入PoseWithCovarianceStamped消息类型，用于接收带有协方差的位姿信息
from geometry_msgs.msg import PoseWithCovarianceStamped
# 导入Odometry消息类型，用于发布里程计信息
from nav_msgs.msg import Odometry
 
# 定义OdomEKF类，用于将robot_pose_ekf的输出转换为标准的Odometry消息
class OdomEKF():
   # 构造函数
   def __init__(self):
       # 初始化ROS节点，命名为'odom_ekf'，anonymous=False表示节点名称不唯一
       rospy.init_node('odom_ekf', anonymous=False)
 
       # 创建一个Publisher，发布类型为nav_msgs/Odometry的消息到'output'话题，队列大小为10
       self.ekf_pub = rospy.Publisher('output', Odometry, queue_size=10)
       
       # 等待'/odom_combined'话题可用，确保在订阅前话题已发布
       rospy.wait_for_message('input', PoseWithCovarianceStamped)
       
       # 订阅'input'话题，接收PoseWithCovarianceStamped类型的消息，并指定回调函数pub_ekf_odom
       rospy.Subscriber('input', PoseWithCovarianceStamped, self.pub_ekf_odom)
       
       # 打印日志信息，表示正在发布组合里程计信息
       rospy.loginfo("Publishing combined odometry on /odom_ekf")
       
   # 回调函数，处理接收到的PoseWithCovarianceStamped消息
   def pub_ekf_odom(self, msg):
       # 创建一个Odometry消息对象
       odom = Odometry()
       # 将接收到的消息的header赋值给odom的header
       odom.header = msg.header
       # 设置odom的frame_id为'/odom'，表示里程计信息所在的坐标系
       odom.header.frame_id = '/odom'
       # 设置odom的child_frame_id为'base_link'，表示里程计信息描述的子坐标系
       odom.child_frame_id = 'base_link'
       # 将接收到的消息的pose赋值给odom的pose
       odom.pose = msg.pose

       
       
       
       # 发布转换后的Odometry消息
       self.ekf_pub.publish(odom)
       
# Python脚本的入口点
if __name__ == '__main__':
   try:
       # 实例化OdomEKF类
       OdomEKF()
       # 进入ROS事件循环，等待回调函数被调用
       rospy.spin()
   except:
       # 捕获异常，通常在节点关闭时发生
       pass
