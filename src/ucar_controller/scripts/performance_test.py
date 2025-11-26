#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 导入rospy库，用于ROS节点编程
import rospy
# 导入Twist和Quaternion消息类型，用于机器人运动控制和姿态表示
from geometry_msgs.msg import Twist, Quaternion
# 导入Odometry消息类型，用于里程计信息
from nav_msgs.msg import Odometry
# 导入tf库，用于处理ROS中的坐标变换
import tf
# 导入数学库，用于弧度转换、符号判断等
from math import radians, copysign
# 导入PyKDL库，用于处理机器人运动学和动力学
import PyKDL
# 导入pi常量
from math import pi
# 导入random库，用于生成随机数
import random

# 将四元数转换为欧拉角（偏航角）
# 参数: quat - geometry_msgs.msg.Quaternion类型的四元数
# 返回: 偏航角（弧度）
def quat_to_angle(quat):
    # 使用PyKDL的Quaternion创建旋转对象
    rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
    # 获取欧拉角（Roll, Pitch, Yaw），并返回Yaw（偏航角）
    return rot.GetRPY()[2]

# 将角度规范化到-pi到pi之间
# 参数: angle - 任意弧度值
# 返回: 规范化后的弧度值
def normalize_angle(angle):
    res = angle
    # 将角度调整到-pi到pi的范围内
    while res > pi:
        res -= 2.0 * pi
    while res < -pi:
        res += 2.0 * pi
    return res

# 定义CalibrateAngular类，用于机器人角度校准和性能测试
class CalibrateAngular():
    # 构造函数
    def __init__(self):
        # 初始化ROS节点，命名为'calibrate_angular'，anonymous=False表示节点名称不唯一
        rospy.init_node('calibrate_angular', anonymous=False)

        # 设置rospy在脚本终止时执行shutdown函数
        rospy.on_shutdown(self.shutdown)

        # 从参数服务器获取检查里程计值的频率，默认为20Hz
        self.rate = rospy.get_param('~rate', 20)

        # 从参数服务器获取机器人旋转速度（弧度/秒），默认为0.3
        self.speed = rospy.get_param('~speed', 0.3) # radians per second
        # 从参数服务器获取容忍度（度），并转换为弧度，默认为1度
        self.tolerance = radians(rospy.get_param('tolerance', 1)) # degrees converted to radians
        # 从参数服务器获取里程计角度比例校正因子，默认为1.0
        self.odom_angular_scale_correction = rospy.get_param('~odom_angular_scale_correction', 1.0)

        # 创建一个Publisher，发布Twist消息到'/cmd_vel'话题，用于控制机器人速度，队列大小为5
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        # 从参数服务器获取机器人基座坐标系名称，默认为'/base_footprint'
        self.base_frame = rospy.get_param('~base_frame', '/base_footprint')

        # 从参数服务器获取里程计坐标系名称，默认为'/odom'
        self.odom_frame = rospy.get_param('~odom_frame', '/odom')

        # 初始化tf监听器
        self.tf_listener = tf.TransformListener()

        # 给tf一些时间来填充其缓冲区
        rospy.sleep(2)

        # 确保能够看到odom和base坐标系之间的变换，等待最长60秒
        self.tf_listener.waitForTransform(self.odom_frame, self.base_frame, rospy.Time(), rospy.Duration(60.0))

        # 打印信息，提示用户启动rqt_reconfigure来控制测试
        rospy.loginfo("Bring up rqt_reconfigure to control the test.")

        # 调用性能测试方法
        self.performance_test()

    # 机器人性能测试方法
    def performance_test(self):
         # 循环直到ROS关闭
         while not rospy.is_shutdown():
            # 从tf获取当前里程计角度
            odom_angle = self.get_odom_angle()

            last_angle = odom_angle
            turn_angle = 0
            # 生成一个随机的测试角度
            test_angle = random.uniform(-3.14,3.14)
            rospy.loginfo('test_angle: %f'%test_angle)
            # 将测试角度加上当前里程计角度
            test_angle = test_angle + odom_angle
            # 计算初始误差
            error = test_angle - turn_angle

            # 设置循环频率
            r = rospy.Rate(self.rate)
            # 当误差大于容忍度时，继续旋转
            while abs(error) > self.tolerance:
                # 如果ROS已关闭，则退出
                if rospy.is_shutdown():
                    return

                # 创建Twist消息，用于控制机器人旋转
                move_cmd = Twist()
                # 根据误差方向设置角速度
                move_cmd.angular.z = copysign(self.speed, error)
                # 发布运动命令
                self.cmd_vel.publish(move_cmd)
                # 按照设定的频率休眠
                r.sleep()

                # 从tf获取当前里程计角度                   
                odom_angle = self.get_odom_angle()

                # 计算自上次测量以来旋转的角度增量
                delta_angle = normalize_angle(odom_angle - last_angle)

                # 累加到总旋转角度
                turn_angle += delta_angle

                # 计算新的误差
                error = test_angle - turn_angle

                # 存储当前角度以备下次比较
                last_angle = odom_angle

            # 停止机器人
            self.cmd_vel.publish(Twist())
            # 休眠0.5秒
            rospy.sleep(0.5)

    # 获取里程计角度
    # 返回: 当前里程计的偏航角（弧度）
    def get_odom_angle(self):
        # 尝试获取odom坐标系和base坐标系之间的变换
        try:
            (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            # 如果发生tf异常，打印日志并返回
            rospy.loginfo("TF Exception")
            return

        # 将四元数旋转转换为欧拉角并返回偏航角
        return quat_to_angle(Quaternion(*rot))


    # 关机函数，在节点关闭时调用
    def shutdown(self):
        # 打印日志信息
        rospy.loginfo("Stopping the robot...")
        # 发布Twist消息停止机器人运动
        self.cmd_vel.publish(Twist())
        # 休眠1秒
        rospy.sleep(1)

# Python脚本的入口点
if __name__ == '__main__':
    try:
        # 实例化CalibrateAngular类
        CalibrateAngular()
    except:
        # 捕获异常，通常在节点关闭时发生
        rospy.loginfo("Calibration terminated.")

