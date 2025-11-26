#! /usr/bin/env python3
# -*- coding: utf-8 -*-

# 导入rospy库，用于ROS节点编程
import rospy
# 从ucar_controller服务中导入所有服务定义
from ucar_controller.srv import *
# 导入threading库，用于多线程操作
import threading
# 导入tf库，用于处理ROS中的坐标变换
import tf

# 定义SensorTFServer类，用于管理和发布传感器（摄像头、激光雷达、IMU）的TF变换
class SensorTFServer:
    # 构造函数
    def __init__(self):
        # 初始化ROS节点，命名为'SensorTFServer'，anonymous=True表示节点名称可以重复
        rospy.init_node('SensorTFServer', anonymous=True)

        # 从参数服务器获取摄像头相对于base_link的位姿参数（x, y, z坐标和欧拉角r, p, y）
        self.camera_pose_x  = rospy.get_param('~camera_pose_x', 0.15)
        self.camera_pose_y  = rospy.get_param('~camera_pose_y', 0.0)
        self.camera_pose_z  = rospy.get_param('~camera_pose_z', 0.15)
        self.camera_euler_r = rospy.get_param('~camera_euler_r', -3.14159 * 115.0/180.0)
        self.camera_euler_p = rospy.get_param('~camera_euler_p', 0.0)
        self.camera_euler_y = rospy.get_param('~camera_euler_y', -3.14159 * 90.0/180.0)

        # 从参数服务器获取激光雷达相对于base_link的位姿参数
        self.lidar_pose_x  = rospy.get_param('~lidar_pose_x',  -0.11)
        self.lidar_pose_y  = rospy.get_param('~lidar_pose_y',  0.0)
        self.lidar_pose_z  = rospy.get_param('~lidar_pose_z',  0.165)
        self.lidar_euler_r = rospy.get_param('~lidar_euler_r', 0.0)
        self.lidar_euler_p = rospy.get_param('~lidar_euler_p', 0.0)
        self.lidar_euler_y = rospy.get_param('~lidar_euler_y', 0)

        # 从参数服务器获取IMU相对于base_link的位姿参数
        self.imu_pose_x  = rospy.get_param('~imu_pose_x',  0.05)
        self.imu_pose_y  = rospy.get_param('~imu_pose_y', -0.05)
        self.imu_pose_z  = rospy.get_param('~imu_pose_z',  0.05)
        self.imu_euler_r = rospy.get_param('~imu_euler_r', 3.14159)
        self.imu_euler_p = rospy.get_param('~imu_euler_p', 0.0)
        self.imu_euler_y = rospy.get_param('~imu_euler_y', 0.0)

        # 从参数服务器获取传感器帧名称和基座帧名称
        self.camera_frame = rospy.get_param("~camera_frame", "cam")
        self.lidar_frame  = rospy.get_param("~lidar_frame" , "lidar")
        self.imu_frame    = rospy.get_param("~imu_frame"   , "imu")
        self.base_frame   = rospy.get_param("~base_frame"  , "base_link")

        # 从参数服务器获取TF发布频率
        self.tf_rate      = rospy.get_param("~tf_rate"   , 30.0)

        # 创建ROS服务，用于设置和获取传感器TF
        self.set_camera_tf_service = rospy.Service('set_camera_tf', SetSensorTF, self.setCameraTFCB)
        self.set_lidar_tf_service  = rospy.Service('set_lidar_tf' , SetSensorTF, self.setLidarTFCB)
        self.set_imu_tf_service    = rospy.Service('set_imu_tf'   , SetSensorTF, self.setIMUTFCB)
        self.get_camera_tf_service = rospy.Service('get_camera_tf', GetSensorTF, self.getCameraTFCB)
        self.get_lidar_tf_service  = rospy.Service('get_lidar_tf' , GetSensorTF, self.getLidarTFCB)
        self.get_imu_tf_service    = rospy.Service('get_imu_tf'   , GetSensorTF, self.getIMUTFCB)
        # 创建TF广播器
        self.tf_br = tf.TransformBroadcaster()
        
        # 创建并启动一个线程，用于周期性发布TF变换
        send_tf_thread = threading.Thread(target=self.sendTFThread, name='T1')
        send_tf_thread.start()

        # 打印服务启动信息
        print("SensorTFServer ready.")
        # 进入ROS事件循环，等待服务请求和消息回调
        rospy.spin()

    # 设置摄像头TF的回调函数
    # req: SetSensorTFRequest服务请求，包含新的位姿参数
    # 返回: SetSensorTFResponse服务响应，指示操作是否成功
    def setCameraTFCB(self,req):
        self.camera_pose_x  = req.pose_x
        self.camera_pose_y  = req.pose_y
        self.camera_pose_z  = req.pose_z
        self.camera_euler_r = req.euler_r
        self.camera_euler_p = req.euler_p
        self.camera_euler_y = req.euler_y
        return SetSensorTFResponse(success = True, message = 'set camera tf success')

    # 获取摄像头TF的回调函数
    # req: GetSensorTFRequest服务请求
    # 返回: GetSensorTFResponse服务响应，包含当前的位姿参数
    def getCameraTFCB(self,req):
        response = GetSensorTFResponse()
        response.pose_x  = self.camera_pose_x
        response.pose_y  = self.camera_pose_y
        response.pose_z  = self.camera_pose_z
        response.euler_r = self.camera_euler_r
        response.euler_p = self.camera_euler_p
        response.euler_y = self.camera_euler_y
        return response
    
    # 设置激光雷达TF的回调函数
    # req: SetSensorTFRequest服务请求，包含新的位姿参数
    # 返回: SetSensorTFResponse服务响应，指示操作是否成功
    def setLidarTFCB(self,req):
        self.lidar_pose_x  = req.pose_x
        self.lidar_pose_y  = req.pose_y
        self.lidar_pose_z  = req.pose_z
        self.lidar_euler_r = req.euler_r
        self.lidar_euler_p = req.euler_p
        self.lidar_euler_y = req.euler_y
        return SetSensorTFResponse(success = True, message = 'set camera tf success')

    # 获取激光雷达TF的回调函数
    # req: GetSensorTFRequest服务请求
    # 返回: GetSensorTFResponse服务响应，包含当前的位姿参数
    def getLidarTFCB(self,req):
        response = GetSensorTFResponse()
        response.pose_x  = self.lidar_pose_x
        response.pose_y  = self.lidar_pose_y
        response.pose_z  = self.lidar_pose_z
        response.euler_r = self.lidar_euler_r
        response.euler_p = self.lidar_euler_p
        response.euler_y = self.lidar_euler_y
        return response

    # 设置IMU TF的回调函数
    # req: SetSensorTFRequest服务请求，包含新的位姿参数
    # 返回: SetSensorTFResponse服务响应，指示操作是否成功
    def setIMUTFCB(self,req):
        self.imu_pose_x  = req.pose_x
        self.imu_pose_y  = req.pose_y
        self.imu_pose_z  = req.pose_z
        self.imu_euler_r = req.euler_r
        self.imu_euler_p = req.euler_p
        self.imu_euler_y = req.euler_y
        return SetSensorTFResponse(success = True, message = 'set camera tf success')

    # 获取IMU TF的回调函数
    # req: GetSensorTFRequest服务请求
    # 返回: GetSensorTFResponse服务响应，包含当前的位姿参数
    def getIMUTFCB(self,req):
        response = GetSensorTFResponse()
        response.pose_x  = self.imu_pose_x
        response.pose_y  = self.imu_pose_y
        response.pose_z  = self.imu_pose_z
        response.euler_r = self.imu_euler_r
        response.euler_p = self.imu_euler_p
        response.euler_y = self.imu_euler_y
        return response

    # 周期性发布TF变换的线程函数
    def sendTFThread(self):
        # 设置TF发布频率
        rate = rospy.Rate(self.tf_rate)
        # 创建线程锁，用于保护共享资源（位姿参数）
        lock = threading.Lock()   
        # 循环直到ROS关闭
        while not rospy.is_shutdown():
            # 获取锁
            lock.acquire()
            # 发布摄像头TF变换
            self.tf_br.sendTransform((self.camera_pose_x, self.camera_pose_y, self.camera_pose_z),
                        tf.transformations.quaternion_from_euler(self.camera_euler_r, self.camera_euler_p, self.camera_euler_y),
                        rospy.Time.now(),
                        self.camera_frame,
                        self.base_frame)
            # 发布激光雷达TF变换 (当前被注释掉)
            # self.tf_br.sendTransform((self.lidar_pose_x, self.lidar_pose_y, self.lidar_pose_z),
            #             tf.transformations.quaternion_from_euler(self.lidar_euler_r, self.lidar_euler_p, self.lidar_euler_y),
            #             rospy.Time.now(),
            #             self.lidar_frame,
            #             self.base_frame)
            # 发布IMU TF变换
            self.tf_br.sendTransform((self.imu_pose_x, self.imu_pose_y, self.imu_pose_z),
                        tf.transformations.quaternion_from_euler(self.imu_euler_r, self.imu_euler_p, self.imu_euler_y),
                        rospy.Time.now(),
                        self.imu_frame,
                        self.base_frame)
            # 释放锁
            lock.release()
            # 按照设定的频率休眠
            rate.sleep()


# Python脚本的入口点
if __name__ == '__main__':
    # 实例化SensorTFServer类，启动TF服务
    server = SensorTFServer()
