/*
 *  YDLIDAR SYSTEM
 *  YDLIDAR ROS Node Client 
 *
 *  Copyright 2015 - 2018 EAI TEAM
 *  http://www.ydlidar.com
 * 
 */

/**
 * @file ydlidar_client.cpp
 * @brief YDLIDAR ROS 节点客户端示例
 * 
 * 该文件提供了一个简单的 ROS 客户端，用于订阅 YDLIDAR 发布的激光扫描数据（/scan 话题），
 * 并将接收到的数据打印到控制台。这对于验证激光雷达数据流和基本功能非常有用。
 */

#include "ros/ros.h" // 包含 ROS 核心库，提供 ROS 节点、话题、服务等功能
#include "sensor_msgs/LaserScan.h" // 包含 LaserScan 消息类型定义，用于处理激光雷达数据

/**
 * @brief 宏定义：将弧度转换为角度
 * 
 * @param x 弧度值
 * @return 对应的角度值
 */
#define RAD2DEG(x) ((x)*180./M_PI)

/**
 * @brief 激光扫描数据回调函数
 * 
 * 当订阅到 /scan 话题的 LaserScan 消息时，此函数会被调用。它解析激光扫描数据，
 * 并打印出雷达的帧ID、扫描点数量、角度范围以及特定角度的距离信息。
 * 
 * @param scan 指向接收到的 LaserScan 消息的常量指针
 */
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    // 计算激光扫描点的数量
    int count = scan->scan_time / scan->time_increment;
    // 打印激光扫描的帧ID和点数量
    printf("[YDLIDAR INFO]: I heard a laser scan %s[%d]:\n", scan->header.frame_id.c_str(), count);
    // 打印激光扫描的角度范围（从弧度转换为角度）
    printf("[YDLIDAR INFO]: angle_range : [%f, %f]\n", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));
  
    // 遍历所有激光扫描点
    for(int i = 0; i < count; i++) {
        // 计算当前扫描点的角度（从弧度转换为角度）
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
	// 如果角度在 -5 到 5 度之间，则打印该点的角度和距离
	if(degree > -5 && degree< 5)
        printf("[YDLIDAR INFO]: angle-distance : [%f, %f, %i]\n", degree, scan->ranges[i], i);
    }
}

/**
 * @brief 主函数
 * 
 * ROS 程序的入口点。初始化 ROS 节点，创建节点句柄，订阅 /scan 话题，
 * 并进入 ROS 事件循环，等待回调函数的触发。
 * 
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return 0 表示程序正常退出
 */
int main(int argc, char **argv)
{
    // 初始化 ROS 节点，节点名为 "ydlidar_client"
    ros::init(argc, argv, "ydlidar_client");
    // 创建 ROS 节点句柄
    ros::NodeHandle n;
    // 订阅 "/scan" 话题，消息类型为 sensor_msgs::LaserScan，队列大小为 1000，
    // 收到消息时调用 scanCallback 函数
    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);
    // 进入 ROS 事件循环，处理回调函数，直到节点关闭
    ros::spin();

    return 0;
}
