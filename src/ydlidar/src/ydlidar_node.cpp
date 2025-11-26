/*
 *  YDLIDAR SYSTEM
 *  YDLIDAR ROS Node Client 
 *
 *  Copyright 2015 - 2018 EAI TEAM
 *  http://www.ydlidar.com
 * 
 */

/**
 * @file ydlidar_node.cpp
 * @brief YDLIDAR ROS 节点的主要实现文件
 * 
 * 该文件包含了 YDLIDAR 激光雷达的 ROS 节点实现。它负责与 YDLIDAR 硬件进行通信，
 * 获取激光扫描数据，并将其作为 sensor_msgs/LaserScan 消息发布到 ROS 话题上。
 * 节点还处理各种参数配置，如串口设置、扫描范围、频率等。
 */

#include "ros/ros.h" // 包含 ROS 核心库，提供 ROS 节点、话题、服务等功能
#include "sensor_msgs/LaserScan.h" // 包含 LaserScan 消息类型定义，用于处理激光雷达数据
#include "CYdLidar.h" // 包含 YDLIDAR SDK 的核心库，用于与雷达硬件通信
#include <vector> // 包含标准库的 vector 容器
#include <iostream> // 包含标准库的输入输出流
#include <string> // 包含标准库的字符串操作
#include <signal.h> // 包含信号处理库，用于捕获程序退出信号
#include <sstream> // 包含字符串流，用于字符串分割

using namespace ydlidar; // 使用 ydlidar 命名空间

#define ROSVerision "1.4.2" // 定义 ROS 版本宏

/**
 * @brief 字符串分割函数
 * 
 * 将给定的字符串按照指定的分隔符进行分割，并将分割后的子字符串转换为浮点数存储在 vector 中。
 * 主要用于解析 ROS 参数中的忽略角度数组。
 * 
 * @param s 要分割的字符串
 * @param delim 分隔符
 * @return 包含分割后浮点数的 vector
 */
std::vector<float> split(const std::string &s, char delim) {
    std::vector<float> elems; // 存储分割后的浮点数
    std::stringstream ss(s); // 创建字符串流
    std::string number; // 临时存储每个分割出的数字字符串
    // 循环读取直到没有分隔符
    while(std::getline(ss, number, delim)) {
        elems.push_back(atof(number.c_str())); // 将字符串转换为浮点数并添加到 vector
    }
    return elems; // 返回包含浮点数的 vector
}

/**
 * @brief 主函数
 * 
 * ROS 节点的入口点。负责初始化 ROS、配置 YDLIDAR 传感器、启动数据采集循环并发布激光扫描数据。
 * 
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return 0 表示程序正常退出
 */
int main(int argc, char * argv[]) {
    ros::init(argc, argv, "ydlidar_node"); 
    // 打印 YDLIDAR 的 ASCII 艺术字，增加程序启动时的视觉效果
    printf("__   ______  _     ___ ____    _    ____  \n");
    printf("\\ \ / /  _ \\| |    |_ _|  _ \\  / \  |  _ \\ \n");
    printf(" \\ V /| | | | |     | || | | |/ _ \\ | |_) | \n");
    printf("  | | | |_| | |___  | || |_| / ___ \\|  _ <  \n");
    printf("  |_| |____/|_____|___|____/_/   \\_\\_| \_\\_\ \n");
    printf("\n");
    fflush(stdout); // 刷新输出缓冲区，确保立即显示 ASCII 艺术字
  
    // 声明用于存储 ROS 参数的变量
    std::string port; // 串口名称
    int baudrate=230400; // 串口波特率，默认 230400
    int samp_rate = 9; // 采样率
    std::string frame_id; // 激光雷达数据发布的坐标系 ID
    bool reversion, resolution_fixed; // 扫描方向是否反转，分辨率是否固定
    bool auto_reconnect; // 是否自动重连
    double angle_max,angle_min; // 扫描角度范围
    result_t op_result; // 操作结果
    std::string list; // 忽略角度列表的字符串形式
    std::vector<float> ignore_array;  // 忽略角度的浮点数数组
    double max_range, min_range; // 扫描距离范围
    double frequency; // 扫描频率

    // 创建 ROS 节点句柄和发布器
    ros::NodeHandle nh; // 全局节点句柄
    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1000); // 创建激光扫描数据发布器，话题名为 "scan"
    ros::NodeHandle nh_private("~"); // 私有节点句柄，用于获取节点私有参数

    // 从 ROS 参数服务器获取参数，如果未设置则使用默认值
    nh_private.param<std::string>("port", port, "/dev/ydlidar"); // 获取串口名称参数
    nh_private.param<std::string>("frame_id", frame_id, "laser_frame"); // 获取坐标系 ID 参数
    nh_private.param<bool>("resolution_fixed", resolution_fixed, "true"); // 获取分辨率固定参数
    nh_private.param<bool>("auto_reconnect", auto_reconnect, "true"); // 获取自动重连参数
    nh_private.param<bool>("reversion", reversion, "true"); // 获取扫描方向反转参数
    nh_private.param<double>("angle_max", angle_max , 180); // 获取最大扫描角度参数
    nh_private.param<double>("angle_min", angle_min , -180); // 获取最小扫描角度参数
    nh_private.param<double>("range_max", max_range , 16.0); // 获取最大扫描距离参数
    nh_private.param<double>("range_min", min_range , 0.08); // 获取最小扫描距离参数
    nh_private.param<double>("frequency", frequency , 10.0); // 获取扫描频率参数
    nh_private.param<int>("samp_rate", samp_rate , 9); // 获取采样率参数
    nh_private.param<std::string>("ignore_array",list,""); // 获取忽略角度数组字符串参数

    // 处理忽略角度数组参数
    ignore_array = split(list ,','); // 将字符串分割为浮点数数组
    if(ignore_array.size()%2){ // 检查忽略角度数组的元素数量是否为偶数
        ROS_ERROR_STREAM("ignore array is odd need be even"); // 如果不是偶数，则报错
    }

    // 检查忽略角度的范围是否有效
    for(uint16_t i =0 ; i < ignore_array.size();i++){ // 遍历忽略角度数组
        if(ignore_array[i] < -180 && ignore_array[i] > 180){ // 检查角度是否在 -180 到 180 之间
            ROS_ERROR_STREAM("ignore array should be between 0 and 360"); // 如果不在有效范围，则报错
        }
    }

    // 创建 YDLIDAR 对象并设置参数
    CYdLidar laser; // 实例化 CYdLidar 类
    // 频率参数校验和调整
    if(frequency<5){ // 如果频率小于 5 Hz
       frequency = 7.0; // 强制设置为 7 Hz
    }
    if(frequency>12){ // 如果频率大于 12 Hz
        frequency = 12; // 强制设置为 12 Hz
    }
    // 角度范围参数校验和调整，确保 angle_max 大于 angle_min
    if(angle_max < angle_min){
        double temp = angle_max;
        angle_max = angle_min;
        angle_min = temp;
    }

    // 设置 YDLIDAR 对象的各项参数
    ROS_INFO("[YDLIDAR INFO] Now YDLIDAR ROS SDK VERSION:%s .......", ROSVerision); // 打印 SDK 版本信息
    laser.setSerialPort(port); // 设置串口名称
    laser.setSerialBaudrate(baudrate); // 设置串口波特率
    laser.setMaxRange(max_range); // 设置最大扫描距离
    laser.setMinRange(min_range); // 设置最小扫描距离
    laser.setMaxAngle(angle_max); // 设置最大扫描角度
    laser.setMinAngle(angle_min); // 设置最小扫描角度
    laser.setReversion(reversion); // 设置扫描方向是否反转
    laser.setFixedResolution(resolution_fixed); // 设置分辨率是否固定
    laser.setAutoReconnect(auto_reconnect); // 设置是否自动重连
    laser.setScanFrequency(frequency); // 设置扫描频率
    laser.setSampleRate(samp_rate); // 设置采样率
    laser.setIgnoreArray(ignore_array); // 设置忽略角度数组

    // 初始化 YDLIDAR
    bool ret = laser.initialize(); // 初始化激光雷达硬件和通信
    if (ret) { // 如果初始化成功
        ret = laser.turnOn(); // 尝试开启激光雷达扫描模式
        if (!ret) { // 如果开启失败
            ROS_ERROR("Failed to start scan mode!!!"); // 打印错误信息
        }
    } else { // 如果初始化失败
        ROS_ERROR("Error initializing YDLIDAR Comms and Status!!!"); // 打印错误信息
    }

    // 设置 ROS 循环频率
    ros::Rate rate(20); // 设置循环频率为 20 Hz

    // 主循环：持续获取激光数据并发布
    while (ret&&ros::ok()) { // 当激光雷达初始化成功且 ROS 节点正常运行时
        bool hardError; // 硬件错误标志
        LaserScan scan; // 存储激光扫描数据
        if(laser.doProcessSimple(scan, hardError )){ // 执行单次激光扫描处理
            sensor_msgs::LaserScan scan_msg; // 创建 ROS LaserScan 消息对象
            ros::Time start_scan_time; // 扫描开始时间
            // 将雷达系统时间戳转换为 ROS 时间戳
            start_scan_time.sec = scan.system_time_stamp/1000000000ul;
            start_scan_time.nsec = scan.system_time_stamp%1000000000ul;
            scan_msg.header.stamp = start_scan_time; // 设置消息时间戳
            scan_msg.header.frame_id = frame_id; // 设置消息的坐标系 ID
            scan_msg.angle_min =(scan.config.min_angle); // 设置最小角度
            scan_msg.angle_max = (scan.config.max_angle); // 设置最大角度
            scan_msg.angle_increment = (scan.config.angle_increment); // 设置角度增量
            scan_msg.scan_time = scan.config.scan_time; // 设置扫描时间
            scan_msg.time_increment = scan.config.time_increment; // 设置时间增量
            scan_msg.range_min = (scan.config.min_range); // 设置最小距离
            scan_msg.range_max = (scan.config.max_range); // 设置最大距离
            
            scan_msg.ranges = scan.ranges; // 填充距离数据
            scan_msg.intensities =  scan.intensities; // 填充强度数据
            scan_pub.publish(scan_msg); // 发布 LaserScan 消息
        }  
        rate.sleep(); // 按照设定的频率休眠
        ros::spinOnce(); // 处理一次 ROS 回调事件
    }

    // 关闭激光雷达并断开连接
    laser.turnOff(); // 关闭激光雷达扫描模式
    printf("[YDLIDAR INFO] Now YDLIDAR is stopping .......\n"); // 打印停止信息
    laser.disconnecting(); // 断开与激光雷达的连接
    return 0; // 程序正常退出
}
