/**
 * @file ahrs_driver.h
 * @author FDILink
 * @brief FDILink AHRS 驱动程序的核心头文件。
 * @version 1.0
 * @date 2024-07-29
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#ifndef BASE_DRIVER_H_
#define BASE_DRIVER_H_

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <serial/serial.h> // ROS的串口通信库，用于与IMU设备进行数据交换。
#include <math.h>
#include <fstream>
#include <fdilink_data_struct.h> // 包含FDILink IMU数据结构体的定义。
#include <sensor_msgs/Imu.h> // ROS标准的IMU消息类型。
#include <geometry_msgs/Pose2D.h> // ROS二维姿态消息类型，用于发布指北角。
#include <boost/thread.hpp> // Boost线程库，可能用于多线程处理。
#include <string>
#include <ros/package.h> // ROS包管理工具，用于获取包路径。
#include <crc_table.h> // CRC校验表，用于数据完整性校验。


using namespace std;


namespace FDILink
{
// 定义帧头和帧尾。
#define FRAME_HEAD 0xfc
#define FRAME_END 0xfd
// 定义不同数据包类型。
#define TYPE_IMU 0x40
#define TYPE_AHRS 0x41
#define TYPE_INSGPS 0x42
#define TYPE_GROUND 0xf0
// 定义不同数据包的长度。
#define IMU_LEN  0x38   // 56字节
#define AHRS_LEN 0x30   // 48字节
#define INSGPS_LEN 0x54 // 84字节
// 定义数学常量。
#define PI 3.141592653589793
#define DEG_TO_RAD 0.017453292519943295

/**
 * @brief ahrsBringup 类，负责FDILink IMU设备的串口通信、数据解析和ROS消息发布。
 */
class ahrsBringup
{
public:
  /**
   * @brief 构造函数，初始化ROS节点句柄和参数。
   */
  ahrsBringup();

  /**
   * @brief 析构函数，清理资源。
   */
  ~ahrsBringup();

  /**
   * @brief 主处理循环，负责串口数据的读取、解析和发布。
   */
  void processLoop();

  /**
   * @brief 检查CRC8校验和。
   * @param len 数据长度。
   * @return bool 如果校验成功返回true，否则返回false。
   */
  bool checkCS8(int len);

  /**
   * @brief 检查CRC16校验和。
   * @param len 数据长度。
   * @return bool 如果校验成功返回true，否则返回false。
   */
  bool checkCS16(int len);

  /**
   * @brief 检查数据包的序列号，处理丢包情况。
   * @param type 数据包类型（IMU, AHRS, INSGPS）。
   */
  void checkSN(int type);

  /**
   * @brief 根据磁力计数据计算偏航角。
   * @param roll 翻滚角。
   * @param pitch 俯仰角。
   * @param magyaw 计算出的偏航角。
   * @param magx 磁力计X轴数据。
   * @param magy 磁力计Y轴数据。
   * @param magz 磁力计Z轴数据。
   */
  void magCalculateYaw(double roll, double pitch, double &magyaw, double magx, double magy, double magz);

  ros::NodeHandle nh_;

private:
  bool if_debug_; // 是否开启调试模式。
  // 统计信息。
  int sn_lost_ = 0; // 丢包计数。
  int crc_error_ = 0; // CRC校验错误计数。
  uint8_t read_sn_ = 0; // 读取到的序列号。
  bool frist_sn_; // 是否是第一个序列号。
  int device_type_ = 1; // 设备类型，用于坐标系转换。

  // 串口通信相关。
  serial::Serial serial_; // 串口对象。
  std::string serial_port_; // 串口名称。
  int serial_baud_; // 串口波特率。
  int serial_timeout_; // 串口超时时间。
  // 数据帧结构体。
  FDILink::imu_frame_read  imu_frame_; // IMU数据帧。
  FDILink::ahrs_frame_read ahrs_frame_; // AHRS数据帧。
  FDILink::insgps_frame_read insgps_frame_; // INSGPS数据帧。

  // ROS话题和帧ID。
  string imu_frame_id_; // IMU话题的frame_id。

  // ROS话题名称。
  string imu_topic_, mag_pose_2d_topic_; // IMU和二维姿态话题名称。
  
  // ROS发布器。
  ros::Publisher imu_pub_; // IMU数据发布器。
  ros::Publisher mag_pose_pub_; // 二维姿态数据发布器。

}; //ahrsBringup
} // namespace FDILink

#endif