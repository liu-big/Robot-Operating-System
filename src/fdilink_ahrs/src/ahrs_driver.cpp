/**
 * @file ahrs_driver.cpp
 * @brief AHRS (Attitude and Heading Reference System) 驱动程序实现文件。
 * 
 * 该文件包含了 `ahrsBringup` 类的实现，负责与AHRS设备进行串口通信，
 * 解析传感器数据（IMU、AHRS、INSGPS），并将其作为ROS消息发布。
 * 支持CRC校验、序列号检查和坐标系变换。
 */

#include <ahrs_driver.h>
#include <Eigen/Eigen>

namespace FDILink
{
/**
 * @brief ahrsBringup类的构造函数。
 * 
 * 构造函数负责初始化AHRS数据读取和ROS发布节点。它从ROS参数服务器加载配置，
 * 设置串口通信参数，打开串口，并初始化ROS消息发布器。
 * 
 * @param frist_sn_ 内部成员变量，初始化为false，用于标记是否第一次成功读取到序列号。
 * @param serial_timeout_ 内部成员变量，初始化为20ms，定义串口读取操作的超时时间。
 */
ahrsBringup::ahrsBringup() :frist_sn_(false), serial_timeout_(20)
{
  // 创建私有ROS节点句柄，用于获取私有参数
  ros::NodeHandle pravite_nh("~");

  // --- ROS参数配置 ---
  // 获取是否开启调试模式参数，默认为false
  pravite_nh.param("debug",     if_debug_,  false);
  // 获取设备类型参数，默认为1 (单IMU)
  pravite_nh.param("device_type", device_type_, 1); 
  // 获取IMU话题名称参数，默认为"/imu"
  pravite_nh.param("imu_topic", imu_topic_, std::string("/imu"));
  // 获取IMU坐标系ID参数，默认为"imu"
  pravite_nh.param("imu_frame", imu_frame_id_, std::string("imu")); 
  // 获取2D磁力计姿态话题名称参数，默认为"/mag_pose_2d"
  pravite_nh.param("mag_pose_2d_topic", mag_pose_2d_topic_, std::string("/mag_pose_2d"));
                                                 
  // 获取串口端口号参数，默认为"/dev/ttyS0"
  pravite_nh.param("port", serial_port_, std::string("/dev/ttyS0")); 
  // 获取串口波特率参数，默认为115200
  pravite_nh.param("baud", serial_baud_, 115200);

  // --- ROS发布器初始化 ---
  // 初始化IMU数据发布器，发布到imu_topic_，队列大小为10
  imu_pub_ = nh_.advertise<sensor_msgs::Imu>(imu_topic_.c_str(), 10);
  // 初始化2D磁力计姿态发布器，发布到mag_pose_2d_topic_，队列大小为10
  mag_pose_pub_ = nh_.advertise<geometry_msgs::Pose2D>(mag_pose_2d_topic_.c_str(), 10);

  // --- 串口设置与打开 ---
  try
  {
    // 设置串口端口
    serial_.setPort(serial_port_);
    // 设置串口波特率
    serial_.setBaudrate(serial_baud_);
    // 设置流控制，无流控制
    serial_.setFlowcontrol(serial::flowcontrol_none);
    // 设置奇偶校验，无奇偶校验
    serial_.setParity(serial::parity_none); 
    // 设置停止位，1位停止位
    serial_.setStopbits(serial::stopbits_one);
    // 设置数据位，8位数据位
    serial_.setBytesize(serial::eightbits);
    // 设置串口读取超时时间
    serial::Timeout time_out = serial::Timeout::simpleTimeout(serial_timeout_);
    serial_.setTimeout(time_out);
    // 打开串口
    serial_.open();
  }
  catch (serial::IOException &e)
  {
    // 捕获串口异常，打印错误信息并退出程序
    ROS_ERROR_STREAM("Unable to open port ");
    exit(0);
  }

  // 检查串口是否成功打开
  if (serial_.isOpen())
  {
    ROS_INFO_STREAM("Serial Port initialized");
  }
  else
  {
    // 如果串口未能打开，打印错误信息并退出程序
    ROS_ERROR_STREAM("Unable to initial Serial port ");
    exit(0);
  }

  // 启动数据处理循环
  processLoop();
}

/**
 * @brief ahrsBringup类的析构函数。
 * 
 * 在 `ahrsBringup` 对象被销毁时调用。此析构函数负责清理资源，
 * 特别是关闭已打开的串口连接，以确保资源正确释放，避免内存泄漏或端口占用。
 */
ahrsBringup::~ahrsBringup()
{
  // 检查串口是否处于打开状态，如果串口已打开，则关闭串口连接。
  if (serial_.isOpen())
    serial_.close();
}

/**
 * @brief 主处理循环函数。
 * 
 * `processLoop` 函数是 `ahrsBringup` 类的核心，它在一个ROS循环中持续运行，
 * 负责从串口读取AHRS设备发送的原始数据，并进行解析、校验和ROS消息发布。
 * 
 * 该函数的主要流程包括：
 * 1. 检查串口是否打开。
 * 2. 循环读取数据，每次尝试读取一个完整的帧。
 * 3. 校验帧头 (`FRAME_HEAD`)，确保数据包的起始正确性。
 * 4. 识别数据类型 (`head_type`)，支持IMU、AHRS、INSGPS等多种数据类型。
 * 5. 校验数据长度 (`check_len`)，确保数据包的完整性。
 * 6. 处理特殊数据类型（如地面站数据），并跳过不相关的数据。
 * 7. 读取并校验帧头中的序列号 (`check_sn`)、CRC8 (`head_crc8`) 和 CRC16 (`head_crc16_H`, `head_crc16_L`)。
 * 8. 根据数据类型填充对应的帧数据结构，并进行CRC8校验。
 * 9. 检查数据包的序列号，统计丢包情况。
 * 10. 读取数据部分，并进行CRC16校验和帧尾 (`FRAME_END`) 校验。
 * 11. 根据解析出的数据类型，发布相应的ROS话题，例如IMU数据和2D磁力计姿态数据。
 * 
 * @note 该函数会持续运行直到ROS节点关闭。在调试模式下 (`if_debug_` 为true)，
 *       会输出详细的调试信息到控制台。
 */
void ahrsBringup::processLoop()
{
  ROS_INFO("ahrsBringup::processLoop: start");
  // 循环，直到ROS关闭
  while (ros::ok())
  {
    // 检查串口是否打开
    if (!serial_.isOpen())
    {
      ROS_WARN("serial unopen");
    }

    // --- 1. 检查帧头 ---
    uint8_t check_head[1] = {0xff};
    // 读取一个字节作为帧头
    size_t head_s = serial_.read(check_head, 1);
    if (if_debug_){
      if (head_s != 1)
      {
        ROS_ERROR("Read serial port time out! can't read pack head.");
      }
      std::cout << std::endl;
      std::cout << "check_head: " << std::hex << (int)check_head[0] << std::dec << std::endl;
    }
    // 如果帧头不正确，则跳过当前循环
    if (check_head[0] != FRAME_HEAD)
    {
      continue;
    }

    // --- 2. 检查数据类型 ---
    uint8_t head_type[1] = {0xff};
    // 读取一个字节作为数据类型
    size_t type_s = serial_.read(head_type, 1);
    if (if_debug_){
      std::cout << "head_type:  " << std::hex << (int)head_type[0] << std::dec << std::endl;
    }
    // 如果数据类型不属于已知类型，则跳过当前循环
    if (head_type[0] != TYPE_IMU && head_type[0] != TYPE_AHRS && head_type[0] != TYPE_INSGPS && head_type[0] != 0x50 && head_type[0] != TYPE_GROUND)
    {
      ROS_WARN("head_type error: %02X",head_type[0]);
      continue;
    }

    // --- 3. 检查数据长度 ---
    uint8_t check_len[1] = {0xff};
    // 读取一个字节作为数据长度
    size_t len_s = serial_.read(check_len, 1);
    if (if_debug_){
      std::cout << "check_len: "<< std::dec << (int)check_len[0]  << std::endl;
    }
    // 根据数据类型检查数据长度是否正确
    if (head_type[0] == TYPE_IMU && check_len[0] != IMU_LEN)
    {
      ROS_WARN("head_len error (imu)");
      continue;
    }else if (head_type[0] == TYPE_AHRS && check_len[0] != AHRS_LEN)
    {
      ROS_WARN("head_len error (ahrs)");
      continue;
    }else if (head_type[0] == TYPE_INSGPS && check_len[0] != INSGPS_LEN)
    {
      ROS_WARN("head_len error (insgps)");
      continue;
    }
    // 处理地面站数据或其他未知数据类型
    else if (head_type[0] == TYPE_GROUND || head_type[0] == 0x50) // 未知数据，防止记录失败
    {
      uint8_t ground_sn[1];
      // 读取序列号
      size_t ground_sn_s = serial_.read(ground_sn, 1);
      // 检查序列号是否连续，并计算丢包数
      if (++read_sn_ != ground_sn[0])
      {
        if ( ground_sn[0] < read_sn_)
        {
          if(if_debug_){
            ROS_WARN("detected sn lost.");
          }
          sn_lost_ += 256 - (int)(read_sn_ - ground_sn[0]);
          read_sn_ = ground_sn[0];
          // continue;
        }
        else
        {
          if(if_debug_){
            ROS_WARN("detected sn lost.");
          }
          sn_lost_ += (int)(ground_sn[0] - read_sn_);
          read_sn_ = ground_sn[0];
          // continue;
        }
      }
      uint8_t ground_ignore[500];
      size_t ground_ignore_s = serial_.read(ground_ignore, (check_len[0]+4));
      continue;
    }

    // --- 4. 读取帧头剩余部分 (序列号, CRC8, CRC16) ---
    uint8_t check_sn[1] = {0xff};
    size_t sn_s = serial_.read(check_sn, 1);
    uint8_t head_crc8[1] = {0xff};
    size_t crc8_s = serial_.read(head_crc8, 1);
    uint8_t head_crc16_H[1] = {0xff};
    uint8_t head_crc16_L[1] = {0xff};
    size_t crc16_H_s = serial_.read(head_crc16_H, 1);
    size_t crc16_L_s = serial_.read(head_crc16_L, 1);

    if (if_debug_){
      std::cout << "check_sn: "     << std::hex << (int)check_sn[0]     << std::dec << std::endl;
      std::cout << "head_crc8: "    << std::hex << (int)head_crc8[0]    << std::dec << std::endl;
      std::cout << "head_crc16_H: " << std::hex << (int)head_crc16_H[0] << std::dec << std::endl;
      std::cout << "head_crc16_L: " << std::hex << (int)head_crc16_L[0] << std::dec << std::endl;
    }

    // --- 5. 填充帧头数据结构，检查CRC8，并统计序列号丢失 ---
    if (head_type[0] == TYPE_IMU)
    {
      // 填充IMU帧头数据
      imu_frame_.frame.header.header_start   = check_head[0];
      imu_frame_.frame.header.data_type      = head_type[0];
      imu_frame_.frame.header.data_size      = check_len[0];
      imu_frame_.frame.header.serial_num     = check_sn[0];
      imu_frame_.frame.header.header_crc8    = head_crc8[0];
      imu_frame_.frame.header.header_crc16_h = head_crc16_H[0];
      imu_frame_.frame.header.header_crc16_l = head_crc16_L[0];
      // 计算并检查CRC8
      uint8_t CRC8 = CRC8_Table(imu_frame_.read_buf.frame_header, 4);
      if (CRC8 != imu_frame_.frame.header.header_crc8)
      {
        ROS_WARN("header_crc8 error");
        continue;
      }
      // 首次读取序列号时进行初始化
      if(!frist_sn_){
        read_sn_  = imu_frame_.frame.header.serial_num - 1;
        frist_sn_ = true;
      }
      // 检查IMU序列号
      ahrsBringup::checkSN(TYPE_IMU);
    }
    else if (head_type[0] == TYPE_AHRS)
    {
      // 填充AHRS帧头数据
      ahrs_frame_.frame.header.header_start   = check_head[0];
      ahrs_frame_.frame.header.data_type      = head_type[0];
      ahrs_frame_.frame.header.data_size      = check_len[0];
      ahrs_frame_.frame.header.serial_num     = check_sn[0];
      ahrs_frame_.frame.header.header_crc8    = head_crc8[0];
      ahrs_frame_.frame.header.header_crc16_h = head_crc16_H[0];
      ahrs_frame_.frame.header.header_crc16_l = head_crc16_L[0];
      // 计算并检查CRC8
      uint8_t CRC8 = CRC8_Table(ahrs_frame_.read_buf.frame_header, 4);
      if (CRC8 != ahrs_frame_.frame.header.header_crc8)
      {
        ROS_WARN("header_crc8 error");
        continue;
      }
      // 首次读取序列号时进行初始化
      if(!frist_sn_){
        read_sn_  = ahrs_frame_.frame.header.serial_num - 1;
        frist_sn_ = true;
      }
      // 检查AHRS序列号
      ahrsBringup::checkSN(TYPE_AHRS);
    }
    else if (head_type[0] == TYPE_INSGPS)
    {
      // 填充INSGPS帧头数据
      insgps_frame_.frame.header.header_start   = check_head[0];
      insgps_frame_.frame.header.data_type      = head_type[0];
      insgps_frame_.frame.header.data_size      = check_len[0];
      insgps_frame_.frame.header.serial_num     = check_sn[0];
      insgps_frame_.frame.header.header_crc8    = head_crc8[0];
      insgps_frame_.frame.header.header_crc16_h = head_crc16_H[0];
      insgps_frame_.frame.header.header_crc16_l = head_crc16_L[0];
      // 计算并检查CRC8
      uint8_t CRC8 = CRC8_Table(insgps_frame_.read_buf.frame_header, 4);
      if (CRC8 != insgps_frame_.frame.header.header_crc8)
      {
        ROS_WARN("header_crc8 error");
        continue;
      }
      else if(if_debug_)
      {
        std::cout << "header_crc8 matched." << std::endl;
      }
      // 检查INSGPS序列号
      ahrsBringup::checkSN(TYPE_INSGPS);
    }
    if (head_type[0] == TYPE_IMU)
    {
      uint16_t head_crc16_l = imu_frame_.frame.header.header_crc16_l;
      uint16_t head_crc16_h = imu_frame_.frame.header.header_crc16_h;
      uint16_t head_crc16 = head_crc16_l + (head_crc16_h << 8);
      size_t data_s = serial_.read(imu_frame_.read_buf.read_msg, (IMU_LEN + 1)); //48+1
      // if (if_debug_){
      //   for (size_t i = 0; i < (IMU_LEN + 1); i++)
      //   {
      //     std::cout << std::hex << (int)imu_frame_.read_buf.read_msg[i] << " ";
      //   }
      //   std::cout << std::dec << std::endl;
      // }
      uint16_t CRC16 = CRC16_Table(imu_frame_.frame.data.data_buff, IMU_LEN);
      if (if_debug_){          
        std::cout << "CRC16:        " << std::hex << (int)CRC16 << std::dec << std::endl;
        std::cout << "head_crc16:   " << std::hex << (int)head_crc16 << std::dec << std::endl;
        std::cout << "head_crc16_h: " << std::hex << (int)head_crc16_h << std::dec << std::endl;
        std::cout << "head_crc16_l: " << std::hex << (int)head_crc16_l << std::dec << std::endl;
        bool if_right = ((int)head_crc16 == (int)CRC16);
        std::cout << "if_right: " << if_right << std::endl;
      }
      
      if (head_crc16 != CRC16)
      {
        ROS_WARN("check crc16 faild(imu).");
        continue;
      }
      else if(imu_frame_.frame.frame_end != FRAME_END)
      {
        ROS_WARN("check frame end.");
        continue;
      }
      
    }
    else if (head_type[0] == TYPE_AHRS)
    {
      uint16_t head_crc16_l = ahrs_frame_.frame.header.header_crc16_l;
      uint16_t head_crc16_h = ahrs_frame_.frame.header.header_crc16_h;
      uint16_t head_crc16 = head_crc16_l + (head_crc16_h << 8);
      size_t data_s = serial_.read(ahrs_frame_.read_buf.read_msg, (AHRS_LEN + 1)); //48+1
      // if (if_debug_){
      //   for (size_t i = 0; i < (AHRS_LEN + 1); i++)
      //   {
      //     std::cout << std::hex << (int)ahrs_frame_.read_buf.read_msg[i] << " ";
      //   }
      //   std::cout << std::dec << std::endl;
      // }
      uint16_t CRC16 = CRC16_Table(ahrs_frame_.frame.data.data_buff, AHRS_LEN);
      if (if_debug_){          
        std::cout << "CRC16:        " << std::hex << (int)CRC16 << std::dec << std::endl;
        std::cout << "head_crc16:   " << std::hex << (int)head_crc16 << std::dec << std::endl;
        std::cout << "head_crc16_h: " << std::hex << (int)head_crc16_h << std::dec << std::endl;
        std::cout << "head_crc16_l: " << std::hex << (int)head_crc16_l << std::dec << std::endl;
        bool if_right = ((int)head_crc16 == (int)CRC16);
        std::cout << "if_right: " << if_right << std::endl;
      }
      
      if (head_crc16 != CRC16)
      {
        ROS_WARN("check crc16 faild(ahrs).");
        continue;
      }
      else if(ahrs_frame_.frame.frame_end != FRAME_END)
      {
        ROS_WARN("check frame end.");
        continue;
      }
    }
    else if (head_type[0] == TYPE_INSGPS)
    {
      uint16_t head_crc16 = insgps_frame_.frame.header.header_crc16_l + ((uint16_t)insgps_frame_.frame.header.header_crc16_h << 8);
      size_t data_s = serial_.read(insgps_frame_.read_buf.read_msg, (INSGPS_LEN + 1)); //48+1
      uint16_t CRC16 = CRC16_Table(insgps_frame_.frame.data.data_buff, INSGPS_LEN);
      if (head_crc16 != CRC16)
      {
        ROS_WARN("check crc16 faild(insgps).");
        continue;
      }
      else if(insgps_frame_.frame.frame_end != FRAME_END)
      {
        ROS_WARN("check frame end.");
        continue;
      }
      
    }

    // publish magyaw topic
    if (head_type[0] == TYPE_AHRS)
    {
      // publish imu topic
      sensor_msgs::Imu imu_data;
      imu_data.header.stamp = ros::Time::now();
      imu_data.header.frame_id = imu_frame_id_.c_str();
      Eigen::Quaterniond q_ahrs(ahrs_frame_.frame.data.data_pack.Qw,
                                ahrs_frame_.frame.data.data_pack.Qx,
                                ahrs_frame_.frame.data.data_pack.Qy,
                                ahrs_frame_.frame.data.data_pack.Qz);
      Eigen::Quaterniond q_r =                          
          Eigen::AngleAxisd( 3.14159, Eigen::Vector3d::UnitZ()) * 
          Eigen::AngleAxisd( 3.14159, Eigen::Vector3d::UnitY()) * 
          Eigen::AngleAxisd( 0.00000, Eigen::Vector3d::UnitX());
      Eigen::Quaterniond q_rr =                          
          Eigen::AngleAxisd( 0.00000, Eigen::Vector3d::UnitZ()) * 
          Eigen::AngleAxisd( 0.00000, Eigen::Vector3d::UnitY()) * 
          Eigen::AngleAxisd( 3.14159, Eigen::Vector3d::UnitX());
      Eigen::Quaterniond q_xiao_rr =
          Eigen::AngleAxisd( 3.14159/2, Eigen::Vector3d::UnitZ()) * 
          Eigen::AngleAxisd( 0.00000, Eigen::Vector3d::UnitY()) * 
          Eigen::AngleAxisd( 3.14159, Eigen::Vector3d::UnitX());
      if (device_type_ == 0)         //未经变换的原始数据
      {
        imu_data.orientation.w = ahrs_frame_.frame.data.data_pack.Qw;
        imu_data.orientation.x = ahrs_frame_.frame.data.data_pack.Qx;
        imu_data.orientation.y = ahrs_frame_.frame.data.data_pack.Qy;
        imu_data.orientation.z = ahrs_frame_.frame.data.data_pack.Qz;
        imu_data.angular_velocity.x = ahrs_frame_.frame.data.data_pack.RollSpeed;
        imu_data.angular_velocity.y = ahrs_frame_.frame.data.data_pack.PitchSpeed;
        imu_data.angular_velocity.z = ahrs_frame_.frame.data.data_pack.HeadingSpeed;
        imu_data.linear_acceleration.x = imu_frame_.frame.data.data_pack.accelerometer_x;
        imu_data.linear_acceleration.y = imu_frame_.frame.data.data_pack.accelerometer_y;
        imu_data.linear_acceleration.z = imu_frame_.frame.data.data_pack.accelerometer_z;
      }
      else if (device_type_ == 1)    //imu单品ROS标准下的坐标变换
      {
        
        Eigen::Quaterniond q_out =  q_r * q_ahrs * q_rr;
        imu_data.orientation.w = q_out.w();
        imu_data.orientation.x = q_out.x();
        imu_data.orientation.y = q_out.y();
        imu_data.orientation.z = q_out.z();
        cout <<"imu_data.orientation.z"<<imu_data.orientation.z<<endl;
        imu_data.angular_velocity.x = ahrs_frame_.frame.data.data_pack.RollSpeed;
        imu_data.angular_velocity.y = -ahrs_frame_.frame.data.data_pack.PitchSpeed;
        imu_data.angular_velocity.z = -ahrs_frame_.frame.data.data_pack.HeadingSpeed;
        imu_data.linear_acceleration.x = -imu_frame_.frame.data.data_pack.accelerometer_x;
        imu_data.linear_acceleration.y = imu_frame_.frame.data.data_pack.accelerometer_y;
        imu_data.linear_acceleration.z = imu_frame_.frame.data.data_pack.accelerometer_z;
      }
      imu_pub_.publish(imu_data);

      Eigen::Quaterniond rpy_q(imu_data.orientation.w,
                               imu_data.orientation.x,
                               imu_data.orientation.y,
                               imu_data.orientation.z);
      geometry_msgs::Pose2D pose_2d;
      double magx, magy, magz, roll, pitch;
      if (device_type_ == 0){        //未经变换的原始数据//
        magx  = imu_frame_.frame.data.data_pack.magnetometer_x;
        magy  = imu_frame_.frame.data.data_pack.magnetometer_y;
        magz  = imu_frame_.frame.data.data_pack.magnetometer_z;
        roll  = ahrs_frame_.frame.data.data_pack.Roll;
        pitch = ahrs_frame_.frame.data.data_pack.Pitch;
      }
      else if (device_type_ == 1){   //小车以及imu单品ROS标准下的坐标变换//
        magx  = -imu_frame_.frame.data.data_pack.magnetometer_x;
        magy  = imu_frame_.frame.data.data_pack.magnetometer_y;
        magz  = imu_frame_.frame.data.data_pack.magnetometer_z;
        
        Eigen::Vector3d EulerAngle = rpy_q.matrix().eulerAngles(2, 1, 0);
        roll  = EulerAngle[2];
        pitch = EulerAngle[1];
      } 
      double magyaw;
      magCalculateYaw(roll, pitch, magyaw, magx, magy, magz);
      pose_2d.theta = magyaw;
      mag_pose_pub_.publish(pose_2d);
    }
  }
}

/**
 * @brief 计算磁航向角（Magnetic Yaw）。
 * 
 * 该函数根据设备的横滚角、俯仰角以及磁力计在X、Y、Z轴上的读数，
 * 计算出磁航向角。此计算通常用于姿态解算中，结合磁场信息来修正航向。
 * 
 * @param roll 设备的横滚角（弧度）。
 * @param pitch 设备的俯仰角（弧度）。
 * @param magyaw 计算得到的磁航向角（弧度），通过引用返回。
 * @param magx 磁力计在X轴上的读数。
 * @param magy 磁力计在Y轴上的读数。
 * @param magz 磁力计在Z轴上的读数。
 * 
 * @return 无。磁航向角通过 `magyaw` 引用参数返回。
 */
void ahrsBringup::magCalculateYaw(double roll, double pitch, double &magyaw, double magx, double magy, double magz)
{
  // 根据横滚角、俯仰角和磁力计读数计算磁航向角
  double temp1 = magy * cos(roll) + magz * sin(roll);
  double temp2 = magx * cos(pitch) + magy * sin(pitch) * sin(roll) - magz * sin(pitch) * cos(roll);
  magyaw = atan2(-temp1, temp2);
  // 将磁航向角调整到0到2*PI的范围内
  if(magyaw < 0)
  {
    magyaw = magyaw + 2 * PI;
  }
}

/**
 * @brief 检查数据帧的序列号，并统计丢包情况。
 * 
 * 该函数用于验证接收到的数据帧的序列号是否连续。如果检测到序列号不连续，
 * 则会计算丢包数量并进行记录。这对于评估数据传输的可靠性非常重要。
 * 
 * @param type 数据帧的类型，可以是 `TYPE_IMU`、`TYPE_AHRS` 或 `TYPE_INSGPS`。
 *             根据不同的类型，函数会检查对应数据帧结构的序列号。
 */
void ahrsBringup::checkSN(int type)
{
  switch (type)
  {
  case TYPE_IMU:
    // 检查IMU数据帧的序列号
    if (++read_sn_ != imu_frame_.frame.header.serial_num)
    {
      // 如果当前序列号小于上一次读取的序列号，说明发生了溢出（从255回到0），需要特殊处理
      if ( imu_frame_.frame.header.serial_num < read_sn_)
      {
        sn_lost_ += 256 - (int)(read_sn_ - imu_frame_.frame.header.serial_num);
        if(if_debug_){
          ROS_WARN("detected sn lost."); // 调试模式下输出丢包警告
        }
      }
      else
      {
        // 正常情况下的序列号不连续，直接计算丢包数量
        sn_lost_ += (int)(imu_frame_.frame.header.serial_num - read_sn_);
        if(if_debug_){
          ROS_WARN("detected sn lost."); // 调试模式下输出丢包警告
        }
      }
    }
    read_sn_ = imu_frame_.frame.header.serial_num; // 更新已读取的序列号
    break;

  case TYPE_AHRS:
    // 检查AHRS数据帧的序列号，逻辑同IMU类型
    if (++read_sn_ != ahrs_frame_.frame.header.serial_num)
    {
      if ( ahrs_frame_.frame.header.serial_num < read_sn_)
      {
        sn_lost_ += 256 - (int)(read_sn_ - ahrs_frame_.frame.header.serial_num);
        if(if_debug_){
          ROS_WARN("detected sn lost.");
        }
      }
      else
      {
        sn_lost_ += (int)(ahrs_frame_.frame.header.serial_num - read_sn_);
        if(if_debug_){
          ROS_WARN("detected sn lost.");
        }
      }
    }
    read_sn_ = ahrs_frame_.frame.header.serial_num;
    break;

  case TYPE_INSGPS:
    // 检查INSGPS数据帧的序列号，逻辑同IMU类型
    if (++read_sn_ != insgps_frame_.frame.header.serial_num)
    {
      if ( insgps_frame_.frame.header.serial_num < read_sn_)
      {
        sn_lost_ += 256 - (int)(read_sn_ - insgps_frame_.frame.header.serial_num);
        if(if_debug_){
          ROS_WARN("detected sn lost.");
        }
      }
      else
      {
        sn_lost_ += (int)(insgps_frame_.frame.header.serial_num - read_sn_);
        if(if_debug_){
          ROS_WARN("detected sn lost.");
        }
      }
    }
    read_sn_ = insgps_frame_.frame.header.serial_num;
    break;

  default:
    break;
  }
}

} //namespace FDILink

/**
 * @brief 主函数。
 * 
 * 这是AHRS ROS驱动程序的入口点。它负责初始化ROS节点，
 * 创建 `ahrsBringup` 类的实例，并启动数据处理循环。
 * 
 * @param argc 命令行参数的数量。
 * @param argv 命令行参数的数组。
 * @return 程序的退出状态码。0表示成功，非0表示错误。
 */
int main(int argc, char **argv)
{
  // 初始化ROS节点，节点名为 "ahrs_bringup"
  ros::init(argc, argv, "ahrs_bringup");
  // 创建ahrsBringup类的实例，这将启动串口通信和数据处理循环
  FDILink::ahrsBringup bp;

  return 0; // 程序正常退出
}
