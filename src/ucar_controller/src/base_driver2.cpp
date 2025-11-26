#include <ucar_controller/base_driver.h>
#include <Eigen/Eigen>
#include <tf/tf.h>

double yaw; // 定义全局变量yaw，可能用于存储机器人当前的偏航角

namespace ucarController
{
/**
 * @brief baseBringup类构造函数
 * @details 该构造函数负责初始化ROS节点句柄、参数、订阅器、发布器、服务以及串口通信。
 * 同时创建并启动数据写入和处理的独立线程，确保系统能够并发地进行数据传输和处理。
 * 初始化过程中会从ROS参数服务器加载各种配置，包括但不限于：
 * - 里程计TF的发布设置
 * - 速度控制、Joystick、里程计和电池状态等ROS话题名称
 * - 串口通信参数（端口、波特率、超时时间）
 * - 机器人运动学参数（编码器分辨率、轮子半径、控制周期、机器人形状参数）
 * - 速度限制
 * - 里程信息文件的路径
 * - IMU和磁力计相关话题及坐标系
 * - 各种状态变量的初始值
 * 在参数加载完成后，会尝试打开串口并进行初始化。如果串口打开失败，程序将退出。
 * 最后，启动ROS的异步Spinning，使ROS回调函数能够并发执行。
 * @param 无
 * @return 无
 */
baseBringup::baseBringup() :x_(0), y_(0), th_(0) // 初始化里程计位置x, y和偏航角th为0
{
  // 创建私有ROS节点句柄，用于从参数服务器获取参数
  ros::NodeHandle pravite_nh("~");

  // 从参数服务器获取各种配置参数，如果未设置则使用默认值
  pravite_nh.param("provide_odom_tf", provide_odom_tf_,true); // 配置是否发布里程计TF（Transform Frame），默认为true
  pravite_nh.param("vel_topic", vel_topic_,std::string("/cmd_vel")); // 配置速度控制话题，例如"/smooth_cmd_vel"，默认为"/cmd_vel"
  
  pravite_nh.param("joy_topic",  joy_topic_, std::string("/joy")); // 配置Joystick（手柄）输入话题，默认为"/joy"
  pravite_nh.param("odom_topic", odom_topic_,std::string("/odom")); // 配置里程计数据发布话题，默认为"/odom"
  pravite_nh.param("battery_topic", battery_topic_,std::string("/battery_state")); // 配置电池状态信息发布话题，默认为"/battery_state"
  
  // 串口通信相关参数配置
  pravite_nh.param("port", port_, std::string("/dev/base_serial_port")); // 串口端口名称，默认为"/dev/base_serial_port"
  pravite_nh.param("baud", baud_, 115200);                // 串口波特率，默认为115200
  pravite_nh.param("serial_timeout", serial_timeout_, 50);// 串口读取超时时间，单位：毫秒，默认为50
  pravite_nh.param("rate", rate_, 20);                    // 主循环频率，单位：赫兹，默认为20
  pravite_nh.param("duration", duration_, 0.01); // 持续时间，可能用于控制周期或延时，默认为0.01
  pravite_nh.param("cmd_timeout", cmd_dt_threshold_, 0.2); // 命令超时阈值，单位：秒，用于判断速度命令是否过期，默认为0.2
  
  // 机器人坐标系相关参数配置
  pravite_nh.param("base_frame", base_frame_, std::string("base_footprint")); // 机器人基座坐标系名称，默认为"base_footprint"
  pravite_nh.param("odom_frame", odom_frame_, std::string("odom")); // 里程计坐标系名称，默认为"odom"

  // 机器人运动学参数配置
  pravite_nh.param("encode_resolution", encode_resolution_, 270);  // 编码器分辨率，每圈的脉冲数，默认为270
  pravite_nh.param("wheel_radius", wheel_radius_, 0.04657);  // 轮子半径，单位：米，默认为0.04657
  pravite_nh.param("period", period_, 50.0); // 控制周期，单位：毫秒，默认为50.0
  pravite_nh.param("base_shape_a", base_shape_a_, 0.2169);  // 机器人形状参数A，可能代表轴距或轮距的一半，默认为0.2169
  pravite_nh.param("base_shape_b", base_shape_b_, 0.0);  // 机器人形状参数B，可能代表轮距或轴距的另一半，默认为0.0

  // 速度限制参数配置
  pravite_nh.param("linear_speed_max",   linear_speed_max_, 3.0);  // 最大线速度，单位：米/秒，默认为3.0
  pravite_nh.param("angular_speed_max", angular_speed_max_, 3.14);// 最大角速度，单位：弧度/秒，默认为3.14
  pravite_nh.setParam("linear_speed_max" ,linear_speed_max_); // 将最大线速度设置到ROS参数服务器
  pravite_nh.setParam("angular_speed_max",angular_speed_max_); // 将最大角速度设置到ROS参数服务器

  // 里程信息文件参数配置
  pravite_nh.param("Mileage_file_name", Mileage_file_name_, std::string("car_Mileage_info.txt"));// 里程信息记录文件名，默认为"car_Mileage_info.txt"
  // 构建里程信息文件的完整路径，位于ucar_controller包的log_info目录下
  Mileage_file_name_        = ros::package::getPath("ucar_controller") + std::string("/log_info/") + Mileage_file_name_; 
  Mileage_backup_file_name_ = Mileage_file_name_ + ".bp"; // 里程信息备份文件路径
  pravite_nh.param("debug_log", debug_log_, true);// 是否开启调试日志输出（如ROS_INFO等），默认为true

  // IMU和磁力计话题参数配置
  pravite_nh.param("imu_topic", imu_topic_, std::string("/imu")); // IMU数据发布话题，默认为"/imu"
  pravite_nh.param("imu_frame", imu_frame_id_, std::string("imu")); // IMU坐标系ID，默认为"imu"
  pravite_nh.param("mag_pose_2d_topic", mag_pose_2d_topic_, std::string("/mag_pose_2d")); // 磁力计2D姿态发布话题，默认为"/mag_pose_2d"
  
  // 内部状态变量初始化
  read_first_ = false; // 标记是否首次成功读取到数据，初始化为false
  imu_frist_sn_ = false; // 标记IMU是否首次获取到序列号，初始化为false
  controll_type_ = MOTOR_MODE_CMD; // 默认控制类型为命令模式（通过/cmd_vel话题控制）
  linear_gain_   = 0.3; // 线速度增益，用于Joystick控制，默认为0.3
  twist_gain_    = 0.7; // 角速度增益，用于Joystick控制，默认为0.7
  linear_speed_min_  = 0; // 最小线速度（当前未使用，保留），初始化为0
  angular_speed_min_ = 0; // 最小角速度（当前未使用，保留），初始化为0
  current_battery_percent_ = -1; // 当前电池百分比，-1表示尚未获取到有效值，初始化为-1
  led_mode_type_   = 0; // LED模式类型，0表示默认模式，初始化为0
  led_frequency_   = 0; // LED闪烁频率，初始化为0
  led_red_value_   = 0; // LED红色分量值，初始化为0
  led_green_value_ = 0; // LED绿色分量值，初始化为0
  led_blue_value_  = 0; // LED蓝色分量值，初始化为0

  // 调用函数获取机器人历史里程信息
  getMileage();

  // ROS发布器（Publisher）初始化
  odom_pub_    = nh_.advertise<nav_msgs::Odometry>(odom_topic_.c_str(),10); // 初始化里程计数据发布器，话题名为odom_topic_，队列大小为10
  battery_pub_ = nh_.advertise<sensor_msgs::BatteryState>(battery_topic_.c_str(),10); // 初始化电池状态发布器，话题名为battery_topic_，队列大小为10
  imu_pub_     = nh_.advertise<sensor_msgs::Imu>(imu_topic_.c_str(), 10); // 初始化IMU数据发布器，话题名为imu_topic_，队列大小为10
  mag_pose_pub_ = nh_.advertise<geometry_msgs::Pose2D>(mag_pose_2d_topic_.c_str(), 10); // 初始化磁力计2D姿态发布器，话题名为mag_pose_2d_topic_，队列大小为10

  // ROS订阅器（Subscriber）初始化
  vel_sub_     = nh_.subscribe<geometry_msgs::Twist>(vel_topic_.c_str(), 1, &baseBringup::velCallback,this); // 订阅速度命令话题，话题名为vel_topic_，队列大小为1，回调函数为velCallback
  joy_sub_     = nh_.subscribe<sensor_msgs::Joy>(joy_topic_.c_str(),     1, &baseBringup::joyCallback,this); // 订阅Joystick话题，话题名为joy_topic_，队列大小为1，回调函数为joyCallback
  
  // ROS服务服务器（Service Server）初始化
  stop_move_server_   = nh_.advertiseService("stop_move", &baseBringup::stopMoveCB, this); // 提供停止移动服务，服务名为"stop_move"，回调函数为stopMoveCB
  set_max_vel_server_ = nh_.advertiseService("set_max_vel", &baseBringup::setMaxVelCB, this); // 提供设置最大速度服务，服务名为"set_max_vel"，回调函数为setMaxVelCB
  get_max_vel_server_ = nh_.advertiseService("get_max_vel", &baseBringup::getMaxVelCB, this); // 提供获取最大速度服务，服务名为"get_max_vel"，回调函数为getMaxVelCB
  get_battery_state_server_ = nh_.advertiseService("get_battery_state", &baseBringup::getBatteryStateCB, this); // 提供获取电池状态服务，服务名为"get_battery_state"，回调函数为getBatteryStateCB
  set_led_server_     = nh_.advertiseService("set_led_light", &baseBringup::setLEDCallBack, this); // 提供设置LED灯服务，服务名为"set_led_light"，回调函数为setLEDCallBack
  
  // 尝试打开串口并配置其参数
  try
  {
    serial_.setPort(port_); // 设置串口端口
    serial_.setBaudrate(baud_); // 设置波特率
    serial_.setFlowcontrol(serial::flowcontrol_none); // 设置流控制为无
    serial_.setParity(serial::parity_none);// 默认无奇偶校验
    serial_.setStopbits(serial::stopbits_one); // 设置停止位为1
    serial_.setBytesize(serial::eightbits); // 设置数据位为8
    serial::Timeout time_out = serial::Timeout::simpleTimeout(serial_timeout_); // 创建简单超时对象
    serial_.setTimeout(time_out); // 应用超时设置
    serial_.open(); // 打开串口
  }
  catch (serial::IOException& e) // 捕获串口I/O异常
  {
    ROS_ERROR_STREAM("无法打开串口: " << e.what()); // 串口打开失败时打印错误信息
    exit(0); // 退出程序
  }
  
  // 检查串口是否成功打开，并打印相应信息
  if(serial_.isOpen()) 
  {
    ROS_INFO_STREAM("串口初始化成功"); // 串口初始化成功信息
  }else{
    ROS_ERROR_STREAM("无法初始化串口"); // 串口初始化失败错误信息
    exit(0); // 退出程序
  } 
  
  // 时间戳初始化，用于里程计计算等
  current_time_ = ros::Time::now(); // 获取当前ROS时间
  last_time_    = ros::Time::now(); // 记录上次时间，初始与当前时间相同
  
  // 再次调用串口设置和打开函数，确保串口状态正确
  setSerial();
  openSerial();
  
  // 创建数据写入和处理的独立线程
  writeThread_   = new boost::thread(boost::bind(&baseBringup::writeLoop,   this)); // 启动写入线程，负责向串口发送数据，绑定writeLoop函数
  processThread_ = new boost::thread(boost::bind(&baseBringup::processLoop, this)); // 启动处理线程，负责从串口读取和处理数据，绑定processLoop函数

  // 启动ROS异步Spinning，允许ROS回调函数在单独的线程中并发执行
  ros::AsyncSpinner spinner(2); // 创建2个线程的异步Spinning
  spinner.start(); // 启动Spinning
  ROS_INFO("ucarController 已准备就绪!"); // 控制器准备就绪信息
  ros::waitForShutdown(); // 阻塞主线程，直到ROS节点关闭
} // baseBringup::baseBringup 构造函数结束


/**
 * @brief 设置串口通信参数
 * @details 该函数用于配置串口的各项通信参数，确保数据能够正确地发送和接收。
 * 它会根据在构造函数中从ROS参数服务器加载的配置来设置串口的波特率、数据位、奇偶校验位、停止位和流控制。
 * 这些设置对于建立稳定可靠的串口连接至关重要。
 * @param 无
 * @return 无
 * @note 此函数通常在串口打开之前或重新配置时调用。
 */
void baseBringup::setSerial()
{
  serial_.setBaudrate(baud_); // 设置串口波特率，从ROS参数获取
  serial_.setBytesize(serial::eightbits); // 设置数据位为8位
  serial_.setParity(serial::parity_none); // 设置无奇偶校验位
  serial_.setStopbits(serial::stopbits_one); // 设置停止位为1位
  serial_.setFlowcontrol(serial::flowcontrol_none); // 设置无流控制
}

/**
 * @brief 打开串口
 * @details 该函数用于尝试打开配置好的串口。如果串口已经处于打开状态，则直接返回。
 * 如果串口未打开，它会尝试执行打开操作。在打开过程中，会捕获 `serial::IOException` 异常，
 * 这通常表示串口无法访问或配置错误。捕获到异常时，会打印错误信息，并递归地尝试重新打开串口，
 * 直到成功打开为止。成功打开后，会打印初始化成功的消息；如果最终未能打开，也会打印失败消息并继续尝试。
 * @param 无
 * @return 无
 * @note 这是一个阻塞函数，会持续尝试打开串口直到成功。在实际应用中，可能需要考虑添加重试次数限制或超时机制。
 */
void baseBringup::openSerial()
{
  if(serial_.isOpen()) return; // 如果串口已打开，则直接返回，避免重复操作
  try
  {
    serial_.open(); // 尝试打开串口
  }
  catch (serial::IOException& e) // 捕获串口I/O异常
  {
    ROS_ERROR_STREAM("无法打开串口: " << e.what()); // 打印串口打开失败的详细错误信息
    // 递归调用自身，持续尝试打开串口。在实际应用中，可能需要加入延时或重试计数，避免无限循环。
    openSerial(); 
  }
  // 检查串口是否成功打开，并打印相应信息
  if(serial_.isOpen()) 
  {
    ROS_INFO_STREAM("串口初始化成功"); // 打印串口初始化成功信息
  }else{
    ROS_ERROR_STREAM("无法初始化串口"); // 打印串口初始化失败错误信息
    // 递归调用自身，持续尝试打开串口。
    openSerial(); 
  }
}

/**
 * @brief 串口数据写入循环函数
 * @details 该函数在一个无限循环中以固定频率向串口发送数据。它主要负责：
 * 1. 根据当前的控制模式（Joystick、速度命令或运动模式）获取线速度和角速度。
 * 2. 对获取到的速度值进行限幅处理，确保不超过最大允许速度。
 * 3. 根据机器人运动学模型计算出每个轮子的脉冲数。
 * 4. 构造包含轮子脉冲数和LED控制信息的串口数据包。
 * 5. 计算数据包的校验和并将其添加到数据包末尾。
 * 6. 通过串口发送完整的数据包。
 * 7. 在发送过程中处理可能发生的异常，并在异常发生时尝试重新打开串口。
 * @param 无
 * @return 无
 */
void baseBringup::writeLoop()
{
  ROS_INFO("baseBringup::writeLoop: start"); // 打印写入循环启动信息
  led_timer = 0; // 初始化LED计时器
  // 设置循环频率，确保数据以固定频率发送
  ros::Rate loop_rate(rate_);
  while(ros::ok()) // 只要ROS节点还在运行就持续循环
  {
    try
    {
      boost::unique_lock<boost::recursive_mutex> lock(Control_mutex_); // 获取互斥锁，保护共享数据
      int cur_controll_type = controll_type_; // 获取当前控制类型
      lock.unlock(); // 释放互斥锁

      double linear_x; // 机器人X方向线速度
      double linear_y; // 机器人Y方向线速度
      double angular_z; // 机器人Z轴角速度

      switch (cur_controll_type)
      {
        case MOTOR_MODE_JOY:{ // Joystick控制模式
          lock.lock(); // 获取互斥锁
          linear_x  = joy_linear_x_; // 从Joystick数据中获取X方向线速度
          linear_y  = joy_linear_y_; // 从Joystick数据中获取Y方向线速度
          angular_z = joy_angular_z_; // 从Joystick数据中获取Z轴角速度
          lock.unlock(); // 释放互斥锁
          break;
        }
        case MOTOR_MODE_CMD:{ // 速度命令控制模式
          lock.lock(); // 获取互斥锁
          double dt = (ros::Time::now() - last_cmd_time_).toSec(); // 计算距离上次接收命令的时间间隔
          if (dt > cmd_dt_threshold_) // 如果命令超时
          {
            linear_x  = 0; // 停止线速度
            linear_y  = 0; // 停止线速度
            angular_z = 0; // 停止角速度
          }        
          else // 命令未超时
          {
            linear_x  = cmd_linear_x_; // 从速度命令中获取X方向线速度
            linear_y  = cmd_linear_y_; // 从速度命令中获取Y方向线速度
            angular_z = cmd_angular_z_; // 从速度命令中获取Z轴角速度
          }
          lock.unlock(); // 释放互斥锁
          break;
        }
        case MOTOR_MODE_MOVE:{ // 运动模式（当前代码中未见具体设置该模式的逻辑，保留）
          lock.lock(); // 获取互斥锁
          linear_x  = move_linear_x_; // 获取运动模式下的X方向线速度
          linear_y  = move_linear_y_; // 获取运动模式下的Y方向线速度
          angular_z = move_angular_z_; // 获取运动模式下的Z轴角速度
          lock.unlock(); // 释放互斥锁
          break;
        }
        default:
          ROS_ERROR("base_driver-writeLoop: controll_type_ error!"); // 未知的控制类型，打印错误信息
          break;
      }
      // 对线速度进行限幅处理，确保不超过最大值和最小值
      if(linear_x > linear_speed_max_)
        linear_x = linear_speed_max_; // 限制X方向线速度不超过最大值
      else if(linear_x < -linear_speed_max_)
        linear_x = -linear_speed_max_; // 限制X方向线速度不低于最小值
      
      if(linear_y > linear_speed_max_)
        linear_y = linear_speed_max_; // 限制Y方向线速度不超过最大值
      else if(linear_y < -linear_speed_max_)
        linear_y = -linear_speed_max_; // 限制Y方向线速度不低于最小值

      // 对角速度进行限幅处理，确保不超过最大值和最小值
      if(angular_z > angular_speed_max_)
        angular_z = angular_speed_max_; // 限制Z轴角速度不超过最大值
      else if(angular_z < -angular_speed_max_)
        angular_z = -angular_speed_max_; // 限制Z轴角速度不低于最小值

      // 根据运动学模型计算四个轮子的角速度 (vw1, vw2, vw3, vw4)
      // 这些公式通常基于麦克纳姆轮或四轮差速驱动机器人的运动学
      double vw1 = linear_x - linear_y - angular_z* (base_shape_a_ + base_shape_b_);
      double vw2 = linear_x + linear_y + angular_z* (base_shape_a_ + base_shape_b_);
      double vw3 = linear_x - linear_y + angular_z* (base_shape_a_ + base_shape_b_);
      double vw4 = linear_x + linear_y - angular_z* (base_shape_a_ + base_shape_b_);
      
      lock.lock(); // 获取互斥锁，保护pack_write_成员
      pack_write_.write_tmp[0] = 0x63; // 设置数据包帧头第一个字节
      pack_write_.write_tmp[1] = 0x75; // 设置数据包帧头第二个字节
      pack_write_.pack.ver     = 0;                 // 0x00  version，数据包版本
      pack_write_.pack.len     = 11; // motor + led，数据包长度
      // pack_write_.pack.sn_num  = write_sn_; // 序列号（注释掉，可能未使用或在其他地方处理）
      // 将线速度转换为轮子脉冲数并赋值给数据包
      pack_write_.pack.data.pluse_w1 = -period_/1000.0*vw1*(encode_resolution_/(2.0*Pi*wheel_radius_));
      pack_write_.pack.data.pluse_w2 =  period_/1000.0*vw2*(encode_resolution_/(2.0*Pi*wheel_radius_));
      pack_write_.pack.data.pluse_w3 = -period_/1000.0*vw4*(encode_resolution_/(2.0*Pi*wheel_radius_));
      pack_write_.pack.data.pluse_w4 =  period_/1000.0*vw3*(encode_resolution_/(2.0*Pi*wheel_radius_));
      
      //LED value
      int cur_led_mode = led_mode_type_; // 获取当前LED模式类型

      led_timer++; // LED计时器递增
      lock.unlock(); // 释放互斥锁

      switch (cur_led_mode) // 根据LED模式类型设置LED颜色值
      {
        // case LED_MODE_NORMAL: // 正常模式（旧的枚举，可能已废弃）
        case ucar_controller::SetLEDMode::Request::MODE_NORMAL : // LED正常模式
        {
          lock.lock(); // 获取互斥锁
          pack_write_.pack.red_value   = (int)led_red_value_;   // 设置红色分量
          pack_write_.pack.green_value = (int)led_green_value_; // 设置绿色分量
          pack_write_.pack.blue_value  = (int)led_blue_value_;  // 设置蓝色分量
          lock.unlock(); // 释放互斥锁
          break;
        }
        case ucar_controller::SetLEDMode::Request::MODE_BLINK: // LED闪烁模式
        {
          double t = (double)led_timer/(double)rate_; // 计算时间（秒）
          lock.lock(); // 获取互斥锁
          // double t = (int)ros::Time::now().toSec(); // 另一种时间计算方式，注释掉
          double f = led_frequency_; // 获取闪烁频率
          int blink = (int)(2.0*t*f)%2; // 计算闪烁状态（0或1）
          pack_write_.pack.red_value   = led_red_value_   * blink; // 根据闪烁状态设置红色分量
          pack_write_.pack.green_value = led_green_value_ * blink; // 根据闪烁状态设置绿色分量
          pack_write_.pack.blue_value  = led_blue_value_  * blink; // 根据闪烁状态设置蓝色分量
          lock.unlock(); // 释放互斥锁
          break;
        }
        case ucar_controller::SetLEDMode::Request::MODE_BREATH: // LED呼吸模式
        {
          double t = ros::Time::now().toSec(); // 获取当前ROS时间（秒）
          lock.lock(); // 获取互斥锁
          // double t = (double)led_timer/(double)rate_; // 另一种时间计算方式，注释掉
          double w = 2 * Pi * led_frequency_; // 计算角频率
          // 根据正弦波设置呼吸效果
          pack_write_.pack.red_value   = 0.5 * (led_red_value_   + led_red_value_   * sin(w * t));
          pack_write_.pack.green_value = 0.5 * (led_green_value_ + led_green_value_ * sin(w * t));
          pack_write_.pack.blue_value  = 0.5 * (led_blue_value_  + led_blue_value_  * sin(w * t));
          lock.unlock(); // 释放互斥锁
          break;
        }
        default:{ // 默认模式（与正常模式相同）
          lock.lock(); // 获取互斥锁
          pack_write_.pack.red_value   = (int)led_red_value_;   // 设置红色分量
          pack_write_.pack.green_value = (int)led_green_value_; // 设置绿色分量
          pack_write_.pack.blue_value  = (int)led_blue_value_;  // 设置蓝色分量
          lock.unlock(); // 释放互斥锁
          break;
        }
      }
      lock.lock(); // 获取互斥锁
      setWriteCS(WRITE_MSG_LONGTH); // 计算并设置数据包的校验和
      size_t pack_write_s = serial_.write(pack_write_.write_tmp,WRITE_MSG_LONGTH); // 通过串口发送数据包
      lock.unlock(); // 释放互斥锁
      if(debug_log_){ // 如果开启调试日志
        cout << "write buf:" << endl; // 打印发送缓冲区内容
        for (size_t i = 0; i < WRITE_MSG_LONGTH; i++)
        {
          cout << std::hex << (int)pack_write_.write_tmp[i] << " "; // 以十六进制格式打印每个字节
        }
        cout << std::dec << endl; // 切换回十进制
      }
      loop_rate.sleep(); // 按照设定的频率休眠
    }
    catch(const std::exception& e) // 捕获标准异常
    {
      ROS_ERROR("AIcarController writeLoop: %s\n", e.what()); // 打印异常信息
      ROS_ERROR("AIcarController writeLoop error, waitfor reopen serial port\n"); // 提示重新打开串口
      setSerial(); // 重新设置串口参数
      openSerial(); // 尝试重新打开串口
    }
    catch(...) // 捕获其他未知异常
    {
      ROS_ERROR("AIcarController writeLoop error, waitfor reopen serial port\n"); // 提示重新打开串口
      setSerial(); // 重新设置串口参数
      openSerial(); // 尝试重新打开串口
    }
  }  
}

/**
 * @brief baseBringup类析构函数
 * @details 该析构函数负责在baseBringup对象销毁时，关闭已打开的串口连接。
 * 这确保了系统资源的正确释放，避免了串口句柄泄露。
 * @param 无
 * @return 无
 */
baseBringup::~baseBringup()
{
  if( serial_.isOpen() )   // 检查串口是否处于打开状态
    serial_.close();       // 如果打开，则关闭串口
}

/**
 * @brief 获取电池状态服务回调函数
 * @details 该函数作为ROS服务回调，用于响应客户端获取机器人当前电池状态的请求。
 * 它会检查`current_battery_percent_`的值来判断是否已获取到有效的电池信息。
 * 如果未获取到（值为-1），则返回UNKNOWN状态；如果已获取到，则返回DISCHARGING状态
 * 和当前的电池百分比。函数使用互斥锁保护对`current_battery_percent_`的访问。
 * @param req `ucar_controller::GetBatteryInfo::Request`类型的请求消息，当前未被使用。
 * @param res `ucar_controller::GetBatteryInfo::Response`类型的响应消息，用于填充电池状态信息。
 * @return `bool` 返回`true`表示服务调用成功。
 */
bool baseBringup::getBatteryStateCB(ucar_controller::GetBatteryInfo::Request &req,
                                    ucar_controller::GetBatteryInfo::Response &res)
{
  // 如果当前电池百分比为-1，表示尚未获取到有效的电池信息
  if (current_battery_percent_ == -1)  
  {
    res.battery_state.power_supply_status     = 0; // 设置电源状态为未知 (UNKNOWN)
    res.battery_state.power_supply_health     = 0; // 设置电源健康状况为未知 (UNKNOWN)
    res.battery_state.power_supply_technology = 0; // 设置电池技术类型为未知 (UNKNOWN)
    // 使用互斥锁保护对共享变量的访问
    boost::unique_lock<boost::recursive_mutex> lock(Control_mutex_); 
    res.battery_state.percentage = current_battery_percent_; // 设置电池百分比为当前值（-1）
    lock.unlock(); // 释放互斥锁
    return true; // 返回成功
  }
  else // 已经获取到有效的电池信息
  {
    res.battery_state.power_supply_status     = 2; // 设置电源状态为正在放电 (DISCHARGING)
    res.battery_state.power_supply_health     = 1; // 设置电源健康状况为良好 (GOOD)
    res.battery_state.power_supply_technology = 0; // 设置电池技术类型为未知 (UNKNOWN)
    res.battery_state.present    = true;           // 标记电池存在
    // 使用互斥锁保护对共享变量的访问
    boost::unique_lock<boost::recursive_mutex> lock(Control_mutex_); 
    res.battery_state.percentage = current_battery_percent_; // 设置电池百分比为当前值
    lock.unlock(); // 释放互斥锁
    return true; // 返回成功
  }
}
// 这部分代码似乎是重复的，且与上方的逻辑有所冲突，通常只保留一个实现
// {
//   res.battery_state.power_supply_status     = 0; // UNKNOWN
//   res.battery_state.power_supply_health     = 0; // UNKNOWN
//   res.battery_state.power_supply_technology = 0; // UNKNOWN
//   boost::unique_lock<boost::recursive_mutex> lock(Control_mutex_); 
//   res.battery_state.percentage = current_battery_percent_;
//   lock.unlock();
//   return true;
// }

/**
 * @brief 设置LED模式服务回调函数
 * @details 该函数作为ROS服务回调，用于响应客户端设置机器人LED灯模式的请求。
 * 它会检查是否已成功连接到底层MCU（通过`read_first_`标志）。
 * 如果未连接，则返回失败信息。如果已连接，则从请求中获取LED的模式类型、
 * 频率以及红、绿、蓝三色值，并更新到内部成员变量，同时重置LED计时器。
 * 最后，返回设置成功的信息。
 * @param req `ucar_controller::SetLEDMode::Request`类型的请求消息，包含LED模式、频率和颜色值。
 * @param res `ucar_controller::SetLEDMode::Response`类型的响应消息，用于指示操作是否成功及返回消息。
 * @return `bool` 返回`true`表示服务调用成功。
 */
bool baseBringup::setLEDCallBack(ucar_controller::SetLEDMode::Request &req, 
                                 ucar_controller::SetLEDMode::Response &res)
{
  // 检查是否已成功连接到底层MCU
  if (!read_first_)
  {
    res.success = false; // 设置操作失败
    res.message = "无法连接到底层MCU，请检查连接."; // 返回错误消息
    return true; // 服务调用成功，但操作失败
  }
  else
  {
    // 使用互斥锁保护对共享LED参数的访问
    boost::unique_lock<boost::recursive_mutex> lock(Control_mutex_); 
    led_mode_type_   = req.mode_type;   // 设置LED模式类型
    led_frequency_   = req.frequency;   // 设置LED闪烁或呼吸频率
    led_red_value_   = req.red_value;   // 设置LED红色分量值
    led_green_value_ = req.green_value; // 设置LED绿色分量值
    led_blue_value_  = req.blue_value;  // 设置LED蓝色分量值
    led_t_0 = ros::Time::now().toSec(); // 记录LED模式设置的起始时间
    led_timer = 0; // 重置LED计时器
    lock.unlock(); // 释放互斥锁
    res.success = true; // 设置操作成功
    res.message = "LED设置成功."; // 返回成功消息
    return true; // 服务调用成功
  }
}


/**
 * @brief 获取最大速度服务回调函数
 * @details 该函数作为ROS服务回调，用于响应客户端获取机器人当前最大线速度和最大角速度的请求。
 * 它会从内部成员变量`linear_speed_max_`和`angular_speed_max_`中读取当前设定的最大速度值，
 * 并将其填充到响应消息中返回给客户端。
 * @param req `ucar_controller::GetMaxVel::Request`类型的请求消息，当前未被使用。
 * @param res `ucar_controller::GetMaxVel::Response`类型的响应消息，用于填充最大线速度和最大角速度。
 * @return `bool` 返回`true`表示服务调用成功。
 */
bool baseBringup::getMaxVelCB(ucar_controller::GetMaxVel::Request &req, ucar_controller::GetMaxVel::Response &res)
{
  // 使用互斥锁保护对共享速度参数的访问
  boost::unique_lock<boost::recursive_mutex> lock(Control_mutex_);
  res.max_linear_velocity  = linear_speed_max_ ; // 获取当前设定的最大线速度
  res.max_angular_velocity = angular_speed_max_; // 获取当前设定的最大角速度
  lock.unlock(); // 释放互斥锁
  return true; // 返回成功
}
/**
 * @brief 设置最大速度服务回调函数
 * @details 该函数作为ROS服务回调，用于响应客户端设置机器人最大线速度和最大角速度的请求。
 * 它会从请求消息中获取新的最大线速度和最大角速度值，并更新到内部成员变量`linear_speed_max_`和`angular_speed_max_`。
 * 同时，将更新后的值也设置到ROS参数服务器，以便其他节点或下次启动时可以获取到这些配置。
 * @param req `ucar_controller::SetMaxVel::Request`类型的请求消息，包含新的最大线速度和最大角速度。
 * @param res `ucar_controller::SetMaxVel::Response`类型的响应消息，用于指示操作是否成功及返回消息。
 * @return `bool` 返回`true`表示服务调用成功。
 */
bool baseBringup::setMaxVelCB(ucar_controller::SetMaxVel::Request &req, ucar_controller::SetMaxVel::Response &res)
{
  // 使用互斥锁保护对共享速度参数的访问
  boost::unique_lock<boost::recursive_mutex> lock(Control_mutex_); 
  linear_speed_max_  = req.max_linear_velocity ; // 更新最大线速度
  angular_speed_max_ = req.max_angular_velocity; // 更新最大角速度
  lock.unlock(); // 释放互斥锁

  // 将更新后的最大速度值设置到ROS参数服务器
  ros::NodeHandle nh_private("~");
  nh_private.setParam("linear_speed_max" ,linear_speed_max_);
  nh_private.setParam("angular_speed_max",angular_speed_max_);

  res.success = true; // 设置操作成功
  res.message = "最大速度设置成功."; // 返回成功消息
  return true; // 服务调用成功
}

/**
 * @brief 停止移动服务回调函数
 * @details 该函数作为ROS服务回调，用于响应客户端停止机器人移动的请求。
 * 它会将机器人的线速度和角速度都设置为0，从而使机器人停止运动。
 * @param req `ucar_controller::StopMove::Request`类型的请求消息，当前未被使用。
 * @param res `ucar_controller::StopMove::Response`类型的响应消息，用于指示操作是否成功及返回消息。
 * @return `bool` 返回`true`表示服务调用成功。
 */
bool baseBringup::stopMoveCB(ucar_controller::StopMove::Request &req, ucar_controller::StopMove::Response &res)
{
  // 使用互斥锁保护对共享速度参数的访问
  boost::unique_lock<boost::recursive_mutex> lock(Control_mutex_);
  linear_speed_ = 0;  // 将线速度设置为0
  angular_speed_ = 0; // 将角速度设置为0
  lock.unlock(); // 释放互斥锁

  res.success = true; // 设置操作成功
  res.message = "停止移动成功."; // 返回成功消息
  return true; // 服务调用成功
}

/**
 * @brief 串口数据处理循环函数
 * @details 该函数在一个无限循环中持续读取和处理来自串口的数据。它负责：
 * 1. 检查串口连接状态，如果未打开则打印错误信息。
 * 2. 循环读取串口数据，通过识别帧头（0x63 0x76 for base data, 0xfc 0x40/0x41 for IMU/AHRS data）
 * 来确定数据包类型。
 * 3. 根据数据包类型读取相应长度的数据。
 * 4. 对读取到的数据进行校验和检查。
 * 5. 如果校验通过，则调用相应的处理函数（如`processBattery`、`processOdometry`、`processIMU`、`processAHRS`）
 * 来解析和发布ROS消息。
 * 6. 处理串口通信过程中可能发生的异常，并在异常发生时尝试重新打开串口。
 * @param 无
 * @return 无
 */
void baseBringup::processLoop()
{
  ROS_INFO("baseBringup::processLoop: 启动串口数据处理循环"); // 打印串口数据处理循环启动信息
  uint8_t check_head_last[1]    = {0xFF}; // 用于存储上一个读取到的字节，以便识别双字节帧头
  uint8_t check_head_current[1] = {0xFF}; // 用于存储当前读取到的字节

  // 主循环，只要ROS节点还在运行就持续处理
  while(ros::ok()){
    // 检查串口是否打开
    if (!serial_.isOpen())
    {
      ROS_ERROR("串口未打开"); // 打印错误信息，但此处不尝试重连，重连逻辑可能在其他地方处理
    }
    try
    {
      int head_type = 0; // 用于标记识别到的帧头类型
      // 循环读取单个字节，直到识别到有效帧头
      while(ros::ok()) 
      {                        
        size_t head_s = serial_.read(check_head_current,1); // 读取一个字节
        // 识别base数据帧头 (0x63 0x76)
        if (check_head_last[0] == 0x63 && check_head_current[0] == 0x76)
        {
          boost::unique_lock<boost::recursive_mutex> lock(Control_mutex_); 
          pack_read_.read_msg.head[0] = check_head_last[0];    // 存储帧头第一个字节
          pack_read_.read_msg.head[1] = check_head_current[0]; // 存储帧头第二个字节
          check_head_last[0] = 0xFF; // 重置上一个字节，避免误识别
          lock.unlock(); // 释放互斥锁
          head_type = 1; // 标记为base数据类型
          break; // 帧头识别成功，跳出内层循环
        }
        // 识别IMU/AHRS/INSGPS/GROUND数据帧头 (0xfc 0x40/0x41/TYPE_INSGPS/TYPE_GROUND/0x50)
        else if (check_head_last[0] == 0xfc && (check_head_current[0] == 0x40 || check_head_current[0] == 0x41 || head_type == TYPE_INSGPS || 
                                                check_head_current[0] == TYPE_GROUND || check_head_current[0] == 0x50))
        {
          boost::unique_lock<boost::recursive_mutex> lock(Control_mutex_);
          if      (check_head_current[0] == 0x40) // IMU数据 (0xfc 0x40)
          {
            imu_frame_.frame.header.header_start = 0xfc; // 设置IMU帧头起始字节
            imu_frame_.frame.header.data_type    = 0x40; // 设置IMU数据类型
            head_type = 0x40; // 标记为IMU数据类型
          }
          else if (check_head_current[0] == 0x41) // AHRS数据 (0xfc 0x41)
          {
            ahrs_frame_.frame.header.header_start = 0xfc; // 设置AHRS帧头起始字节
            ahrs_frame_.frame.header.data_type    = 0x41; // 设置AHRS数据类型
            head_type = 0x41; // 标记为AHRS数据类型
          }
          else if (check_head_current[0] == TYPE_GROUND){ // GROUND数据
            head_type = TYPE_GROUND; // 标记为GROUND数据类型
          }
          else if (check_head_current[0] == 0x50) // 其他类型数据 (0xfc 0x50)
          {
            head_type = 0x50; // 标记为其他数据类型
          }
          check_head_last[0] = 0xFF; // 重置上一个字节
          lock.unlock(); // 释放互斥锁
          if(debug_log_){
            cout << "head_type: " << head_type << endl; // 打印识别到的帧头类型
          }
          break; // 帧头识别成功，跳出内层循环
        }
        check_head_last[0] = check_head_current[0]; // 更新上一个读取到的字节
      }
      if (head_type == 1) // 如果是base数据类型
      {
        size_t res = serial_.read(pack_read_.read_msg.read_msg,READ_DATA_LONGTH); // 读取剩余的数据包内容
        if(debug_log_){
          cout << "serial_read: " <<endl; // 打印读取到的串口数据
          for (size_t i = 0; i < READ_MSG_LONGTH; i++)
          {
            cout << std::hex << (int)pack_read_.read_tmp[i] << " "; // 以十六进制格式打印每个字节
          }
          cout << std::dec << endl; // 切换回十进制
        }
        if(!checkCS(READ_MSG_LONGTH)) // 检查校验和
        {
          ROS_WARN("check cs error"); // 校验和错误，打印警告
        }
        else // 校验和正确
        {
          processBattery(); // 处理电池数据
          processOdometry(); // 处理里程计数据
        }
        if(!read_first_) // 如果是首次成功读取数据
        {
          read_first_ = true; // 标记已首次成功读取数据
        }
      }
      else if (head_type == 0x40 || head_type == 0x41|| head_type == TYPE_GROUND || head_type == 0x50 || head_type == TYPE_INSGPS) // 如果是IMU/AHRS/INSGPS/GROUND数据类型
      {
        processIMU(head_type); // 处理IMU相关数据
      }
      else // 未知帧头类型
      {
        if(debug_log_)
        {
          ROS_DEBUG("head_type ERROR."); // 调试模式下打印帧头错误信息
        }
      }
    }// try end
    catch(const std::exception& e) // 捕获标准异常
    {
      ROS_ERROR("AIcarController readLoop: %s\n", e.what()); // 打印异常信息
      ROS_ERROR("AIcarController readLoop error, try to reopen serial port\n"); // 提示重新打开串口
      setSerial(); // 重新设置串口参数
      openSerial(); // 尝试重新打开串口
    }
    catch(...) // 捕获其他未知异常
    {
      ROS_ERROR("AIcarController readLoop error, try to reopen serial port\n"); // 提示重新打开串口
      setSerial(); // 重新设置串口参数
      openSerial(); // 尝试重新打开串口
    }
  }
}


/**
 * @brief 处理IMU/AHRS/INSGPS等传感器数据
 * @details 该函数负责从串口读取并解析IMU、AHRS、INSGPS等传感器数据包。
 * 它首先读取数据包的长度和序列号等头部信息，并进行CRC8校验。
 * 接着，根据数据类型读取相应长度的数据体，并进行CRC16校验。
 * 如果校验通过，对于AHRS数据，还会将其转换成ROS的`sensor_msgs::Imu`消息并发布，
 * 同时计算磁力计的偏航角并发布`geometry_msgs::Pose2D`消息。
 * 函数还会检查数据包的序列号，以检测是否存在丢包情况。
 * @param head_type `uint8_t` 表示数据包的类型（IMU、AHRS、INSGPS、GROUND等）。
 * @return 无
 */
void baseBringup::processIMU(uint8_t head_type)
{
  uint8_t check_len[1] = {0xff}; // 用于存储数据长度
  size_t len_s = serial_.read(check_len, 1); // 读取数据长度字节
  if (debug_log_){
    std::cout << "check_len: "<< std::dec << (int)check_len[0]  << std::endl; // 打印数据长度（十进制）
  }
  // 检查数据长度是否与预期类型匹配
  if (head_type == TYPE_IMU && check_len[0] != IMU_LEN)
  {
    ROS_WARN("head_len error (imu)"); // IMU数据长度错误
    return;
  }else if (head_type == TYPE_AHRS && check_len[0] != AHRS_LEN)
  {
    ROS_WARN("head_len error (ahrs)"); // AHRS数据长度错误
    return;
  }else if (head_type == TYPE_INSGPS && check_len[0] != INSGPS_LEN)
  {
    ROS_WARN("head_len error (insgps)"); // INSGPS数据长度错误
    return;
  }
  else if (head_type == TYPE_GROUND || head_type == 0x50) // GROUND或未知数据类型，跳过
  {
    uint8_t ground_sn[1]; // 用于存储序列号
    size_t ground_sn_s = serial_.read(ground_sn, 1); // 读取序列号
    if (++read_sn_ != ground_sn[0]) // 检查序列号是否连续
    {
      if ( ground_sn[0] < read_sn_) // 序列号回绕或丢失
      {
        if(debug_log_){
          ROS_WARN("detected sn lost_1."); // 调试模式下打印丢包信息
        }
        sn_lost_ += 256 - (int)(read_sn_ - ground_sn[0]); // 计算丢包数量
        read_sn_ = ground_sn[0]; // 更新当前序列号
      }
      else // 序列号跳跃，可能是丢包
      {
        if(debug_log_){
          ROS_WARN("detected sn lost_2."); // 调试模式下打印丢包信息
        }
        sn_lost_ += (int)(ground_sn[0] - read_sn_); // 计算丢包数量
        read_sn_ = ground_sn[0]; // 更新当前序列号
      }
    }
    uint8_t ground_ignore[500]; // 用于忽略无效数据
    size_t ground_ignore_s = serial_.read(ground_ignore, (check_len[0]+4)); // 读取并忽略剩余数据
    return; // 返回，不处理无效数据
  }
  //read head sn 
  uint8_t check_sn[1] = {0xff}; // 存储序列号
  size_t sn_s = serial_.read(check_sn, 1); // 读取序列号
  uint8_t head_crc8[1] = {0xff}; // 存储头部CRC8
  size_t crc8_s = serial_.read(head_crc8, 1); // 读取头部CRC8
  uint8_t head_crc16_H[1] = {0xff}; // 存储头部CRC16高字节
  uint8_t head_crc16_L[1] = {0xff}; // 存储头部CRC16低字节
  size_t crc16_H_s = serial_.read(head_crc16_H, 1); // 读取头部CRC16高字节
  size_t crc16_L_s = serial_.read(head_crc16_L, 1); // 读取头部CRC16低字节
  if (debug_log_){
    std::cout << "check_sn: "     << std::hex << (int)check_sn[0]     << std::dec << std::endl; // 打印序列号
    std::cout << "head_crc8: "    << std::hex << (int)head_crc8[0]    << std::dec << std::endl; // 打印头部CRC8
    std::cout << "head_crc16_H: " << std::hex << (int)head_crc16_H[0] << std::dec << std::endl; // 打印头部CRC16高字节
    std::cout << "head_crc16_L: " << std::hex << (int)head_crc16_L[0] << std::dec << std::endl; // 打印头部CRC16低字节
  }
  // put header & check crc8 & count sn lost
  if (head_type == TYPE_IMU) // 如果是IMU数据
  {
    imu_frame_.frame.header.data_size      = check_len[0]; // 设置数据长度
    imu_frame_.frame.header.serial_num     = check_sn[0]; // 设置序列号
    imu_frame_.frame.header.header_crc8    = head_crc8[0]; // 设置头部CRC8
    imu_frame_.frame.header.header_crc16_h = head_crc16_H[0]; // 设置头部CRC16高字节
    imu_frame_.frame.header.header_crc16_l = head_crc16_L[0]; // 设置头部CRC16低字节
    uint8_t CRC8 = CRC8_Table(imu_frame_.read_buf.frame_header, 4); // 计算CRC8
    if (CRC8 != imu_frame_.frame.header.header_crc8) // 检查CRC8
    {
      ROS_WARN("header_crc8 error"); // 头部CRC8错误
      return;
    }
    if(!imu_frist_sn_){ // 如果是首次获取IMU序列号
      read_sn_  = imu_frame_.frame.header.serial_num - 1; // 初始化read_sn_
      imu_frist_sn_ = true; // 标记已首次获取IMU序列号
    }
    //check sn 
    baseBringup::checkSN(TYPE_IMU); // 检查序列号
  }
  else if (head_type == TYPE_AHRS) // 如果是AHRS数据
  {
    ahrs_frame_.frame.header.data_size      = check_len[0]; // 设置数据长度
    ahrs_frame_.frame.header.serial_num     = check_sn[0]; // 设置序列号
    ahrs_frame_.frame.header.header_crc8    = head_crc8[0]; // 设置头部CRC8
    ahrs_frame_.frame.header.header_crc16_h = head_crc16_H[0]; // 设置头部CRC16高字节
    ahrs_frame_.frame.header.header_crc16_l = head_crc16_L[0]; // 设置头部CRC16低字节
    uint8_t CRC8 = CRC8_Table(ahrs_frame_.read_buf.frame_header, 4); // 计算CRC8
    if (CRC8 != ahrs_frame_.frame.header.header_crc8) // 检查CRC8
    {
      ROS_WARN("header_crc8 error"); // 头部CRC8错误
      return;
    }
    if(!imu_frist_sn_){ // 如果是首次获取IMU序列号
      read_sn_  = ahrs_frame_.frame.header.serial_num - 1; // 初始化read_sn_
      imu_frist_sn_ = true; // 标记已首次获取IMU序列号
    }
    //check sn 
    baseBringup::checkSN(TYPE_AHRS); // 检查序列号
  }
  else if (head_type == TYPE_INSGPS) // 如果是INSGPS数据
  {
    insgps_frame_.frame.header.header_start   = 0xfc; // 设置帧头起始字节
    insgps_frame_.frame.header.data_type      = TYPE_INSGPS; // 设置数据类型
    insgps_frame_.frame.header.data_size      = check_len[0]; // 设置数据长度
    insgps_frame_.frame.header.serial_num     = check_sn[0]; // 设置序列号
    insgps_frame_.frame.header.header_crc8    = head_crc8[0]; // 设置头部CRC8
    insgps_frame_.frame.header.header_crc16_h = head_crc16_H[0]; // 设置头部CRC16高字节
    insgps_frame_.frame.header.header_crc16_l = head_crc16_L[0]; // 设置头部CRC16低字节
    uint8_t CRC8 = CRC8_Table(insgps_frame_.read_buf.frame_header, 4); // 计算CRC8
    if (CRC8 != insgps_frame_.frame.header.header_crc8) // 检查CRC8
    {
      ROS_WARN("header_crc8 error"); // 头部CRC8错误
      return;
    }
    else if(debug_log_)
    {
      std::cout << "header_crc8 matched." << std::endl; // 调试模式下打印CRC8匹配信息
    }
    
    baseBringup::checkSN(TYPE_INSGPS); // 检查序列号
  }
  if (head_type == TYPE_IMU) // 如果是IMU数据
  {
    uint16_t head_crc16_l = imu_frame_.frame.header.header_crc16_l; // 获取头部CRC16低字节
    uint16_t head_crc16_h = imu_frame_.frame.header.header_crc16_h; // 获取头部CRC16高字节
    uint16_t head_crc16 = head_crc16_l + (head_crc16_h << 8); // 组合成完整的CRC16
    size_t data_s = serial_.read(imu_frame_.read_buf.read_msg, (IMU_LEN + 1)); // 读取数据体和帧尾
    uint16_t CRC16 = CRC16_Table(imu_frame_.frame.data.data_buff, IMU_LEN); // 计算数据体CRC16
    if (debug_log_){          
      std::cout << "CRC16:        " << std::hex << (int)CRC16 << std::dec << std::endl; // 打印计算出的CRC16
      std::cout << "head_crc16:   " << std::hex << (int)head_crc16 << std::dec << std::endl; // 打印头部CRC16
      std::cout << "head_crc16_h: " << std::hex << (int)head_crc16_h << std::dec << std::endl; // 打印头部CRC16高字节
      std::cout << "head_crc16_l: " << std::hex << (int)head_crc16_l << std::dec << std::endl; // 打印头部CRC16低字节
      bool if_right = ((int)head_crc16 == (int)CRC16); // 判断CRC16是否匹配
      std::cout << "if_right: " << if_right << std::endl; // 打印匹配结果
    }
    
    if (head_crc16 != CRC16) // 检查CRC16
    {
      ROS_WARN("check crc16 faild(imu)."); // CRC16校验失败
      return;
    }
    else if(imu_frame_.frame.frame_end != FRAME_END) // 检查帧尾
    {
      ROS_WARN("check frame end."); // 帧尾错误
      return;
    }
    
  }
  else if (head_type == TYPE_AHRS) // 如果是AHRS数据
  {
    uint16_t head_crc16_l = ahrs_frame_.frame.header.header_crc16_l; // 获取头部CRC16低字节
    uint16_t head_crc16_h = ahrs_frame_.frame.header.header_crc16_h; // 获取头部CRC16高字节
    uint16_t head_crc16 = head_crc16_l + (head_crc16_h << 8); // 组合成完整的CRC16
    size_t data_s = serial_.read(ahrs_frame_.read_buf.read_msg, (AHRS_LEN + 1)); // 读取数据体和帧尾
    uint16_t CRC16 = CRC16_Table(ahrs_frame_.frame.data.data_buff, AHRS_LEN); // 计算数据体CRC16
    if (debug_log_){          
      std::cout << "CRC16:        " << std::hex << (int)CRC16 << std::dec << std::endl; // 打印计算出的CRC16
      std::cout << "head_crc16:   " << std::hex << (int)head_crc16 << std::dec << std::endl; // 打印头部CRC16
      std::cout << "head_crc16_h: " << std::hex << (int)head_crc16_h << std::dec << std::endl; // 打印头部CRC16高字节
      std::cout << "head_crc16_l: " << std::hex << (int)head_crc16_l << std::dec << std::endl; // 打印头部CRC16低字节
      bool if_right = ((int)head_crc16 == (int)CRC16); // 判断CRC16是否匹配
      std::cout << "if_right: " << if_right << std::endl; // 打印匹配结果
    }
    
    if (head_crc16 != CRC16) // 检查CRC16
    {
      ROS_WARN("check crc16 faild(ahrs)."); // CRC16校验失败
      return;
    }
    else if(ahrs_frame_.frame.frame_end != FRAME_END) // 检查帧尾
    {
      ROS_WARN("check frame end."); // 帧尾错误
      return;
    }
  }
  else if (head_type == TYPE_INSGPS) // 如果是INSGPS数据
  {
    uint16_t head_crc16 = insgps_frame_.frame.header.header_crc16_l + ((uint16_t)insgps_frame_.frame.header.header_crc16_h << 8); // 组合成完整的CRC16
    size_t data_s = serial_.read(insgps_frame_.read_buf.read_msg, (INSGPS_LEN + 1)); // 读取数据体和帧尾
    uint16_t CRC16 = CRC16_Table(insgps_frame_.frame.data.data_buff, INSGPS_LEN); // 计算数据体CRC16
    if (head_crc16 != CRC16) // 检查CRC16
    {
      ROS_WARN("check crc16 faild(insgps)."); // CRC16校验失败
      return;
    }
    else if(insgps_frame_.frame.frame_end != FRAME_END) // 检查帧尾
    {
      ROS_WARN("check frame end."); // 帧尾错误
      return;
    }
    
  }

  // publish magyaw topic
  if (head_type == TYPE_AHRS) // 如果是AHRS数据，发布IMU和磁力计话题
  {
    // publish imu topic
    sensor_msgs::Imu imu_data; // 创建Imu消息
    imu_data.header.stamp = ros::Time::now(); // 设置时间戳
    imu_data.header.frame_id = imu_frame_id_.c_str(); // 设置帧ID
    // 从AHRS数据中获取四元数，并进行坐标系转换（旋转）
    Eigen::Quaterniond q_ahrs(ahrs_frame_.frame.data.data_pack.Qw,
                              ahrs_frame_.frame.data.data_pack.Qx,
                              ahrs_frame_.frame.data.data_pack.Qy,
                              ahrs_frame_.frame.data.data_pack.Qz);
    // 定义旋转四元数，用于将IMU数据从其传感器坐标系转换到机器人基座坐标系
    Eigen::Quaterniond q_r =                          
        Eigen::AngleAxisd( 3.14159, Eigen::Vector3d::UnitZ()) * // 绕Z轴旋转180度
        Eigen::AngleAxisd( 3.14159, Eigen::Vector3d::UnitY()) * // 绕Y轴旋转180度
        Eigen::AngleAxisd( 0.00000, Eigen::Vector3d::UnitX()); // 绕X轴旋转0度
    Eigen::Quaterniond q_rr =                          
        Eigen::AngleAxisd( 0.00000, Eigen::Vector3d::UnitZ()) * // 绕Z轴旋转0度
        Eigen::AngleAxisd( 0.00000, Eigen::Vector3d::UnitY()) * // 绕Y轴旋转0度
        Eigen::AngleAxisd( 3.14159, Eigen::Vector3d::UnitX()); // 绕X轴旋转180度
    Eigen::Quaterniond q_xiao_rr = // 另一个旋转四元数，可能用于不同配置
        Eigen::AngleAxisd( 3.14159/2, Eigen::Vector3d::UnitZ()) * // 绕Z轴旋转90度
        Eigen::AngleAxisd( 0.00000, Eigen::Vector3d::UnitY()) * // 绕Y轴旋转0度
        Eigen::AngleAxisd( 3.14159, Eigen::Vector3d::UnitX()); // 绕X轴旋转180度
      
    Eigen::Quaterniond q_out =  q_r * q_ahrs * q_rr; // 应用旋转转换
    imu_data.orientation.w = q_out.w(); // 设置Imu消息的四元数W分量
    imu_data.orientation.x = q_out.x(); // 设置Imu消息的四元数X分量
    imu_data.orientation.y = q_out.y(); // 设置Imu消息的四元数Y分量
    imu_data.orientation.z = q_out.z(); // 设置Imu消息的四元数Z分量

    // 设置角速度，并进行坐标系转换（符号反转）
    imu_data.angular_velocity.x = ahrs_frame_.frame.data.data_pack.RollSpeed;
    imu_data.angular_velocity.y = -ahrs_frame_.frame.data.data_pack.PitchSpeed;
    imu_data.angular_velocity.z = -ahrs_frame_.frame.data.data_pack.HeadingSpeed;
    // 设置线加速度，并进行坐标系转换（符号反转）
    imu_data.linear_acceleration.x = -imu_frame_.frame.data.data_pack.accelerometer_x;
    imu_data.linear_acceleration.y = imu_frame_.frame.data.data_pack.accelerometer_y;
    imu_data.linear_acceleration.z = imu_frame_.frame.data.data_pack.accelerometer_z;

    imu_pub_.publish(imu_data); // 发布Imu消息

    Eigen::Quaterniond rpy_q(imu_data.orientation.w, // 创建四元数用于欧拉角转换
                              imu_data.orientation.x,
                              imu_data.orientation.y,
                              imu_data.orientation.z);
    geometry_msgs::Pose2D pose_2d; // 创建Pose2D消息用于发布磁力计姿态
    double magx, magy, magz, roll, pitch;
    // 获取磁力计原始数据，并进行坐标系转换（符号反转）
    magx  = -imu_frame_.frame.data.data_pack.magnetometer_x;
    magy  = imu_frame_.frame.data.data_pack.magnetometer_y;
    magz  = imu_frame_.frame.data.data_pack.magnetometer_z;
    Eigen::Vector3d EulerAngle = rpy_q.matrix().eulerAngles(2, 1, 0); // 将四元数转换为欧拉角（Z-Y-X顺序）
    roll  = EulerAngle[2]; // 获取roll角
    pitch = EulerAngle[1]; // 获取pitch角

    tf::Quaternion tf_quat(imu_data.orientation.x,imu_data.orientation.y,imu_data.orientation.z,imu_data.orientation.w); // 将ROS四元数转换为TF四元数
    double p,r,y;
    tf::Matrix3x3(tf_quat).getRPY(r,p,y); // 从TF四元数获取roll, pitch, yaw

    double magyaw;
    magCalculateYaw(roll, pitch, magyaw, magx, magy, magz); // 计算磁力计偏航角
    pose_2d.theta = magyaw; // 设置Pose2D消息的偏航角
    yaw = y; // 更新全局变量yaw为Imu发布的yaw角
    //printf("yaw:%f\n",yaw/3.14*180); // 调试打印yaw角（注释掉）
    mag_pose_pub_.publish(pose_2d); // 发布磁力计2D姿态消息
  }
}

// void baseBringup::magCalculateYaw(double roll, double pitch, double &magyaw, double magx, double magy, double magz)
// {
//   double temp1 = magy * cos(roll) + magz * sin(roll);
//   double temp2 = magx * cos(pitch) + magy * sin(pitch) * sin(roll) - magz * sin(pitch) * cos(roll);
//   magyaw = atan2(-temp1, temp2);
//   if(magyaw < 0)
//   {
//     magyaw = magyaw + 2 * PI;
//   }
//   // return magyaw;
// }

/**
 * @brief 计算磁力计偏航角
 * @details 该函数根据机器人的roll、pitch角以及磁力计在X、Y、Z轴上的读数，
 * 计算出机器人的偏航角（Yaw）。此方法通常用于补偿倾斜对磁力计读数的影响。
 * 计算结果会被限制在$[-\pi, \pi]$的范围内。
 * @param roll `double` 机器人当前的滚转角（弧度）。
 * @param pitch `double` 机器人当前的俯仰角（弧度）。
 * @param magyaw `double&` 引用参数，用于返回计算出的偏航角（弧度）。
 * @param magx `double` 磁力计在X轴上的读数。
 * @param magy `double` 磁力计在Y轴上的读数。
 * @param magz `double` 磁力计在Z轴上的读数。
 * @return 无
 */
void baseBringup::magCalculateYaw(double roll, double pitch, double &magyaw, double magx, double magy, double magz)
{
    // 计算在水平面投影后的Y分量
    double temp1 = magy * cos(roll) + magz * sin(roll);
    // 计算在水平面投影后的X分量
    double temp2 = magx * cos(pitch) + magy * sin(pitch) * sin(roll) - magz * sin(pitch) * cos(roll);
    // 使用atan2计算偏航角，提供更广的范围和正确的象限
    magyaw = atan2(-temp1, temp2);

    // 将计算出来的yaw角度限制在 -π 到 π 的范围内
    magyaw = fmod(magyaw, 2 * M_PI); // 将角度限制在 [0, 2π) 或 (-2π, 0]
    if (magyaw < -M_PI) { // 如果角度小于 -π，则加上 2π
        magyaw += 2 * M_PI;
    } else if (magyaw > M_PI) { // 如果角度大于 π，则减去 2π
        magyaw -= 2 * M_PI;
    }
}

/**
 * @brief 检查传感器数据包的序列号
 * @details 该函数用于检查接收到的传感器数据包的序列号是否连续。
 * 如果序列号不连续，则会检测到丢包情况，并更新丢包计数`sn_lost_`。
 * 这对于评估传感器数据的完整性和可靠性非常重要。
 * @param type `int` 表示传感器数据包的类型（TYPE_IMU, TYPE_AHRS, TYPE_INSGPS）。
 * @return 无
 */
void baseBringup::checkSN(int type)
{
  switch (type)
  {
  case TYPE_IMU: // IMU数据包
    if (++read_sn_ != imu_frame_.frame.header.serial_num) // 预期的序列号与实际序列号不匹配
    {
      if ( imu_frame_.frame.header.serial_num < read_sn_) // 序列号回绕或丢失
      {
        sn_lost_ += 256 - (int)(read_sn_ - imu_frame_.frame.header.serial_num); // 计算丢包数量（考虑256回绕）
        if(debug_log_){
          ROS_WARN("detected sn lost_3."); // 调试模式下打印丢包信息
        }
      }
      else // 序列号跳跃，可能是丢包
      {
        sn_lost_ += (int)(imu_frame_.frame.header.serial_num - read_sn_); // 计算丢包数量
        if(debug_log_){
          ROS_WARN("detected sn lost_4."); // 调试模式下打印丢包信息
        }
      }
    }
    read_sn_ = imu_frame_.frame.header.serial_num; // 更新当前读取到的序列号
    break;

  case TYPE_AHRS: // AHRS数据包
    if (++read_sn_ != ahrs_frame_.frame.header.serial_num) // 预期的序列号与实际序列号不匹配
    {
      if ( ahrs_frame_.frame.header.serial_num < read_sn_) // 序列号回绕或丢失
      {
        sn_lost_ += 256 - (int)(read_sn_ - ahrs_frame_.frame.header.serial_num); // 计算丢包数量（考虑256回绕）
        if(debug_log_){
          ROS_WARN("detected sn lost_5."); // 调试模式下打印丢包信息
        }
      }
      else // 序列号跳跃，可能是丢包
      {
        sn_lost_ += (int)(ahrs_frame_.frame.header.serial_num - read_sn_); // 计算丢包数量
        if(debug_log_){
          ROS_WARN("detected sn lost_6."); // 调试模式下打印丢包信息
        }
      }
    }
    read_sn_ = ahrs_frame_.frame.header.serial_num; // 更新当前读取到的序列号
    break;

  case TYPE_INSGPS: // INSGPS数据包
    if (++read_sn_ != insgps_frame_.frame.header.serial_num) // 预期的序列号与实际序列号不匹配
    {
      if ( insgps_frame_.frame.header.serial_num < read_sn_) // 序列号回绕或丢失
      {
        sn_lost_ += 256 - (int)(read_sn_ - insgps_frame_.frame.header.serial_num); // 计算丢包数量（考虑256回绕）
        if(debug_log_){
          ROS_WARN("detected sn lost_7."); // 调试模式下打印丢包信息
        }
      }
      else // 序列号跳跃，可能是丢包
      {
        sn_lost_ += (int)(insgps_frame_.frame.header.serial_num - read_sn_); // 计算丢包数量
        if(debug_log_){
          ROS_WARN("detected sn lost_8."); // 调试模式下打印丢包信息
        }
      }
    }
    read_sn_ = insgps_frame_.frame.header.serial_num; // 更新当前读取到的序列号
    break;

  default:
    break;
  }
}

/**
 * @brief 设置写入数据包的校验和
 * @details 该函数计算即将通过串口发送的数据包的校验和（8位累加和），并将其写入数据包的最后一个字节。
 * 校验和用于确保数据在传输过程中的完整性。
 * @param len `int` 数据包的总长度（包括校验和字节）。
 * @return 无
 */
void baseBringup::setWriteCS(int len)
{
	uint8_t ck = 0x00; // 初始化校验和为0
  boost::unique_lock<boost::recursive_mutex> lock(Control_mutex_); // 获取互斥锁，保护pack_write_成员
	for (size_t i = 0; i < len - 1; i++) // 遍历数据包除最后一个字节（校验和字节）之外的所有字节
	{
		ck += pack_write_.write_tmp[i]; // 累加每个字节的值
	}
	pack_write_.write_tmp[len - 1] = ck; // 将计算出的校验和写入数据包的最后一个字节
  lock.unlock(); // 释放互斥锁
}

/**
 * @brief Joystick消息回调函数
 * @details 该函数作为ROS订阅器回调，用于处理从`/joy`话题接收到的Joystick（手柄）输入消息。
 * 它主要负责：
 * 1. 根据Joystick按钮切换机器人的控制模式（从命令模式切换到Joystick模式，反之亦然）。
 * 2. 根据Joystick轴输入调整线速度和角速度的增益。
 * 3. 将Joystick的轴数据转换为机器人的线速度和角速度，并更新内部成员变量。
 * 函数使用互斥锁保护对控制模式和速度变量的访问。
 * @param msg `const sensor_msgs::Joy::ConstPtr&` 指向Joystick消息的常量指针。
 * @return 无
 */
void baseBringup::joyCallback(const sensor_msgs::Joy::ConstPtr& msg){
  if(debug_log_){
    std::cout << "joyCallback" << std::endl; // 调试模式下打印进入joyCallback
  }
  //mode_switch 
  boost::unique_lock<boost::recursive_mutex> lock(Control_mutex_); // 获取互斥锁，保护共享数据
  if (msg->buttons[0]==1)//button 0 pressed, turn off joy mode
  {
    controll_type_ = MOTOR_MODE_CMD; // 切换到命令模式
    std::cout << "controll_type_ turn to MOTOR_MODE_CMD:" << controll_type_ << std::endl; // 打印模式切换信息

  }
  else if (msg->buttons[1] == 1)//button 1 pressed, turn on joy mode
  {
    controll_type_ = MOTOR_MODE_JOY; // 切换到Joystick模式
    std::cout << "controll_type_ turn to MOTOR_MODE_JOY:" << controll_type_ << std::endl; // 打印模式切换信息
  }
  
  //set speed 
  if (msg->axes[6] == 1)        //axes 6 positive, twist_speed down
  {
    if(debug_log_){
      std::cout << "twist_speed down" << std::endl; // 调试模式下打印角速度增益降低
    }
    twist_gain_ -= 0.1; // 降低角速度增益

  }else if(msg->axes[6] == -1)  //axes 6 negative, twist_speed up
  {
    if(debug_log_){
      std::cout << "twist_speed up" << std::endl; // 调试模式下打印角速度增益升高
    }
    twist_gain_ += 0.1; // 提高角速度增益
  }
  if (msg->axes[7] == -1)      //axes 7 negative, linear_speed down
  {
    if(debug_log_){
      std::cout << "linear_speed down" << std::endl; // 调试模式下打印线速度增益降低
    }
    linear_gain_ -= 0.1; // 降低线速度增益
  }else if(msg->axes[7] == 1)  //axes 7 positive, linear_speed up
  {
    if(debug_log_){
      std::cout << "linear_speed up" << std::endl; // 调试模式下打印线速度增益升高
    }
    linear_gain_ += 0.1; // 提高线速度增益
  }
  //write speed
  double linear_x  = linear_gain_ * msg->axes[1];  // linear_gain_ = 0.3, 使用axes[1]控制X方向线速度
  double linear_y  = linear_gain_ * msg->axes[0];  // twist_gain_  = 0.2, 使用axes[0]控制Y方向线速度 
  double angular_z = twist_gain_  * msg->axes[2];  // msg->axes[]  = [-1,1], 使用axes[2]控制Z轴角速度

  if (debug_log_){
    cout << "linear_x=" << linear_x << "linear_y=" << linear_y << "angular_z=" << angular_z << endl; // 调试模式下打印计算出的线速度和角速度
  }  
  joy_linear_x_  =  linear_x; // 更新Joystick模式下的X方向线速度
  joy_linear_y_  =  linear_y; // 更新Joystick模式下的Y方向线速度
  joy_angular_z_ =  angular_z; // 更新Joystick模式下的Z轴角速度
  lock.unlock(); // 释放互斥锁
  return;
}

/**
 * @brief 速度命令消息回调函数
 * @details 该函数作为ROS订阅器回调，用于处理从`/cmd_vel`话题接收到的速度命令消息。
 * 它会检查当前的控制模式是否为命令模式（`MOTOR_MODE_CMD`）。
 * 如果是，则将接收到的线速度和角速度更新到内部成员变量，并记录命令的接收时间。
 * 如果不是命令模式，则忽略此消息，避免冲突。
 * 函数使用互斥锁保护对速度变量和控制模式的访问。
 * @param msg `const geometry_msgs::Twist::ConstPtr&` 指向Twist消息的常量指针，包含线速度和角速度。
 * @return 无
 */
void baseBringup::velCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  boost::unique_lock<boost::recursive_mutex> lock(Control_mutex_); // 获取互斥锁，保护共享数据
  if (controll_type_ != MOTOR_MODE_CMD) // 如果当前控制模式不是命令模式
  {
    lock.unlock(); // 释放互斥锁
    return; // 直接返回，不处理该速度命令
  }
  if(debug_log_){
    std::cout << "vel mode" << std::endl; // 调试模式下打印进入速度命令模式
  }
  cmd_linear_x_  =  msg -> linear.x;  // 更新命令模式下的X方向线速度
  cmd_linear_y_  =  msg -> linear.y;  // 更新命令模式下的Y方向线速度
  cmd_angular_z_ =  msg -> angular.z; // 更新命令模式下的Z轴角速度
  last_cmd_time_ = ros::Time::now();  // 记录最后一次接收到命令的时间
  lock.unlock(); // 释放互斥锁
  return;
}

/**
 * @brief 检查接收数据包的校验和
 * @details 该函数计算接收到的串口数据包的校验和（8位累加和），并将其与数据包中提供的校验和进行比较。
 * 用于验证接收数据的完整性和准确性。
 * @param len `int` 接收数据包的总长度（包括校验和字节）。
 * @return `bool` 如果计算的校验和与接收的校验和匹配，则返回`true`；否则返回`false`。
 */
bool baseBringup::checkCS(int len){
  uint8_t ck = 0; // 初始化校验和为0
	for (size_t i = 0; i < len - 1; i++) // 遍历数据包除最后一个字节（校验和字节）之外的所有字节
	{
		ck += pack_read_.read_tmp[i]; // 累加每个字节的值
	}
  return pack_read_.read_tmp[len - 1] == ck; // 比较计算出的校验和与接收到的校验和
  // return true; // 调试时可能直接返回true，跳过校验
}

/**
 * @brief 处理电池状态数据
 * @details 该函数从接收到的串口数据包中解析电池百分比信息，并更新内部成员变量`current_battery_percent_`。
 * 随后，它将电池状态封装成ROS的`sensor_msgs::BatteryState`消息，并发布到`/battery_state`话题。
 * 消息中包含了电源状态、健康状况、技术类型和电池是否存在等信息。
 * @param 无
 * @return 无
 */
void baseBringup::processBattery()
{
  boost::unique_lock<boost::recursive_mutex> lock(Control_mutex_); // 获取互斥锁，保护current_battery_percent_成员
  current_battery_percent_ = (int)pack_read_.pack.battery_percent; // 从数据包中获取电池百分比并转换为整型
  lock.unlock(); // 释放互斥锁
  sensor_msgs::BatteryState battery_msg; // 创建BatteryState消息
  battery_msg.power_supply_status     = 2; // 设置电源状态为放电中 (DISCHARGING)
  battery_msg.power_supply_health     = 1; // 设置电源健康状况为良好 (GOOD)
  battery_msg.power_supply_technology = 0; // 设置电池技术类型为未知 (UNKNOWN)
  battery_msg.present    = true;           // 标记电池存在
  battery_msg.percentage = current_battery_percent_; // 设置电池电量百分比
  battery_pub_.publish(battery_msg); // 发布电池状态消息
}

/**
 * @brief 处理里程计数据
 * @details 该函数从接收到的串口数据包中解析轮子脉冲数据，并结合机器人运动学参数，
 * 计算出机器人的线速度和角速度。接着，它通过积分计算机器人的当前位姿（x, y, th），
 * 并将这些信息封装成ROS的`nav_msgs::Odometry`消息发布。
 * 同时，如果`provide_odom_tf_`为true，还会发布里程计到机器人基座的TF变换。
 * 最后，函数会更新机器人的总里程信息。
 * @param 无
 * @return 无
 */
void baseBringup::processOdometry(){
  boost::unique_lock<boost::recursive_mutex> lock(Control_mutex_); // 获取互斥锁，保护里程计相关成员
  current_time_ = ros::Time::now(); // 获取当前ROS时间
  double dt = (current_time_ - last_time_).toSec(); // 计算时间间隔
  last_time_ = current_time_; // 更新上次时间
  lock.unlock(); // 释放互斥锁
  double vw1,vw2,vw3,vw4;
  // 将轮子脉冲数转换为轮子线速度。这里使用了编码器分辨率、轮子半径和控制周期来计算每个轮子的实际速度。
  // 注意，脉冲数前面的负号或正号取决于轮子的安装方向和编码器计数方向。
  vw1 =-pack_read_.pack.data.pluse_w1 * 2 * Pi * wheel_radius_ / (encode_resolution_ * period_/1000.0);
  vw2 = pack_read_.pack.data.pluse_w2 * 2 * Pi * wheel_radius_ / (encode_resolution_ * period_/1000.0);
  vw4 =-pack_read_.pack.data.pluse_w3 * 2 * Pi * wheel_radius_ / (encode_resolution_ * period_/1000.0);
  vw3 = pack_read_.pack.data.pluse_w4 * 2 * Pi * wheel_radius_ / (encode_resolution_ * period_/1000.0);

  double Vx,Vy,Vth;
  // 根据四个轮子的线速度计算机器人整体的线速度（Vx, Vy）和角速度（Vth）。
  // 这些公式通常是基于麦克纳姆轮或四轮差速驱动机器人的运动学逆解。
  Vx  = ( vw1+vw2+vw3+vw4)/4; // 计算机器人前进方向（X轴）的线速度，通常是所有轮子速度的平均值。
  //cout << "VX="<< Vx <<endl; // 调试打印Vx（注释掉）
  Vy  = 0.975*(-vw1+vw2-vw3+vw4)/4; // 计算机器人侧向（Y轴）的线速度。0.975可能是一个经验修正系数。
  Vth = (-vw1+vw2+vw3-vw4)/(4*(base_shape_a_+base_shape_b_)); // 计算机器人绕Z轴的角速度。base_shape_a_和base_shape_b_是机器人形状参数，例如轮距或轴距的一半。

  // 计算在dt时间间隔内的位姿增量
  double delta_x = (Vx * cos(th_) - Vy * sin(th_)) * dt; // 计算X方向的位移增量
  double delta_y = (Vx * sin(th_) + Vy * cos(th_)) * dt; // 计算Y方向的位移增量
  double delta_th = Vth * dt; // 计算偏航角的增量
  lock.lock(); // 再次获取互斥锁，保护位姿更新
  x_  += delta_x; // 更新机器人X坐标
  y_  += delta_y; // 更新机器人Y坐标
  th_ += delta_th; // 更新机器人偏航角
  // printf("th_:%f\n",th_/3.14*180); // 调试打印th_（注释掉）
  // printf("yaw:%f\n",yaw/3.14*180); // 调试打印yaw（注释掉）
  lock.unlock(); // 释放互斥锁
  nav_msgs::Odometry odom_tmp; // 创建一个nav_msgs::Odometry消息对象
  odom_tmp.header.stamp = ros::Time::now(); // 设置里程计消息的时间戳为当前ROS时间
  odom_tmp.header.frame_id = odom_frame_.c_str(); // 设置里程计坐标系（父坐标系）的ID
  odom_tmp.child_frame_id  = base_frame_.c_str(); // 设置机器人基座坐标系（子坐标系）的ID
  odom_tmp.pose.pose.position.x = x_; // 设置机器人在里程计坐标系下的X坐标
  odom_tmp.pose.pose.position.y = y_; // 设置机器人在里程计坐标系下的Y坐标
  odom_tmp.pose.pose.position.z = 0.0; // 设置机器人在里程计坐标系下的Z坐标（通常为0，因为是2D平面运动）
  //geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th_); // 使用th_（积分得到的偏航角）创建四元数，但这里被注释掉了
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(yaw); // 使用全局变量yaw（通常来自IMU）创建表示偏航角的四元数
  odom_tmp.pose.pose.orientation = odom_quat; // 设置机器人在里程计坐标系下的姿态四元数
  if (!Vx||!Vy||!Vth){ // 如果机器人速度为零（静止状态）
    odom_tmp.pose.covariance = ODOM_POSE_COVARIANCE; // 设置静止状态下的位姿协方差
  }else{ // 如果机器人正在运动
    odom_tmp.pose.covariance = ODOM_POSE_COVARIANCE2; // 设置运动状态下的位姿协方差
  }
  odom_tmp.twist.twist.linear.x  = Vx; // 设置机器人X方向的线速度
  odom_tmp.twist.twist.linear.y  = Vy; // 设置机器人Y方向的线速度
  odom_tmp.twist.twist.linear.z  = 0.0; // 设置机器人Z方向的线速度（通常为0）
  odom_tmp.twist.twist.angular.x = 0.0; // 设置机器人绕X轴的角速度（通常为0）
  odom_tmp.twist.twist.angular.y = 0.0; // 设置机器人绕Y轴的角速度（通常为0）
  odom_tmp.twist.twist.angular.z = Vth; // 设置机器人绕Z轴的角速度

  //printf("Vth:%f\n",Vth); // 调试打印Vth（注释掉）
  
  if (!Vx||!Vy||!Vth){ // 如果机器人速度为零（静止状态）
    odom_tmp.twist.covariance = ODOM_TWIST_COVARIANCE; // 设置静止状态下的速度协方差
  }else{ // 如果机器人正在运动
    odom_tmp.twist.covariance = ODOM_TWIST_COVARIANCE2; // 设置运动状态下的速度协方差
  }
  odom_pub_.publish(odom_tmp); // 发布里程计消息到ROS话题
  
  lock.lock(); // 获取互斥锁
  current_odom_ = odom_tmp; // 更新当前里程计数据（供其他部分使用）
  lock.unlock(); // 释放互斥锁

  // 检查是否应该发布TF变换
  if (ros::param::has("publish_odom_tf")) // 检查ROS参数服务器中是否存在"publish_odom_tf"参数
  {
    ros::param::get("publish_odom_tf", provide_odom_tf_); // 从参数服务器获取"publish_odom_tf"参数的值，更新provide_odom_tf_成员变量
  }
  
  if(provide_odom_tf_) // 如果配置为发布TF变换
  {
    geometry_msgs::TransformStamped odom_trans;     /* first, we'll publish the transform over tf */ // 创建一个TransformStamped消息对象，用于TF变换
    odom_trans.header.stamp = ros::Time::now(); // 设置TF变换的时间戳为当前ROS时间
    odom_trans.header.frame_id = odom_frame_.c_str(); // 设置TF变换的父坐标系ID（里程计坐标系）
    odom_trans.child_frame_id  = base_frame_.c_str(); // 设置TF变换的子坐标系ID（机器人基座坐标系）
    odom_trans.transform.translation.x = x_; // 设置X方向平移量
    odom_trans.transform.translation.y = y_; // 设置Y方向平移量
    odom_trans.transform.translation.z = 0.0; // 设置Z方向平移量（2D运动通常为0）
    odom_trans.transform.rotation = odom_quat; // 设置旋转四元数
    odom_broadcaster_.sendTransform(odom_trans);    /* send the transform */ // 发送里程计到机器人基座的TF变换
  }
  updateMileage(odom_tmp.twist.twist.linear.x,odom_tmp.twist.twist.linear.y,dt); // 更新总里程信息，传入线速度和时间间隔
}

/**
 * @brief 获取历史里程信息
 * @details 该函数尝试从预设的文件（`Mileage_file_name_`或其备份文件`Mileage_backup_file_name_`）
 * 中读取机器人历史的总里程信息。如果文件不存在或为空，则将里程计总和初始化为0。
 * 读取到的里程信息会存储在`Mileage_sum_`成员变量中，并同时设置到ROS参数服务器。
 * @param 无
 * @return `bool` 如果成功读取到里程信息，返回`true`；否则返回`false`。
 */
bool baseBringup::getMileage(){
  std::fstream fin; // 文件输入流对象
  std::fstream fin_b; // 备份文件输入流对象
  std::string str_in; // 用于存储从主文件读取的里程字符串
  std::string str_in_b; // 用于存储从备份文件读取的里程字符串
  std::stringstream ss; // 字符串流对象，用于字符串和数值之间的转换
  ros::NodeHandle pravite_nh("~"); // 私有ROS节点句柄，用于访问ROS参数服务器
  
	fin.open(Mileage_file_name_.c_str()); // 尝试打开主里程信息文件（以C字符串形式传入文件名）
  fin_b.open(Mileage_backup_file_name_.c_str()); // 尝试打开备份里程信息文件
  if (fin.fail() && fin_b.fail()) // 如果主文件和备份文件都无法打开（例如文件不存在）
  {
    ROS_ERROR("open Mileage files error, will creat a new file! \n"); // 打印错误信息，提示将创建新文件
    Mileage_sum_ = 0.0; // 初始化里程总和为0
    pravite_nh.setParam("Mileage_sum",Mileage_sum_); // 将里程总和设置到ROS参数服务器
    return false; // 返回false表示获取里程失败
  }
  if (!fin.fail()){ // 如果主里程信息文件成功打开
    fin >> str_in; // 从主文件中读取一行字符串（期望是里程值）
    fin.close(); // 关闭主文件
  }
  if (!fin_b.fail()){ // 如果备份里程信息文件成功打开
      fin_b >> str_in_b; // 从备份文件中读取一行字符串
      fin_b.close(); // 关闭备份文件
  }
  if (str_in != "") // 如果从主文件读取到了内容（字符串不为空）
  {
    ss << str_in; // 将读取到的字符串放入字符串流
    ss >> Mileage_sum_; // 从字符串流中提取数值，赋值给Mileage_sum_
    pravite_nh.setParam("Mileage_sum",Mileage_sum_); // 将获取到的里程总和设置到ROS参数服务器
    ss.clear(); // 清空字符串流的状态标志和内容，以便下次使用
  }
  else if (str_in_b != "") // 如果主文件为空，但备份文件不为空
  {
    ss << str_in_b; // 将备份文件中的字符串放入字符串流
    ss >> Mileage_sum_; // 从字符串流中提取数值，赋值给Mileage_sum_
    ss.clear(); // 清空字符串流
    pravite_nh.setParam("Mileage_sum",Mileage_sum_); // 将获取到的里程总和设置到ROS参数服务器
  }
  else // 如果主文件和备份文件都为空
  {
    ROS_ERROR("Mileage_files empty. \n"); // 打印错误信息，提示里程文件为空
    Mileage_sum_ = 0.0; // 初始化里程总和为0
    pravite_nh.setParam("Mileage_sum",Mileage_sum_); // 将里程总和设置到ROS参数服务器
  }
  return true; // 返回true表示尝试获取里程已完成
}

/**
 * @brief 更新总里程信息
 * @details 该函数根据机器人的线速度（Vx, Vy）和时间间隔（dt）计算出在这段时间内的行驶距离，
 * 并将其累加到总里程`Mileage_sum_`中。为了避免频繁写入文件，它会检查与上次写入里程的差值。
 * 如果累积里程超过一定阈值（0.1米），则将更新后的总里程写入主里程信息文件和备份文件中，
 * 并更新ROS参数服务器中的里程值。
 * @param vx `double` 机器人X方向的线速度。
 * @param vy `double` 机器人Y方向的线速度。
 * @param dt `double` 时间间隔（秒）。
 * @return `bool` 总是返回`true`，表示尝试更新操作已完成。
 */
bool baseBringup::updateMileage(double vx, double vy, double dt){
  double speed  = sqrt(vx*vx + vy*vy); // 计算机器人的合速度（欧几里得范数）
  boost::unique_lock<boost::recursive_mutex> lock(Control_mutex_); // 获取互斥锁，保护Mileage_sum_成员
  Mileage_sum_ += speed * dt; // 将速度乘以时间间隔得到行驶距离，累加到总里程
  lock.unlock(); // 释放互斥锁
  double d_Mileage = abs(Mileage_sum_ - Mileage_last_); // 计算当前总里程与上次记录里程之间的差值
  if (d_Mileage < 0.1) // 如果差值小于0.1米（阈值），则认为不需要立即写入文件
  {
    return true; // 返回true，表示更新操作已处理（但不一定写入了文件）
  }
  FILE* fout = std::fopen(Mileage_file_name_.c_str(), "w"); // 以写入模式打开主里程信息文件。如果文件不存在，会创建新文件；如果存在，会清空内容。
  if (fout) // 如果文件成功打开
	{
		std::fprintf(fout,"%lf\n",Mileage_sum_); // 将格式化后的总里程写入文件
		std::fclose(fout); // 关闭文件
	}
  FILE* fout_b = std::fopen(Mileage_backup_file_name_.c_str(), "w"); // 以写入模式打开备份里程信息文件
  if (fout_b) // 如果备份文件成功打开
	{
		std::fprintf(fout_b,"%lf\n",Mileage_sum_); // 将格式化后的总里程写入备份文件
		std::fclose(fout_b); // 关闭备份文件
	}
  ros::NodeHandle pravite_nh("~"); // 私有ROS节点句柄
  pravite_nh.setParam("Mileage_sum",Mileage_sum_); // 将更新后的里程总和设置到ROS参数服务器，供其他ROS节点查询
  Mileage_last_ = Mileage_sum_; // 更新上次记录的里程值，以便下次比较
  return true; // 返回true，表示更新操作已完成并可能写入了文件
}

/**
 * @brief 调用串口打开函数
 * @details 该函数简单地调用`serial_.open()`来尝试打开串口。
 * 这是一个辅助函数，可能在需要手动触发串口重新连接或初始化时使用。
 * @param 无
 * @return 无
 */
void baseBringup::callHandle()
{
  serial_.open(); // 尝试打开串口
}

}//namespace ucarController (命名空间ucarController的结束)

/**
 * @brief 主函数
 * @details 这是ROS节点的主入口点。它负责：
 * 1. 初始化ROS系统。
 * 2. 创建`ucarController::baseBringup`类的实例，该实例将初始化机器人控制器、
 * 设置ROS通信、启动串口读写线程等。
 * 3. 程序会阻塞在这里，直到ROS节点被关闭。
 * @param argc `int` 命令行参数计数。
 * @param argv `char**` 命令行参数数组。
 * @return `int` 程序退出码。
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "base_bringup"); // 初始化ROS节点，将其命名为"base_bringup"。这是所有ROS程序的必备步骤。
  ucarController::baseBringup bp; // 创建ucarController::baseBringup类的一个实例。这将触发该类的构造函数，从而完成所有的初始化工作（包括参数加载、ROS发布/订阅/服务设置、串口初始化和线程启动）。

  return 0; // 程序正常退出，通常在ros::waitForShutdown()被调用且ROS节点关闭后才会真正达到这里。
}
