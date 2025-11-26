#include <ucar_controller/base_driver.h>
#include <Eigen/Eigen>
namespace ucarController
{
baseBringup::baseBringup() :x_(0), y_(0), th_(0)
{
  ros::NodeHandle pravite_nh("~");
  pravite_nh.param("provide_odom_tf", provide_odom_tf_,false);
  pravite_nh.param("vel_topic", vel_topic_,std::string("/cmd_vel"));///smooth_cmd_vel
  
  pravite_nh.param("joy_topic",  joy_topic_, std::string("/joy"));
  pravite_nh.param("odom_topic", odom_topic_,std::string("/odom"));
  pravite_nh.param("battery_topic", battery_topic_,std::string("/battery_state"));
  //serial
  pravite_nh.param("port", port_, std::string("/dev/base_serial_port"));
  pravite_nh.param("baud", baud_, 115200);                
  pravite_nh.param("serial_timeout", serial_timeout_, 50);//ms
  pravite_nh.param("rate", rate_, 20);                    //hz
  pravite_nh.param("duration", duration_, 0.01);
  pravite_nh.param("cmd_timeout", cmd_dt_threshold_, 0.2);
  
  pravite_nh.param("base_frame", base_frame_, std::string("base_footprint"));
  pravite_nh.param("odom_frame", odom_frame_, std::string("odom"));

  pravite_nh.param("encode_resolution", encode_resolution_, 270);  //   
  pravite_nh.param("wheel_radius", wheel_radius_, 0.04657);  //   m
  pravite_nh.param("period", period_, 50.0); //ms
  pravite_nh.param("base_shape_a", base_shape_a_, 0.2169);  //   m
  pravite_nh.param("base_shape_b", base_shape_b_, 0.0);  //   m

  pravite_nh.param("linear_speed_max",   linear_speed_max_, 3.0);  //   m/s
  pravite_nh.param("angular_speed_max", angular_speed_max_, 3.14);// rad/s
  pravite_nh.setParam("linear_speed_max" ,linear_speed_max_);
  pravite_nh.setParam("angular_speed_max",angular_speed_max_);

  pravite_nh.param("Mileage_file_name", Mileage_file_name_, std::string("car_Mileage_info.txt"));//
  Mileage_file_name_        = ros::package::getPath("ucar_controller") + std::string("/log_info/") + Mileage_file_name_;
  Mileage_backup_file_name_ = Mileage_file_name_ + ".bp";
  pravite_nh.param("debug_log", debug_log_, true);//  true rosinfo 等等  打印log数据

  pravite_nh.param("imu_topic", imu_topic_, std::string("/imu"));
  pravite_nh.param("imu_frame", imu_frame_id_, std::string("imu")); 
  pravite_nh.param("mag_pose_2d_topic", mag_pose_2d_topic_, std::string("/mag_pose_2d"));
  read_first_ = false;
  imu_frist_sn_ = false;
  controll_type_ = MOTOR_MODE_CMD; // 1:vel_mode 0:joy_node
  linear_gain_   = 0.3;
  twist_gain_    = 0.7;
  linear_speed_min_  = 0;
  angular_speed_min_ = 0;
  current_battery_percent_ = -1;
  led_mode_type_   = 0;
  led_frequency_   = 0;
  led_red_value_   = 0;
  led_green_value_ = 0;
  led_blue_value_  = 0;

  getMileage();

  odom_pub_    = nh_.advertise<nav_msgs::Odometry>(odom_topic_.c_str(),10);
  battery_pub_ = nh_.advertise<sensor_msgs::BatteryState>(battery_topic_.c_str(),10);
  vel_sub_     = nh_.subscribe<geometry_msgs::Twist>(vel_topic_.c_str(), 1, &baseBringup::velCallback,this);
  joy_sub_     = nh_.subscribe<sensor_msgs::Joy>(joy_topic_.c_str(),     1, &baseBringup::joyCallback,this);
  
  stop_move_server_   = nh_.advertiseService("stop_move", &baseBringup::stopMoveCB, this);
  set_max_vel_server_ = nh_.advertiseService("set_max_vel", &baseBringup::setMaxVelCB, this);
  get_max_vel_server_ = nh_.advertiseService("get_max_vel", &baseBringup::getMaxVelCB, this);
  get_battery_state_server_ = nh_.advertiseService("get_battery_state", &baseBringup::getBatteryStateCB, this);
  set_led_server_     = nh_.advertiseService("set_led_light", &baseBringup::setLEDCallBack, this);
  
  try
  {
    serial_.setPort(port_); 
    serial_.setBaudrate(baud_); 
    serial_.setFlowcontrol(serial::flowcontrol_none);
    serial_.setParity(serial::parity_none);//default is parity_none
    serial_.setStopbits(serial::stopbits_one);
    serial_.setBytesize(serial::eightbits);
    serial::Timeout time_out = serial::Timeout::simpleTimeout(serial_timeout_); 
    serial_.setTimeout(time_out); 
    serial_.open(); 
  }

/**
 * @brief 获取里程信息。
 * 
 * 该函数用于从文件中读取机器人当前的里程信息（总里程和单次里程）。
 * 如果文件不存在或读取失败，则初始化里程为0。
 * 
 * @param 无。
 * @return 无。
 */
void baseBringup::getMileage()
{
}

  catch (serial::IOException& e)
  {
    ROS_ERROR_STREAM("Unable to open port "); 
    exit(0); 
  }

/**
 * @brief 获取里程信息。
 * 
 * 该函数用于从文件中读取机器人当前的里程信息（总里程和单次里程）。
 * 如果文件不存在或读取失败，则初始化里程为0。
 * 
 * @param 无。
 * @return 无。
 */
void baseBringup::getMileage()
{
}

  if(serial_.isOpen()) 
  { 
    ROS_INFO_STREAM("Serial Port initialized"); 
  }

/**
 * @brief 获取里程信息。
 * 
 * 该函数用于从文件中读取机器人当前的里程信息（总里程和单次里程）。
 * 如果文件不存在或读取失败，则初始化里程为0。
 * 
 * @param 无。
 * @return 无。
 */
void baseBringup::getMileage()
{
}
else{ 
    ROS_ERROR_STREAM("Unable to initial Serial port "); 
    exit(0);
  }

/**
 * @brief 获取里程信息。
 * 
 * 该函数用于从文件中读取机器人当前的里程信息（总里程和单次里程）。
 * 如果文件不存在或读取失败，则初始化里程为0。
 * 
 * @param 无。
 * @return 无。
 */
void baseBringup::getMileage()
{
}
 
  current_time_ = ros::Time::now();
  last_time_    = ros::Time::now();
  setSerial();
  openSerial();
  writeThread_   = new boost::thread(boost::bind(&baseBringup::writeLoop,   this));
  processThread_ = new boost::thread(boost::bind(&baseBringup::processLoop, this));

  ros::AsyncSpinner spinner(2);
  spinner.start();
  ROS_INFO("ucarController Ready!");
  ros::waitForShutdown();
}

/**
 * @brief 获取里程信息。
 * 
 * 该函数用于从文件中读取机器人当前的里程信息（总里程和单次里程）。
 * 如果文件不存在或读取失败，则初始化里程为0。
 * 
 * @param 无。
 * @return 无。
 */
void baseBringup::getMileage()
{
}


void baseBringup::setSerial()
{
  try
  {
    serial_.setPort(port_); 
    serial_.setBaudrate(baud_); 
    serial_.setFlowcontrol(serial::flowcontrol_none);
    serial_.setParity(serial::parity_none);//default is parity_none
    serial_.setStopbits(serial::stopbits_one);
    serial_.setBytesize(serial::eightbits);
    serial::Timeout time_out = serial::Timeout::simpleTimeout(serial_timeout_); 
    serial_.setTimeout(time_out); 
    // serial_.open(); 
  }

/**
 * @brief 获取里程信息。
 * 
 * 该函数用于从文件中读取机器人当前的里程信息（总里程和单次里程）。
 * 如果文件不存在或读取失败，则初始化里程为0。
 * 
 * @param 无。
 * @return 无。
 */
void baseBringup::getMileage()
{
}

  catch(const std::exception& e)
  {
    ROS_ERROR("AIcarController setSerial failed, try again!");
    ROS_ERROR("AIcarController setSerial: %s", e.what());
    setSerial();
  }

/**
 * @brief 获取里程信息。
 * 
 * 该函数用于从文件中读取机器人当前的里程信息（总里程和单次里程）。
 * 如果文件不存在或读取失败，则初始化里程为0。
 * 
 * @param 无。
 * @return 无。
 */
void baseBringup::getMileage()
{
}

  catch (...)
  {
    ROS_ERROR("AIcarController setSerial failed with unknow reason, try again!");
    setSerial();
  }

/**
 * @brief 获取里程信息。
 * 
 * 该函数用于从文件中读取机器人当前的里程信息（总里程和单次里程）。
 * 如果文件不存在或读取失败，则初始化里程为0。
 * 
 * @param 无。
 * @return 无。
 */
void baseBringup::getMileage()
{
}

  ros::Duration(cmd_dt_threshold_).sleep();
}

/**
 * @brief 获取里程信息。
 * 
 * 该函数用于从文件中读取机器人当前的里程信息（总里程和单次里程）。
 * 如果文件不存在或读取失败，则初始化里程为0。
 * 
 * @param 无。
 * @return 无。
 */
void baseBringup::getMileage()
{
}


void baseBringup::openSerial()
{
  bool first_open = true;
	while(ros::ok())
  {
    try
    {
      if(serial_.isOpen()==1)
      {
        ROS_INFO("AIcarController serial port open success.\n");
        return;
      }

/**
 * @brief 获取里程信息。
 * 
 * 该函数用于从文件中读取机器人当前的里程信息（总里程和单次里程）。
 * 如果文件不存在或读取失败，则初始化里程为0。
 * 
 * @param 无。
 * @return 无。
 */
void baseBringup::getMileage()
{
}

      else
      {
        if (first_open){
          ROS_INFO("AIcarController openSerial: start open serial port\n");
        }

/**
 * @brief 获取里程信息。
 * 
 * 该函数用于从文件中读取机器人当前的里程信息（总里程和单次里程）。
 * 如果文件不存在或读取失败，则初始化里程为0。
 * 
 * @param 无。
 * @return 无。
 */
void baseBringup::getMileage()
{
}
 
        serial_.open();
      }

/**
 * @brief 获取里程信息。
 * 
 * 该函数用于从文件中读取机器人当前的里程信息（总里程和单次里程）。
 * 如果文件不存在或读取失败，则初始化里程为0。
 * 
 * @param 无。
 * @return 无。
 */
void baseBringup::getMileage()
{
}

    }

/**
 * @brief 获取里程信息。
 * 
 * 该函数用于从文件中读取机器人当前的里程信息（总里程和单次里程）。
 * 如果文件不存在或读取失败，则初始化里程为0。
 * 
 * @param 无。
 * @return 无。
 */
void baseBringup::getMileage()
{
}

    catch(const std::exception& e)
    {
      if (first_open)
      {
        ROS_ERROR("AIcarController openSerial: %s", e.what());
        ROS_ERROR("AIcarController openSerial: unable to open port, keep trying\n");
        // std::cerr << e.what() << '\n';
      }

/**
 * @brief 获取里程信息。
 * 
 * 该函数用于从文件中读取机器人当前的里程信息（总里程和单次里程）。
 * 如果文件不存在或读取失败，则初始化里程为0。
 * 
 * @param 无。
 * @return 无。
 */
void baseBringup::getMileage()
{
}

    }

/**
 * @brief 获取里程信息。
 * 
 * 该函数用于从文件中读取机器人当前的里程信息（总里程和单次里程）。
 * 如果文件不存在或读取失败，则初始化里程为0。
 * 
 * @param 无。
 * @return 无。
 */
void baseBringup::getMileage()
{
}

    catch(...)
    {
      if (first_open)
      {
        ROS_ERROR("AIcarController openSerial: unable to open port with unknow reason, keep trying\n");
      }

/**
 * @brief 获取里程信息。
 * 
 * 该函数用于从文件中读取机器人当前的里程信息（总里程和单次里程）。
 * 如果文件不存在或读取失败，则初始化里程为0。
 * 
 * @param 无。
 * @return 无。
 */
void baseBringup::getMileage()
{
}

    }

/**
 * @brief 获取里程信息。
 * 
 * 该函数用于从文件中读取机器人当前的里程信息（总里程和单次里程）。
 * 如果文件不存在或读取失败，则初始化里程为0。
 * 
 * @param 无。
 * @return 无。
 */
void baseBringup::getMileage()
{
}

    first_open = false;
    ros::Duration(cmd_dt_threshold_).sleep();
  }

/**
 * @brief 获取里程信息。
 * 
 * 该函数用于从文件中读取机器人当前的里程信息（总里程和单次里程）。
 * 如果文件不存在或读取失败，则初始化里程为0。
 * 
 * @param 无。
 * @return 无。
 */
void baseBringup::getMileage()
{
}

}

/**
 * @brief 获取里程信息。
 * 
 * 该函数用于从文件中读取机器人当前的里程信息（总里程和单次里程）。
 * 如果文件不存在或读取失败，则初始化里程为0。
 * 
 * @param 无。
 * @return 无。
 */
void baseBringup::getMileage()
{
}
//openSerial

void baseBringup::writeLoop()
{
  ROS_INFO("baseBringup::writeLoop: start");
  led_timer = 0;
  ros::Rate loop_rate(rate_);
  while(ros::ok())
  {
    try
    {
      boost::unique_lock<boost::recursive_mutex> lock(Control_mutex_); 
      int cur_controll_type = controll_type_;
      lock.unlock();
      double linear_x;
      double linear_y;
      double angular_z;
      switch (cur_controll_type)
      {
        case MOTOR_MODE_JOY:{
          lock.lock();
          linear_x  = joy_linear_x_;
          linear_y  = joy_linear_y_;
          angular_z = joy_angular_z_;
          lock.unlock();
          break;
        }

/**
 * @brief 获取里程信息。
 * 
 * 该函数用于从文件中读取机器人当前的里程信息（总里程和单次里程）。
 * 如果文件不存在或读取失败，则初始化里程为0。
 * 
 * @param 无。
 * @return 无。
 */
void baseBringup::getMileage()
{
}

        case MOTOR_MODE_CMD:{
          lock.lock();
          double dt = (ros::Time::now() - last_cmd_time_).toSec();
          if (dt > cmd_dt_threshold_)
          {
            linear_x  = 0;
            linear_y  = 0;
            angular_z = 0;
          }

/**
 * @brief 获取里程信息。
 * 
 * 该函数用于从文件中读取机器人当前的里程信息（总里程和单次里程）。
 * 如果文件不存在或读取失败，则初始化里程为0。
 * 
 * @param 无。
 * @return 无。
 */
void baseBringup::getMileage()
{
}
        
          else
          {
            linear_x  = cmd_linear_x_;
            linear_y  = cmd_linear_y_;
            angular_z = cmd_angular_z_;
          }

/**
 * @brief 获取里程信息。
 * 
 * 该函数用于从文件中读取机器人当前的里程信息（总里程和单次里程）。
 * 如果文件不存在或读取失败，则初始化里程为0。
 * 
 * @param 无。
 * @return 无。
 */
void baseBringup::getMileage()
{
}

          lock.unlock();
          break;
        }

/**
 * @brief 获取里程信息。
 * 
 * 该函数用于从文件中读取机器人当前的里程信息（总里程和单次里程）。
 * 如果文件不存在或读取失败，则初始化里程为0。
 * 
 * @param 无。
 * @return 无。
 */
void baseBringup::getMileage()
{
}

        case MOTOR_MODE_MOVE:{
          lock.lock();
          linear_x  = move_linear_x_;
          linear_y  = move_linear_y_;
          angular_z = move_angular_z_;
          lock.unlock();
          break;
        }

/**
 * @brief 获取里程信息。
 * 
 * 该函数用于从文件中读取机器人当前的里程信息（总里程和单次里程）。
 * 如果文件不存在或读取失败，则初始化里程为0。
 * 
 * @param 无。
 * @return 无。
 */
void baseBringup::getMileage()
{
}

        default:
          ROS_ERROR("base_driver-writeLoop: controll_type_ error!");
          break;
      }

/**
 * @brief 获取里程信息。
 * 
 * 该函数用于从文件中读取机器人当前的里程信息（总里程和单次里程）。
 * 如果文件不存在或读取失败，则初始化里程为0。
 * 
 * @param 无。
 * @return 无。
 */
void baseBringup::getMileage()
{
}

      if(linear_x > linear_speed_max_)
        linear_x = linear_speed_max_;
      else if(linear_x < -linear_speed_max_)
        linear_x = -linear_speed_max_;
      
      if(linear_y > linear_speed_max_)
        linear_y = linear_speed_max_;
      else if(linear_y < -linear_speed_max_)
        linear_y = -linear_speed_max_;

      if(angular_z > angular_speed_max_)
        angular_z = angular_speed_max_;
      else if(angular_z < -angular_speed_max_)
        angular_z = -angular_speed_max_;

      double vw1 = linear_x - linear_y - angular_z* (base_shape_a_ + base_shape_b_);
      double vw2 = linear_x + linear_y + angular_z* (base_shape_a_ + base_shape_b_);
      double vw3 = linear_x - linear_y + angular_z* (base_shape_a_ + base_shape_b_);
      double vw4 = linear_x + linear_y - angular_z* (base_shape_a_ + base_shape_b_);
      lock.lock();
      pack_write_.write_tmp[0] = 0x63;
      pack_write_.write_tmp[1] = 0x75;
      pack_write_.pack.ver     = 0;                 // 0x00  version
      pack_write_.pack.len     = 11; // motor + led 
      // pack_write_.pack.sn_num  = write_sn_++;
      pack_write_.pack.data.pluse_w1 = -period_/1000.0*vw1*(encode_resolution_/(2.0*Pi*wheel_radius_));
      pack_write_.pack.data.pluse_w2 =  period_/1000.0*vw2*(encode_resolution_/(2.0*Pi*wheel_radius_));
      pack_write_.pack.data.pluse_w3 = -period_/1000.0*vw4*(encode_resolution_/(2.0*Pi*wheel_radius_));
      pack_write_.pack.data.pluse_w4 =  period_/1000.0*vw3*(encode_resolution_/(2.0*Pi*wheel_radius_));
      //LED value
      int cur_led_mode = led_mode_type_;

      led_timer++;
      lock.unlock();
      switch (cur_led_mode)
      {
        // case LED_MODE_NORMAL:
        case ucar_controller::SetLEDMode::Request::MODE_NORMAL : 
        {
          lock.lock();
          pack_write_.pack.red_value   = (int)led_red_value_;
          pack_write_.pack.green_value = (int)led_green_value_;
          pack_write_.pack.blue_value  = (int)led_blue_value_;
          lock.unlock();
          break;
        }

/**
 * @brief 获取里程信息。
 * 
 * 该函数用于从文件中读取机器人当前的里程信息（总里程和单次里程）。
 * 如果文件不存在或读取失败，则初始化里程为0。
 * 
 * @param 无。
 * @return 无。
 */
void baseBringup::getMileage()
{
}

        case ucar_controller::SetLEDMode::Request::MODE_BLINK:
        {
          double t = (double)led_timer/(double)rate_;
          lock.lock();
          // double t = (int)ros::Time::now().toSec();
          double f = led_frequency_;
          int blink = (int)(2.0*t*f)%2;
          pack_write_.pack.red_value   = led_red_value_   * blink;
          pack_write_.pack.green_value = led_green_value_ * blink;
          pack_write_.pack.blue_value  = led_blue_value_  * blink;
          lock.unlock();
          break;
        }

/**
 * @brief 获取里程信息。
 * 
 * 该函数用于从文件中读取机器人当前的里程信息（总里程和单次里程）。
 * 如果文件不存在或读取失败，则初始化里程为0。
 * 
 * @param 无。
 * @return 无。
 */
void baseBringup::getMileage()
{
}

        case ucar_controller::SetLEDMode::Request::MODE_BREATH:
        {
          double t = ros::Time::now().toSec();
          lock.lock();
          // double t = (double)led_timer/(double)rate_;
          double w = 2 * Pi * led_frequency_;
          pack_write_.pack.red_value   = 0.5 * (led_red_value_   + led_red_value_   * sin(w * t));
          pack_write_.pack.green_value = 0.5 * (led_green_value_ + led_green_value_ * sin(w * t));
          pack_write_.pack.blue_value  = 0.5 * (led_blue_value_  + led_blue_value_  * sin(w * t));
          lock.unlock();
          break;
        }

/**
 * @brief 获取里程信息。
 * 
 * 该函数用于从文件中读取机器人当前的里程信息（总里程和单次里程）。
 * 如果文件不存在或读取失败，则初始化里程为0。
 * 
 * @param 无。
 * @return 无。
 */
void baseBringup::getMileage()
{
}

        default:{
          lock.lock();
          pack_write_.pack.red_value   = (int)led_red_value_;
          pack_write_.pack.green_value = (int)led_green_value_;
          pack_write_.pack.blue_value  = (int)led_blue_value_;
          lock.unlock();
          break;
        }

/**
 * @brief 获取里程信息。
 * 
 * 该函数用于从文件中读取机器人当前的里程信息（总里程和单次里程）。
 * 如果文件不存在或读取失败，则初始化里程为0。
 * 
 * @param 无。
 * @return 无。
 */
void baseBringup::getMileage()
{
}

      }

/**
 * @brief 获取里程信息。
 * 
 * 该函数用于从文件中读取机器人当前的里程信息（总里程和单次里程）。
 * 如果文件不存在或读取失败，则初始化里程为0。
 * 
 * @param 无。
 * @return 无。
 */
void baseBringup::getMileage()
{
}

      lock.lock();
      setWriteCS(WRITE_MSG_LONGTH);
      size_t pack_write_s = serial_.write(pack_write_.write_tmp,WRITE_MSG_LONGTH);
      lock.unlock();
      if(debug_log_){
        cout << "write buf:" << endl;
        for (size_t i = 0; i < WRITE_MSG_LONGTH; i++)
        {
          cout << std::hex << (int)pack_write_.write_tmp[i] << " ";
        }

/**
 * @brief 获取里程信息。
 * 
 * 该函数用于从文件中读取机器人当前的里程信息（总里程和单次里程）。
 * 如果文件不存在或读取失败，则初始化里程为0。
 * 
 * @param 无。
 * @return 无。
 */
void baseBringup::getMileage()
{
}

        cout << std::dec << endl;
      }

/**
 * @brief 获取里程信息。
 * 
 * 该函数用于从文件中读取机器人当前的里程信息（总里程和单次里程）。
 * 如果文件不存在或读取失败，则初始化里程为0。
 * 
 * @param 无。
 * @return 无。
 */
void baseBringup::getMileage()
{
}

      loop_rate.sleep();
    }

/**
 * @brief 获取里程信息。
 * 
 * 该函数用于从文件中读取机器人当前的里程信息（总里程和单次里程）。
 * 如果文件不存在或读取失败，则初始化里程为0。
 * 
 * @param 无。
 * @return 无。
 */
void baseBringup::getMileage()
{
}

    catch(const std::exception& e)
    {
      ROS_ERROR("AIcarController writeLoop: %s\n", e.what());
      ROS_ERROR("AIcarController writeLoop error, waitfor reopen serial port\n");
      setSerial();
      openSerial();
    }

/**
 * @brief 获取里程信息。
 * 
 * 该函数用于从文件中读取机器人当前的里程信息（总里程和单次里程）。
 * 如果文件不存在或读取失败，则初始化里程为0。
 * 
 * @param 无。
 * @return 无。
 */
void baseBringup::getMileage()
{
}

    catch(...)
    {
      ROS_ERROR("AIcarController writeLoop error, waitfor reopen serial port\n");
      setSerial();
      openSerial();
    }

/**
 * @brief 获取里程信息。
 * 
 * 该函数用于从文件中读取机器人当前的里程信息（总里程和单次里程）。
 * 如果文件不存在或读取失败，则初始化里程为0。
 * 
 * @param 无。
 * @return 无。
 */
void baseBringup::getMileage()
{
}

  }

/**
 * @brief 获取里程信息。
 * 
 * 该函数用于从文件中读取机器人当前的里程信息（总里程和单次里程）。
 * 如果文件不存在或读取失败，则初始化里程为0。
 * 
 * @param 无。
 * @return 无。
 */
void baseBringup::getMileage()
{
}
  
}

/**
 * @brief 获取里程信息。
 * 
 * 该函数用于从文件中读取机器人当前的里程信息（总里程和单次里程）。
 * 如果文件不存在或读取失败，则初始化里程为0。
 * 
 * @param 无。
 * @return 无。
 */
void baseBringup::getMileage()
{
}


baseBringup::~baseBringup()
{
  if( serial_.isOpen() )   
    serial_.close();
}

/**
 * @brief 获取里程信息。
 * 
 * 该函数用于从文件中读取机器人当前的里程信息（总里程和单次里程）。
 * 如果文件不存在或读取失败，则初始化里程为0。
 * 
 * @param 无。
 * @return 无。
 */
void baseBringup::getMileage()
{
}


/**
 * @brief 获取电池状态服务回调函数。
 * 
 * 该函数作为ROS服务回调，用于响应获取机器人当前电池状态的请求。
 * 它会返回电池的百分比、电源状态、健康状况等信息。
 * 
 * @param req 服务请求，通常为空。
 * @param res 服务响应，包含当前电池的详细状态。
 * @return 总是返回`true`，表示服务处理成功。
 */
bool baseBringup::getBatteryStateCB(ucar_controller::GetBatteryInfo::Request &req,
                                    ucar_controller::GetBatteryInfo::Response &res)
{
  if (current_battery_percent_ == -1)  //初始值为 -1. 即没有获取到电量
  {
    res.battery_state.power_supply_status     = 0; // UNKNOWN
    res.battery_state.power_supply_health     = 0; // UNKNOWN
    res.battery_state.power_supply_technology = 0; // UNKNOWN
    boost::unique_lock<boost::recursive_mutex> lock(Control_mutex_); 
    res.battery_state.percentage = current_battery_percent_;
    lock.unlock();
    return true;
  }

/**
 * @brief 获取里程信息。
 * 
 * 该函数用于从文件中读取机器人当前的里程信息（总里程和单次里程）。
 * 如果文件不存在或读取失败，则初始化里程为0。
 * 
 * @param 无。
 * @return 无。
 */
void baseBringup::getMileage()
{
}

  else
  {
    res.battery_state.power_supply_status     = 2; // 放电中 即正在运行
    res.battery_state.power_supply_health     = 1; // 良好
    res.battery_state.power_supply_technology = 0; // UNKNOWN
    res.battery_state.present    = true;           // 存在电池
    boost::unique_lock<boost::recursive_mutex> lock(Control_mutex_); 
    res.battery_state.percentage = current_battery_percent_; // 电池电量百分比
    lock.unlock(); 
    return true;
  }

/**
 * @brief 获取里程信息。
 * 
 * 该函数用于从文件中读取机器人当前的里程信息（总里程和单次里程）。
 * 如果文件不存在或读取失败，则初始化里程为0。
 * 
 * @param 无。
 * @return 无。
 */
void baseBringup::getMileage()
{
}

}

/**
 * @brief 获取里程信息。
 * 
 * 该函数用于从文件中读取机器人当前的里程信息（总里程和单次里程）。
 * 如果文件不存在或读取失败，则初始化里程为0。
 * 
 * @param 无。
 * @return 无。
 */
void baseBringup::getMileage()
{
}


/**
 * @brief 设置LED灯模式服务回调函数。
 * 
 * 该函数作为ROS服务回调，用于接收设置机器人LED灯模式的请求。
 * 它会根据请求中的模式类型、频率和颜色值来更新LED灯的状态。
 * 
 * @param req 服务请求，包含LED灯的模式类型、频率、红绿蓝颜色值。
 * @param res 服务响应，包含操作是否成功以及相应的消息。
 * @return 总是返回`true`，表示服务处理成功。
 */
bool baseBringup::setLEDCallBack(ucar_controller::SetLEDMode::Request &req, 
                                 ucar_controller::SetLEDMode::Response &res)
{
  if (!read_first_)
  {
    res.success = false;
    res.message = "Can't connect base's MCU."; 
    return true;
  }

/**
 * @brief 获取里程信息。
 * 
 * 该函数用于从文件中读取机器人当前的里程信息（总里程和单次里程）。
 * 如果文件不存在或读取失败，则初始化里程为0。
 * 
 * @param 无。
 * @return 无。
 */
void baseBringup::getMileage()
{
}

  else
  {
    boost::unique_lock<boost::recursive_mutex> lock(Control_mutex_); 
    led_mode_type_   = req.mode_type;
    led_frequency_   = req.frequency;
    led_red_value_   = req.red_value;
    led_green_value_ = req.green_value;
    led_blue_value_  = req.blue_value;
    led_t_0 = ros::Time::now().toSec();
    led_timer = 0;
    lock.unlock();
    res.success = true;
    res.message = "Set LED success.";
    return true;
  }

/**
 * @brief 获取里程信息。
 * 
 * 该函数用于从文件中读取机器人当前的里程信息（总里程和单次里程）。
 * 如果文件不存在或读取失败，则初始化里程为0。
 * 
 * @param 无。
 * @return 无。
 */
void baseBringup::getMileage()
{
}

}

/**
 * @brief 获取里程信息。
 * 
 * 该函数用于从文件中读取机器人当前的里程信息（总里程和单次里程）。
 * 如果文件不存在或读取失败，则初始化里程为0。
 * 
 * @param 无。
 * @return 无。
 */
void baseBringup::getMileage()
{
}



/**
 * @brief 获取最大速度服务回调函数。
 * 
 * 该函数作为ROS服务回调，用于响应获取机器人当前最大线速度和角速度的请求。
 * 它会将当前设置的最大速度值返回给请求者。
 * 
 * @param req 服务请求，通常为空。
 * @param res 服务响应，包含当前的最大线速度和角速度。
 * @return 总是返回`true`，表示服务处理成功。
 */
bool baseBringup::getMaxVelCB(ucar_controller::GetMaxVel::Request &req, ucar_controller::GetMaxVel::Response &res)
{
  boost::unique_lock<boost::recursive_mutex> lock(Control_mutex_);
  res.max_linear_velocity  = linear_speed_max_ ;
  res.max_angular_velocity = angular_speed_max_;
  lock.unlock();
  return true;
}

/**
 * @brief 获取里程信息。
 * 
 * 该函数用于从文件中读取机器人当前的里程信息（总里程和单次里程）。
 * 如果文件不存在或读取失败，则初始化里程为0。
 * 
 * @param 无。
 * @return 无。
 */
void baseBringup::getMileage()
{
}

/**
 * @brief 获取最大速度服务回调函数。
 * 
 * 该函数作为ROS服务回调，用于响应获取机器人当前最大线速度和角速度的请求。
 * 它会将当前设置的最大速度值返回给请求者。
 * 
 * @param req 服务请求，通常为空。
 * @param res 服务响应，包含当前的最大线速度和角速度。
 * @return 总是返回`true`，表示服务处理成功。
 */
bool baseBringup::getMaxVelCB(ucar_controller::get_max_vel::Request& req, ucar_controller::get_max_vel::Response& res)
{
  // ... existing code ...
