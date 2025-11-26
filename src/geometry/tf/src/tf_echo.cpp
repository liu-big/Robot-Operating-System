/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file tf_echo.cpp
 * @brief TF变换回显工具。
 * 该程序是一个命令行工具，用于周期性地打印指定源坐标系到目标坐标系之间的TF变换。
 * 可以显示平移、四元数旋转以及RPY（翻滚、俯仰、偏航）角度。
 */

#include <cstdio>
#include "tf/transform_listener.h"
#include "ros/ros.h"

#define _USE_MATH_DEFINES

/**
 * @brief echoListener类，用于监听TF变换并打印。
 */
class echoListener
{
public:

  tf::TransformListener tf; ///< TF变换监听器。

  /**
   * @brief 构造函数。
   */
  echoListener()
  {

  }

  /**
   * @brief 析构函数。
   */
  ~echoListener()
  {

  }

private:

};

/**
 * @brief 主函数，TF变换回显工具的入口点。
 * @param argc 命令行参数数量。
 * @param argv 命令行参数数组。
 * @return 程序退出码。
 * @details 该函数初始化ROS节点，解析命令行参数以获取源帧、目标帧和可选的回显频率。
 *          它会周期性地查找并打印从源帧到目标帧的TF变换信息，包括平移和旋转（四元数和RPY）。
 *          如果找不到变换或发生其他TF异常，会打印相应的错误信息。
 *          用法: tf_echo source_frame target_frame [echo_rate]
 */
int main(int argc, char ** argv)
{
  // 初始化ROS节点，节点名为 "tf_echo"
  ros::init(argc, argv, "tf_echo", ros::init_options::AnonymousName);

  // 允许2或3个命令行参数
  if (argc < 3 || argc > 4)
  {
    printf("Usage: tf_echo source_frame target_frame [echo_rate]\n\n");
    printf("This will echo the transform from the coordinate frame of the source_frame\n");
    printf("to the coordinate frame of the target_frame. \n");
    printf("Note: This is the transform to get data from target_frame into the source_frame.\n");
    printf("Default echo rate is 1 if echo_rate is not given.\n");
    return -1;
  }

  ros::NodeHandle nh("~"); // 私有节点句柄

  double rate_hz;
  if (argc == 4)
  {
    // 从命令行读取回显频率
    rate_hz = atof(argv[3]);
  }
  else
  {
    // 从参数服务器读取回显频率参数，默认为1.0
    nh.param("rate", rate_hz, 1.0);
  }
  if (rate_hz <= 0.0)
  {
    std::cerr << "Echo rate must be > 0.0\n";
    return -1;
  }
  ros::Rate rate(rate_hz); // 设置ROS循环频率

  int precision(3); // 默认输出精度
  // 从参数服务器获取精度参数
  if (nh.getParam("precision", precision))
  {
    if (precision < 1)
    {
      std::cerr << "Precision must be > 0\n";
      return -1;
    }
    printf("Precision default value was overriden, new value: %d\n", precision);
  }

  // 实例化一个本地监听器对象
  echoListener echoListener;

  // 获取源帧和目标帧ID
  std::string source_frameid = std::string(argv[1]);
  std::string target_frameid = std::string(argv[2]);

  // 等待最多一秒，直到第一个变换可用
  echoListener.tf.waitForTransform(source_frameid, target_frameid, ros::Time(), ros::Duration(1.0));

  // 除了等待退出，不需要做任何事情
  // 监听器类中的回调函数将处理所有事情
  while(nh.ok())
    {
      try
      {
        tf::StampedTransform echo_transform; // 存储获取到的变换
        // 查找从源帧到目标帧的变换
        echoListener.tf.lookupTransform(source_frameid, target_frameid, ros::Time(), echo_transform);
        std::cout.precision(precision); // 设置输出精度
        std::cout.setf(std::ios::fixed,std::ios::floatfield); // 设置浮点数输出格式
        std::cout << "At time " << echo_transform.stamp_.toSec() << std::endl; // 打印时间戳
        double yaw, pitch, roll;
        echo_transform.getBasis().getRPY(roll, pitch, yaw); // 获取RPY角度
        tf::Quaternion q = echo_transform.getRotation(); // 获取四元数
        tf::Vector3 v = echo_transform.getOrigin(); // 获取平移向量
        std::cout << "- Translation: [" << v.getX() << ", " << v.getY() << ", " << v.getZ() << "]" << std::endl; // 打印平移
        std::cout << "- Rotation: in Quaternion [" << q.getX() << ", " << q.getY() << ", " 
                  << q.getZ() << ", " << q.getW() << "]" << std::endl
                  << "            in RPY (radian) [" <<  roll << ", " << pitch << ", " << yaw << "]" << std::endl
                  << "            in RPY (degree) [" <<  roll*180.0/M_PI << ", " << pitch*180.0/M_PI << ", " << yaw*180.0/M_PI << "]" << std::endl; // 打印旋转

        // 打印变换 (注释掉，因为上面已经打印了)
      }
      catch(tf::TransformException& ex)
      {
        std::cout << "Failure at "<< ros::Time::now() << std::endl;
        std::cout << "Exception thrown:" << ex.what()<< std::endl;
        std::cout << "The current list of frames is:" <<std::endl;
        std::cout << echoListener.tf.allFramesAsString()<<std::endl;
        
      }
      rate.sleep(); // 休眠以控制循环频率
    }

  return 0;
}

