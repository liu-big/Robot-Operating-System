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
 * @file static_transform_publisher.cpp
 * @brief 静态TF变换发布器。
 * 该程序是一个命令行工具，用于周期性地发布给定的TF变换。
 * 支持欧拉角（yaw, pitch, roll）和四元数两种方式指定旋转。
 */

#include <cstdio>
#include "tf/transform_broadcaster.h"

/**
 * @brief TransformSender类，用于发送TF变换。
 */
class TransformSender
{
public:
  ros::NodeHandle node_; ///< ROS节点句柄。

  /**
   * @brief 构造函数，使用欧拉角初始化变换。
   * @param x 变换的X坐标。
   * @param y 变换的Y坐标。
   * @param z 变换的Z坐标。
   * @param yaw 绕Z轴的旋转角度（偏航）。
   * @param pitch 绕Y轴的旋转角度（俯仰）。
   * @param roll 绕X轴的旋转角度（翻滚）。
   * @param time 变换的时间戳。
   * @param frame_id 父坐标系的ID。
   * @param child_frame_id 子坐标系的ID。
   */
  TransformSender(double x, double y, double z, double yaw, double pitch, double roll, ros::Time time, const std::string& frame_id, const std::string& child_frame_id)
  {
    tf::Quaternion q;
    q.setRPY(roll, pitch,yaw);
    transform_ = tf::StampedTransform(tf::Transform(q, tf::Vector3(x,y,z)), time, frame_id, child_frame_id );
  };

  /**
   * @brief 构造函数，使用四元数初始化变换。
   * @param x 变换的X坐标。
   * @param y 变换的Y坐标。
   * @param z 变换的Z坐标。
   * @param qx 四元数的X分量。
   * @param qy 四元数的Y分量。
   * @param qz 四元数的Z分量。
   * @param qw 四元数的W分量。
   * @param time 变换的时间戳。
   * @param frame_id 父坐标系的ID。
   * @param child_frame_id 子坐标系的ID。
   */
  TransformSender(double x, double y, double z, double qx, double qy, double qz, double qw, ros::Time time, const std::string& frame_id, const std::string& child_frame_id) :
    transform_(tf::Transform(tf::Quaternion(qx,qy,qz,qw), tf::Vector3(x,y,z)), time, frame_id, child_frame_id){};

  /**
   * @brief 析构函数，清理ROS连接。
   */
  ~TransformSender() { }

  tf::TransformBroadcaster broadcaster; ///< TF变换广播器。

  /**
   * @brief 周期性发送TF变换。
   * @param time 变换的时间戳。
   */
  void send (ros::Time time) {
    transform_.stamp_ = time;
    broadcaster.sendTransform(transform_);
  };

private:
  tf::StampedTransform transform_; ///< 存储的TF变换。

};

/**
 * @brief 主函数，静态TF变换发布器的入口点。
 * @param argc 命令行参数数量。
 * @param argv 命令行参数数组。
 * @return 程序退出码。
 * @details 解析命令行参数，创建静态变换，并以指定频率发布。
 *          命令行参数格式：x y z yaw pitch roll frame_id child_frame_id [period_in_ms]
 *          或 x y z qx qy qz qw frame_id child_frame_id [period_in_ms]
 */
int main(int argc, char ** argv)
{
  // 初始化ROS节点，节点名为 "static_transform_publisher"
  ros::init(argc, argv,"static_transform_publisher", ros::init_options::AnonymousName);

  // 根据参数数量判断使用哪种模式
  if(argc == 11) // 使用欧拉角或四元数，并指定周期
  {
    // 将周期参数从毫秒转换为ros::Duration
    ros::Duration sleeper(atof(argv[10])/1000.0);

    // 检查父子坐标系是否相同
    if (strcmp(argv[8], argv[9]) == 0)
      ROS_FATAL("target_frame and source frame are the same (%s, %s) this cannot work", argv[8], argv[9]);

    // 创建TransformSender对象，初始化变换
    TransformSender tf_sender(atof(argv[1]), atof(argv[2]), atof(argv[3]),
                              atof(argv[4]), atof(argv[5]), atof(argv[6]), atof(argv[7]),
                              ros::Time() + sleeper, // 未来时间戳，允许较慢发送而不会超时
                              argv[8], argv[9]);

    // 循环发送变换直到ROS节点关闭
    while(tf_sender.node_.ok())
    {
      tf_sender.send(ros::Time::now() + sleeper);
      ROS_DEBUG("Sending transform from %s with parent %s\n", argv[8], argv[9]);
      sleeper.sleep(); // 休眠指定周期
    }

    return 0;
  }
  else if (argc == 10) // 使用欧拉角或四元数，并指定周期（旧版本兼容）
  {
    // 将周期参数从毫秒转换为ros::Duration
    ros::Duration sleeper(atof(argv[9])/1000.0);

    // 检查父子坐标系是否相同
    if (strcmp(argv[7], argv[8]) == 0)
      ROS_FATAL("target_frame and source frame are the same (%s, %s) this cannot work", argv[7], argv[8]);

    // 创建TransformSender对象，初始化变换
    TransformSender tf_sender(atof(argv[1]), atof(argv[2]), atof(argv[3]),
                              atof(argv[4]), atof(argv[5]), atof(argv[6]),
                              ros::Time() + sleeper, // 未来时间戳，允许较慢发送而不会超时
                              argv[7], argv[8]);

    // 循环发送变换直到ROS节点关闭
    while(tf_sender.node_.ok())
    {
      tf_sender.send(ros::Time::now() + sleeper);
      ROS_DEBUG("Sending transform from %s with parent %s\n", argv[7], argv[8]);
      sleeper.sleep(); // 休眠指定周期
    }

    return 0;

  }
  else // 参数数量不正确，打印使用说明
  {
    printf("A command line utility for manually sending a transform.\n");
    printf("It will periodicaly republish the given transform. \n");
    printf("Usage: static_transform_publisher x y z yaw pitch roll frame_id child_frame_id  period(milliseconds) \n");
    printf("OR \n");
    printf("Usage: static_transform_publisher x y z qx qy qz qw frame_id child_frame_id  period(milliseconds) \n");
    printf("\nThis transform is the transform of the coordinate frame from frame_id into the coordinate frame \n");
    printf("of the child_frame_id.  \n");
    ROS_ERROR("static_transform_publisher exited due to not having the right number of arguments");
    return -1;
  }


}

