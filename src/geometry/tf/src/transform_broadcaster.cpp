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
 * @file transform_broadcaster.cpp
 * @brief 这是一个ROS TF（Transform Frame）库中用于广播变换的C++实现。
 * 它提供了将TF变换发布到ROS消息系统中的功能，支持多种类型的变换消息。
 * @author Tully Foote
 */


#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"

#include <tf2_ros/transform_broadcaster.h>



namespace tf {

/**
 * @brief TransformBroadcaster类的构造函数。
 * 初始化内部的tf2_ros::TransformBroadcaster。
 */
TransformBroadcaster::TransformBroadcaster():
  tf2_broadcaster_()
{
}

/**
 * @brief 发送单个geometry_msgs::TransformStamped类型的变换。
 * @param msgtf 要发送的变换消息。
 */
void TransformBroadcaster::sendTransform(const geometry_msgs::TransformStamped & msgtf)
{
  tf2_broadcaster_.sendTransform(msgtf);
}

/**
 * @brief 发送单个tf::StampedTransform类型的变换。
 * @param transform 要发送的StampedTransform对象。
 */
void TransformBroadcaster::sendTransform(const StampedTransform & transform)
{
  geometry_msgs::TransformStamped msgtf;
  transformStampedTFToMsg(transform, msgtf);
  tf2_broadcaster_.sendTransform(msgtf);
}

/**
 * @brief 发送多个geometry_msgs::TransformStamped类型的变换。
 * @param msgtf 包含多个变换消息的向量。
 */
void TransformBroadcaster::sendTransform(const std::vector<geometry_msgs::TransformStamped> & msgtf)
{
  tf2_broadcaster_.sendTransform(msgtf);
}

/**
 * @brief 发送多个tf::StampedTransform类型的变换。
 * @param transforms 包含多个StampedTransform对象的向量。
 */
void TransformBroadcaster::sendTransform(const std::vector<StampedTransform> & transforms)
{
  std::vector<geometry_msgs::TransformStamped> msgtfs;
  for (std::vector<StampedTransform>::const_iterator it = transforms.begin(); it != transforms.end(); ++it)
  {
    geometry_msgs::TransformStamped msgtf;
    transformStampedTFToMsg(*it, msgtf);
    msgtfs.push_back(msgtf);

  }
  tf2_broadcaster_.sendTransform(msgtfs);
}
  



}


