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
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"和任何明示或暗示的保证，包括但不限于适销性和特定用途适用性的暗示保证。
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Tully Foote */

/**
 * @file empty_listener.cpp
 * @brief 一个简单的TF监听器示例。
 * 该节点仅初始化ROS和TF监听器，然后进入ros::spin()循环，
 * 不执行任何实际的TF查询或转换操作。主要用于测试TF库的基本功能或作为最小示例。
 */

#include "ros/ros.h"
#include "tf/transform_listener.h"

/**
 * @brief 主函数，empty_listener节点的入口点。
 * @param argc 命令行参数数量。
 * @param argv 命令行参数数组。
 * @return 程序退出码。
 * @details 初始化ROS节点和TF监听器，然后进入ROS事件循环。
 */
int main(int argc, char** argv)
{
  // 初始化ROS节点，节点名为"empty_listener"
  ros::init(argc, argv, "empty_listener");
  // 创建一个ROS节点句柄
  ros::NodeHandle node;

  // 创建一个tf::TransformListener对象，用于监听和查询TF变换
  tf::TransformListener listener;

  // 进入ROS事件循环，处理回调函数，直到节点关闭
  ros::spin();

  // 程序正常退出
  return 0;
}
