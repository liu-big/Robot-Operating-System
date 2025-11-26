

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
 * @file tf_monitor.cpp
 * @brief 这是一个用于监控ROS中TF（Transform Frame）变换的C++程序。
 * 它能够监听TF消息，并计算变换的延迟、发布频率以及每个变换的发布者。
 * 用户可以选择监控特定的变换链，或者监控所有发布的TF变换。
 * @author Wim Meeusen
 */


#include "tf/tf.h"
#include "tf/transform_listener.h"
#include <string>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include "ros/ros.h"

using namespace tf;
using namespace ros;
using namespace std;

/**
 * @class TFMonitor
 * @brief TFMonitor类用于监听和分析ROS中的TF变换。
 * 它维护了TF变换的延迟、发布者和发布频率等信息，并提供输出这些信息的功能。
 */
class TFMonitor
{
public:
  std::string framea_, frameb_; ///< 存储TF变换的源帧和目标帧。
  bool using_specific_chain_; ///< 标志，指示是否监控特定的变换链。
  
  ros::NodeHandle node_; ///< ROS节点句柄。
  ros::Subscriber subscriber_tf_, subscriber_tf_static_; ///< TF和静态TF消息的订阅者。
  std::vector<std::string> chain_; ///< 存储特定变换链中的帧。
  std::map<std::string, std::string> frame_authority_map; ///< 存储每个子帧的发布者（authority）。
  std::map<std::string, std::vector<double> > delay_map; ///< 存储每个子帧的变换延迟。
  std::map<std::string, std::vector<double> > authority_map; ///< 存储每个发布者的平均延迟。
  std::map<std::string, std::vector<double> > authority_frequency_map; ///< 存储每个发布者的发布时间戳，用于计算频率。
  
  TransformListener tf_; ///< TF监听器，用于查询TF变换。

  tf::tfMessage message_; ///< TF消息对象。

  boost::mutex map_lock_; ///< 用于保护共享数据结构（如map）的互斥锁。

  /**
   * @brief TF消息的回调函数。
   * 当接收到TF消息时，此函数会被调用，并处理消息内容。
   * @param msg_evt 包含TF消息和发布者信息的事件对象。
   */
  void callback(const ros::MessageEvent<tf::tfMessage const>& msg_evt)
  {
    const tf::tfMessage& message = *(msg_evt.getConstMessage());
    std::string authority = msg_evt.getPublisherName(); // 获取发布者名称
    process_callback(message, authority, false);
  }
  
  /**
   * @brief 静态TF消息的回调函数。
   * 当接收到静态TF消息时，此函数会被调用，并处理消息内容。
   * @param msg_evt 包含静态TF消息和发布者信息的事件对象。
   */
  void static_callback(const ros::MessageEvent<tf::tfMessage const>& msg_evt)
  {
    const tf::tfMessage& message = *(msg_evt.getConstMessage());
    std::string authority = msg_evt.getPublisherName() + std::string("(static)"); // 获取发布者名称并标记为静态
    process_callback(message, authority, true);
  }

  /**
   * @brief 处理TF消息的通用函数。
   * 此函数解析TF消息，计算延迟，并更新内部的各种map数据结构。
   * @param message TF消息对象。
   * @param authority 发布者名称。
   * @param is_static 标志，指示是否为静态TF变换。
   */
  void process_callback(const tf::tfMessage& message, const std::string & authority, bool is_static)
  {
    double average_offset = 0;
    boost::mutex::scoped_lock my_lock(map_lock_); // 锁定互斥锁以保护共享数据
    for (unsigned int i = 0; i < message.transforms.size(); i++)
    {
      frame_authority_map[message.transforms[i].child_frame_id] = authority; // 记录子帧的发布者

      double offset;
      if (is_static)
      {
        offset = 0.0; // 静态变换延迟为0
      }
      else
      {
        offset = (ros::Time::now() - message.transforms[i].header.stamp).toSec(); // 计算动态变换的延迟
      }
      average_offset  += offset;
      
      // 更新延迟map
      std::map<std::string, std::vector<double> >::iterator it = delay_map.find(message.transforms[i].child_frame_id);
      if (it == delay_map.end())
      {
        delay_map[message.transforms[i].child_frame_id] = std::vector<double>(1,offset);
      }
      else
      {
        it->second.push_back(offset);
        if (it->second.size() > 1000) 
          it->second.erase(it->second.begin()); // 保持最多1000个延迟数据
      }
      
    } 
    
    average_offset /= max((size_t) 1, message.transforms.size()); // 计算平均延迟

    // 更新发布者平均延迟map
    std::map<std::string, std::vector<double> >::iterator it2 = authority_map.find(authority);
    if (it2 == authority_map.end())
    {
      authority_map[authority] = std::vector<double>(1,average_offset);
    }
    else
    {
      it2->second.push_back(average_offset);
      if (it2->second.size() > 1000) 
        it2->second.erase(it2->second.begin()); // 保持最多1000个平均延迟数据
    }
    
    // 更新发布者频率map
    std::map<std::string, std::vector<double> >::iterator it3 = authority_frequency_map.find(authority);
    if (it3 == authority_frequency_map.end())
    {
      authority_frequency_map[authority] = std::vector<double>(1,ros::Time::now().toSec());
    }
    else
    {
      it3->second.push_back(ros::Time::now().toSec());
      if (it3->second.size() > 1000) 
        it3->second.erase(it3->second.begin()); // 保持最多1000个时间戳数据
    }
    
  };

  /**
   * @brief TFMonitor类的构造函数。
   * 根据是否监控特定变换链来初始化TFMonitor对象，并订阅TF消息。
   * @param using_specific_chain 标志，指示是否监控特定的变换链。
   * @param framea 源帧名称（如果监控特定变换链）。
   * @param frameb 目标帧名称（如果监控特定变换链）。
   */
  TFMonitor(bool using_specific_chain, std::string framea  = "", std::string frameb = ""):
    framea_(framea), frameb_(frameb),
    using_specific_chain_(using_specific_chain)
  {
    
    if (using_specific_chain_)
    {
      cout << "Waiting for transform chain to become available between "<< framea_ << " and " << frameb_<< " " << flush;
      // 等待TF变换可用
      while (node_.ok() && !tf_.waitForTransform(framea_, frameb_, Time(), Duration(1.0)))
        cout << "." << flush;
      cout << endl;
     
      try{
        // 获取变换链
        tf_.chainAsVector(frameb_, ros::Time(), framea_, ros::Time(), frameb_, chain_);
      }
      catch(tf::TransformException& ex){
        ROS_WARN("Transform Exception %s", ex.what());
        return;
      } 

      // 订阅TF消息
      subscriber_tf_ = node_.subscribe("tf/tfMessage", 100, &TFMonitor::callback, this);
      subscriber_tf_static_ = node_.subscribe("tf_static/tfMessage", 100, &TFMonitor::static_callback, this);
    }
    else
    {
      // 订阅TF消息
      subscriber_tf_ = node_.subscribe("tf/tfMessage", 100, &TFMonitor::callback, this);
      subscriber_tf_static_ = node_.subscribe("tf_static/tfMessage", 100, &TFMonitor::static_callback, this);
    }
  };

  /**
   * @brief 打印TF变换信息。
   * @details 根据是否监控特定变换链，打印相应的TF变换信息，包括延迟、发布者和频率。
   */
  void print_info()
  {
    boost::mutex::scoped_lock my_lock(map_lock_); // 锁定互斥锁以保护共享数据
    if (using_specific_chain_)
    {
      // 打印特定变换链的信息
      cout << endl << "Results for " << framea_ << " to " << frameb_ << endl;
      cout << "Frames:" << endl;
      for (unsigned int i = 0; i < chain_.size(); i++)
      {
        cout << chain_[i] << " published by " << frame_authority_map[chain_[i]] << endl;
      }
      cout << endl;

      // 打印延迟信息
      cout << "Average Delay:" << endl;
      for (unsigned int i = 0; i < chain_.size(); i++)
      {
        if (delay_map.find(chain_[i]) != delay_map.end())
        {
          double sum = 0;
          for (unsigned int j = 0; j < delay_map[chain_[i]].size(); j++)
          {
            sum += delay_map[chain_[i]][j];
          }
          cout << chain_[i] << ": " << sum / delay_map[chain_[i]].size() << endl;
        }
      }
      cout << endl;

      // 打印发布者信息
      cout << "Publishers:" << endl;
      std::map<std::string, std::vector<double> >::iterator it;
      for (it = authority_map.begin(); it != authority_map.end(); ++it)
      {
        double sum = 0;
        for (unsigned int j = 0; j < it->second.size(); j++)
        {
          sum += it->second[j];
        }
        cout << it->first << ": Avg Delay: " << sum / it->second.size() << " Max Delay: " << *std::max_element(it->second.begin(), it->second.end()) << " Min Delay: " << *std::min_element(it->second.begin(), it->second.end()) << endl;

        // 计算并打印发布频率
        if (authority_frequency_map.find(it->first) != authority_frequency_map.end())
        {
          std::vector<double>& times = authority_frequency_map[it->first];
          if (times.size() > 1)
          {
            double total_time = times.back() - times.front();
            cout << "  Frequency: " << (times.size() - 1) / total_time << " Hz" << endl;
          }
        }
      }
    }
    else
    {
      // 打印所有TF变换的信息
      cout << endl << "Results for All Transforms:" << endl;
      cout << "Frames:" << endl;
      std::map<std::string, std::string>::iterator it_frame_authority;
      for (it_frame_authority = frame_authority_map.begin(); it_frame_authority != frame_authority_map.end(); ++it_frame_authority)
      {
        cout << it_frame_authority->first << " published by " << it_frame_authority->second << endl;
      }
      cout << endl;

      // 打印所有发布者的信息
      cout << "Publishers:" << endl;
      std::map<std::string, std::vector<double> >::iterator it_authority;
      for (it_authority = authority_map.begin(); it_authority != authority_map.end(); ++it_authority)
      {
        double sum = 0;
        for (unsigned int j = 0; j < it_authority->second.size(); j++)
        {
          sum += it_authority->second[j];
        }
        cout << it_authority->first << ": Avg Delay: " << sum / it_authority->second.size() << " Max Delay: " << *std::max_element(it_authority->second.begin(), it_authority->second.end()) << " Min Delay: " << *std::min_element(it_authority->second.begin(), it_authority->second.end()) << endl;

        // 计算并打印发布频率
        if (authority_frequency_map.find(it_authority->first) != authority_frequency_map.end())
        {
          std::vector<double>& times = authority_frequency_map[it_authority->first];
          if (times.size() > 1)
          {
            double total_time = times.back() - times.front();
            cout << "  Frequency: " << (times.size() - 1) / total_time << " Hz" << endl;
          }
        }
      }
    }
  }

};

/**
 * @brief 主函数，TF监控工具的入口点。
 * @param argc 命令行参数数量。
 * @param argv 命令行参数数组。
 * @return 程序退出码。
 * @details 该函数解析命令行参数，根据参数数量决定是监控特定变换链还是所有TF变换。
 *          然后创建一个TFMonitor对象，并周期性地打印TF变换信息。
 *          用法: tf_monitor [source_frame] [target_frame]
 */
int main(int argc, char ** argv)
{
  ros::init(argc, argv, "tf_monitor");

  TFMonitor* monitor;
  if (argc == 3)
  {
    // 监控特定变换链
    monitor = new TFMonitor(true, argv[1], argv[2]);
  }
  else if (argc == 1)
  {
    // 监控所有TF变换
    monitor = new TFMonitor(false);
  }
  else
  {
    // 参数错误，打印使用说明
    printf("Usage: tf_monitor [source_frame] [target_frame]\n");
    return -1;
  }

  ros::Rate rate(1.0);
  while (ros::ok())
  {
    monitor->print_info(); // 打印TF信息
    ros::spinOnce(); // 处理ROS回调
    rate.sleep(); // 休眠以控制频率
  }

  delete monitor;
  return 0;
}
