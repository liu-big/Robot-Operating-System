/*
 * Copyright (c) 2009, Willow Garage, Inc.
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

/** \author Tully Foote */

/**
 * @file change_notifier.cpp
 * @brief TF状态变化通知程序。
 * 该程序用于提供TF状态变化的通知，例如机器人位置的更新，
 * 主要用于按需更新Web图形，不建议用于实时操作反馈。
 */

#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "xmlrpcpp/XmlRpcValue.h"

/**
 * @brief FramePair类，用于存储源帧和目标帧之间的关系以及更新距离。
 */
class FramePair
{
public:
  /**
   * @brief FramePair构造函数。
   * @param source_frame 源帧的名称。
   * @param target_frame 目标帧的名称。
   * @param translational_update_distance 平移更新距离阈值。
   * @param angular_update_distance 角度更新距离阈值。
   * @details 初始化FramePair对象，设置源帧、目标帧以及平移和角度更新距离。
   */
  FramePair(const std::string& source_frame, const std::string& target_frame, double translational_update_distance, double angular_update_distance) :
    source_frame_(source_frame),
    target_frame_(target_frame),
    translational_update_distance_(translational_update_distance),
    angular_update_distance_(angular_update_distance)
  {
    // 初始化pose_in_，表示源帧中的姿态，初始为单位姿态。
    pose_in_ = tf::Stamped<tf::Pose>(tf::Pose(tf::createIdentityQuaternion(), tf::Vector3(0, 0, 0)), ros::Time(), source_frame_);
  }

public:
  std::string source_frame_; ///< 源帧名称。
  std::string target_frame_; ///< 目标帧名称。

  tf::Stamped<tf::Pose> pose_in_; ///< 输入姿态，通常是源帧中的姿态。
  tf::Stamped<tf::Pose> pose_out_; ///< 输出姿态，通常是目标帧中的姿态。
  tf::Stamped<tf::Pose> last_sent_pose_; ///< 上次发送的姿态，用于判断是否需要更新。
  
  double translational_update_distance_; ///< 平移更新距离阈值。
  double angular_update_distance_; ///< 角度更新距离阈值。
};

/**
 * @brief 从ROS参数服务器获取帧对配置。
 * @param local_node ROS节点句柄。
 * @param frame_pairs 存储解析出的FramePair对象的向量。
 * @param default_translational_update_distance 默认的平移更新距离。
 * @param default_angular_update_distance 默认的角度更新距离。
 * @return 如果成功获取帧对则返回true，否则返回false。
 * @details 从ROS参数服务器读取帧对配置，并将其存储在frame_pairs向量中。
 */
bool getFramePairs(const ros::NodeHandle& local_node, std::vector<FramePair>& frame_pairs, double default_translational_update_distance, double default_angular_update_distance)
{
  XmlRpc::XmlRpcValue frame_pairs_param;
  // 尝试从参数服务器获取 "frame_pairs" 参数
  if (!local_node.getParam("frame_pairs", frame_pairs_param))
  {
    // 如果未提供 "frame_pairs" 参数，则默认使用 base_link->map
    frame_pairs.push_back(FramePair("base_link", "map", default_translational_update_distance, default_angular_update_distance));
    return true;
  }

  // 检查参数类型是否为数组
  if (frame_pairs_param.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("Expecting a list for frame_pairs parameter");
    return false;
  }
  // 遍历数组中的每个帧对配置
  for (int i = 0; i < frame_pairs_param.size(); i++)
  {
    XmlRpc::XmlRpcValue frame_pair_param = frame_pairs_param[i];
    // 检查帧对配置类型是否为结构体
    if (frame_pair_param.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      ROS_ERROR("frame_pairs must be specified as maps, but they are XmlRpcType: %d", frame_pair_param.getType());
      return false;
    }

    // 获取源帧 (source_frame)
    if (!frame_pair_param.hasMember("source_frame"))
    {
      ROS_ERROR("frame_pair does not specified source_frame");
      return false;
    }
    XmlRpc::XmlRpcValue source_frame_param = frame_pair_param["source_frame"];
    if (source_frame_param.getType() != XmlRpc::XmlRpcValue::TypeString)
    {
      ROS_ERROR("source_frame must be a string, but it is XmlRpcType: %d", source_frame_param.getType());
      return false;
    }
    std::string source_frame = source_frame_param;

    // 获取目标帧 (target_frame)
    if (!frame_pair_param.hasMember("target_frame"))
    {
      ROS_ERROR("frame_pair does not specified target_frame");
      return false;
    }
    XmlRpc::XmlRpcValue target_frame_param = frame_pair_param["target_frame"];
    if (target_frame_param.getType() != XmlRpc::XmlRpcValue::TypeString)
    {
      ROS_ERROR("target_frame must be a string, but it is XmlRpcType: %d", target_frame_param.getType());
      return false;
    }
    std::string target_frame = target_frame_param;

    // 获取可选的平移更新距离 (translational_update_distance)
    double translational_update_distance = default_translational_update_distance;
    if (frame_pair_param.hasMember("translational_update_distance"))
    {
      XmlRpc::XmlRpcValue translational_update_distance_param = frame_pair_param["translational_update_distance"];
      if (translational_update_distance_param.getType() != XmlRpc::XmlRpcValue::TypeDouble &&
          translational_update_distance_param.getType() != XmlRpc::XmlRpcValue::TypeInt)
      {
        ROS_ERROR("translational_update_distance must be either an integer or a double, but it is XmlRpcType: %d", translational_update_distance_param.getType());
        return false;
      }
      translational_update_distance = translational_update_distance_param;
    }

    // 获取可选的角度更新距离 (angular_update_distance)
    double angular_update_distance = default_angular_update_distance;
    if (frame_pair_param.hasMember("angular_update_distance"))
    {
      XmlRpc::XmlRpcValue angular_update_distance_param = frame_pair_param["angular_update_distance"];
      if (angular_update_distance_param.getType() != XmlRpc::XmlRpcValue::TypeDouble &&
          angular_update_distance_param.getType() != XmlRpc::XmlRpcValue::TypeInt)
      {
        ROS_ERROR("angular_update_distance must be either an integer or a double, but it is XmlRpcType: %d", angular_update_distance_param.getType());
        return false;
      }
      angular_update_distance = angular_update_distance_param;
    }

    ROS_INFO("Notifying change on %s -> %s (translational update distance: %.4f, angular update distance: %.4f)", source_frame.c_str(), target_frame.c_str(), translational_update_distance, angular_update_distance);

    // 将解析出的帧对添加到向量中
    frame_pairs.push_back(FramePair(source_frame, target_frame, translational_update_distance, angular_update_distance));
  }

  return true;
}

/**
 * @brief 主函数，TF状态变化通知程序的入口点。
 * @param argc 命令行参数数量。
 * @param argv 命令行参数数组。
 * @return 程序退出码。
 * @details 初始化ROS节点，设置参数，并循环检查TF变换是否需要更新。
 */
int main(int argc, char** argv)
{
  // 初始化ROS节点
  ros::init(argc, argv, "change_notifier", ros::init_options::AnonymousName);
  ros::NodeHandle node; // 全局节点句柄
  ros::NodeHandle local_node("~"); // 私有节点句柄

  double polling_frequency, translational_update_distance, angular_update_distance;
  // 从参数服务器获取轮询频率、平移更新距离和角度更新距离
  local_node.param(std::string("polling_frequency"),             polling_frequency,             10.0);
  local_node.param(std::string("translational_update_distance"), translational_update_distance,  0.10);
  local_node.param(std::string("angular_update_distance"),       angular_update_distance,        0.10);

  std::vector<FramePair> frame_pairs;
  // 获取帧对配置
  if (!getFramePairs(local_node, frame_pairs, translational_update_distance, angular_update_distance))
  {
    ROS_ERROR("Failed to get frame pairs from parameter server.");
    return -1;
  }

  tf::TransformListener tf;

  ros::Rate rate(polling_frequency);
  while(node.ok())
  {
    for (std::vector<FramePair>::iterator it = frame_pairs.begin(); it != frame_pairs.end(); ++it)
    {
      try
      {
        // 尝试获取从源帧到目标帧的最新变换
        tf.lookupLatest(it->target_frame_, it->source_frame_, it->pose_in_.stamp_, it->pose_out_);
      }
      catch (tf::TransformException& ex)
      {
        ROS_ERROR("Failure %s\n", ex.what());
      }

      // 计算当前姿态与上次发送姿态之间的平移和角度距离
      double translational_distance = it->pose_out_.getOrigin().distance(it->last_sent_pose_.getOrigin());
      double angular_distance = it->pose_out_.getRotation().angleShortestPath(it->last_sent_pose_.getRotation());

      // 如果距离超过阈值，则认为需要通知更新
      if (translational_distance > it->translational_update_distance_ ||
          angular_distance > it->angular_update_distance_)
      {
        ROS_INFO("Notifying change on %s -> %s (translational distance: %.4f, angular distance: %.4f)", it->source_frame_.c_str(), it->target_frame_.c_str(), translational_distance, angular_distance);
        it->last_sent_pose_ = it->pose_out_;
      }
    }
    rate.sleep();
  }

  return 0;
}
