/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of code must retain the above copyright
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
 * @file transform_listener.cpp
 * @brief 这是一个ROS TF（Transform Frame）库中用于监听和查询变换的C++实现。
 * 它提供了多种方法来获取和转换不同类型的数据（如点、向量、姿态等）在不同坐标系之间的变换。
 * @author Tully Foote
 */

#include "tf/transform_listener.h"


using namespace tf;

/**
 * @brief 对帧ID进行重映射。
 * 此函数通过ROS参数服务器获取前缀，并将其应用于给定的帧ID，实现帧ID的重映射。
 * @param frame_id 原始的帧ID。
 * @return 重映射后的帧ID。
 */
std::string tf::remap(const std::string& frame_id)
{
  ros::NodeHandle n("~");
  return tf::resolve(getPrefixParam(n), frame_id);
}

/**
 * @brief TransformListener类的构造函数。
 * 初始化TF监听器，设置缓存时间，并可选择是否在单独线程中进行自旋。
 * @param max_cache_time 变换数据的最大缓存时间。
 * @param spin_thread 是否在单独的线程中自旋ROS回调。
 */
TransformListener::TransformListener(ros::Duration max_cache_time, bool spin_thread):
  Transformer(true, max_cache_time), tf2_listener_(*Transformer::tf2_buffer_ptr_, node_, spin_thread)
{
  // 所有初始化都在tf2内部完成
}

/**
 * @brief TransformListener类的构造函数（带NodeHandle参数）。
 * 使用给定的ROS NodeHandle初始化TF监听器，设置缓存时间，并可选择是否在单独线程中进行自旋。
 * @param nh ROS NodeHandle对象。
 * @param max_cache_time 变换数据的最大缓存时间。
 * @param spin_thread 是否在单独的线程中自旋ROS回调。
 */
TransformListener::TransformListener(const ros::NodeHandle& nh, ros::Duration max_cache_time, bool spin_thread):
  Transformer(true, max_cache_time), node_(nh), tf2_listener_(*Transformer::tf2_buffer_ptr_, nh, spin_thread)
{
  // 所有初始化都在tf2内部完成
}

/**
 * @brief TransformListener类的析构函数。
 * 清理资源。
 */
TransformListener::~TransformListener()
{
  // 所有清理都在tf2内部完成
}

/**
 * @brief 检查ROS系统是否正常运行。
 * @return 如果ROS正常运行则返回true，否则返回false。
 */
//Override Transformer::ok() for ticket:4882
bool TransformListener::ok() const { return ros::ok(); }


/**
 * @brief 将QuaternionStamped消息从一个坐标系转换到另一个坐标系。
 * @param target_frame 目标坐标系的名称。
 * @param msg_in 输入的QuaternionStamped消息。
 * @param msg_out 转换后的QuaternionStamped消息。
 */
void TransformListener::transformQuaternion(const std::string& target_frame,
    const geometry_msgs::QuaternionStamped& msg_in,
    geometry_msgs::QuaternionStamped& msg_out) const
{
  tf::assertQuaternionValid(msg_in.quaternion);

  Stamped<Quaternion> pin, pout;
  quaternionStampedMsgToTF(msg_in, pin);  
  transformQuaternion(target_frame, pin, pout);
  quaternionStampedTFToMsg(pout, msg_out);
}

/**
 * @brief 将Vector3Stamped消息从一个坐标系转换到另一个坐标系。
 * @param target_frame 目标坐标系的名称。
 * @param msg_in 输入的Vector3Stamped消息。
 * @param msg_out 转换后的Vector3Stamped消息。
 */
void TransformListener::transformVector(const std::string& target_frame,
    const geometry_msgs::Vector3Stamped& msg_in,
    geometry_msgs::Vector3Stamped& msg_out) const
{
  Stamped<Vector3> pin, pout;
  vector3StampedMsgToTF(msg_in, pin);
  transformVector(target_frame, pin, pout);
  vector3StampedTFToMsg(pout, msg_out);
}

/**
 * @brief 将PointStamped消息从一个坐标系转换到另一个坐标系。
 * @param target_frame 目标坐标系的名称。
 * @param msg_in 输入的PointStamped消息。
 * @param msg_out 转换后的PointStamped消息。
 */
void TransformListener::transformPoint(const std::string& target_frame,
    const geometry_msgs::PointStamped& msg_in,
    geometry_msgs::PointStamped& msg_out) const
{
  Stamped<Point> pin, pout;
  pointStampedMsgToTF(msg_in, pin);
  transformPoint(target_frame, pin, pout);
  pointStampedTFToMsg(pout, msg_out);
}

/**
 * @brief 将PoseStamped消息从一个坐标系转换到另一个坐标系。
 * @param target_frame 目标坐标系的名称。
 * @param msg_in 输入的PoseStamped消息。
 * @param msg_out 转换后的PoseStamped消息。
 */
void TransformListener::transformPose(const std::string& target_frame,
    const geometry_msgs::PoseStamped& msg_in,
    geometry_msgs::PoseStamped& msg_out) const
{
  tf::assertQuaternionValid(msg_in.pose.orientation);

  Stamped<Pose> pin, pout;
  poseStampedMsgToTF(msg_in, pin);
  transformPose(target_frame, pin, pout);
  poseStampedTFToMsg(pout, msg_out);
}
/* http://www.ros.org/wiki/tf/Reviews/2010-03-12_API_Review
void TransformListener::transformTwist(const std::string& target_frame,
    const geometry_msgs::TwistStamped& msg_in,
    geometry_msgs::TwistStamped& msg_out) const
{
  tf::Vector3 twist_rot(msg_in.twist.angular.x,
                        msg_in.twist.angular.y,
                        msg_in.twist.angular.z);
  tf::Vector3 twist_vel(msg_in.twist.linear.x,
                        msg_in.twist.linear.y,
                        msg_in.twist.linear.z);

  tf::StampedTransform transform;
  lookupTransform(target_frame,msg_in.header.frame_id,  msg_in.header.stamp, transform);


  tf::Vector3 out_rot = transform.getBasis() * twist_rot;
  tf::Vector3 out_vel = transform.getBasis()* twist_vel + transform.getOrigin().cross(out_rot);

  geometry_msgs::TwistStamped interframe_twist;
  lookupVelocity(target_frame, msg_in.header.frame_id, msg_in.header.stamp, ros::Duration(0.1), interframe_twist); //\todo get rid of hard coded number

  msg_out.header.stamp = msg_in.header.stamp;
  msg_out.header.frame_id = target_frame;
  msg_out.twist.linear.x =  out_vel.x() + interframe_twist.twist.linear.x;
  msg_out.twist.linear.y =  out_vel.y() + interframe_twist.twist.linear.y;
  msg_out.twist.linear.z =  out_vel.z() + interframe_twist.twist.linear.z;
  msg_out.twist.angular.x =  out_rot.x() + interframe_twist.twist.angular.x;
  msg_out.twist.angular.y =  out_rot.y() + interframe_twist.twist.angular.y;
  msg_out.twist.angular.z =  out_rot.z() + interframe_twist.twist.angular.z;

  }*/

/**
 * @brief 将QuaternionStamped消息从一个坐标系在特定时间点转换到另一个坐标系。
 * @param target_frame 目标坐标系的名称。
 * @param target_time 目标时间点。
 * @param msg_in 输入的QuaternionStamped消息。
 * @param fixed_frame 固定坐标系的名称。
 * @param msg_out 转换后的QuaternionStamped消息。
 */
void TransformListener::transformQuaternion(const std::string& target_frame, const ros::Time& target_time,
    const geometry_msgs::QuaternionStamped& msg_in,
    const std::string& fixed_frame, geometry_msgs::QuaternionStamped& msg_out) const
{
  tf::assertQuaternionValid(msg_in.quaternion);
  Stamped<Quaternion> pin, pout;
  quaternionStampedMsgToTF(msg_in, pin);
  transformQuaternion(target_frame, target_time, pin, fixed_frame, pout);
  quaternionStampedTFToMsg(pout, msg_out);
}

/**
 * @brief 将Vector3Stamped消息从一个坐标系在特定时间点转换到另一个坐标系。
 * @param target_frame 目标坐标系的名称。
 * @param target_time 目标时间点。
 * @param msg_in 输入的Vector3Stamped消息。
 * @param fixed_frame 固定坐标系的名称。
 * @param msg_out 转换后的Vector3Stamped消息。
 */
void TransformListener::transformVector(const std::string& target_frame, const ros::Time& target_time,
    const geometry_msgs::Vector3Stamped& msg_in,
    const std::string& fixed_frame, geometry_msgs::Vector3Stamped& msg_out) const
{
  Stamped<Vector3> pin, pout;
  vector3StampedMsgToTF(msg_in, pin);
  transformVector(target_frame, target_time, pin, fixed_frame, pout);
  vector3StampedTFToMsg(pout, msg_out);
}

/**
 * @brief 将PointStamped消息从一个坐标系在特定时间点转换到另一个坐标系。
 * @param target_frame 目标坐标系的名称。
 * @param target_time 目标时间点。
 * @param msg_in 输入的PointStamped消息。
 * @param fixed_frame 固定坐标系的名称。
 * @param msg_out 转换后的PointStamped消息。
 */
void TransformListener::transformPoint(const std::string& target_frame, const ros::Time& target_time,
    const geometry_msgs::PointStamped& msg_in,
    const std::string& fixed_frame, geometry_msgs::PointStamped& msg_out) const
{
  Stamped<Point> pin, pout;
  pointStampedMsgToTF(msg_in, pin);
  transformPoint(target_frame, target_time, pin, fixed_frame, pout);
  pointStampedTFToMsg(pout, msg_out);
}

/**
 * @brief 将PoseStamped消息从一个坐标系在特定时间点转换到另一个坐标系。
 * @param target_frame 目标坐标系的名称。
 * @param target_time 目标时间点。
 * @param msg_in 输入的PoseStamped消息。
 * @param fixed_frame 固定坐标系的名称。
 * @param msg_out 转换后的PoseStamped消息。
 */
void TransformListener::transformPose(const std::string& target_frame, const ros::Time& target_time,
    const geometry_msgs::PoseStamped& msg_in,
    const std::string& fixed_frame, geometry_msgs::PoseStamped& msg_out) const
{
  tf::assertQuaternionValid(msg_in.pose.orientation);

  Stamped<Pose> pin, pout;
  poseStampedMsgToTF(msg_in, pin);
  transformPose(target_frame, target_time, pin, fixed_frame, pout);
  poseStampedTFToMsg(pout, msg_out);
}

/**
 * @brief 将PointCloud消息从一个坐标系转换到另一个坐标系。
 * @param target_frame 目标坐标系的名称。
 * @param cloudIn 输入的PointCloud消息。
 * @param cloudOut 转换后的PointCloud消息。
 */
void TransformListener::transformPointCloud(const std::string & target_frame, const sensor_msgs::PointCloud & cloudIn, sensor_msgs::PointCloud & cloudOut) const
{
  StampedTransform transform;
  lookupTransform(target_frame, cloudIn.header.frame_id, cloudIn.header.stamp, transform);

  transformPointCloud(target_frame, transform, cloudIn.header.stamp, cloudIn, cloudOut);
}

/**
 * @brief 将PointCloud消息从一个坐标系在特定时间点转换到另一个坐标系。
 * @param target_frame 目标坐标系的名称。
 * @param target_time 目标时间点。
 * @param cloudIn 输入的PointCloud消息。
 * @param fixed_frame 固定坐标系的名称。
 * @param cloudOut 转换后的PointCloud消息。
 */
void TransformListener::transformPointCloud(const std::string& target_frame, const ros::Time& target_time, 
    const sensor_msgs::PointCloud& cloudIn,
    const std::string& fixed_frame, sensor_msgs::PointCloud& cloudOut) const
{
  StampedTransform transform;
  lookupTransform(target_frame, target_time,
      cloudIn.header.frame_id, cloudIn.header.stamp,
      fixed_frame,
      transform);

  transformPointCloud(target_frame, transform, target_time, cloudIn, cloudOut);


}

inline void transformPointMatVec(const tf::Vector3 &origin, const tf::Matrix3x3 &basis, const geometry_msgs::Point32 &in, geometry_msgs::Point32 &out)
{
  // Use temporary variables in case &in == &out
  double x = basis[0].x() * in.x + basis[0].y() * in.y + basis[0].z() * in.z + origin.x();
  double y = basis[1].x() * in.x + basis[1].y() * in.y + basis[1].z() * in.z + origin.y();
  double z = basis[2].x() * in.x + basis[2].y() * in.y + basis[2].z() * in.z + origin.z();
  
  out.x = x; out.y = y; out.z = z;
}


void TransformListener::transformPointCloud(const std::string & target_frame, const tf::Transform& net_transform,
                                            const ros::Time& target_time, const sensor_msgs::PointCloud & cloudIn, 
                                            sensor_msgs::PointCloud & cloudOut) const
{
  tf::Vector3 origin = net_transform.getOrigin();
  tf::Matrix3x3 basis  = net_transform.getBasis();

  unsigned int length = cloudIn.points.size();

  // Copy relevant data from cloudIn, if needed
  if (&cloudIn != &cloudOut)
  {
    cloudOut.header = cloudIn.header;
    cloudOut.points.resize(length);
    cloudOut.channels.resize(cloudIn.channels.size());
    for (unsigned int i = 0 ; i < cloudIn.channels.size() ; ++i)
      cloudOut.channels[i] = cloudIn.channels[i];
  }

  // Transform points
  cloudOut.header.stamp = target_time;
  cloudOut.header.frame_id = target_frame;
  for (unsigned int i = 0; i < length ; i++) {
    transformPointMatVec(origin, basis, cloudIn.points[i], cloudOut.points[i]);
  }
}
