/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

// 包含 kdl_conversions 包的 kdl_msg.h 头文件，该文件定义了 KDL 和 geometry_msgs 之间的转换函数声明。
#include "kdl_conversions/kdl_msg.h"

// 定义 tf 命名空间，所有转换函数的实现都在此命名空间下。
namespace tf {

  /**
   * @brief 将 geometry_msgs::Point 消息转换为 KDL::Vector。
   * @param m 输入的 geometry_msgs::Point 消息。
   * @param k 输出的 KDL::Vector。
   */
  void pointMsgToKDL(const geometry_msgs::Point &m, KDL::Vector &k)
  {
    k[0] = m.x;
    k[1] = m.y;
    k[2] = m.z;
  }

  /**
   * @brief 将 KDL::Vector 转换为 geometry_msgs::Point 消息。
   * @param k 输入的 KDL::Vector。
   * @param m 输出的 geometry_msgs::Point 消息。
   */
  void pointKDLToMsg(const KDL::Vector &k, geometry_msgs::Point &m)
  {
    m.x = k[0];
    m.y = k[1];
    m.z = k[2];
  }

  /**
   * @brief 将 geometry_msgs::Pose 消息转换为 KDL::Frame。
   *        Pose 包含位置 (Point) 和方向 (Quaternion)。
   * @param m 输入的 geometry_msgs::Pose 消息。
   * @param k 输出的 KDL::Frame。
   */
  void poseMsgToKDL(const geometry_msgs::Pose &m, KDL::Frame &k)
  {
    // 转换位置部分
    k.p[0] = m.position.x;
    k.p[1] = m.position.y;
    k.p[2] = m.position.z;
    
    // 转换方向部分，使用 KDL::Rotation::Quaternion 从四元数创建旋转矩阵。
    k.M = KDL::Rotation::Quaternion( m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w);
  }

  /**
   * @brief 将 KDL::Frame 转换为 geometry_msgs::Pose 消息。
   * @param k 输入的 KDL::Frame。
   * @param m 输出的 geometry_msgs::Pose 消息。
   */
  void poseKDLToMsg(const KDL::Frame &k, geometry_msgs::Pose &m)
  {
    // 转换位置部分
    m.position.x = k.p[0];
    m.position.y = k.p[1];
    m.position.z = k.p[2];
    
    // 转换方向部分，从 KDL::Rotation 获取四元数。
    k.M.GetQuaternion(m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w);
  }

  /**
   * @brief 将 geometry_msgs::Quaternion 消息转换为 KDL::Rotation。
   * @param m 输入的 geometry_msgs::Quaternion 消息。
   * @param k 输出的 KDL::Rotation。
   */
  void quaternionMsgToKDL(const geometry_msgs::Quaternion &m, KDL::Rotation &k)
  {
    k = KDL::Rotation::Quaternion(m.x, m.y, m.z, m.w);
  }

  /**
   * @brief 将 KDL::Rotation 转换为 geometry_msgs::Quaternion 消息。
   * @param k 输入的 KDL::Rotation。
   * @param m 输出的 geometry_msgs::Quaternion 消息。
   */
  void quaternionKDLToMsg(const KDL::Rotation &k, geometry_msgs::Quaternion &m)
  {
    k.GetQuaternion(m.x, m.y, m.z, m.w);
  }

  /**
   * @brief 将 geometry_msgs::Transform 消息转换为 KDL::Frame。
   *        Transform 包含平移 (Vector3) 和旋转 (Quaternion)。
   * @param m 输入的 geometry_msgs::Transform 消息。
   * @param k 输出的 KDL::Frame。
   */
  void transformMsgToKDL(const geometry_msgs::Transform &m, KDL::Frame &k)
  {
    // 转换平移部分
    k.p[0] = m.translation.x;
    k.p[1] = m.translation.y;
    k.p[2] = m.translation.z;
    
    // 转换旋转部分
    k.M = KDL::Rotation::Quaternion( m.rotation.x, m.rotation.y, m.rotation.z, m.rotation.w);
  }

  /**
   * @brief 将 KDL::Frame 转换为 geometry_msgs::Transform 消息。
   * @param k 输入的 KDL::Frame。
   * @param m 输出的 geometry_msgs::Transform 消息。
   */
  void transformKDLToMsg(const KDL::Frame &k, geometry_msgs::Transform &m)
  {
    // 转换平移部分
    m.translation.x = k.p[0];
    m.translation.y = k.p[1];
    m.translation.z = k.p[2];
    
    // 转换旋转部分
    k.M.GetQuaternion(m.rotation.x, m.rotation.y, m.rotation.z, m.rotation.w);
  }

  /**
   * @brief 将 KDL::Twist 转换为 geometry_msgs::Twist 消息。
   *        Twist 包含线速度 (linear) 和角速度 (angular)。
   * @param t 输入的 KDL::Twist。
   * @param m 输出的 geometry_msgs::Twist 消息。
   */
  void twistKDLToMsg(const KDL::Twist &t, geometry_msgs::Twist &m)
  {
    m.linear.x = t.vel[0];
    m.linear.y = t.vel[1];
    m.linear.z = t.vel[2];
    m.angular.x = t.rot[0];
    m.angular.y = t.rot[1];
    m.angular.z = t.rot[2];
  }

  /**
   * @brief 将 geometry_msgs::Twist 消息转换为 KDL::Twist。
   * @param m 输入的 geometry_msgs::Twist 消息。
   * @param t 输出的 KDL::Twist。
   */
  void twistMsgToKDL(const geometry_msgs::Twist &m, KDL::Twist &t)
  {
    t.vel[0] = m.linear.x;
    t.vel[1] = m.linear.y;
    t.vel[2] = m.linear.z;
    t.rot[0] = m.angular.x;
    t.rot[1] = m.angular.y;
    t.rot[2] = m.angular.z;
  }

  /**
   * @brief 将 geometry_msgs::Vector3 消息转换为 KDL::Vector。
   * @param m 输入的 geometry_msgs::Vector3 消息。
   * @param k 输出的 KDL::Vector。
   */
  void vectorMsgToKDL(const geometry_msgs::Vector3 &m, KDL::Vector &k)
  {
    k[0] = m.x;
    k[1] = m.y;
    k[2] = m.z;
  }

  /**
   * @brief 将 KDL::Vector 转换为 geometry_msgs::Vector3 消息。
   * @param k 输入的 KDL::Vector。
   * @param m 输出的 geometry_msgs::Vector3 消息。
   */
  void vectorKDLToMsg(const KDL::Vector &k, geometry_msgs::Vector3 &m)
  {
    m.x = k[0];
    m.y = k[1];
    m.z = k[2];
  }

  /**
   * @brief 将 geometry_msgs::Wrench 消息转换为 KDL::Wrench。
   *        Wrench 包含力和力矩。
   * @param m 输入的 geometry_msgs::Wrench 消息。
   * @param k 输出的 KDL::Wrench。
   */
  void wrenchMsgToKDL(const geometry_msgs::Wrench &m, KDL::Wrench &k)
  {
    k[0] = m.force.x;
    k[1] = m.force.y;
    k[2] = m.force.z;
    k[3] = m.torque.x;
    k[4] = m.torque.y;
    k[5] = m.torque.z;
  }

  /**
   * @brief 将 KDL::Wrench 转换为 geometry_msgs::Wrench 消息。
   * @param k 输入的 KDL::Wrench。
   * @param m 输出的 geometry_msgs::Wrench 消息。
   */
  void wrenchKDLToMsg(const KDL::Wrench &k, geometry_msgs::Wrench &m)
  {
    m.force.x  = k[0];
    m.force.y  = k[1];
    m.force.z  = k[2];
    m.torque.x = k[3];
    m.torque.y = k[4];
    m.torque.z = k[5];
  }


  // 以下是为向后兼容性而提供的已废弃方法。

  /**
   * @brief (已废弃) 将 geometry_msgs::Pose 消息转换为 KDL::Frame。
   *        此函数内部调用 poseMsgToKDL。
   * @param m 输入的 geometry_msgs::Pose 消息。
   * @param k 输出的 KDL::Frame。
   */
  void PoseMsgToKDL(const geometry_msgs::Pose &m, KDL::Frame &k) { poseMsgToKDL(m, k);}

  /**
   * @brief (已废弃) 将 KDL::Frame 转换为 geometry_msgs::Pose 消息。
   *        此函数内部调用 poseKDLToMsg。
   * @param k 输入的 KDL::Frame。
   * @param m 输出的 geometry_msgs::Pose 消息。
   */
  void PoseKDLToMsg(const KDL::Frame &k, geometry_msgs::Pose &m) { poseKDLToMsg(k, m);}

  /**
   * @brief (已废弃) 将 geometry_msgs::Twist 消息转换为 KDL::Twist。
   *        此函数内部调用 twistMsgToKDL。
   * @param m 输入的 geometry_msgs::Twist 消息。
   * @param k 输出的 KDL::Twist。
   */
  void TwistMsgToKDL(const geometry_msgs::Twist &m, KDL::Twist &k) {twistMsgToKDL(m, k);};

  /**
   * @brief (已废弃) 将 KDL::Twist 转换为 geometry_msgs::Twist 消息。
   *        此函数内部调用 twistKDLToMsg。
   * @param k 输入的 KDL::Twist。
   * @param m 输出的 geometry_msgs::Twist 消息。
   */
  void TwistKDLToMsg(const KDL::Twist &k, geometry_msgs::Twist &m){twistKDLToMsg(k, m);};


}  // namespace tf

