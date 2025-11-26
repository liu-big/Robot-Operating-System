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
 * 
 * 作者: Adam Leeper
 */

// 防止头文件被多次包含，避免重复定义错误。
#ifndef CONVERSIONS_KDL_MSG_H
#define CONVERSIONS_KDL_MSG_H

// 包含 KDL 库的 frames.hpp 头文件，定义了 KDL 的几何类型，如 Frame, Vector, Rotation 等。
#include "kdl/frames.hpp"
// 包含 ROS geometry_msgs 消息类型头文件，用于定义各种几何消息，如点、姿态、四元数、变换、扭曲、向量和力。
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Wrench.h>

// 包含 ROS 宏定义，例如用于标记废弃函数的 ROS_DEPRECATED 宏。
#include <ros/macros.h>

// 定义 tf 命名空间，所有转换函数都在此命名空间下。
namespace tf {
/// KDL 和 geometry_msgs 类型之间相互转换的函数。

/**
 * @brief 将 geometry_msgs::Point 消息转换为 KDL::Vector。
 * @param m 输入的 geometry_msgs::Point 消息。
 * @param k 输出的 KDL::Vector。
 */
void pointMsgToKDL(const geometry_msgs::Point &m, KDL::Vector &k);

/**
 * @brief 将 KDL::Vector 转换为 geometry_msgs::Point 消息。
 * @param k 输入的 KDL::Vector。
 * @param m 输出的 geometry_msgs::Point 消息。
 */
void pointKDLToMsg(const KDL::Vector &k, geometry_msgs::Point &m);

/**
 * @brief 将 geometry_msgs::Pose 消息转换为 KDL::Frame。
 * @param m 输入的 geometry_msgs::Pose 消息。
 * @param k 输出的 KDL::Frame。
 */
void poseMsgToKDL(const geometry_msgs::Pose &m, KDL::Frame &k);

/**
 * @brief 将 KDL::Frame 转换为 geometry_msgs::Pose 消息。
 * @param k 输入的 KDL::Frame。
 * @param m 输出的 geometry_msgs::Pose 消息。
 */
void poseKDLToMsg(const KDL::Frame &k, geometry_msgs::Pose &m);

/**
 * @brief 将 geometry_msgs::Quaternion 消息转换为 KDL::Rotation。
 * @param m 输入的 geometry_msgs::Quaternion 消息。
 * @param k 输出的 KDL::Rotation。
 */
void quaternionMsgToKDL(const geometry_msgs::Quaternion &m, KDL::Rotation &k);

/**
 * @brief 将 KDL::Rotation 转换为 geometry_msgs::Quaternion 消息。
 * @param k 输入的 KDL::Rotation。
 * @param m 输出的 geometry_msgs::Quaternion 消息。
 */
void quaternionKDLToMsg(const KDL::Rotation &k, geometry_msgs::Quaternion &m);

/**
 * @brief 将 geometry_msgs::Transform 消息转换为 KDL::Frame。
 * @param m 输入的 geometry_msgs::Transform 消息。
 * @param k 输出的 KDL::Frame。
 */
void transformMsgToKDL(const geometry_msgs::Transform &m, KDL::Frame &k);

/**
 * @brief 将 KDL::Frame 转换为 geometry_msgs::Transform 消息。
 * @param k 输入的 KDL::Frame。
 * @param m 输出的 geometry_msgs::Transform 消息。
 */
void transformKDLToMsg(const KDL::Frame &k, geometry_msgs::Transform &m);

/**
 * @brief 将 geometry_msgs::Twist 消息转换为 KDL::Twist。
 * @param m 输入的 geometry_msgs::Twist 消息。
 * @param k 输出的 KDL::Twist。
 */
void twistMsgToKDL(const geometry_msgs::Twist &m, KDL::Twist &k);

/**
 * @brief 将 KDL::Twist 转换为 geometry_msgs::Twist 消息。
 * @param k 输入的 KDL::Twist。
 * @param m 输出的 geometry_msgs::Twist 消息。
 */
void twistKDLToMsg(const KDL::Twist &k, geometry_msgs::Twist &m);

/**
 * @brief 将 geometry_msgs::Vector3 消息转换为 KDL::Vector。
 * @param m 输入的 geometry_msgs::Vector3 消息。
 * @param k 输出的 KDL::Vector。
 */
void vectorMsgToKDL(const geometry_msgs::Vector3 &m, KDL::Vector &k);

/**
 * @brief 将 KDL::Vector 转换为 geometry_msgs::Vector3 消息。
 * @param k 输入的 KDL::Vector。
 * @param m 输出的 geometry_msgs::Vector3 消息。
 */
void vectorKDLToMsg(const KDL::Vector &k, geometry_msgs::Vector3 &m);

/**
 * @brief 将 geometry_msgs::Wrench 消息转换为 KDL::Wrench。
 * @param m 输入的 geometry_msgs::Wrench 消息。
 * @param k 输出的 KDL::Wrench。
 */
void wrenchMsgToKDL(const geometry_msgs::Wrench &m, KDL::Wrench &k);

/**
 * @brief 将 KDL::Wrench 转换为 geometry_msgs::Wrench 消息。
 * @param k 输入的 KDL::Wrench。
 * @param m 输出的 geometry_msgs::Wrench 消息。
 */
void wrenchKDLToMsg(const KDL::Wrench &k, geometry_msgs::Wrench &m);


// 以下是已废弃的方法，建议使用上面没有 ROS_DEPRECATED 宏标记的新方法。

/**
 * @brief (已废弃) 将 geometry_msgs::Pose 消息转换为 KDL::Frame。
 * @param m 输入的 geometry_msgs::Pose 消息。
 * @param k 输出的 KDL::Frame。
 */
ROS_DEPRECATED void PoseMsgToKDL(const geometry_msgs::Pose &m, KDL::Frame &k);

/**
 * @brief (已废弃) 将 KDL::Frame 转换为 geometry_msgs::Pose 消息。
 * @param k 输入的 KDL::Frame。
 * @param m 输出的 geometry_msgs::Pose 消息。
 */
ROS_DEPRECATED void PoseKDLToMsg(const KDL::Frame &k, geometry_msgs::Pose &m);

/**
 * @brief (已废弃) 将 geometry_msgs::Twist 消息转换为 KDL::Twist。
 * @param m 输入的 geometry_msgs::Twist 消息。
 * @param k 输出的 KDL::Twist。
 */
ROS_DEPRECATED void TwistMsgToKDL(const geometry_msgs::Twist &m, KDL::Twist &k);

/**
 * @brief (已废弃) 将 KDL::Twist 转换为 geometry_msgs::Twist 消息。
 * @param k 输入的 KDL::Twist。
 * @param m 输出的 geometry_msgs::Twist 消息。
 */
ROS_DEPRECATED void TwistKDLToMsg(const KDL::Twist &k, geometry_msgs::Twist &m);


}


#endif



