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
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * 作者: Stuart Glaser
 */

#ifndef EIGEN_MSG_CONVERSIONS_H // 如果未定义 EIGEN_MSG_CONVERSIONS_H，则定义它，防止头文件被多次包含
#define EIGEN_MSG_CONVERSIONS_H // 定义 EIGEN_MSG_CONVERSIONS_H

// 包含ROS标准消息类型和几何消息类型头文件
#include <std_msgs/Float64MultiArray.h> // 包含 std_msgs/Float64MultiArray 消息类型定义
#include <geometry_msgs/Point.h>        // 包含 geometry_msgs/Point 消息类型定义
#include <geometry_msgs/Pose.h>         // 包含 geometry_msgs/Pose 消息类型定义
#include <geometry_msgs/Quaternion.h>   // 包含 geometry_msgs/Quaternion 消息类型定义
#include <geometry_msgs/Transform.h>    // 包含 geometry_msgs/Transform 消息类型定义
#include <geometry_msgs/Twist.h>        // 包含 geometry_msgs/Twist 消息类型定义
#include <geometry_msgs/Vector3.h>      // 包含 geometry_msgs/Vector3 消息类型定义
#include <geometry_msgs/Wrench.h>       // 包含 geometry_msgs/Wrench 消息类型定义

// 包含Eigen库核心和几何模块头文件
#include <Eigen/Core>     // 包含 Eigen 核心模块，提供矩阵和向量的基本定义
#include <Eigen/Geometry> // 包含 Eigen 几何模块，提供几何变换（如旋转、平移）的定义

namespace tf { // 定义 tf 命名空间，用于存放与 ROS tf 相关的转换函数

/// 将 Point 消息转换为 Eigen Vector
/// @param m 输入的 geometry_msgs::Point 消息
/// @param e 输出的 Eigen::Vector3d 向量
void pointMsgToEigen(const geometry_msgs::Point &m, Eigen::Vector3d &e);

/// 将 Eigen Vector 转换为 Point 消息
/// @param e 输入的 Eigen::Vector3d 向量
/// @param m 输出的 geometry_msgs::Point 消息
void pointEigenToMsg(const Eigen::Vector3d &e, geometry_msgs::Point &m);

/// 将 Pose 消息转换为 Eigen Affine3d
/// @param m 输入的 geometry_msgs::Pose 消息
/// @param e 输出的 Eigen::Affine3d 仿射变换矩阵
void poseMsgToEigen(const geometry_msgs::Pose &m, Eigen::Affine3d &e);

/// 将 Pose 消息转换为 Eigen Isometry3d
/// @param m 输入的 geometry_msgs::Pose 消息
/// @param e 输出的 Eigen::Isometry3d 等距变换矩阵
void poseMsgToEigen(const geometry_msgs::Pose &m, Eigen::Isometry3d &e);

/// 将 Eigen Affine3d 转换为 Pose 消息
/// @param e 输入的 Eigen::Affine3d 仿射变换矩阵
/// @param m 输出的 geometry_msgs::Pose 消息
void poseEigenToMsg(const Eigen::Affine3d &e, geometry_msgs::Pose &m);

/// 将 Eigen Isometry3d 转换为 Pose 消息
/// @param e 输入的 Eigen::Isometry3d 等距变换矩阵
/// @param m 输出的 geometry_msgs::Pose 消息
void poseEigenToMsg(const Eigen::Isometry3d &e, geometry_msgs::Pose &m);

/// 将 Quaternion 消息转换为 Eigen Quaternion
/// @param m 输入的 geometry_msgs::Quaternion 消息
/// @param e 输出的 Eigen::Quaterniond 四元数
void quaternionMsgToEigen(const geometry_msgs::Quaternion &m, Eigen::Quaterniond &e);

/// 将 Eigen Quaternion 转换为 Quaternion 消息
/// @param e 输入的 Eigen::Quaterniond 四元数
/// @param m 输出的 geometry_msgs::Quaternion 消息
void quaternionEigenToMsg(const Eigen::Quaterniond &e, geometry_msgs::Quaternion &m);

/// 将 Transform 消息转换为 Eigen Affine3d
/// @param m 输入的 geometry_msgs::Transform 消息
/// @param e 输出的 Eigen::Affine3d 仿射变换矩阵
void transformMsgToEigen(const geometry_msgs::Transform &m, Eigen::Affine3d &e);

/// 将 Transform 消息转换为 Eigen Isometry3d
/// @param m 输入的 geometry_msgs::Transform 消息
/// @param e 输出的 Eigen::Isometry3d 等距变换矩阵
void transformMsgToEigen(const geometry_msgs::Transform &m, Eigen::Isometry3d &e);

/// 将 Eigen Affine3d 转换为 Transform 消息
/// @param e 输入的 Eigen::Affine3d 仿射变换矩阵
/// @param m 输出的 geometry_msgs::Transform 消息
void transformEigenToMsg(const Eigen::Affine3d &e, geometry_msgs::Transform &m);

/// 将 Eigen Isometry3d 转换为 Transform 消息
/// @param e 输入的 Eigen::Isometry3d 等距变换矩阵
/// @param m 输出的 geometry_msgs::Transform 消息
void transformEigenToMsg(const Eigen::Isometry3d &e, geometry_msgs::Transform &m);

/// 将 Twist 消息转换为 Eigen 矩阵
/// @param m 输入的 geometry_msgs::Twist 消息
/// @param e 输出的 Eigen::Matrix<double,6,1> 矩阵 (表示线速度和角速度)
void twistMsgToEigen(const geometry_msgs::Twist &m, Eigen::Matrix<double,6,1> &e);

/// 将 Eigen 矩阵转换为 Twist 消息
/// @param e 输入的 Eigen::Matrix<double,6,1> 矩阵
/// @param m 输出的 geometry_msgs::Twist 消息
void twistEigenToMsg(const Eigen::Matrix<double,6,1> &e, geometry_msgs::Twist &m);

/// 将 Vector 消息转换为 Eigen Vector
/// @param m 输入的 geometry_msgs::Vector3 消息
/// @param e 输出的 Eigen::Vector3d 向量
void vectorMsgToEigen(const geometry_msgs::Vector3 &m, Eigen::Vector3d &e);

/// 将 Eigen Vector 转换为 Vector 消息
/// @param e 输入的 Eigen::Vector3d 向量
/// @param m 输出的 geometry_msgs::Vector3 消息
void vectorEigenToMsg(const Eigen::Vector3d &e, geometry_msgs::Vector3 &m);

/// 将 Wrench 消息转换为 Eigen 矩阵
/// @param m 输入的 geometry_msgs::Wrench 消息
/// @param e 输出的 Eigen::Matrix<double,6,1> 矩阵 (表示力和扭矩)
void wrenchMsgToEigen(const geometry_msgs::Wrench &m, Eigen::Matrix<double,6,1> &e);

/// 将 Eigen 矩阵转换为 Wrench 消息
/// @param e 输入的 Eigen::Matrix<double,6,1> 矩阵
/// @param m 输出的 geometry_msgs::Wrench 消息
void wrenchEigenToMsg(const Eigen::Matrix<double,6,1> &e, geometry_msgs::Wrench &m);

/// 将 Eigen 矩阵转换为 Float64MultiArray 消息
/// @tparam Derived Eigen矩阵的派生类型
/// @param e 输入的 Eigen::MatrixBase<Derived> 矩阵
/// @param m 输出的 std_msgs::Float64MultiArray 消息
template <class Derived>
void matrixEigenToMsg(const Eigen::MatrixBase<Derived> &e, std_msgs::Float64MultiArray &m)
{
  // 如果布局维度不为2，则调整为2
  if (m.layout.dim.size() != 2)
    m.layout.dim.resize(2);
  // 设置第一个维度（行）的步长和大小
  m.layout.dim[0].stride = e.rows() * e.cols();
  m.layout.dim[0].size = e.rows();
  // 设置第二个维度（列）的步长和大小
  m.layout.dim[1].stride = e.cols();
  m.layout.dim[1].size = e.cols();
  // 如果数据大小不匹配，则调整数据大小
  if ((int)m.data.size() != e.size())
    m.data.resize(e.size());
  int ii = 0;
  // 遍历Eigen矩阵，将数据填充到消息中
  for (int i = 0; i < e.rows(); ++i)
    for (int j = 0; j < e.cols(); ++j)
      m.data[ii++] = e.coeff(i, j);
}

} // namespace tf 结束

#endif // EIGEN_MSG_CONVERSIONS_H 结束
