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
 * 作者: Adam Leeper, Stuart Glaser
 */

#ifndef EIGEN_KDL_CONVERSIONS_H // 如果未定义 EIGEN_KDL_CONVERSIONS_H，则定义它，防止头文件被多次包含
#define EIGEN_KDL_CONVERSIONS_H // 定义 EIGEN_KDL_CONVERSIONS_H

// 包含Eigen库核心和几何模块头文件
#include <Eigen/Core>     // 包含 Eigen 核心模块，提供矩阵和向量的基本定义
#include <Eigen/Geometry> // 包含 Eigen 几何模块，提供几何变换（如旋转、平移）的定义

// 包含KDL库的frames模块头文件
#include <kdl/frames.hpp> // 包含 KDL 框架模块，提供 KDL 几何类型（如 Frame, Rotation, Vector）的定义

namespace tf { // 定义 tf 命名空间，用于存放与 ROS tf 相关的转换函数

/// 将 KDL 旋转转换为 Eigen 四元数
/// @param k 输入的 KDL::Rotation 旋转
/// @param e 输出的 Eigen::Quaterniond 四元数
void quaternionKDLToEigen(const KDL::Rotation &k, Eigen::Quaterniond &e);

/// 将 Eigen 四元数转换为 KDL 旋转
/// @param e 输入的 Eigen::Quaterniond 四元数
/// @param k 输出的 KDL::Rotation 旋转
void quaternionEigenToKDL(const Eigen::Quaterniond &e, KDL::Rotation &k);

/// 将 KDL 框架转换为 Eigen Affine3d
/// @param k 输入的 KDL::Frame 框架
/// @param e 输出的 Eigen::Affine3d 仿射变换矩阵
void transformKDLToEigen(const KDL::Frame &k, Eigen::Affine3d &e);

/// 将 KDL 框架转换为 Eigen Isometry3d
/// @param k 输入的 KDL::Frame 框架
/// @param e 输出的 Eigen::Isometry3d 等距变换矩阵
void transformKDLToEigen(const KDL::Frame &k, Eigen::Isometry3d &e);

/// 将 Eigen Affine3d 转换为 KDL 框架
/// @param e 输入的 Eigen::Affine3d 仿射变换矩阵
/// @param k 输出的 KDL::Frame 框架
void transformEigenToKDL(const Eigen::Affine3d &e, KDL::Frame &k);

/// 将 Eigen Isometry3d 转换为 KDL 框架
/// @param e 输入的 Eigen::Isometry3d 等距变换矩阵
/// @param k 输出的 KDL::Frame 框架
void transformEigenToKDL(const Eigen::Isometry3d &e, KDL::Frame &k);

/// 将 KDL 扭曲（Twist）转换为 Eigen 矩阵
/// @param k 输入的 KDL::Twist 扭曲
/// @param e 输出的 Eigen::Matrix<double, 6, 1> 矩阵 (表示线速度和角速度)
void twistKDLToEigen(const KDL::Twist &k, Eigen::Matrix<double, 6, 1> &e);

/// 将 Eigen 矩阵转换为 KDL 扭曲（Twist）
/// @param e 输入的 Eigen::Matrix<double, 6, 1> 矩阵
/// @param k 输出的 KDL::Twist 扭曲
void twistEigenToKDL(const Eigen::Matrix<double, 6, 1> &e, KDL::Twist &k);

/// 将 KDL 向量转换为 Eigen 矩阵
/// @param k 输入的 KDL::Vector 向量
/// @param e 输出的 Eigen::Matrix<double, 3, 1> 矩阵
void vectorKDLToEigen(const KDL::Vector &k, Eigen::Matrix<double, 3, 1> &e);

/// 将 Eigen 矩阵转换为 KDL 向量
/// @param e 输入的 Eigen::Matrix<double, 3, 1> 矩阵
/// @param k 输出的 KDL::Vector 向量
void vectorEigenToKDL(const Eigen::Matrix<double, 3, 1> &e, KDL::Vector &k);

/// 将 KDL 力矩（Wrench）转换为 Eigen 矩阵
/// @param k 输入的 KDL::Wrench 力矩
/// @param e 输出的 Eigen::Matrix<double, 6, 1> 矩阵 (表示力和扭矩)
void wrenchKDLToEigen(const KDL::Wrench &k, Eigen::Matrix<double, 6, 1> &e);

/// 将 Eigen 矩阵转换为 KDL 力矩（Wrench）
/// @param e 输入的 Eigen::Matrix<double, 6, 1> 矩阵
/// @param k 输出的 KDL::Wrench 力矩
void wrenchEigenToKDL(const Eigen::Matrix<double, 6, 1> &e, KDL::Wrench &k);

} // namespace tf 结束

#endif // EIGEN_KDL_CONVERSIONS_H 结束
