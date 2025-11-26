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

// 包含 eigen_conversions 库中 eigen_kdl 的头文件，该头文件定义了 KDL 类型和 Eigen 类型之间的转换函数声明。
#include <eigen_conversions/eigen_kdl.h>

namespace tf { // 定义 tf 命名空间，用于存放与 ROS tf 相关的转换函数

/// 将 KDL::Rotation 类型（四元数）转换为 Eigen::Quaterniond 类型。
/// @param k 输入的 KDL::Rotation 对象。
/// @param e 输出的 Eigen::Quaterniond 对象，存储转换后的四元数。
void quaternionKDLToEigen(const KDL::Rotation &k, Eigen::Quaterniond &e)
{
  // KDL::Rotation 的 GetQuaternion 方法直接将四元数分量写入传入的引用参数中。
  k.GetQuaternion(e.x(), e.y(), e.z(), e.w());
}

/// 将 Eigen::Quaterniond 类型转换为 KDL::Rotation 类型（四元数）。
/// @param e 输入的 Eigen::Quaterniond 对象。
/// @param k 输出的 KDL::Rotation 对象，存储转换后的四元数。
void quaternionEigenToKDL(const Eigen::Quaterniond &e, KDL::Rotation &k)
{
  // 使用 KDL::Rotation 的静态方法 Quaternion 从四元数分量构造 KDL::Rotation 对象。
  k = KDL::Rotation::Quaternion(e.x(), e.y(), e.z(), e.w());  
}

namespace { // 匿名命名空间，用于定义只在本文件内部使用的辅助函数模板
  /// 模板函数：将 KDL::Frame 类型转换为 Eigen 的变换类型（如 Eigen::Affine3d 或 Eigen::Isometry3d）。
  /// @tparam T Eigen 的变换类型，可以是 Eigen::Affine3d 或 Eigen::Isometry3d。
  /// @param k 输入的 KDL::Frame 对象，包含位置和旋转。
  /// @param e 输出的 Eigen 变换类型，存储转换后的变换。
  template<typename T>
  void transformKDLToEigenImpl(const KDL::Frame &k, T &e)
  {
    // 转换平移部分
    for (unsigned int i = 0; i < 3; ++i)
      e(i, 3) = k.p[i]; // 将 KDL::Frame 的平移向量分量赋值给 Eigen 变换矩阵的最后一列

    // 转换旋转矩阵部分
    for (unsigned int i = 0; i < 9; ++i)
      e(i/3, i%3) = k.M.data[i]; // 将 KDL::RotationMatrix 的数据按行主序赋值给 Eigen 变换矩阵的旋转部分

    // 设置 "齐次" 行，确保是有效的齐次变换矩阵
    e(3,0) = 0.0;
    e(3,1) = 0.0;
    e(3,2) = 0.0;
    e(3,3) = 1.0;
  }

  /// 模板函数：将 Eigen 的变换类型（如 Eigen::Affine3d 或 Eigen::Isometry3d）转换为 KDL::Frame 类型。
  /// @tparam T Eigen 的变换类型，可以是 Eigen::Affine3d 或 Eigen::Isometry3d。
  /// @param e 输入的 Eigen 变换类型。
  /// @param k 输出的 KDL::Frame 对象，存储转换后的变换。
  template<typename T>
  void transformEigenToKDLImpl(const T &e, KDL::Frame &k)
  {
    // 转换平移部分
    for (unsigned int i = 0; i < 3; ++i)
      k.p[i] = e(i, 3); // 将 Eigen 变换矩阵的最后一列赋值给 KDL::Frame 的平移向量分量
    // 转换旋转矩阵部分
    for (unsigned int i = 0; i < 9; ++i)
      k.M.data[i] = e(i/3, i%3); // 将 Eigen 变换矩阵的旋转部分赋值给 KDL::RotationMatrix 的数据
  }

} // 匿名命名空间结束

/// 将 KDL::Frame 类型转换为 Eigen::Affine3d 类型。
/// @param k 输入的 KDL::Frame 对象。
/// @param e 输出的 Eigen::Affine3d 仿射变换矩阵。
void transformKDLToEigen(const KDL::Frame &k, Eigen::Affine3d &e)
{
  transformKDLToEigenImpl(k, e); // 调用模板实现进行转换
}

/// 将 KDL::Frame 类型转换为 Eigen::Isometry3d 类型。
/// @param k 输入的 KDL::Frame 对象。
/// @param e 输出的 Eigen::Isometry3d 等距变换矩阵。
void transformKDLToEigen(const KDL::Frame &k, Eigen::Isometry3d &e)
{
  transformKDLToEigenImpl(k, e); // 调用模板实现进行转换
}

/// 将 Eigen::Affine3d 类型转换为 KDL::Frame 类型。
/// @param e 输入的 Eigen::Affine3d 仿射变换矩阵。
/// @param k 输出的 KDL::Frame 对象。
void transformEigenToKDL(const Eigen::Affine3d &e, KDL::Frame &k)
{
  transformEigenToKDLImpl(e, k); // 调用模板实现进行转换
}

/// 将 Eigen::Isometry3d 类型转换为 KDL::Frame 类型。
/// @param e 输入的 Eigen::Isometry3d 等距变换矩阵。
/// @param k 输出的 KDL::Frame 对象。
void transformEigenToKDL(const Eigen::Isometry3d &e, KDL::Frame &k)
{
  transformEigenToKDLImpl(e, k); // 调用模板实现进行转换
}

/// 将 Eigen::Matrix<double, 6, 1> 类型（表示 Twist）转换为 KDL::Twist 类型。
/// @param e 输入的 Eigen::Matrix<double, 6, 1> 向量，前三维为线速度，后三维为角速度。
/// @param k 输出的 KDL::Twist 对象，存储转换后的 Twist。
void twistEigenToKDL(const Eigen::Matrix<double, 6, 1> &e, KDL::Twist &k)
{
  for(int i = 0; i < 6; ++i)
    k[i] = e[i]; // 逐分量赋值
}

/// 将 KDL::Twist 类型转换为 Eigen::Matrix<double, 6, 1> 类型。
/// @param k 输入的 KDL::Twist 对象。
/// @param e 输出的 Eigen::Matrix<double, 6, 1> 向量，存储转换后的 Twist。
void twistKDLToEigen(const Eigen::Matrix<double, 6, 1> &e, KDL::Twist &k)
{
  for(int i = 0; i < 6; ++i)
    e[i] = k[i]; // 逐分量赋值
}

/// 将 Eigen::Matrix<double, 3, 1> 类型（表示 Vector）转换为 KDL::Vector 类型。
/// @param e 输入的 Eigen::Matrix<double, 3, 1> 向量。
/// @param k 输出的 KDL::Vector 对象，存储转换后的 Vector。
void vectorEigenToKDL(const Eigen::Matrix<double, 3, 1> &e, KDL::Vector &k)
{
  for(int i = 0; i < 3; ++i)
    k[i] = e[i]; // 逐分量赋值
}

/// 将 KDL::Vector 类型转换为 Eigen::Matrix<double, 3, 1> 类型。
/// @param k 输入的 KDL::Vector 对象。
/// @param e 输出的 Eigen::Matrix<double, 3, 1> 向量，存储转换后的 Vector。
void vectorKDLToEigen(const KDL::Vector &k, Eigen::Matrix<double, 3, 1> &e)
{
  for(int i = 0; i < 3; ++i)
    e[i] = k[i]; // 逐分量赋值
}

/// 将 KDL::Wrench 类型转换为 Eigen::Matrix<double, 6, 1> 类型。
/// @param k 输入的 KDL::Wrench 对象，包含力和扭矩。
/// @param e 输出的 Eigen::Matrix<double, 6, 1> 向量，存储转换后的 Wrench。
void wrenchKDLToEigen(const KDL::Wrench &k, Eigen::Matrix<double, 6, 1> &e)
{
  for(int i = 0; i < 6; ++i)
    e[i] = k[i]; // 逐分量赋值
}

/// 将 Eigen::Matrix<double, 6, 1> 类型（表示 Wrench）转换为 KDL::Wrench 类型。
/// @param e 输入的 Eigen::Matrix<double, 6, 1> 向量，前三维为力，后三维为扭矩。
/// @param k 输出的 KDL::Wrench 对象，存储转换后的 Wrench。
void wrenchEigenToKDL(const Eigen::Matrix<double, 6, 1> &e, KDL::Wrench &k)
{
  for(int i = 0; i < 6; ++i)
    k[i] = e[i]; // 逐分量赋值
}


} // namespace tf 结束
