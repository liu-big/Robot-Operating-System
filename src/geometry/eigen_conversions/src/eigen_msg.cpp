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

// 包含 eigen_conversions 库中 eigen_msg 的头文件，该头文件定义了 ROS 消息类型和 Eigen 类型之间的转换函数声明。
#include <eigen_conversions/eigen_msg.h>

namespace tf { // 定义 tf 命名空间，用于存放与 ROS tf 相关的转换函数

/// 将 geometry_msgs::Point 消息类型转换为 Eigen::Vector3d 类型。
/// @param m 输入的 geometry_msgs::Point 消息，包含 x, y, z 坐标。
/// @param e 输出的 Eigen::Vector3d 向量，存储转换后的三维坐标。
void pointMsgToEigen(const geometry_msgs::Point &m, Eigen::Vector3d &e)
{
  e(0) = m.x; // 将 ROS Point 的 x 坐标赋值给 Eigen 向量的第一个分量
  e(1) = m.y; // 将 ROS Point 的 y 坐标赋值给 Eigen 向量的第二个分量
  e(2) = m.z; // 将 ROS Point 的 z 坐标赋值给 Eigen 向量的第三个分量
}

/// 将 Eigen::Vector3d 类型转换为 geometry_msgs::Point 消息类型。
/// @param e 输入的 Eigen::Vector3d 向量，包含三维坐标。
/// @param m 输出的 geometry_msgs::Point 消息，存储转换后的 x, y, z 坐标。
void pointEigenToMsg(const Eigen::Vector3d &e, geometry_msgs::Point &m)
{
  m.x = e(0); // 将 Eigen 向量的第一个分量赋值给 ROS Point 的 x 坐标
  m.y = e(1); // 将 Eigen 向量的第二个分量赋值给 ROS Point 的 y 坐标
  m.z = e(2); // 将 Eigen 向量的第三个分量赋值给 ROS Point 的 z 坐标
}

namespace { // 匿名命名空间，用于定义只在本文件内部使用的辅助函数模板
  /// 模板函数：将 geometry_msgs::Pose 消息类型转换为 Eigen 的姿态类型（如 Eigen::Affine3d 或 Eigen::Isometry3d）。
  /// @tparam T Eigen 的姿态类型，可以是 Eigen::Affine3d 或 Eigen::Isometry3d。
  /// @param m 输入的 geometry_msgs::Pose 消息，包含位置和方向。
  /// @param e 输出的 Eigen 姿态类型，存储转换后的姿态。
  template<typename T>
  void poseMsgToEigenImpl(const geometry_msgs::Pose &m, T &e)
  {
    // 通过平移向量和四元数构建 Eigen 姿态
    e = Eigen::Translation3d(m.position.x, // 从 ROS Pose 的位置信息创建 Eigen 平移向量
                             m.position.y,
                             m.position.z) *
      Eigen::Quaterniond(m.orientation.w, // 从 ROS Pose 的方向信息创建 Eigen 四元数
                         m.orientation.x,
                         m.orientation.y,
                         m.orientation.z);
  }

  /// 模板函数：将 Eigen 的姿态类型（如 Eigen::Affine3d 或 Eigen::Isometry3d）转换为 geometry_msgs::Pose 消息类型。
  /// @tparam T Eigen 的姿态类型，可以是 Eigen::Affine3d 或 Eigen::Isometry3d。
  /// @param e 输入的 Eigen 姿态类型，包含位置和方向。
  /// @param m 输出的 geometry_msgs::Pose 消息，存储转换后的姿态。
  template<typename T>
  void poseEigenToMsgImpl(const T &e, geometry_msgs::Pose &m)
  {
    m.position.x = e.translation()[0]; // 将 Eigen 姿态的平移向量 x 分量赋值给 ROS Pose 的位置 x
    m.position.y = e.translation()[1]; // 将 Eigen 姿态的平移向量 y 分量赋值给 ROS Pose 的位置 y
    m.position.z = e.translation()[2]; // 将 Eigen 姿态的平移向量 z 分量赋值给 ROS Pose 的位置 z
    Eigen::Quaterniond q = (Eigen::Quaterniond)e.linear(); // 从 Eigen 姿态的线性部分获取旋转四元数
    m.orientation.x = q.x(); // 将 Eigen 四元数 x 分量赋值给 ROS Pose 的方向 x
    m.orientation.y = q.y(); // 将 Eigen 四元数 y 分量赋值给 ROS Pose 的方向 y
    m.orientation.z = q.z(); // 将 Eigen 四元数 z 分量赋值给 ROS Pose 的方向 z
    m.orientation.w = q.w(); // 将 Eigen 四元数 w 分量赋值给 ROS Pose 的方向 w
    // 确保四元数的 w 分量为正，保持一致性
    if (m.orientation.w < 0) { 
      m.orientation.x *= -1;
      m.orientation.y *= -1;
      m.orientation.z *= -1;
      m.orientation.w *= -1;
    }
  }

  /// 模板函数：将 geometry_msgs::Transform 消息类型转换为 Eigen 的变换类型（如 Eigen::Affine3d 或 Eigen::Isometry3d）。
  /// @tparam T Eigen 的变换类型，可以是 Eigen::Affine3d 或 Eigen::Isometry3d。
  /// @param m 输入的 geometry_msgs::Transform 消息，包含平移和旋转。
  /// @param e 输出的 Eigen 变换类型，存储转换后的变换。
  template<typename T>
  void transformMsgToEigenImpl(const geometry_msgs::Transform &m, T &e)
  {
    // 通过平移向量和四元数构建 Eigen 变换
    e = Eigen::Translation3d(m.translation.x, // 从 ROS Transform 的平移信息创建 Eigen 平移向量
                             m.translation.y,
                             m.translation.z) *
      Eigen::Quaterniond(m.rotation.w, // 从 ROS Transform 的旋转信息创建 Eigen 四元数
                         m.rotation.x,
                         m.rotation.y,
                         m.rotation.z);
  }

  /// 模板函数：将 Eigen 的变换类型（如 Eigen::Affine3d 或 Eigen::Isometry3d）转换为 geometry_msgs::Transform 消息类型。
  /// @tparam T Eigen 的变换类型，可以是 Eigen::Affine3d 或 Eigen::Isometry3d。
  /// @param e 输入的 Eigen 变换类型，包含平移和旋转。
  /// @param m 输出的 geometry_msgs::Transform 消息，存储转换后的变换。
  template<typename T>
  void transformEigenToMsgImpl(const T &e, geometry_msgs::Transform &m)
  {
    m.translation.x = e.translation()[0]; // 将 Eigen 变换的平移向量 x 分量赋值给 ROS Transform 的平移 x
    m.translation.y = e.translation()[1]; // 将 Eigen 变换的平移向量 y 分量赋值给 ROS Transform 的平移 y
    m.translation.z = e.translation()[2]; // 将 Eigen 变换的平移向量 z 分量赋值给 ROS Transform 的平移 z
    Eigen::Quaterniond q = (Eigen::Quaterniond)e.linear(); // 从 Eigen 变换的线性部分获取旋转四元数
    m.rotation.x = q.x(); // 将 Eigen 四元数 x 分量赋值给 ROS Transform 的旋转 x
    m.rotation.y = q.y(); // 将 Eigen 四元数 y 分量赋值给 ROS Transform 的旋转 y
    m.rotation.z = q.z(); // 将 Eigen 四元数 z 分量赋值给 ROS Transform 的旋转 z
    m.rotation.w = q.w(); // 将 Eigen 四元数 w 分量赋值给 ROS Transform 的旋转 w
    // 确保四元数的 w 分量为正，保持一致性
    if (m.rotation.w < 0) {
      m.rotation.x *= -1;
      m.rotation.y *= -1;
      m.rotation.z *= -1;
      m.rotation.w *= -1;
    }
  }
} // 匿名命名空间结束

/// 将 geometry_msgs::Pose 消息类型转换为 Eigen::Affine3d 类型。
/// @param m 输入的 geometry_msgs::Pose 消息。
/// @param e 输出的 Eigen::Affine3d 仿射变换矩阵。
void poseMsgToEigen(const geometry_msgs::Pose &m, Eigen::Affine3d &e)
{
  poseMsgToEigenImpl(m, e); // 调用模板实现进行转换
}

/// 将 geometry_msgs::Pose 消息类型转换为 Eigen::Isometry3d 类型。
/// @param m 输入的 geometry_msgs::Pose 消息。
/// @param e 输出的 Eigen::Isometry3d 等距变换矩阵。
void poseMsgToEigen(const geometry_msgs::Pose &m, Eigen::Isometry3d &e)
{
  poseMsgToEigenImpl(m, e); // 调用模板实现进行转换
}

/// 将 Eigen::Affine3d 类型转换为 geometry_msgs::Pose 消息类型。
/// @param e 输入的 Eigen::Affine3d 仿射变换矩阵。
/// @param m 输出的 geometry_msgs::Pose 消息。
void poseEigenToMsg(const Eigen::Affine3d &e, geometry_msgs::Pose &m)
{
  poseEigenToMsgImpl(e, m); // 调用模板实现进行转换
}

/// 将 Eigen::Isometry3d 类型转换为 geometry_msgs::Pose 消息类型。
/// @param e 输入的 Eigen::Isometry3d 等距变换矩阵。
/// @param m 输出的 geometry_msgs::Pose 消息。
void poseEigenToMsg(const Eigen::Isometry3d &e, geometry_msgs::Pose &m)
{
  poseEigenToMsgImpl(e, m); // 调用模板实现进行转换
}

/// 将 geometry_msgs::Quaternion 消息类型转换为 Eigen::Quaterniond 类型。
/// @param m 输入的 geometry_msgs::Quaternion 消息，包含 x, y, z, w 分量。
/// @param e 输出的 Eigen::Quaterniond 四元数，存储转换后的四元数。
void quaternionMsgToEigen(const geometry_msgs::Quaternion &m, Eigen::Quaterniond &e)
{
  e = Eigen::Quaterniond(m.w, m.x, m.y, m.z); // 使用 ROS Quaternion 的 w, x, y, z 分量构造 Eigen 四元数
}

/// 将 Eigen::Quaterniond 类型转换为 geometry_msgs::Quaternion 消息类型。
/// @param e 输入的 Eigen::Quaterniond 四元数，包含 x, y, z, w 分量。
/// @param m 输出的 geometry_msgs::Quaternion 消息，存储转换后的四元数。
void quaternionEigenToMsg(const Eigen::Quaterniond &e, geometry_msgs::Quaternion &m)
{
  m.x = e.x(); // 将 Eigen 四元数 x 分量赋值给 ROS Quaternion 的 x
  m.y = e.y(); // 将 Eigen 四元数 y 分量赋值给 ROS Quaternion 的 y
  m.z = e.z(); // 将 Eigen 四元数 z 分量赋值给 ROS Quaternion 的 z
  m.w = e.w(); // 将 Eigen 四元数 w 分量赋值给 ROS Quaternion 的 w
}

/// 将 geometry_msgs::Transform 消息类型转换为 Eigen::Affine3d 类型。
/// @param m 输入的 geometry_msgs::Transform 消息。
/// @param e 输出的 Eigen::Affine3d 仿射变换矩阵。
void transformMsgToEigen(const geometry_msgs::Transform &m, Eigen::Affine3d &e)
{
  transformMsgToEigenImpl(m, e); // 调用模板实现进行转换
}

/// 将 geometry_msgs::Transform 消息类型转换为 Eigen::Isometry3d 类型。
/// @param m 输入的 geometry_msgs::Transform 消息。
/// @param e 输出的 Eigen::Isometry3d 等距变换矩阵。
void transformMsgToEigen(const geometry_msgs::Transform &m, Eigen::Isometry3d &e)
{
  transformMsgToEigenImpl(m, e); // 调用模板实现进行转换
}

/// 将 Eigen::Affine3d 类型转换为 geometry_msgs::Transform 消息类型。
/// @param e 输入的 Eigen::Affine3d 仿射变换矩阵。
/// @param m 输出的 geometry_msgs::Transform 消息。
void transformEigenToMsg(const Eigen::Affine3d &e, geometry_msgs::Transform &m)
{
  transformEigenToMsgImpl(e, m); // 调用模板实现进行转换
}

/// 将 Eigen::Isometry3d 类型转换为 geometry_msgs::Transform 消息类型。
/// @param e 输入的 Eigen::Isometry3d 等距变换矩阵。
/// @param m 输出的 geometry_msgs::Transform 消息。
void transformEigenToMsg(const Eigen::Isometry3d &e, geometry_msgs::Transform &m)
{
  transformEigenToMsgImpl(e, m); // 调用模板实现进行转换
}

/// 将 geometry_msgs::Vector3 消息类型转换为 Eigen::Vector3d 类型。
/// @param m 输入的 geometry_msgs::Vector3 消息，包含 x, y, z 分量。
/// @param e 输出的 Eigen::Vector3d 向量，存储转换后的三维向量。
void vectorMsgToEigen(const geometry_msgs::Vector3 &m, Eigen::Vector3d &e)
{
  e(0) = m.x; // 将 ROS Vector3 的 x 分量赋值给 Eigen 向量的第一个分量
  e(1) = m.y; // 将 ROS Vector3 的 y 分量赋值给 Eigen 向量的第二个分量
  e(2) = m.z; // 将 ROS Vector3 的 z 分量赋值给 Eigen 向量的第三个分量
}

/// 将 Eigen::Vector3d 类型转换为 geometry_msgs::Vector3 消息类型。
/// @param e 输入的 Eigen::Vector3d 向量，包含三维向量。
/// @param m 输出的 geometry_msgs::Vector3 消息，存储转换后的 x, y, z 分量。
void vectorEigenToMsg(const Eigen::Vector3d &e, geometry_msgs::Vector3 &m)
{
  m.x = e(0); // 将 Eigen 向量的第一个分量赋值给 ROS Vector3 的 x
  m.y = e(1); // 将 Eigen 向量的第二个分量赋值给 ROS Vector3 的 y
  m.z = e(2); // 将 Eigen 向量的第三个分量赋值给 ROS Vector3 的 z
}

/// 将 geometry_msgs::Twist 消息类型转换为 Eigen::Matrix<double, 6, 1> 类型。
/// @param m 输入的 geometry_msgs::Twist 消息，包含线速度和角速度。
/// @param e 输出的 Eigen::Matrix<double, 6, 1> 矩阵，存储转换后的六维向量（前三维为线速度，后三维为角速度）。
void twistMsgToEigen(const geometry_msgs::Twist &m, Eigen::Matrix<double,6,1> &e)
{
  e[0] = m.linear.x; // 线速度 x
  e[1] = m.linear.y; // 线速度 y
  e[2] = m.linear.z; // 线速度 z
  e[3] = m.angular.x; // 角速度 x
  e[4] = m.angular.y; // 角速度 y
  e[5] = m.angular.z; // 角速度 z
}

/// 将 Eigen::Matrix<double, 6, 1> 类型转换为 geometry_msgs::Twist 消息类型。
/// @param e 输入的 Eigen::Matrix<double, 6, 1> 矩阵，包含线速度和角速度。
/// @param m 输出的 geometry_msgs::Twist 消息，存储转换后的线速度和角速度。
void twistEigenToMsg(const Eigen::Matrix<double,6,1> &e, geometry_msgs::Twist &m)
{
  m.linear.x = e[0]; // 线速度 x
  m.linear.y = e[1]; // 线速度 y
  m.linear.z = e[2]; // 线速度 z
  m.angular.x = e[3]; // 角速度 x
  m.angular.y = e[4]; // 角速度 y
  m.angular.z = e[5]; // 角速度 z
}

/// 将 geometry_msgs::Wrench 消息类型转换为 Eigen::Matrix<double, 6, 1> 类型。
/// @param m 输入的 geometry_msgs::Wrench 消息，包含力和扭矩。
/// @param e 输出的 Eigen::Matrix<double, 6, 1> 矩阵，存储转换后的六维向量（前三维为力，后三维为扭矩）。
void wrenchMsgToEigen(const geometry_msgs::Wrench &m, Eigen::Matrix<double,6,1> &e)
{
  e[0] = m.force.x; // 力 x
  e[1] = m.force.y; // 力 y
  e[2] = m.force.z; // 力 z
  e[3] = m.torque.x; // 扭矩 x
  e[4] = m.torque.y; // 扭矩 y
  e[5] = m.torque.z; // 扭矩 z
}

/// 将 Eigen::Matrix<double, 6, 1> 类型转换为 geometry_msgs::Wrench 消息类型。
/// @param e 输入的 Eigen::Matrix<double, 6, 1> 矩阵，包含力和扭矩。
/// @param m 输出的 geometry_msgs::Wrench 消息，存储转换后的力和扭矩。
void wrenchEigenToMsg(const Eigen::Matrix<double,6,1> &e, geometry_msgs::Wrench &m)
{
  m.force.x = e[0]; // 力 x
  m.force.y = e[1]; // 力 y
  m.force.z = e[2]; // 力 z
  m.torque.x = e[3]; // 扭矩 x
  m.torque.y = e[4]; // 扭矩 y
  m.torque.z = e[5]; // 扭矩 z
}

/// 将 Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> 类型转换为 std_msgs::Float64MultiArray 消息类型。
/// @param e 输入的 Eigen 动态大小矩阵。
/// @param m 输出的 std_msgs::Float64MultiArray 消息，存储转换后的多维浮点数组。
void eigenMatrixToMsg(const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &e, std_msgs::Float64MultiArray &m)
{
  m.data.resize(e.rows() * e.cols()); // 根据 Eigen 矩阵的行数和列数调整 ROS 消息数据数组的大小
  for (int i = 0; i < e.rows(); ++i) { // 遍历 Eigen 矩阵的每一行
    for (int j = 0; j < e.cols(); ++j) { // 遍历 Eigen 矩阵的每一列
      m.data[i * e.cols() + j] = e(i, j); // 将 Eigen 矩阵的元素按行主序存储到 ROS 消息数据数组中
    }
  }
}

/// 将 std_msgs::Float64MultiArray 消息类型转换为 Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> 类型。
/// @param m 输入的 std_msgs::Float64MultiArray 消息，包含多维浮点数组。
/// @param e 输出的 Eigen 动态大小矩阵，存储转换后的矩阵。
void msgToEigenMatrix(const std_msgs::Float64MultiArray &m, Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &e)
{
  if (m.layout.dim.size() == 0) { // 如果消息的维度信息为空
    e.resize(0, 0); // 将 Eigen 矩阵大小设置为 0x0
    return;
  }

  if (m.layout.dim.size() > 2) { // 如果消息的维度超过 2 维
    ROS_ERROR("Only matrices with 2 or fewer dimensions are supported for conversion to Eigen matrices"); // 报错：只支持二维或更低维度的矩阵转换
    return;
  }

  int rows = m.layout.dim[0].size; // 获取矩阵的行数
  int cols = 1; // 默认列数为 1
  if (m.layout.dim.size() > 1) { // 如果存在第二维
    cols = m.layout.dim[1].size; // 获取矩阵的列数
  }

  e.resize(rows, cols); // 调整 Eigen 矩阵的大小
  for (int i = 0; i < rows; ++i) { // 遍历矩阵的每一行
    for (int j = 0; j < cols; ++j) { // 遍历矩阵的每一列
      e(i, j) = m.data[i * cols + j]; // 将 ROS 消息数据数组的元素按行主序存储到 Eigen 矩阵中
    }
  }
}

} // namespace tf 结束
