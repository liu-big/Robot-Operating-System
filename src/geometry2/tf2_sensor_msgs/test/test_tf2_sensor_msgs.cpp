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


// 包含tf2_sensor_msgs库的头文件，提供了传感器消息的tf2转换功能
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
// 包含geometry_msgs的PoseStamped消息类型头文件
#include <geometry_msgs/PoseStamped.h>
// 包含tf2_ros的TransformListener头文件，用于监听tf变换
#include <tf2_ros/transform_listener.h>
// 包含ros核心库的头文件
#include <ros/ros.h>
// 包含gtest库的头文件，用于编写和运行单元测试
#include <gtest/gtest.h>
// 包含tf2_ros的Buffer头文件，用于存储和查询tf变换
#include <tf2_ros/buffer.h.>

// 声明一个tf2_ros::Buffer指针，用于在测试中进行tf变换查询
tf2_ros::Buffer* tf_buffer;
// 定义一个小的常数EPS，用于浮点数比较时的容差
static const double EPS = 1e-3;


// 定义一个测试用例，测试PointCloud2的tf变换
TEST(Tf2Sensor, PointCloud2)
{
  // 创建一个PointCloud2消息对象
  sensor_msgs::PointCloud2 cloud;
  // 创建一个PointCloud2Modifier对象，用于方便地设置点云字段和大小
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  // 设置点云的字段，这里设置为xyz和rgb，每个点包含3个float类型的坐标和rgb信息
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  // 调整点云大小，这里设置为1个点
  modifier.resize(1);

  // 创建迭代器，用于访问点云中x、y、z坐标
  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

  // 设置点云中第一个点的坐标
  *iter_x = 1;
  *iter_y = 2;
  *iter_z = 3;

  // 设置点云消息的头部时间戳和帧ID
  cloud.header.stamp = ros::Time(2);
  cloud.header.frame_id = "A";

  // 使用简单API进行tf变换
  // 将点云从帧"A"变换到帧"B"，时间戳为ros::Duration(2.0)
  sensor_msgs::PointCloud2 cloud_simple = tf_buffer->transform(cloud, "B", ros::Duration(2.0));
  // 创建迭代器，用于访问变换后点云的x、y、z坐标
  sensor_msgs::PointCloud2Iterator<float> iter_x_after(cloud_simple, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y_after(cloud_simple, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z_after(cloud_simple, "z");
  // 验证变换后的坐标是否接近预期值，使用EPS作为容差
  EXPECT_NEAR(*iter_x_after, -9, EPS);
  EXPECT_NEAR(*iter_y_after, 18, EPS);
  EXPECT_NEAR(*iter_z_after, 27, EPS);

  // 使用高级API进行tf变换
  // 将点云从帧"A"变换到帧"B"，源时间戳为ros::Time(2.0)，目标时间戳为ros::Duration(3.0)
  sensor_msgs::PointCloud2 cloud_advanced = tf_buffer->transform(cloud, "B", ros::Time(2.0),
                                                                 "A", ros::Duration(3.0));
  // 创建迭代器，用于访问高级API变换后点云的x、y、z坐标
  sensor_msgs::PointCloud2Iterator<float> iter_x_advanced(cloud_advanced, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y_advanced(cloud_advanced, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z_advanced(cloud_advanced, "z");
  // 验证变换后的坐标是否接近预期值，使用EPS作为容差
  EXPECT_NEAR(*iter_x_advanced, -9, EPS);
  EXPECT_NEAR(*iter_y_advanced, 18, EPS);
  EXPECT_NEAR(*iter_z_advanced, 27, EPS);
}

// 主函数，用于运行所有测试
int main(int argc, char **argv){
  // 初始化Google Test框架
  testing::InitGoogleTest(&argc, argv);
  // 初始化ROS节点，节点名为"test"
  ros::init(argc, argv, "test");
  // 创建ROS节点句柄
  ros::NodeHandle n;

  // 实例化tf2_ros::Buffer对象
  tf_buffer = new tf2_ros::Buffer();

  // 填充tf_buffer，添加一个从帧"A"到帧"B"的静态变换
  geometry_msgs::TransformStamped t;
  // 设置平移分量
  t.transform.translation.x = 10;
  t.transform.translation.y = 20;
  t.transform.translation.z = 30;
  // 设置旋转分量（单位四元数，表示无旋转）
  t.transform.rotation.x = 1;
  t.transform.rotation.y = 0;
  t.transform.rotation.z = 0;
  t.transform.rotation.w = 0;
  // 设置变换的时间戳和帧ID
  t.header.stamp = ros::Time(2.0);
  t.header.frame_id = "A";
  t.child_frame_id = "B";
  // 将变换添加到tf_buffer中，并指定发布者名称为"test"
  tf_buffer->setTransform(t, "test");

  // 运行所有定义的测试
  int ret = RUN_ALL_TESTS();
  // 清理tf_buffer对象
  delete tf_buffer;
  // 返回测试结果
  return ret;
}
