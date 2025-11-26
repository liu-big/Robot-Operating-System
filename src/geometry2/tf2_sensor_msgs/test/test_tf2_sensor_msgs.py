#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import unittest
import struct
import tf2_sensor_msgs
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointField
from tf2_ros import TransformStamped
import copy

## @brief PointCloudConversions类是一个Python单元测试示例，用于测试tf2_sensor_msgs库中PointCloud2消息的转换功能。
#  该类继承自unittest.TestCase，包含设置测试环境和执行简单变换测试的方法。
class PointCloudConversions(unittest.TestCase):
    ## @brief setUp方法在每个测试方法执行前运行，用于初始化测试环境。
    #  它创建一个PointCloud2消息，设置其字段和点数据，并定义一个用于变换的TransformStamped消息。
    #  @param self: 类的实例。
    #  @return 无。
    def setUp(self):
        # 初始化输入的PointCloud2消息
        self.point_cloud_in = point_cloud2.PointCloud2()
        # 定义点云的字段：x, y, z，均为FLOAT32类型
        self.point_cloud_in.fields = [PointField('x', 0, PointField.FLOAT32, 1),
                                PointField('y', 4, PointField.FLOAT32, 1),
                                PointField('z', 8, PointField.FLOAT32, 1)]

        # 设置点步长（每个点占用的字节数）
        self.point_cloud_in.point_step = 4 * 3
        # 设置点云高度（1表示无序点云）
        self.point_cloud_in.height = 1
        # 设置点云宽度（2表示包含两个点）
        self.point_cloud_in.width = 2
        # 设置行步长（每行占用的字节数）
        self.point_cloud_in.row_step = self.point_cloud_in.point_step * self.point_cloud_in.width

        # 定义点数据，包含两个点的x, y, z坐标
        points = [1, 2, 0, 10, 20, 30]
        # 将点数据打包成字节串并赋值给point_cloud_in.data
        self.point_cloud_in.data = struct.pack('%sf' % len(points), *points)

        # 初始化一个TransformStamped消息，用于表示变换
        self.transform_translate_xyz_300 = TransformStamped()
        # 设置变换的平移分量
        self.transform_translate_xyz_300.transform.translation.x = 300
        self.transform_translate_xyz_300.transform.translation.y = 300
        self.transform_translate_xyz_300.transform.translation.z = 300
        # 设置变换的旋转分量，w=1表示无旋转
        self.transform_translate_xyz_300.transform.rotation.w = 1  # 无旋转，只设置w分量

        # 验证点云数据是否正确读取
        assert(list(point_cloud2.read_points(self.point_cloud_in)) == [(1.0, 2.0, 0.0), (10.0, 20.0, 30.0)])

    ## @brief test_simple_transform方法测试PointCloud2消息的简单变换功能。
    #  它使用tf2_sensor_msgs.do_transform_cloud函数对点云进行变换，并验证变换结果是否符合预期。
    #  @param self: 类的实例。
    #  @return 无。
    def test_simple_transform(self):
        # 复制原始点云数据，用于后续验证输入点云是否被修改
        old_data = copy.deepcopy(self.point_cloud_in.data)  # 当前为字符串类型，深拷贝不是必需的
        # 对点云进行变换
        point_cloud_transformed = tf2_sensor_msgs.do_transform_cloud(self.point_cloud_in, self.transform_translate_xyz_300)

        # 定义变换偏移量
        k = 300
        # 计算预期变换后的坐标
        expected_coordinates = [(1+k, 2+k, 0+k), (10+k, 20+k, 30+k)]
        # 读取变换后点云的坐标
        new_points = list(point_cloud2.read_points(point_cloud_transformed))
        # 打印新点坐标
        print("new_points are %s" % new_points)
        # 验证变换后的坐标是否与预期相符
        assert(expected_coordinates == new_points)
        # 验证输入点云数据是否未被修改
        assert(old_data == self.point_cloud_in.data)  # 检查输入点云是否未被修改


## @brief PointCloudConversionsMultichannel类是一个简单的单元测试，用于测试tf2_sensor_msgs.do_transform_cloud函数的多通道版本。
#  该类继承自unittest.TestCase，包含设置测试环境和执行多通道变换测试的方法。
class PointCloudConversionsMultichannel(unittest.TestCase):
    # 定义变换偏移距离常量
    TRANSFORM_OFFSET_DISTANCE = 300

    ## @brief setUp方法在每个测试方法执行前运行，用于初始化多通道测试环境。
    #  它创建一个包含x, y, z和index字段的PointCloud2消息，并定义一个用于变换的TransformStamped消息。
    #  @param self: 类的实例。
    #  @return 无。
    def setUp(self):
        # 初始化输入的PointCloud2消息
        self.point_cloud_in = point_cloud2.PointCloud2()
        # 定义点云的字段：x, y, z（FLOAT32）和index（INT32）
        self.point_cloud_in.fields = [PointField('x', 0, PointField.FLOAT32, 1),
                                PointField('y', 4, PointField.FLOAT32, 1),
                                PointField('z', 8, PointField.FLOAT32, 1),
                                PointField('index', 12, PointField.INT32, 1)]

        # 设置点步长（每个点占用的字节数）
        self.point_cloud_in.point_step = 4 * 4
        # 设置点云高度（1表示无序点云）
        self.point_cloud_in.height = 1
        # 设置点云宽度（2表示包含两个点）
        self.point_cloud_in.width = 2
        # 设置行步长（每行占用的字节数）
        self.point_cloud_in.row_step = self.point_cloud_in.point_step * self.point_cloud_in.width

        # 定义点数据，包含两个点的x, y, z和index坐标
        self.points = [(1.0, 2.0, 0.0, 123), (10.0, 20.0, 30.0, 456)]
        # 遍历点数据，将其打包成字节串并追加到point_cloud_in.data
        for point in self.points:
            self.point_cloud_in.data += struct.pack('3fi', *point)

        # 初始化一个TransformStamped消息，用于表示变换
        self.transform_translate_xyz_300 = TransformStamped()
        # 设置变换的平移分量，使用预定义的偏移距离
        self.transform_translate_xyz_300.transform.translation.x = self.TRANSFORM_OFFSET_DISTANCE
        self.transform_translate_xyz_300.transform.translation.y = self.TRANSFORM_OFFSET_DISTANCE
        self.transform_translate_xyz_300.transform.translation.z = self.TRANSFORM_OFFSET_DISTANCE
        # 设置变换的旋转分量，w=1表示无旋转
        self.transform_translate_xyz_300.transform.rotation.w = 1  # 无旋转，只设置w分量

        # 验证点云数据是否正确读取
        assert(list(point_cloud2.read_points(self.point_cloud_in)) == self.points)

    ## @brief test_simple_transform_multichannel方法测试PointCloud2消息的多通道变换功能。
    #  它使用tf2_sensor_msgs.do_transform_cloud函数对点云进行变换，并验证变换结果是否符合预期，特别是多通道数据（如index）是否保持不变。
    #  @param self: 类的实例。
    #  @return 无。
    def test_simple_transform_multichannel(self):
        # 复制原始点云数据，用于后续验证输入点云是否被修改
        old_data = copy.deepcopy(self.point_cloud_in.data)  # 当前为字符串类型，深拷贝不是必需的
        # 对点云进行变换
        point_cloud_transformed = tf2_sensor_msgs.do_transform_cloud(self.point_cloud_in, self.transform_translate_xyz_300)

        # 计算预期变换后的坐标
        expected_coordinates = []
        for point in self.points:
           expected_coordinates += [(
                    point[0] + self.TRANSFORM_OFFSET_DISTANCE,
                    point[1] + self.TRANSFORM_OFFSET_DISTANCE,
                    point[2] + self.TRANSFORM_OFFSET_DISTANCE,
                    point[3] # index通道必须保持不变
                )]

        # 读取变换后点云的坐标
        new_points = list(point_cloud2.read_points(point_cloud_transformed))
        # 打印新点坐标
        print("new_points are %s" % new_points)
        # 验证变换后的坐标是否与预期相符
        assert(expected_coordinates == new_points)
        # 验证输入点云数据是否未被修改
        assert(old_data == self.point_cloud_in.data)  # 检查输入点云是否未被修改


# 当脚本作为主程序运行时执行以下代码
if __name__ == '__main__':
    import rosunit
    # 运行PointCloudConversions测试类中的所有测试方法
    rosunit.unitrun("test_tf2_sensor_msgs", "test_point_cloud_conversion", PointCloudConversions)
    # 运行PointCloudConversionsMultichannel测试类中的所有测试方法
    rosunit.unitrun("test_tf2_sensor_msgs", "test_point_cloud_conversion", PointCloudConversionsMultichannel)

