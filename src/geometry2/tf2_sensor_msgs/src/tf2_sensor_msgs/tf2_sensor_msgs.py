# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# 导入PointCloud2消息类型，用于处理点云数据
from sensor_msgs.msg import PointCloud2
# 导入read_points和create_cloud函数，用于点云数据的读写操作
from sensor_msgs.point_cloud2 import read_points, create_cloud
# 导入PyKDL库，用于处理机器人运动学和动力学，包括坐标变换
import PyKDL
# 导入rospy，ROS Python客户端库
import rospy
# 导入tf2_ros，ROS中用于处理坐标变换的库
import tf2_ros

# 定义一个简单的转换函数，将消息原样返回
# 这个函数用于tf2_ros的转换注册，表示PointCloud2消息可以直接作为目标消息类型
def to_msg_msg(msg):
    """
    将输入消息原样返回。

    Args:
        msg: 任意ROS消息对象。

    Returns:
        与输入相同的ROS消息对象。
    """
    return msg

# 将PointCloud2消息类型注册到tf2_ros的转换系统中，允许其作为目标消息类型
tf2_ros.ConvertRegistration().add_to_msg(PointCloud2, to_msg_msg)

# 定义一个简单的转换函数，将消息原样返回
# 这个函数用于tf2_ros的转换注册，表示PointCloud2消息可以直接作为源消息类型
def from_msg_msg(msg):
    """
    将输入消息原样返回。

    Args:
        msg: 任意ROS消息对象。

    Returns:
        与输入相同的ROS消息对象。
    """
    return msg

# 将PointCloud2消息类型注册到tf2_ros的转换系统中，允许其作为源消息类型
tf2_ros.ConvertRegistration().add_from_msg(PointCloud2, from_msg_msg)

# 将ROS TransformStamped消息转换为PyKDL.Frame对象
def transform_to_kdl(t):
    """
    将ROS TransformStamped消息转换为PyKDL.Frame对象。

    Args:
        t (geometry_msgs.msg.TransformStamped): 包含平移和旋转信息的ROS变换消息。

    Returns:
        PyKDL.Frame: 对应的KDL坐标系对象。
    """
    return PyKDL.Frame(PyKDL.Rotation.Quaternion(t.transform.rotation.x, t.transform.rotation.y,
                                                 t.transform.rotation.z, t.transform.rotation.w),
                       PyKDL.Vector(t.transform.translation.x, 
                                    t.transform.translation.y, 
                                    t.transform.translation.z))

# PointStamped
# 对PointCloud2消息进行坐标变换
def do_transform_cloud(cloud, transform):
    """
    对PointCloud2消息中的点进行坐标变换。

    Args:
        cloud (sensor_msgs.msg.PointCloud2): 输入的点云数据。
        transform (geometry_msgs.msg.TransformStamped): 用于变换的ROS变换消息。

    Returns:
        sensor_msgs.msg.PointCloud2: 变换后的点云数据。
    """
    # 将ROS变换消息转换为KDL的Frame对象
    t_kdl = transform_to_kdl(transform)
    points_out = []
    # 遍历输入点云中的每个点
    for p_in in read_points(cloud):
        # 将点转换为KDL的Vector，并应用KDL变换
        p_out = t_kdl * PyKDL.Vector(p_in[0], p_in[1], p_in[2])
        # 将变换后的点坐标与原始点的其他字段（如颜色、强度等）合并
        points_out.append((p_out[0], p_out[1], p_out[2]) + p_in[3:])
    # 使用变换后的点和原始点云的头部信息、字段信息创建新的PointCloud2消息
    res = create_cloud(transform.header, cloud.fields, points_out)
    return res
# 将do_transform_cloud函数注册到tf2_ros的TransformRegistration中，使其可以处理PointCloud2类型的变换
tf2_ros.TransformRegistration().add(PointCloud2, do_transform_cloud)
