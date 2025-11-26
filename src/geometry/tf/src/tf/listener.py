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
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
# IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# @file listener.py
# @brief TF变换监听器和转换器。
# 该文件定义了用于监听和处理ROS中TF（Transform Frame）变换的Python类。
# 它提供了查询、等待和转换不同数据类型（如点、姿态等）在不同坐标系之间的方法。

import roslib
roslib.load_manifest('tf')
import rospy
import tf2_ros
import tf2_geometry_msgs
import PyKDL

from geometry_msgs.msg import PointStamped, QuaternionStamped, PoseStamped


def xyz_to_mat44(x, y, z):
    """
    @brief 将XYZ坐标转换为4x4齐次变换矩阵。
    @param x X坐标。
    @param y Y坐标。
    @param z Z坐标。
    @return 4x4齐次变换矩阵。
    """
    return PyKDL.Frame(PyKDL.Vector(x, y, z)).M


def xyzw_to_mat44(x, y, z, w):
    """
    @brief 将XYZW四元数转换为4x4齐次变换矩阵。
    @param x 四元数X分量。
    @param y 四元数Y分量。
    @param z 四元数Z分量。
    @param w 四元数W分量。
    @return 4x4齐次变换矩阵。
    """
    return PyKDL.Rotation.Quaternion(x, y, z, w)


def strip_leading_slash(s):
    """
    @brief 移除字符串开头的斜杠（如果存在）。
    @param s 输入字符串。
    @return 移除开头的斜杠后的字符串。
    """
    if s.startswith('/'):
        return s[1:]
    return s


class Transformer(tf2_ros.Buffer):
    """
    @brief Transformer类，继承自tf2_ros.Buffer，提供TF变换的查询和管理功能。
    """

    def __init__(self, cache_time=rospy.Duration(tf2_ros.Buffer.DEFAULT_CACHE_TIME), debug=False):
        """
        @brief 构造函数。
        @param cache_time 缓存时间。
        @param debug 是否开启调试模式。
        """
        tf2_ros.Buffer.__init__(self, cache_time)
        self.debug = debug

    def lookupTransform(self, target_frame, source_frame, time):
        """
        @brief 查找从源帧到目标帧的变换。
        @param target_frame 目标帧。
        @param source_frame 源帧。
        @param time 时间戳。
        @return 变换StampedTransform。
        """
        return self.lookup_transform(target_frame, source_frame, time)

    def lookupTransformFull(self, target_frame, target_time, source_frame, source_time, fixed_frame):
        """
        @brief 查找从源帧到目标帧的完整变换（指定固定帧）。
        @param target_frame 目标帧。
        @param target_time 目标时间。
        @param source_frame 源帧。
        @param source_time 源时间。
        @param fixed_frame 固定帧。
        @return 变换StampedTransform。
        """
        return self.lookup_transform(target_frame, target_time, source_frame, source_time, fixed_frame)

    def canTransform(self, target_frame, source_frame, time, timeout=rospy.Duration(0.0)):
        """
        @brief 检查是否可以进行从源帧到目标帧的变换。
        @param target_frame 目标帧。
        @param source_frame 源帧。
        @param time 时间戳。
        @param timeout 超时时间。
        @return 如果可以变换则返回True，否则返回False。
        """
        return self.can_transform(target_frame, source_frame, time, timeout)

    def canTransformFull(self, target_frame, target_time, source_frame, source_time, fixed_frame, timeout=rospy.Duration(0.0)):
        """
        @brief 检查是否可以进行从源帧到目标帧的完整变换（指定固定帧）。
        @param target_frame 目标帧。
        @param target_time 目标时间。
        @param source_frame 源帧。
        @param source_time 源时间。
        @param fixed_frame 固定帧。
        @param timeout 超时时间。
        @return 如果可以变换则返回True，否则返回False。
        """
        return self.can_transform(target_frame, target_time, source_frame, source_time, fixed_frame, timeout)

    def waitForTransform(self, target_frame, source_frame, time, timeout, polling_sleep_duration=rospy.Duration(0.01)):
        """
        @brief 等待从源帧到目标帧的变换可用。
        @param target_frame 目标帧。
        @param source_frame 源帧。
        @param time 时间戳。
        @param timeout 超时时间。
        @param polling_sleep_duration 轮询间隔。
        @return 如果变换可用则返回True，否则返回False。
        """
        return self.can_transform(target_frame, source_frame, time, timeout)

    def waitForTransformFull(self, target_frame, target_time, source_frame, source_time, fixed_frame, timeout, polling_sleep_duration=rospy.Duration(0.01)):
        """
        @brief 等待从源帧到目标帧的完整变换可用（指定固定帧）。
        @param target_frame 目标帧。
        @param target_time 目标时间。
        @param source_frame 源帧。
        @param source_time 源时间。
        @param fixed_frame 固定帧。
        @param timeout 超时时间。
        @param polling_sleep_duration 轮询间隔。
        @return 如果变换可用则返回True，否则返回False。
        """
        return self.can_transform(target_frame, target_time, source_frame, source_time, fixed_frame, timeout)

    def getLatestCommonTime(self, source_frame, target_frame):
        """
        @brief 获取源帧和目标帧之间最新的共同时间。
        @param source_frame 源帧。
        @param target_frame 目标帧。
        @return 最新的共同时间。
        """
        return self.get_latest_common_time(source_frame, target_frame)

    def getParent(self, frame_id, time):
        """
        @brief 获取指定帧的父帧。
        @param frame_id 帧ID。
        @param time 时间戳。
        @return 父帧ID。
        """
        return self._get_parent(frame_id, time)

    def frameExists(self, frame_id):
        """
        @brief 检查指定帧是否存在。
        @param frame_id 帧ID。
        @return 如果帧存在则返回True，否则返回False。
        """
        return self._frame_exists(frame_id)

    def allFramesAsString(self):
        """
        @brief 获取所有帧的字符串表示。
        @return 所有帧的字符串表示。
        """
        return self.all_frames_as_string()

    def allFramesAsDot(self):
        """
        @brief 获取所有帧的Dot语言表示。
        @return 所有帧的Dot语言表示。
        """
        return self.all_frames_as_dot()

    def getFrameStrings(self):
        """
        @brief 获取所有帧的字符串列表。
        @return 帧字符串列表。
        """
        return self.get_frame_strings()

    def clear(self):
        """
        @brief 清除缓存中的所有变换数据。
        """
        self.clear()

    def setExtrapolationLimit(self, distance):
        """
        @brief 设置外推限制（此方法已弃用，不执行任何操作）。
        @param distance 距离。
        """
        rospy.logwarn("Transformer.setExtrapolationLimit is deprecated and does not do anything")

    def _transform(self, tf_func, target_frame, stamped_in):
        """
        @brief 内部转换方法。
        @param tf_func 实际执行转换的tf2函数。
        @param target_frame 目标帧。
        @param stamped_in 输入的Stamped消息。
        @return 转换后的Stamped消息。
        """
        return tf_func(stamped_in, target_frame)

    def transformQuaternion(self, target_frame, stamped_in):
        """
        @brief 转换QuaternionStamped消息。
        @param target_frame 目标帧。
        @param stamped_in 输入的QuaternionStamped消息。
        @return 转换后的QuaternionStamped消息。
        """
        return self._transform(tf2_geometry_msgs.do_transform_quaternion, target_frame, stamped_in)

    def transformVector3(self, target_frame, stamped_in):
        """
        @brief 转换Vector3Stamped消息。
        @param target_frame 目标帧。
        @param stamped_in 输入的Vector3Stamped消息。
        @return 转换后的Vector3Stamped消息。
        """
        return self._transform(tf2_geometry_msgs.do_transform_vector3, target_frame, stamped_in)

    def transformPoint(self, target_frame, stamped_in):
        """
        @brief 转换PointStamped消息。
        @param target_frame 目标帧。
        @param stamped_in 输入的PointStamped消息。
        @return 转换后的PointStamped消息。
        """
        return self._transform(tf2_geometry_msgs.do_transform_point, target_frame, stamped_in)

    def transformPose(self, target_frame, stamped_in):
        """
        @brief 转换PoseStamped消息。
        @param target_frame 目标帧。
        @param stamped_in 输入的PoseStamped消息。
        @return 转换后的PoseStamped消息。
        """
        return self._transform(tf2_geometry_msgs.do_transform_pose, target_frame, stamped_in)

    def transformTwist(self, target_frame, stamped_in):
        """
        @brief 转换TwistStamped消息。
        @param target_frame 目标帧。
        @param stamped_in 输入的TwistStamped消息。
        @return 转换后的TwistStamped消息。
        """
        return self._transform(tf2_geometry_msgs.do_transform_twist, target_frame, stamped_in)

    def transformPointCloud(self, target_frame, stamped_in):
        """
        @brief 转换PointCloud2消息。
        @param target_frame 目标帧。
        @param stamped_in 输入的PointCloud2消息。
        @return 转换后的PointCloud2消息。
        """
        return self._transform(tf2_geometry_msgs.do_transform_cloud, target_frame, stamped_in)


class TransformerROS(Transformer):
    """
    @brief TransformerROS类，继承自Transformer，并添加了ROS相关的订阅和转换功能。
    """

    def __init__(self, cache_time=rospy.Duration(tf2_ros.Buffer.DEFAULT_CACHE_TIME), debug=False):
        """
        @brief 构造函数。
        @param cache_time 缓存时间。
        @param debug 是否开启调试模式。
        """
        Transformer.__init__(self, cache_time, debug)
        self.tf2_listener = tf2_ros.TransformListener(self)

    def ok(self):
        """
        @brief 检查ROS是否正常运行。
        @return 如果ROS正常运行则返回True，否则返回False。
        """
        return rospy.is_shutdown()

    def transformQuaternion(self, target_frame, stamped_in):
        """
        @brief 转换QuaternionStamped消息。
        @param target_frame 目标帧。
        @param stamped_in 输入的QuaternionStamped消息。
        @return 转换后的QuaternionStamped消息。
        """
        if isinstance(stamped_in, QuaternionStamped):
            return tf2_geometry_msgs.do_transform_quaternion(stamped_in, self.lookup_transform(target_frame, stamped_in.header.frame_id, stamped_in.header.stamp))
        else:
            return Transformer.transformQuaternion(self, target_frame, stamped_in)

    def transformVector3(self, target_frame, stamped_in):
        """
        @brief 转换Vector3Stamped消息。
        @param target_frame 目标帧。
        @param stamped_in 输入的Vector3Stamped消息。
        @return 转换后的Vector3Stamped消息。
        """
        if isinstance(stamped_in, PointStamped):
            return tf2_geometry_msgs.do_transform_vector3(stamped_in, self.lookup_transform(target_frame, stamped_in.header.frame_id, stamped_in.header.stamp))
        else:
            return Transformer.transformVector3(self, target_frame, stamped_in)

    def transformPoint(self, target_frame, stamped_in):
        """
        @brief 转换PointStamped消息。
        @param target_frame 目标帧。
        @param stamped_in 输入的PointStamped消息。
        @return 转换后的PointStamped消息。
        """
        if isinstance(stamped_in, PointStamped):
            return tf2_geometry_msgs.do_transform_point(stamped_in, self.lookup_transform(target_frame, stamped_in.header.frame_id, stamped_in.header.stamp))
        else:
            return Transformer.transformPoint(self, target_frame, stamped_in)

    def transformPose(self, target_frame, stamped_in):
        """
        @brief 转换PoseStamped消息。
        @param target_frame 目标帧。
        @param stamped_in 输入的PoseStamped消息。
        @return 转换后的PoseStamped消息。
        """
        if isinstance(stamped_in, PoseStamped):
            return tf2_geometry_msgs.do_transform_pose(stamped_in, self.lookup_transform(target_frame, stamped_in.header.frame_id, stamped_in.header.stamp))
        else:
            return Transformer.transformPose(self, target_frame, stamped_in)

    def transformTwist(self, target_frame, stamped_in):
        """
        @brief 转换TwistStamped消息。
        @param target_frame 目标帧。
        @param stamped_in 输入的TwistStamped消息。
        @return 转换后的TwistStamped消息。
        """
        if isinstance(stamped_in, PoseStamped):
            return tf2_geometry_msgs.do_transform_twist(stamped_in, self.lookup_transform(target_frame, stamped_in.header.frame_id, stamped_in.header.stamp))
        else:
            return Transformer.transformTwist(self, target_frame, stamped_in)

    def transformPointCloud(self, target_frame, stamped_in):
        """
        @brief 转换PointCloud2消息。
        @param target_frame 目标帧。
        @param stamped_in 输入的PointCloud2消息。
        @return 转换后的PointCloud2消息。
        """
        if isinstance(stamped_in, PoseStamped):
            return tf2_geometry_msgs.do_transform_cloud(stamped_in, self.lookup_transform(target_frame, stamped_in.header.frame_id, stamped_in.header.stamp))
        else:
            return Transformer.transformPointCloud(self, target_frame, stamped_in)


class TransformListener(TransformerROS):

    """
    TransformListener is a subclass of :class:`tf.TransformerROS` that
    subscribes to the ``"/tf"`` message topic, and calls :meth:`tf.Transformer.setTransform`
    with each incoming transformation message.

    In this way a TransformListener object automatically
    stays up to to date with all current transforms.  Typical usage might be::

        import tf
        from geometry_msgs.msg import PointStamped

        class MyNode:

            def __init__(self):

                self.tl = tf.TransformListener()
                rospy.Subscriber("/sometopic", PointStamped, self.some_message_handler)
                ...
            
            def some_message_handler(self, point_stamped):

                # want to work on the point in the "world" frame
                point_in_world = self.tl.transformPoint("world", point_stamped)
                ...
        
    """
    def __init__(self, *args, **kwargs):
        TransformerROS.__init__(self, *args, **kwargs)
        self._listener = tf2_ros.TransformListener(self._buffer)
        self.setUsingDedicatedThread(True)
