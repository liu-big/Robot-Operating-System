# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
@file broadcaster.py
@brief 这是一个ROS TF（Transform Frame）Python库中用于广播变换的模块。
它提供了一个方便的类`TransformBroadcaster`，用于将TF变换更新发布到ROS的"/tf"消息话题。
"""

import geometry_msgs.msg
import tf2_ros.transform_broadcaster

/**
 * @class TransformBroadcaster
 * @brief TransformBroadcaster类提供了一种方便的方式来在"/tf"消息话题上发送变换更新。
 */
class TransformBroadcaster:
    """
    :class:`TransformBroadcaster` is a convenient way to send transformation updates on the ``"/tf"`` message topic.
    """

    /**
     * @brief TransformBroadcaster的构造函数。
     * @param queue_size 消息队列的大小，默认为100。
     */
    def __init__(self, queue_size=100):
        self.tf2_broadcaster = tf2_ros.transform_broadcaster.TransformBroadcaster()

    /**
     * @brief 广播从子坐标系到父坐标系的变换。
     * @param translation 变换的平移部分，格式为元组 (x, y, z)。
     * @param rotation 变换的旋转部分，格式为元组 (x, y, z, w)。
     * @param time 变换的时间戳，为rospy.Time()对象。
     * @param child 子坐标系的名称，字符串类型。
     * @param parent 父坐标系的名称，字符串类型。
     *
     * 将从TF子坐标系到父坐标系的变换广播到ROS话题"/tf"上。
     */
    def sendTransform(self, translation, rotation, time, child, parent):
        """
        :param translation: the translation of the transformtion as a tuple (x, y, z)
        :param rotation: the rotation of the transformation as a tuple (x, y, z, w)
        :param time: the time of the transformation, as a rospy.Time()
        :param child: child frame in tf, string
        :param parent: parent frame in tf, string

        Broadcast the transformation from tf frame child to parent on ROS topic ``"/tf"``.
        """

        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = parent
        t.header.stamp = time
        t.child_frame_id = child
        t.transform.translation.x = translation[0]
        t.transform.translation.y = translation[1]
        t.transform.translation.z = translation[2]

        t.transform.rotation.x = rotation[0]
        t.transform.rotation.y = rotation[1]
        t.transform.rotation.z = rotation[2]
        t.transform.rotation.w = rotation[3]

        self.sendTransformMessage(t)

    /**
     * @brief 广播TransformStamped消息。
     * @param transform geometry_msgs.msg.TransformStamped对象。
     *
     * 将TF子坐标系到父坐标系的变换广播到ROS话题"/tf"上。
     */
    def sendTransformMessage(self, transform):
        """
        :param transform: geometry_msgs.msg.TransformStamped
        Broadcast the transformation from tf frame child to parent on ROS topic ``"/tf"``.
        """
        self.tf2_broadcaster.sendTransform([transform])
