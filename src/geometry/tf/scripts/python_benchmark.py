#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Software License Agreement (BSD License)
#
# Copyright (c) 2011, Willow Garage, Inc.
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

# 导入必要的模块
from __future__ import print_function

import rostest
import rospy
import numpy
import unittest
import sys
import time
try:
    from cStringIO import StringIO
except ImportError:
    from io import StringIO


import tf.transformations
import geometry_msgs.msg

from tf.msg import tfMessage

import tf

# 定义迭代次数
iterations = 10000

# 初始化tf转换器
t = tf.Transformer()

# 定义一个函数用于创建TransformStamped消息
def mkm():
    """
    mkm()
    @brief 创建并返回一个geometry_msgs/TransformStamped消息。
    该消息包含一个从"PARENT"到"THISFRAME"的变换，其中y轴平移5.0，旋转为单位四元数。
    @return geometry_msgs.msg.TransformStamped: 创建的TransformStamped消息。
    """
    m = geometry_msgs.msg.TransformStamped()
    m.header.frame_id = "PARENT"
    m.child_frame_id = "THISFRAME"
    m.transform.translation.y = 5.0
    m.transform.rotation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(0, 0, 0))
    return m

# 创建一个包含20个TransformStamped消息的tfMessage
tm = tfMessage([mkm() for i in range(20)])

# 定义一个函数用于将ROS消息序列化为字符串
def deserel_to_string(o):
    """
    deserel_to_string(o)
    @brief 将ROS消息对象序列化为字符串。
    @param o: 待序列化的ROS消息对象。
    @return str: 序列化后的字符串。
    """
    s = StringIO()
    o.serialize(s)
    return s.getvalue()

# 序列化tfMessage为字符串
mstr = deserel_to_string(tm)

# 定义一个Timer类用于测量函数执行时间
class Timer:
    """
    Timer类用于测量函数的平均执行时间。
    """
    def __init__(self, func):
        """
        __init__(self, func)
        @brief 构造函数，初始化Timer对象。
        @param func: 待测量执行时间的函数。
        """
        self.func = func
    def mean(self, iterations = 1000000):
        """
        mean(self, iterations=1000000)
        @brief 计算函数的平均执行时间。
        @param iterations: 执行函数的迭代次数，默认为1000000。
        @return float: 函数每次执行的平均时间（秒）。
        """
        started = time.time()
        for i in range(iterations):
            self.func()
        took = time.time() - started
        return took / iterations
        
# 导入tf.msg和tf.cMsg，并对tfMessage进行反序列化和断言测试
import tf.msg
import tf.cMsg
for t in [tf.msg.tfMessage, tf.cMsg.tfMessage]:
    m2 = t()
    m2.deserialize(mstr)
    for m in m2.transforms:
        print(type(m), sys.getrefcount(m))
    assert deserel_to_string(m2) == mstr, "deserel screwed up for type %s" % repr(t)

    m2 = t()
    print("deserialize only {} us each".format(1e6 * Timer(lambda: m2.deserialize(mstr)).mean()))

sys.exit(0)

# 性能测试：仅setTransform的平均时间
started = time.time()
for i in range(iterations):
    for m in tm.transforms:
        t.setTransform(m)
took = time.time() - started
print("setTransform only {} took {} us each".format(iterations, took, (1e6 * took / iterations)))

# 性能测试：deserialize+setTransform的平均时间
started = time.time()
for i in range(iterations):
    m2 = tfMessage()
    m2.deserialize(mstr)
    for m in m2.transforms:
        t.setTransform(m)
took = time.time() - started
print("deserialize+setTransform {} took {} us each".format(iterations, took, (1e6 * took / iterations)))

# 导入TransformListener
from tf import TransformListener
