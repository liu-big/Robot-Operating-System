# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
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
#  * Neither the name of Willow Garage, Inc. nor the names of its
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
#
# Revision $Id$

# @file tfwtf.py
# @brief TF诊断工具。
# 该文件包含用于诊断TF（Transform Frame）相关问题的roswtf插件。
# 它定义了多种规则来检测TF消息中的异常情况，如时间戳差异、父子关系重定义、循环以及非规范化旋转等。

from __future__ import print_function

import time

from roswtf.rules import warning_rule, error_rule
import rosgraph
import rospy

import tf.msg

import math


# 全局变量：接收到的TF消息列表
_msgs = []

################################################################################
# 规则定义

def rostime_delta(ctx):
    """
    @brief 检查TF消息时间戳与ROS时间戳的差异。
    @param ctx 上下文对象。
    @return 包含时间戳差异过大错误的列表。
    """
    deltas = {}
    for m, stamp, callerid in _msgs:
        for t in m.transforms:
            d = t.header.stamp - stamp
            secs = d.to_sec()
            if abs(secs) > 1.:
                if callerid in deltas:
                    if abs(secs) > abs(deltas[callerid]):
                        deltas[callerid] = secs
                else:
                    deltas[callerid]  = secs

    errors = []
    for k, v in deltas.items():
        errors.append("receiving transform from [{}] that differed from ROS time by {}s".format(k, v))
    return errors

def reparenting(ctx):
    """
    @brief 检测TF帧的父子关系重定义。
    @param ctx 上下文对象。
    @return 包含父子关系重定义错误的列表。
    """
    errors = []
    parent_id_map = {}
    for m, stamp, callerid in _msgs:
        for t in m.transforms:
            frame_id = t.child_frame_id
            parent_id = t.header.frame_id
            if frame_id in parent_id_map and parent_id_map[frame_id] != parent_id:
                msg = "reparenting of [{}] to [{}] by [{}]".format(frame_id, parent_id, callerid)
                parent_id_map[frame_id] = parent_id
                if msg not in errors:
                    errors.append(msg)
            else:
                parent_id_map[frame_id] = parent_id
    return errors

def cycle_detection(ctx):
    """
    @brief 检测TF帧之间的循环依赖。
    @param ctx 上下文对象。
    @return 包含循环依赖错误的列表。
    """
    max_depth = 100
    errors = []
    parent_id_map = {}
    for m, stamp, callerid in _msgs:
        for t in m.transforms:
            frame_id = t.child_frame_id
            parent_id = t.header.frame_id
            parent_id_map[frame_id] = parent_id

    for frame in parent_id_map:
        frame_list = []
        current_frame = frame
        count = 0
        while count < max_depth + 2:
            count = count + 1
            frame_list.append(current_frame)
            try:
                current_frame = parent_id_map[current_frame]
            except KeyError:
                break
            if current_frame == frame:
                errors.append("Frame {} is in a loop. It's loop has elements:\n{}".format(frame, " -> ".join(frame_list)))
                break

    return errors

def multiple_authority(ctx):
    """
    @brief 检测TF帧的多重发布者冲突。
    @param ctx 上下文对象。
    @return 包含多重发布者冲突错误的列表。
    """
    errors = []
    authority_map = {}
    for m, stamp, callerid in _msgs:
        for t in m.transforms:
            frame_id = t.child_frame_id
            parent_id = t.header.frame_id 
            if frame_id in authority_map and authority_map[frame_id] != callerid:
                msg = "node [{}] publishing transform [{}] with parent [{}] already published by node [{}]".format(callerid, frame_id, parent_id, authority_map[frame_id])
                authority_map[frame_id] = callerid
                if msg not in errors:
                    errors.append(msg)
            else:
                authority_map[frame_id] = callerid
    return errors

def no_msgs(ctx):
    """
    @brief 检查是否接收到TF消息。
    @param ctx 上下文对象。
    @return 如果没有接收到消息则返回True，否则返回False。
    """
    return not _msgs

def not_normalized(ctx):
    """
    @brief 检测TF消息中旋转四元数是否未归一化。
    @param ctx 上下文对象。
    @return 包含未归一化旋转错误的列表。
    """
    errors = []
    for m, stamp, callerid in _msgs:
        for t in m.transforms:
            q = t.transform.rotation
            length = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
            if math.fabs(length - 1) > 1e-6:
                errors.append("rotation from [{}] to [{}] is not unit length, {}".format(t.header.frame_id, t.child_frame_id, length))
    return errors

################################################################################
# roswtf 插件接口

# tf_warnings 和 tf_errors 声明了实际检查的规则

tf_warnings = [
  (no_msgs, "No tf messages"),
  (rostime_delta, "Received out-of-date/future transforms:"),  
  (not_normalized, "Received non-normalized rotation in transforms:"),
]
tf_errors = [
  (reparenting, "TF re-parenting contention:"),
  (cycle_detection, "TF cycle detection::"),
  (multiple_authority, "TF multiple authority contention:"),
]

# rospy 订阅器回调函数，用于处理 /tf 话题
def _tf_handle(msg):
    """
    @brief /tf 话题的回调函数，将接收到的消息添加到全局列表中。
    @param msg 接收到的tfMessage消息。
    """
    _msgs.append((msg, rospy.get_rostime(), msg._connection_header['callerid']))

# @return bool: 如果 /tf 有发布者则返回True
def is_tf_active():
    """
    @brief 检查TF话题是否活跃（是否有发布者）。
    @return 如果/tf话题有发布者则返回True，否则返回False。
    """
    master = rosgraph.Master('/tfwtf')
    if master is not None:
        try:
            val = master.getPublishedTopics('/')
            if filter(lambda x: x[0] == '/tf', val):
                return True
        except:
            pass
    return False

# roswtf 在线检查的入口点
def roswtf_plugin_online(ctx):
    """
    @brief roswtf在线插件的入口函数，执行TF相关检查。
    @param ctx 上下文对象。
    """
    # 如果tf不活跃，则不运行插件，因为这些检查需要时间
    if not is_tf_active():
        return

    print("running tf checks, this will take a second...")
    sub1 = rospy.Subscriber('/tf', tf.msg.tfMessage, _tf_handle)
    time.sleep(1.0)
    sub1.unregister()
    print("... tf checks complete")

    for r in tf_warnings:
        warning_rule(r, r[0](ctx), ctx)
    for r in tf_errors:
        error_rule(r, r[0](ctx), ctx)

# 当前没有tf的静态检查
#def roswtf_plugin_static(ctx):
#    for r in tf_warnings:
#        warning_rule(r, r[0](ctx), ctx)
#    for r in tf_errors:
#        error_rule(r, r[0](ctx), ctx)
