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

# 该脚本用于更新Bullet数据类型，将tf从Electric版本更新到Fuerte/Unstable或更高版本。
# 默认情况下，此脚本假定您的文件未使用tf命名空间。
# 如果使用了tf命名空间，请修改下面的for循环，以使用namespaced_rules。

from __future__ import print_function

import subprocess


# 定义sed命令，用于查找并替换文件内容。它会查找当前目录下所有非版本控制文件（.svn-base, .hg, .git），
# 且文件扩展名为.c或.h的文件，然后对这些文件执行sed替换规则。
cmd = "find . -type f ! -name '*.svn-base' -a ! -name '*.hg' -a ! -name '*.git' -a \( -name '*.c*' -o -name '*.h*' \) -exec sed -i '%(rule)s' {} \;"

# 定义一组替换规则，用于更新Bullet库的头文件路径和类型名称。
# 这些规则主要针对包含路径和bt*类型名称的替换。
rules = ['s|LinearMath/bt|tf/LinearMath/|g',  # 包含路径替换
         's/btTransform\.h/Transform\.h/g',  # 头文件名称替换
         's/btMatrix3x3\.h/Matrix3x3\.h/g',
         's/btScalar\.h/Scalar\.h/g',
         's/btQuaternion\.h/Quaternion\.h/g',
         's/btQuadWord\.h/QuadWord\.h/g',
         's/btMinMax\.h/MinMax\.h/g',
         's/btVector3\.h/Vector3\.h/g',
         's/btScalar/tfScalar/g',
         ]

# 定义一组未命名空间化的替换规则，用于将bt*类型替换为tf::*类型。
unnamespaced_rules = [
         's/btTransform/tf::Transform/g',
         's/btQuaternion/tf::Quaternion/g',
         's/btVector3/tf::Vector3/g',
         's/btMatrix3x3/tf::Matrix3x3/g',
         's/btQuadWord/tf::QuadWord/g',

         ]

# 定义一组命名空间化的替换规则，用于将bt*类型替换为*类型（假定已在tf命名空间内）。
namespaced_rules = [ 
         's/btTransform/Transform/g',
         's/btQuaternion/Quaternion/g',
         's/btVector3/Vector3/g',
         's/btMatrix3x3/Matrix3x3/g',
         's/btQuadWord/QuadWord/g',
         #'s/btScalar/Scalar/g',
         ]


# 遍历所有规则（当前使用未命名空间化的规则），并执行sed命令。
# 如果您的文件使用了tf命名空间，请将 `rules + unnamespaced_rules` 更改为 `rules + namespaced_rules`。
for rule in rules + unnamespaced_rules: # 如果文件使用了tf命名空间，请在此处进行更改
    # 格式化完整的sed命令，将当前规则嵌入到命令字符串中。
    full_cmd = cmd%locals()
    print("Running {}".format(full_cmd))
    # 执行shell命令并捕获返回码。
    ret_code = subprocess.call(full_cmd, shell=True)
    # 根据返回码判断命令执行是否成功。
    if ret_code == 0:
        print("success")
    else:
        print("failure")
