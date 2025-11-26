#!/bin/bash
#
# Copyright (c) 2024, UCAR.AI, All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ============================================================================== 

# 文件: rkcat.sh
# 描述: 该脚本用于显示系统传感器信息和NPU（神经网络处理单元）的当前负载情况。
# 作者: UCAR.AI
# 创建日期: 2024-01-01
# 修订历史:
#    2024-01-01: 初始版本

# 显示系统传感器信息
# sensors命令用于显示各种硬件传感器的读数，例如CPU温度、风扇转速、电压等。
sensors

# 查看NPU占用率
# 通过读取/sys/kernel/debug/rknpu/load文件，可以获取NPU的当前负载情况。
echo "当前NPU占用:"
sudo cat /sys/kernel/debug/rknpu/load
