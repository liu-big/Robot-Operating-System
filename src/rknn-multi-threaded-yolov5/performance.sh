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

# 文件: performance.sh
# 描述: 该脚本用于优化系统性能，通过设置CPU、GPU和NPU的频率，以达到最佳运行状态。
# 作者: UCAR.AI
# 创建日期: 2024-01-01
# 修订历史:
#    2024-01-01: 初始版本

# 开启NPU模块
# 使用modprobe命令加载rknpu模块，启用NPU硬件加速功能。
echo "NPU已开启"
sudo modprobe rknpu

# CPU定频设置
# 将CPU频率设置为固定值，以提高性能稳定性。

# 设置CPU0-3的频率
echo "CPU0-3 可用频率:" # 显示CPU0-3可用的频率列表
sudo cat /sys/devices/system/cpu/cpufreq/policy0/scaling_available_frequencies
sudo echo userspace > /sys/devices/system/cpu/cpufreq/policy0/scaling_governor # 设置CPU0-3的调频模式为userspace
sudo echo 1800000 > /sys/devices/system/cpu/cpufreq/policy0/scaling_setspeed # 设置CPU0-3的频率为1.8GHz
echo "CPU0-3 当前频率:" # 显示CPU0-3当前频率
sudo cat /sys/devices/system/cpu/cpufreq/policy0/cpuinfo_cur_freq

# 设置CPU4-5的频率
echo "CPU4-5 可用频率:" # 显示CPU4-5可用的频率列表
sudo cat /sys/devices/system/cpu/cpufreq/policy4/scaling_available_frequencies
sudo echo userspace > /sys/devices/system/cpu/cpufreq/policy4/scaling_governor # 设置CPU4-5的调频模式为userspace
sudo echo 2400000 > /sys/devices/system/cpu/cpufreq/policy4/scaling_setspeed # 设置CPU4-5的频率为2.4GHz
echo "CPU4-5 当前频率:" # 显示CPU4-5当前频率
sudo cat /sys/devices/system/cpu/cpufreq/policy4/cpuinfo_cur_freq

# 设置CPU6-7的频率
echo "CPU6-7 可用频率:" # 显示CPU6-7可用的频率列表
sudo cat /sys/devices/system/cpu/cpufreq/policy6/scaling_available_frequencies
sudo echo userspace > /sys/devices/system/cpu/cpufreq/policy6/scaling_governor # 设置CPU6-7的调频模式为userspace
sudo echo 2400000 > /sys/devices/system/cpu/cpufreq/policy6/scaling_setspeed # 设置CPU6-7的频率为2.4GHz
echo "CPU6-7 当前频率:" # 显示CPU6-7当前频率
sudo cat /sys/devices/system/cpu/cpufreq/policy6/cpuinfo_cur_freq

# GPU定频设置
# 将GPU频率设置为固定值，以提高图形处理性能。
cat /sys/class/devfreq/fb000000.gpu/available_frequencies # 显示GPU可用的频率列表
echo userspace > /sys/class/devfreq/fb000000.gpu/governor # 设置GPU的调频模式为userspace
echo 1000000000 > /sys/class/devfreq/fb000000.gpu/userspace/set_freq # 设置GPU的频率为1GHz
cat /sys/class/devfreq/fb000000.gpu/cur_freq # 显示GPU当前频率

# NPU定频设置
# 将NPU频率设置为固定值，以提高神经网络处理性能。
echo "NPU 可用频率:" # 显示NPU可用的频率列表
sudo cat /sys/class/devfreq/fdab0000.npu/available_frequencies    
sudo echo userspace > /sys/class/devfreq/fdab0000.npu/governor # 设置NPU的调频模式为userspace
sudo echo 1000000000 > /sys/class/devfreq/fdab0000.npu/userspace/set_freq # 设置NPU的频率为1GHz
echo "NPU 当前频率:" # 显示NPU当前频率
sudo cat /sys/class/devfreq/fdab0000.npu/cur_freq
