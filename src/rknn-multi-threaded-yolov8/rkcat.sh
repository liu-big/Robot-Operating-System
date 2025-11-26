#!/bin/bash
# 脚本名称: rkcat.sh
# 功能: 用于配置RKNN NPU的频率。
# 使用方法: 以root用户身份运行此脚本。

# 提示用户切换到root用户
echo "请切换到root用户" # 提示信息

# 设置NPU频率为最高频率 (1GHz)
# 将1000000000写入NPU频率设置文件
sudo echo 1000000000 > /sys/class/devfreq/fdab0000.npu/userspace/set_freq # 设置NPU频率

# 显示当前NPU频率
echo "NPU 当前频率:" # 提示信息
sudo cat /sys/class/devfreq/fdab0000.npu/cur_freq # 读取并显示当前NPU频率
# 查看温度
sensors
# 查看NPU占用
echo "当前NPU占用:"
sudo cat /sys/kernel/debug/rknpu/load
