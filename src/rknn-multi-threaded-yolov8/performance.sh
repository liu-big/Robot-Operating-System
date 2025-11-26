#!/bin/bash
# 脚本名称: performance.sh
# 功能: 用于测试RKNN模型在不同线程数下的性能。
# 使用方法: 直接运行此脚本。

# 定义一个包含不同线程数的数组
threads=(1 2 3 4 5 6 7 8)

# 遍历线程数组，对每个线程数进行测试
for t in ${threads[@]}
do
    echo "Testing with $t threads..." # 输出当前测试的线程数
    # 运行Python脚本，并传递线程数作为参数
    # 注意: 这里的Python脚本路径和名称需要根据实际情况进行调整
    python3 main.py $t # 假设main.py接受一个线程数参数
    echo "--------------------"
done

echo "Performance test completed." # 性能测试完成提示
# 请切换到root用户

# 开启NPU
echo "NPU已开启"
sudo modprobe rknpu
# CPU定频
echo "CPU0-3 可用频率:"
sudo cat /sys/devices/system/cpu/cpufreq/policy0/scaling_available_frequencies
sudo echo userspace > /sys/devices/system/cpu/cpufreq/policy0/scaling_governor
sudo echo 1800000 > /sys/devices/system/cpu/cpufreq/policy0/scaling_setspeed
echo "CPU0-3 当前频率:"
sudo cat /sys/devices/system/cpu/cpufreq/policy0/cpuinfo_cur_freq

echo "CPU4-5 可用频率:"
sudo cat /sys/devices/system/cpu/cpufreq/policy4/scaling_available_frequencies
sudo echo userspace > /sys/devices/system/cpu/cpufreq/policy4/scaling_governor
sudo echo 2400000 > /sys/devices/system/cpu/cpufreq/policy4/scaling_setspeed
echo "CPU4-5 当前频率:"
sudo cat /sys/devices/system/cpu/cpufreq/policy4/cpuinfo_cur_freq

echo "CPU6-7 可用频率:"
sudo cat /sys/devices/system/cpu/cpufreq/policy6/scaling_available_frequencies
sudo echo userspace > /sys/devices/system/cpu/cpufreq/policy6/scaling_governor
sudo echo 2400000 > /sys/devices/system/cpu/cpufreq/policy6/scaling_setspeed
echo "CPU6-7 当前频率:"
sudo cat /sys/devices/system/cpu/cpufreq/policy6/cpuinfo_cur_freq
# GPU定频
cat /sys/class/devfreq/fb000000.gpu/available_frequencies
echo userspace > /sys/class/devfreq/fb000000.gpu/governor
echo 1000000000 > /sys/class/devfreq/fb000000.gpu/userspace/set_freq
cat /sys/class/devfreq/fb000000.gpu/cur_freq
# NPU定频
echo "NPU 可用频率:"
sudo cat /sys/class/devfreq/fdab0000.npu/available_frequencies    
sudo echo userspace > /sys/class/devfreq/fdab0000.npu/governor
sudo echo 1000000000 > /sys/class/devfreq/fdab0000.npu/userspace/set_freq
echo "NPU 当前频率:"
sudo cat /sys/class/devfreq/fdab0000.npu/cur_freq
