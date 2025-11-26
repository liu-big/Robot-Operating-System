#!/bin/bash
#
# @file initdev_mini.sh
# @brief 用于初始化开发环境的脚本，主要配置 udev 规则和 ROS 环境变量。
#
# 该脚本执行以下操作：
# 1. 添加 udev 规则，为特定的 USB 摄像头设备创建符号链接。
# 2. 重新加载并重启 udev 服务，使新规则生效。
# 3. 将 ROS 工作空间的 setup.bash 脚本添加到用户的 .bashrc 文件中，以便每次登录时自动加载 ROS 环境变量。

# 添加 udev 规则：为 type-c 摄像头设备创建符号链接 'ucar_video'
# 这使得该摄像头可以通过 /dev/ucar_video 访问，而不是 /dev/videoX
echo 'KERNEL=="video*", SUBSYSTEMS=="usb",ATTRS{idVendor}=="0edc",ATTRS{idProduct}=="2050",KERNELS=="1-1",NAME="video0",SYMLINK+="ucar_video"' >>  /etc/udev/rules.d/ucar.rules

# 重新加载 udev 规则
service udev reload
# 等待 2 秒，确保 udev 服务有足够时间处理规则
sleep 2
# 重启 udev 服务，使新规则完全生效
service udev restart

# 将 ROS 工作空间的 setup.bash 脚本添加到 /home/ucar/.bashrc 文件中
# 这确保了每次打开终端时，ROS 环境变量都会被正确设置，方便小车底盘的 ROS 开发
echo 'source /home/ucar/ucar_ws/devel/setup.bash' >> /home/ucar/.bashrc

