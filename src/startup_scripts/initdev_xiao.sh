#!/bin/bash
#
# @file initdev_xiao.sh
# @brief 用于初始化开发环境的脚本，主要配置 udev 规则和 ROS 环境变量。
#
# 该脚本执行以下操作：
# 1. 将 ROS 工作空间的 setup.bash 脚本添加到用户的 .bashrc 文件中。
# 2. 添加 udev 规则，为特定的 USB 串口设备（底盘和激光雷达）和 USB 摄像头设备创建符号链接。
# 3. 添加 udev 规则，设置麦克风阵列和 ttyTHS1 设备的权限。
# 4. 重新加载并重启 udev 服务，使新规则生效。

# 将 ROS 工作空间的 setup.bash 脚本添加到 /home/ucar/.bashrc 文件中
# 这确保了每次打开终端时，ROS 环境变量都会被正确设置，方便小车底盘的 ROS 开发
echo 'source /home/ucar/ucar_ws/devel/setup.bash' >> /home/ucar/.bashrc

# 添加 udev 规则：为底盘串口设备创建符号链接 'base_serial_port'
# 这使得底盘串口可以通过 /dev/base_serial_port 访问
echo 'KERNEL=="ttyUSB*", SUBSYSTEMS=="usb",ATTRS{idVendor}=="10c4",ATTRS{idProduct}=="ea60",KERNELS=="1-2.3",NAME="ttyUSB0",SYMLINK+="base_serial_port"' >   /etc/udev/rules.d/ucar.rules

# 添加 udev 规则：为激光雷达串口设备创建符号链接 'lidar_serial_port'
# 这使得激光雷达串口可以通过 /dev/lidar_serial_port 访问
echo 'KERNEL=="ttyUSB*", SUBSYSTEMS=="usb",ATTRS{idVendor}=="10c4",ATTRS{idProduct}=="ea60",KERNELS=="1-2.1",NAME="ttyUSB1",SYMLINK+="lidar_serial_port"' >> /etc/udev/rules.d/ucar.rules

# 添加 udev 规则：为 type-c 摄像头设备创建符号链接 'ucar_video'
# 这使得该摄像头可以通过 /dev/ucar_video 访问
echo 'KERNEL=="video*", SUBSYSTEMS=="usb",ATTRS{idVendor}=="0edc",ATTRS{idProduct}=="2050",KERNELS=="1-2.4",NAME="video0",SYMLINK+="ucar_video"' >>  /etc/udev/rules.d/ucar.rules

# 添加 udev 规则：设置麦克风阵列设备的权限为 0666 (读写)
# 确保应用程序可以访问麦克风阵列
echo 'ATTRS{idVendor}=="10d6", ATTRS{idProduct}=="b003", MODE="0666"' >>  /etc/udev/rules.d/ucar.rules

# 添加 udev 规则：设置 ttyTHS1 设备的权限为 0666 (读写)
# 确保应用程序可以访问 ttyTHS1 串口
echo 'KERNEL=="ttyTHS1" MODE="0666"' >>  /etc/udev/rules.d/ucar.rules

# 重新加载 udev 规则
service udev reload
# 等待 2 秒，确保 udev 服务有足够时间处理规则
sleep 2
# 重启 udev 服务，使新规则完全生效
service udev restart
