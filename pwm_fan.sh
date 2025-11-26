#!/bin/bash

# 脚本功能：该脚本用于持续控制风扇以最大PWM转速运行，确保设备散热。

# 定义风扇硬件监控目录
HWMON_DIR="/sys/class/hwmon/hwmon8"

# 函数：enable_pwm_control
# 描述：检查并启用风扇的PWM控制功能。如果对应的PWM启用文件存在，则将其设置为启用状态。
# 参数：无
# 使用方法：在脚本启动时调用此函数，确保风扇可以通过PWM进行调速。
enable_pwm_control() {
    # 检查PWM启用文件是否存在
    if [ -f $HWMON_DIR/pwm1_enable ]; then
        # 如果存在，则写入1启用PWM控制
        echo 1 | sudo tee $HWMON_DIR/pwm1_enable
        echo "PWM控制已启用。" # 添加日志输出
    else
        echo "PWM启用文件不存在：$HWMON_DIR/pwm1_enable。无法启用PWM控制。" # 添加错误日志
    fi
}

# 函数：set_fan_speed_loop
# 描述：持续设置风扇转速到最大值（255），并通过短暂休眠来控制设置频率，避免CPU占用过高。
# 参数：无
# 使用方法：在后台持续运行此函数，以保持风扇全速运转。
set_fan_speed_loop() {
    echo "开始设置风扇转速为最大值（255）。" # 添加日志输出
    while true; do
        # 设置PWM值为255（最大转速），通过sudo tee写入，需要用户密码
        echo 255 | sudo tee $HWMON_DIR/pwm1
        # 短暂休眠0.1秒，避免CPU占用过高，同时允许其他进程运行
        sleep 0.1
    done
}

# 主程序入口
# 1. 启用PWM控制
enable_pwm_control
# 2. 持续设置风扇转速
set_fan_speed_loop
