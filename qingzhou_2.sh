# qingzhou_2.sh 脚本
# 描述：该脚本用于启动多个ROS节点和YOLOv8目标检测相关的程序，
#      通过gnome-terminal在不同的tab页中运行，方便开发和调试。

gnome-terminal --window -e 'bash -c "exec bash"' \ # 启动一个新的gnome-terminal窗口
# Tab 1: 启动语音命令节点
# 描述：加载ROS环境，然后启动speech_command包中的speech_command.launch文件，
#      该文件通常用于启动语音识别和命令处理节点。
# 延时：1秒，确保ROS环境已准备就绪。
--tab -e 'bash -c "sleep 1; source ~/ucar_ws/devel/setup.bash; roslaunch speech_command speech_command.launch; exec bash"' \ # 第一个标签页：等待1秒后，加载ROS环境，启动语音命令节点
# Tab 2: 启动YOLOv8性能优化脚本
# 描述：进入rknn-multi-threaded-yolov8目录，执行performance.sh脚本，
#      该脚本通常用于设置NPU、CPU和GPU的频率，以优化YOLOv8的推理性能。
# 延时：5秒，确保系统有足够时间启动。
--tab -e 'bash -c "sleep 5; cd src/rknn-multi-threaded-yolov8; echo iflytek | sudo -S sh performance.sh; exec bash"' \ # 第二个标签页：等待5秒后，进入YOLOv8性能优化脚本目录，执行performance.sh脚本
# Tab 3: 启动YOLOv8主程序
# 描述：进入rknn-multi-threaded-yolov8目录，激活conda环境yolov8，
#      然后运行main.py，启动YOLOv8目标检测的主程序。
# 延时：5秒，等待性能优化脚本执行完毕。
--tab -e 'bash -c "sleep 5; cd src/rknn-multi-threaded-yolov8; source ~/miniconda3/etc/profile.d/conda.sh && conda activate yolov8 && python3 main.py; exec bash"' \ # 第三个标签页：等待5秒后，进入YOLOv8主程序目录，激活yolov8 conda环境，并运行main.py
# Tab 4: 启动ROS process0_copy_3.py脚本
# 描述：进入src目录，运行process0_copy_3.py脚本，该脚本通常处理ROS话题数据或执行其他ROS相关任务。
# 延时：5秒，等待YOLOv8程序启动。
--tab -e 'bash -c "sleep 5; cd src; python3 process0_copy_3.py; exec bash"' \ # 第四个标签页：等待5秒后，进入src目录，运行process0_copy_3.py脚本
# Tab 5: 启动ucar_controller的base_driver节点
# 描述：加载ROS环境，然后启动ucar_controller包中的base_driver.launch文件，
#      该文件通常用于启动机器人底盘驱动节点。
# 延时：1秒，确保ROS环境已准备就绪。
--tab -e 'bash -c "sleep 1; source ~/ucar_ws/devel/setup.bash; roslaunch ucar_controller base_driver.launch; exec bash"' \ # 第五个标签页：等待1秒后，加载ROS环境，启动ucar_controller包中的base_driver.launch
# Tab 6: 启动ucar_nav的导航节点
# 描述：加载ROS环境，然后启动ucar_nav包中的ucar_navigation.launch文件，
#      该文件通常用于启动ROS导航栈，包括地图服务、AMCL、move_base等。
# 延时：5秒，等待底盘驱动节点启动。
--tab -e 'bash -c "sleep 5; source ~/ucar_ws/devel/setup.bash; roslaunch ucar_nav ucar_navigation.launch; exec bash"' # 第六个标签页：等待5秒后，加载ROS环境，启动ucar_nav包中的ucar_navigation.launch




