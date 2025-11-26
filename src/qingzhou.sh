# 该脚本用于启动多个gnome-terminal终端窗口，并在每个标签页中执行特定的任务。
# 整体功能：启动YOLOv8性能优化脚本、YOLOv8主程序、数据处理脚本、ROS底盘驱动和ROS导航。
gnome-terminal --window -e 'bash -c "exec bash"' \ # 启动一个新的gnome-terminal窗口
--tab -e 'bash -c "sleep 5; cd src/rknn-multi-threaded-yolov8; echo iflytek | sudo -S sh performance.sh; exec bash"' \ # 第一个标签页：等待5秒后，进入YOLOv8性能优化脚本目录，执行performance.sh脚本（用于YOLOv8性能优化）
--tab -e 'bash -c "sleep 5; cd src/rknn-multi-threaded-yolov8; echo iflytek | sudo -S -k && conda activate yolov8 && python3 main.py; exec bash"' \ # 第二个标签页：等待5秒后，进入YOLOv8主程序目录，激活yolov8 conda环境，并运行main.py（YOLOv8主程序）
--tab -e 'bash -c "sleep 5; cd src; python3 process.py; exec bash"' \ # 第三个标签页：等待5秒后，进入src目录，运行process.py脚本（数据处理脚本）
--tab -e 'bash -c "sleep 1; source ~/ucar_ws/devel/setup.bash; roslaunch ucar_controller base_driver.launch; exec bash"' \ # 第四个标签页：等待1秒后，加载ROS环境，启动ucar_controller包中的base_driver.launch（ROS底盘驱动）
--tab -e 'bash -c "sleep 5; source ~/ucar_ws/devel/setup.bash; roslaunch ucar_nav ucar_navigation.launch; exec bash"' # 第五个标签页：等待5秒后，加载ROS环境，启动ucar_nav包中的ucar_navigation.launch（ROS导航）











