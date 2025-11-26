### with abot###
# canda.sh 脚本用于启动多个终端窗口，并在每个窗口中执行特定的任务。
# 主要功能包括：
# 1. 启动一个主终端窗口。
# 2. 启动 YOLOv8 性能优化脚本。
# 3. 启动 YOLOv8 主程序。

gnome-terminal --window -e 'bash -c "exec bash"' \
# 第一个标签页：启动 YOLOv8 性能优化脚本。
# 延迟 5 秒后，进入 `src/rknn-multi-threaded-yolov8` 目录，并以 sudo 权限运行 `performance.sh` 脚本。
--tab -e 'bash -c "sleep 5; cd src/rknn-multi-threaded-yolov8; echo iflytek | sudo -S sh performance.sh; exec bash"' \
# 第二个标签页：启动 YOLOv8 主程序。
# 延迟 5 秒后，进入 `src/rknn-multi-threaded-yolov8` 目录，并运行 `main.py` Python 程序。
--tab -e 'bash -c "sleep 5; cd src/rknn-multi-threaded-yolov8; python main.py; exec bash"' \












