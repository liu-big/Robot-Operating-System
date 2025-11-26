# -*- coding: utf-8 -*-
# 导入必要的库
import cv2 # 导入OpenCV库，用于图像处理
import time # 导入time库，用于时间相关操作
import numpy as np # 导入NumPy库，用于数值计算
from func import myFunc # 从func模块导入myFunc函数，可能用于RKNN模型推理
import rospy # 导入rospy库，用于ROS节点开发
from std_msgs.msg import Float32MultiArray # 导入Float32MultiArray消息类型，用于发布浮点数数组
from std_msgs.msg import Int32 # 导入Int32消息类型，用于发布整数
from time import sleep # 从time库导入sleep函数，用于延时

# 定义相机内参矩阵，用于图像畸变矫正
camera_matrix = np.array([[408.12625, 0, 319.03663],
                          [0, 408.87682, 235.3358],
                          [0, 0, 1]], dtype=np.float32)

# 定义相机畸变参数，用于图像畸变矫正
dist_coeffs = np.array([-0.325698, 0.111425, 0.002560, 0.000459, 0.000000], dtype=np.float32)

stop_flag = 0 # 停止标志，可能用于控制机器人停止行为

# 函数：mid
# 功能：巡线数据提取函数，用于计算赛道中值和偏差角度。
# 参数：
#   follow: 图像帧，用于绘制中间线和前瞻点。
#   mask: 二值化图像掩码，用于识别赛道。
# 返回值：
#   error: 图像中点与指定提取中点的误差，正数表示右转，负数表示左转。
#   mid_angle: 偏差角度，正数表示右转，负数表示左转。
def mid(follow, mask):
    height = follow.shape[0] # 获取图像高度
    width = follow.shape[1] # 获取图像宽度

    half_width = width // 2 # 图像半宽
    half = half_width  # 从下往上扫描赛道，最下端取图片中线为分割线
    mid_output = 0 # 初始化赛道中值
    mid_angle = 0 # 初始化偏差角度
    # white_pixels = np.count_nonzero(mask[180:220] == 255)
    # print(white_pixels)
    for y in range(height - 1, -1, -1): # 从图像底部向上遍历每一行
        # 判断分割线左端是否有赛道
        if (mask[y][max(0, half - half_width):half] == np.zeros_like(
                mask[y][max(0, half - half_width):half])).all():  
            left = max(0, half - half_width)  # 如果无赛道，取图片左边界
            cv2.circle(follow, (int(left), y), 1, (255, 255, 255), -1) # 绘制左边界点
        else:
            left = np.average(np.where(mask[y][0:half] == 255))  # 计算分割线左端白色像素的平均位置
            cv2.circle(follow, (int(left), y), 1, (255, 255, 255), -1) # 绘制左侧赛道点

        # 判断分割线右端是否有赛道
        if (mask[y][half:min(width, half + half_width)] == np.zeros_like(
                mask[y][half:min(width, half + half_width)])).all():  
            right = min(width, half + half_width)  # 如果无赛道，取图片右边界
            cv2.circle(follow, (int(right), y), 1, (255, 255, 255), -1) # 绘制右边界点
        else:
            right = np.average(np.where(mask[y][half:width] == 255)) + half  # 计算分割线右端白色像素的平均位置
            cv2.circle(follow, (int(right), y), 1, (255, 255, 255), -1) # 绘制右侧赛道点

        mid = int((left + right) // 2) # 计算拟合中点
        
        half = mid  # 递归，从下往上确定分割线，使分割线跟随赛道中心
        cv2.circle(follow, (mid, y), 1, (255, 255, 255), -1)  # 绘制中线点

        if y == 150:  # 设置前瞻点位置（距离图像底部150像素处）
            mid_output = int(mid) # 获取前瞻点的中值
            cv2.circle(follow, (mid_output, 150), 5, (255, 255, 255), -1)  # 绘制指定提取中点

        error = follow.shape[1] // 2 - mid_output  # 计算图片中点与指定提取中点的误差
        mid_angle = np.arctan(error / height) * 180 / np.pi # 计算偏差角度
        mid_angle = mid_angle # 角度值

        # 以下是被注释掉的霍夫变换直线检测代码，可能用于调试或备用
        # line_mask = follow.copy()
        # # edges = cv2.Canny(line_mask, 50, 150)
        # lines = cv2.HoughLinesP(line_mask, 1, np.pi / 180, threshold=50, minLineLength=100, maxLineGap=10)
        # if lines is not None:
        #     for line in lines:
        #         x1, y1, x2, y2 = line[0]
        #         cv2.line(follow, (x1, y1), (x2, y2), (0, 255, 0), 2)

        #         # 检查是否为横线（中间部分）
        #         if abs(y2 - y1) < 50 and (220 < (y1 + y2) / 2 < 590):
        #             stop_flag = 1
    # 以下是被注释掉的停止线检测逻辑，可能用于调试或备用
    # if white_pixels > stop_line_area_pixels:  # 如果实心矩形穿过100行的直线的白色像素面积大于阈值
    #    error = 0.0
    #    stop_flag = 1.0       
    #    print("stop")
    # else:
    #    stop_flag = 0.0
    #    print("go")

    return error, mid_angle  # 返回误差和角度，error为正数右转，为负数左转；角度同理

# 函数：detect_line
# 功能：ROS订阅回调函数，用于更新任务标志。
# 参数：
#   msg: 接收到的Int32消息，包含任务标志数据。
# 返回值：
#   无
def detect_line(msg):
    global mission_flag # 声明使用全局变量mission_flag
    mission_flag = msg.data # 更新任务标志为接收到的消息数据
    print(mission_flag) # 打印当前任务标志

if __name__ == "__main__":
    # 主程序入口
    # 任务标志，0表示识别任务，1表示巡线任务
    mission_flag = 0
    # 打开摄像头，参数0表示默认摄像头
    cap = cv2.VideoCapture(0)

    # cap = cv2.VideoCapture('a8f1.mp4') # 备用：从视频文件读取帧
    # 初始化 ROS 节点，命名为 "detect"
    rospy.init_node("detect")
    # 导入rknn-lite包（因为与ros有冲突，必须放在初始化ros节点后）
    from rknnpool import rknnPoolExecutor
    # 定义ros消息发布器，发布到 "vision_msg_list" 话题，消息类型为Float32MultiArray
    vision_pub = rospy.Publisher("vision_msg_list", Float32MultiArray, queue_size=10)
    vision_msg = Float32MultiArray() # 创建Float32MultiArray消息对象
    # 定义ros消息订阅器，订阅 "/mission_msg" 话题，消息类型为Int32，回调函数为detect_line
    vision_sub = rospy.Subscriber('/mission_msg',Int32,detect_line,queue_size=10) # 订阅视觉话题 
    
    # 视觉消息列表，用于存储识别或巡线结果
    vision_msg_list = []
    # 导入RKNN模型路径
    modelPath = "./rknnModel/iflytek_new_i8.rknn"
    # 线程数，增大可提高帧率
    TPEs = 1
    # 初始化RKNN池，用于管理RKNN模型推理线程
    pool = rknnPoolExecutor(
        rknnModel=modelPath,
        TPEs=TPEs,
        func=myFunc) # myFunc是RKNN推理的实际处理函数
    # 初始化异步处理所需的图像帧
    if (cap.isOpened()): # 检查摄像头是否成功打开
        for i in range(TPEs + 1): # 预先读取TPEs+1帧图像，填充RKNN池
            ret, frame = cap.read() # 读取一帧图像
            if not ret: # 如果读取失败
                cap.release() # 释放摄像头资源
                del pool # 删除RKNN池对象
                exit(-1) # 退出程序
            frame = frame[0:480, 20:620] # 裁剪图像，取[0:480]行，[20:620]列
            pool.put(frame) # 将图像放入RKNN池进行异步处理
    frames, loopTime, initTime = 0, time.time(), time.time() # 初始化帧数、循环时间和起始时间
    # 图像处理主循环
    while (cap.isOpened()): # 循环直到摄像头关闭
        frames += 1 # 帧数加1
        ret, frame = cap.read() # 读取一帧图像
        if not ret: # 如果读取失败
            break # 退出循环
        # 裁剪图像并水平镜像
        frame = frame[0:480, 20:620] # 裁剪图像
        frame = cv2.flip(frame, 1) # 水平镜像图像
        # 畸变矫正
        frame = cv2.undistort(frame, camera_matrix, dist_coeffs) # 使用相机内参和畸变参数进行畸变矫正

        # 识别任务处理逻辑
        if mission_flag ==0:       
            pool.put(frame) # 将当前帧放入RKNN池进行识别
            frame, flag = pool.get() # 从RKNN池获取处理结果
            
            if flag: # 如果处理成功
                img, class_array, center_array = frame # 解包结果：图像、类别数组、中心点数组
                img = np.asarray(frame[0]) # 将图像转换为NumPy数组
                vision_msg_list = class_array + center_array[0] + center_array[1] + center_array[2] # 组合视觉消息列表
            else:
                pass # 处理失败则跳过
            # 计算帧率
            fps = frames / (time.time() - initTime) # 计算当前帧率
            
            # 在图像上叠加帧率信息
            cv2.putText(img, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2) # 在图像上显示帧率
            cv2.imshow('test', img) # 显示图像
            if cv2.waitKey(1) & 0xFF == ord('q'): # 等待按键，如果按下'q'则退出
                break
        # 巡线任务处理逻辑
        if mission_flag == 1:
            cv2.destroyWindow('test') # 销毁识别任务的显示窗口
            # 设定感兴趣区，并水平镜像画面
            # 检查frame是否有效且包含预期列表
            if isinstance(frame, (list, tuple)) and len(frame) == 3: 
                frame, class_array, center_array = frame # 解包frame
            # 将图像列表转换为 NumPy 数组
                frame = np.asarray(frame) # 转换为NumPy数组
            else:
                print("Frame is None or does not contain the expected lists") # 打印错误信息
            roi_frame = frame[220:480, 0:640] # 定义感兴趣区域（ROI），裁剪图像
            gray = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2GRAY) # 将ROI转换为灰度图
            # gray = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2GRAY)
            # 进行高斯滤波，灰度，自适应阈值处理
            # gauss = cv2.GaussianBlur(gray, (3, 3), 0)
            # #地面不反光使用
            _, mask = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY) # 对灰度图进行二值化处理，阈值为200
            # 以下是被注释掉的图像处理代码，可能用于调试或备用
            # gauss = cv2.GaussianBlur(roi_frame, (3, 3), 0)
            # color = cv2.inRange(gauss, (220, 160 ,160), (250,250,250))
            # gray = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2GRAY)
            # 进行高斯滤波，灰度，自适应阈值处理
            # #地面不反光使用
            # _, mask = cv2.threshold(gauss, 180, 255, cv2.THRESH_BINARY)
            # 地面反光使用
            # mask = cv.adaptiveThreshold(gauss, 255, cv.ADAPTIVE_THRESH_MEAN_C, cv.THRESH_BINARY_INV, 7, 7)
            # 使用闭，膨胀操作，将断开的线补全
            # kernel = np.ones((3, 3), np.uint8)
            # mask = cv2.morphologyEx(color, cv2.MORPH_CLOSE, kernel)
            # kernel = np.ones((3, 3), np.uint8)
            # mask = cv2.dilate(mask, kernel, iterations=1)
            follow = mask.copy() # 复制掩码图像用于绘制
            error, mid_angle = mid(follow, mask) # 调用mid函数计算误差和角度
            lines = cv2.HoughLinesP(mask, 1, np.pi / 180, threshold=400, minLineLength=100, maxLineGap=10) # 使用霍夫变换检测直线
            if lines is not None: # 如果检测到直线
                for line in lines: # 遍历每条直线
                    x1, y1, x2, y2 = line[0] # 获取直线端点坐标
                    angle = np.arctan2(y2 - y1, x2 - x1) * 180 / np.pi # 计算直线角度
                    cv2.line(roi_frame, (x1, y1), (x2, y2), (0, 255, 0), 2) # 在ROI上绘制直线
                    if (abs(angle) < 5 or abs(angle - 180)) < 5 and (140 <= y1 <= 180 or 140 <= y2 <= 180):  # 设置角度阈值，例如小于10度或接近180度，并检查Y坐标范围
                        stop_flag = 1 # 设置停止标志
                        
            else:
                stop_flag = 0 # 未检测到直线则停止标志为0
                    
            vision_msg_list_lines =  [error, mid_angle, stop_flag]
            vision_msg_list = vision_msg_list_lines
            fps = frames / (time.time() - initTime)
            print(f"error:{error}")
            print(f"angle:{mid_angle}")
            #显示图像
            cv2.putText(frame, f"FPS: {fps:.2f}", (230, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow("frame", roi_frame)
            cv2.imshow("mask", mask)
            cv2.imshow("follow", follow)
            #退出键
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            pass

        # 发布消息
        vision_msg.data = vision_msg_list
        vision_pub.publish(vision_msg)
        #print(mission_flag)


    # 释放cap和rknn线程池
    cap.release()
    cv2.destroyAllWindows()
    pool.release()
