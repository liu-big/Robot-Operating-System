# -*- coding: utf-8 -*-
# 导入必要的库
import queue # 导入队列模块，用于线程间通信
import threading # 导入线程模块，用于多线程处理
import numpy as np # 导入NumPy库，用于数值计算
from rknn.api import RKNNLite # 从rknn.api导入RKNNLite，用于RKNN模型推理

# 定义RKNN池执行器类
class rknnPoolExecutor(threading.Thread):
    # 初始化方法
    # 参数:
    #   rknnModel: RKNN模型文件路径。
    #   TPEs: 线程池中的线程数量。
    #   func: 每个线程执行的函数，通常是RKNN模型的后处理函数。
    def __init__(self, rknnModel, TPEs, func):
        threading.Thread.__init__(self) # 调用父类Thread的初始化方法
        self.queue = queue.Queue() # 创建一个队列，用于存放待处理的图像帧
        self.rknnModel = rknnModel # RKNN模型路径
        self.TPEs = TPEs # 线程池大小
        self.func = func # 处理函数
        self.flag = True # 运行标志
        self.rknn_list = [] # RKNN上下文列表
        self.rknn_init_finish = False # RKNN初始化完成标志
        self.rknn_init_lock = threading.Lock() # RKNN初始化锁，防止多线程同时初始化
        self.rknn_init_condition = threading.Condition(self.rknn_init_lock) # RKNN初始化条件变量
        self.start() # 启动线程

    # 线程运行方法
    def run(self):
        # RKNN模型初始化
        for i in range(self.TPEs): # 遍历线程池大小
            rknn = RKNNLite() # 创建RKNNLite对象
            ret = rknn.load_rknn(self.rknnModel) # 加载RKNN模型
            if ret != 0: # 如果加载失败
                print('load rknn model failed!') # 打印错误信息
                exit(ret) # 退出程序
            ret = rknn.init_runtime() # 初始化RKNN运行时环境
            if ret != 0: # 如果初始化失败
                print('init rknn runtime failed!') # 打印错误信息
                exit(ret) # 退出程序
            self.rknn_list.append(rknn) # 将初始化好的RKNN对象添加到列表中
        # 通知所有等待的线程RKNN初始化已完成
        with self.rknn_init_condition:
            self.rknn_init_finish = True # 设置初始化完成标志
            self.rknn_init_condition.notify_all() # 通知所有等待的线程

        # 循环处理图像帧
        while self.flag: # 当运行标志为True时循环
            if self.queue.empty(): # 如果队列为空
                continue # 继续下一次循环
            # 从队列中获取图像帧和对应的RKNN对象索引
            data, index = self.queue.get() # 获取数据和索引
            # 执行RKNN推理
            outputs = self.rknn_list[index].inference(inputs=[data]) # 执行推理
            # 调用处理函数对推理结果进行后处理
            self.queue.task_done() # 标记任务完成
            self.queue.put((self.func(outputs), True)) # 将处理结果放入队列

    # 放入图像帧进行处理
    # 参数:
    #   data: 待处理的图像帧。
    def put(self, data):
        # 等待RKNN初始化完成
        with self.rknn_init_condition:
            while not self.rknn_init_finish: # 如果RKNN未初始化完成
                self.rknn_init_condition.wait() # 等待条件变量
        # 将图像帧和RKNN对象索引放入队列
        self.queue.put((data, self.queue.qsize() % self.TPEs)) # 放入数据和索引

    # 获取处理结果
    # 返回值:
    #   处理后的图像帧和处理成功标志。
    def get(self):
        # 等待RKNN初始化完成
        with self.rknn_init_condition:
            while not self.rknn_init_finish: # 如果RKNN未初始化完成
                self.rknn_init_condition.wait() # 等待条件变量
        # 循环直到队列中有处理结果
        while True: # 无限循环
            if self.queue.empty(): # 如果队列为空
                continue # 继续下一次循环
            data, flag = self.queue.get() # 获取数据和标志
            if flag: # 如果标志为True，表示是处理结果
                self.queue.task_done() # 标记任务完成
                return data, flag # 返回数据和标志
            else: # 如果标志为False，表示是原始图像帧，重新放入队列
                self.queue.task_done() # 标记任务完成
                self.queue.put((data, flag)) # 重新放入队列

    # 释放资源
    def release(self):
        self.flag = False # 设置运行标志为False，停止线程循环
        self.join() # 等待线程结束
        for rknn in self.rknn_list: # 遍历RKNN对象列表
            rknn.release() # 释放RKNN资源
