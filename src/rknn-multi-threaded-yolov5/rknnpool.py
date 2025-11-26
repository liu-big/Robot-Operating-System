# -*- coding: utf-8 -*-
#
# Copyright (c) 2024, UCAR.AI, All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ============================================================================== 

"""
文件: rknnpool.py
描述: 该文件定义了RKNN模型池执行器，用于管理RKNN模型的加载、初始化和多线程推理。
作者: UCAR.AI
创建日期: 2024-01-01
修订历史:
    2024-01-01: 初始版本
"""

from queue import Queue
from rknnlite.api import RKNNLite
from concurrent.futures import ThreadPoolExecutor


# initRKNN: 初始化单个RKNN模型。
# 参数:
#   rknnModel (str): RKNN模型的路径，默认为"./rknnModel/yolov5s.rknn"。
#   id (int): NPU核心ID，用于指定模型运行在哪个NPU核心上。默认为0。
#             - 0: NPU_CORE_0
#             - 1: NPU_CORE_1
#             - 2: NPU_CORE_2
#             - -1: NPU_CORE_0_1_2 (所有核心)
#             - 其他值: 默认核心
# 返回值:
#   RKNNLite: 初始化后的RKNNLite对象。
# 使用说明:
#   该函数加载指定的RKNN模型，并根据id参数在相应的NPU核心上初始化运行时环境。
#   如果加载或初始化失败，程序将退出。
def initRKNN(rknnModel="./rknnModel/yolov5s.rknn", id=0):
    rknn_lite = RKNNLite() # 创建RKNNLite实例
    ret = rknn_lite.load_rknn(rknnModel) # 加载RKNN模型
    if ret != 0:
        print("Load RKNN rknnModel failed")
        exit(ret)
    
    # 根据id选择NPU核心进行初始化
    if id == 0:
        ret = rknn_lite.init_runtime(core_mask=RKNNLite.NPU_CORE_0)
    elif id == 1:
        ret = rknn_lite.init_runtime(core_mask=RKNNLite.NPU_CORE_1)
    elif id == 2:
        ret = rknn_lite.init_runtime(core_mask=RKNNLite.NPU_CORE_2)
    elif id == -1:
        ret = rknn_lite.init_runtime(core_mask=RKNNLite.NPU_CORE_0_1_2)
    else:
        ret = rknn_lite.init_runtime()
    
    if ret != 0:
        print("Init runtime environment failed")
        exit(ret)
    print(rknnModel, "\t\tdone") # 打印加载完成信息
    return rknn_lite


# initRKNNs: 初始化多个RKNN模型实例。
# 参数:
#   rknnModel (str): RKNN模型的路径，默认为"./rknnModel/yolov5s.rknn"。
#   TPEs (int): 需要初始化的RKNN模型实例数量，默认为1。
# 返回值:
#   list: 包含多个RKNNLite对象的列表。
# 使用说明:
#   该函数根据TPEs的数量，循环调用initRKNN函数，创建并初始化多个RKNN模型实例，
#   并将它们存储在一个列表中返回。每个实例会轮流分配到不同的NPU核心（0, 1, 2）。
def initRKNNs(rknnModel="./rknnModel/yolov5s.rknn", TPEs=1):
    rknn_list = [] # 初始化RKNNLite对象列表
    for i in range(TPEs):
        rknn_list.append(initRKNN(rknnModel, i % 3)) # 轮流分配NPU核心
    return rknn_list


# rknnPoolExecutor: RKNN模型池执行器类，用于管理RKNN模型的并发推理。
class rknnPoolExecutor():
    # __init__: 构造函数，初始化RKNN模型池执行器。
    # 参数:
    #   rknnModel (str): RKNN模型的路径。
    #   TPEs (int): 线程池中的线程数量，对应RKNN模型实例的数量。
    #   func (function): 用于处理推理结果的函数。
    def __init__(self, rknnModel, TPEs, func):
        self.TPEs = TPEs # 线程池大小
        self.queue = Queue() # 任务队列，用于存储Future对象
        self.rknnPool = initRKNNs(rknnModel, TPEs) # 初始化RKNN模型实例池
        self.pool = ThreadPoolExecutor(max_workers=TPEs) # 创建线程池
        self.func = func # 推理结果处理函数
        self.num = 0 # 计数器，用于轮询分配RKNN实例

    # put: 将推理任务提交到线程池。
    # 参数:
    #   frame: 输入图像帧。
    # 使用说明:
    #   该方法将图像帧和对应的RKNN模型实例（通过轮询选择）提交给线程池进行异步推理。
    #   推理任务的结果（Future对象）会被放入任务队列。
    def put(self, frame):
        self.queue.put(self.pool.submit(
            self.func, self.rknnPool[self.num % self.TPEs], frame)) # 提交任务到线程池
        self.num += 1 # 计数器递增

    # get: 从任务队列中获取推理结果。
    # 返回值:
    #   tuple: (result, bool)
    #     result: 推理结果，如果队列为空则为None。
    #     bool: 表示是否成功获取结果（True表示成功，False表示队列为空）。
    # 使用说明:
    #   该方法从任务队列中获取一个已完成的推理任务的Future对象，并返回其结果。
    #   如果队列为空，则返回None和False。
    def get(self):
        if self.queue.empty():
            return None, False # 队列为空，返回None和False
        fut = self.queue.get() # 获取Future对象
        return fut.result(), True # 返回推理结果和True

    # release: 释放线程池和RKNN模型资源。
    # 使用说明:
    #   该方法关闭线程池，并遍历RKNN模型实例池，释放每个RKNNLite对象的资源。
    def release(self):
        self.pool.shutdown() # 关闭线程池
        for rknn_lite in self.rknnPool:
            rknn_lite.release() # 释放RKNNLite资源
