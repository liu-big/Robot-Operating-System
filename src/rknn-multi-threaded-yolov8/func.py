# -*- coding: utf-8 -*-
# @Author: guo
# @Date: 2024-06-20
# @Description: 该脚本包含了YOLOv8模型推理的后处理函数，
#               包括边界框过滤、非极大值抑制(NMS)、DFL(Distribution Focal Loss)处理、
#               图像预处理(letterbox)以及结果绘制等功能。
#               主要用于对RKNN推理引擎输出的结果进行解析和可视化。

# 以下代码改自https://github.com/rockchip-linux/rknn-toolkit2/tree/master/examples/onnx/yolov5
import cv2
import numpy as np

# 定义目标检测的阈值、非极大值抑制阈值和图像大小
# OBJ_THRESH: 目标置信度阈值，低于此值的检测框将被过滤。
# NMS_THRESH: 非极大值抑制阈值，用于去除重叠的检测框。
# IMG_SIZE: 模型输入图像的尺寸，这里是640x640。
OBJ_THRESH, NMS_THRESH, IMG_SIZE = 0.4, 0.25, 640

# 定义模型检测的类别名称
CLASSES = ("terrorist1","terrorist2","terrorist3","spontoon","bulltproo_vest","teargas","first_aid_kit")


def filter_boxes(boxes, box_confidences, box_class_probs):
    """
    根据对象置信度阈值过滤边界框。

    参数:
        boxes (np.ndarray): 边界框坐标数组。
        box_confidences (np.ndarray): 边界框的对象置信度数组。
        box_class_probs (np.ndarray): 边界框的类别概率数组。

    返回:
        tuple: 过滤后的边界框、类别和分数。
    """
    box_confidences = box_confidences.reshape(-1)
    candidate, class_num = box_class_probs.shape

    class_max_score = np.max(box_class_probs, axis=-1)
    classes = np.argmax(box_class_probs, axis=-1)

    _class_pos = np.where(class_max_score* box_confidences >= OBJ_THRESH)
    scores = (class_max_score* box_confidences)[_class_pos]

    boxes = boxes[_class_pos]
    classes = classes[_class_pos]

    return boxes, classes, scores

def nms_boxes(boxes, scores):
    """
    执行非极大值抑制(NMS)以抑制重叠的边界框。

    参数:
        boxes (np.ndarray): 边界框坐标数组。
        scores (np.ndarray): 边界框的置信度分数数组。

    返回:
        np.ndarray: 保留的边界框的索引。
    """
    x = boxes[:, 0]
    y = boxes[:, 1]
    w = boxes[:, 2] - boxes[:, 0]
    h = boxes[:, 3] - boxes[:, 1]

    areas = w * h
    order = scores.argsort()[::-1]

    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(i)

        xx1 = np.maximum(x[i], x[order[1:]])
        yy1 = np.maximum(y[i], y[order[1:]])
        xx2 = np.minimum(x[i] + w[i], x[order[1:]] + w[order[1:]])
        yy2 = np.minimum(y[i] + h[i], y[order[1:]] + h[order[1:]])

        w1 = np.maximum(0.0, xx2 - xx1 + 0.00001)
        h1 = np.maximum(0.0, yy2 - yy1 + 0.00001)
        inter = w1 * h1

        ovr = inter / (areas[i] + areas[order[1:]] - inter)
        inds = np.where(ovr <= NMS_THRESH)[0]
        order = order[inds + 1]
    keep = np.array(keep)
    return keep

# def dfl(position):
#     # Distribution Focal Loss (DFL)
#     import torch
#     x = torch.tensor(position)
#     n,c,h,w = x.shape
#     p_num = 4
#     mc = c//p_num
#     y = x.reshape(n,p_num,mc,h,w)
#     y = y.softmax(2)
#     acc_metrix = torch.tensor(range(mc)).float().reshape(1,1,mc,1,1)
#     y = (y*acc_metrix).sum(2)
#     return y.numpy()

# def dfl(position):
#     # Distribution Focal Loss (DFL)
#     n, c, h, w = position.shape
#     p_num = 4
#     mc = c // p_num
#     y = position.reshape(n, p_num, mc, h, w)
#     exp_y = np.exp(y)
#     y = exp_y / np.sum(exp_y, axis=2, keepdims=True)
#     acc_metrix = np.arange(mc).reshape(1, 1, mc, 1, 1).astype(float)
#     y = (y * acc_metrix).sum(2)
#     return y

def dfl(position):
    """
    实现Distribution Focal Loss (DFL)的计算。
    用于YOLOv8中预测边界框的分布表示。

    参数:
        position (np.ndarray): 模型的原始输出，包含边界框的分布信息。

    返回:
        np.ndarray: 经过DFL处理后的边界框坐标。
    """
    # x = np.array(position)
    n,c,h,w = position.shape
    p_num = 4
    mc = c//p_num
    y = position.reshape(n,p_num,mc,h,w)
    
    # Vectorized softmax
    # 对每个分布进行softmax操作，确保概率和为1。
    e_y = np.exp(y - np.max(y, axis=2, keepdims=True))  # subtract max for numerical stability
    y = e_y / np.sum(e_y, axis=2, keepdims=True)
    
    # 计算加权和，得到最终的坐标值。
    acc_metrix = np.arange(mc).reshape(1,1,mc,1,1)
    y = (y*acc_metrix).sum(2)
    return y
    

def box_process(position):
    """
    处理模型输出的边界框位置信息，将其转换为实际的xyxy坐标。

    参数:
        position (np.ndarray): 经过DFL处理后的边界框原始位置信息。

    返回:
        np.ndarray: 转换后的边界框xyxy坐标。
    """
    # 获取特征图的尺寸
    grid_h, grid_w = position.shape[2:4]
    # 生成网格坐标
    col, row = np.meshgrid(np.arange(0, grid_w), np.arange(0, grid_h))
    col = col.reshape(1, 1, grid_h, grid_w)
    row = row.reshape(1, 1, grid_h, grid_w)
    grid = np.concatenate((col, row), axis=1)
    # 计算步长
    stride = np.array([IMG_SIZE//grid_h, IMG_SIZE//grid_w]).reshape(1,2,1,1)

    # 对位置信息进行DFL处理
    position = dfl(position)
    # 计算边界框的左上角和右下角坐标
    box_xy  = grid +0.5 -position[:,0:2,:,:]
    box_xy2 = grid +0.5 +position[:,2:4,:,:]
    # 将坐标拼接成xyxy格式
    xyxy = np.concatenate((box_xy*stride, box_xy2*stride), axis=1)

    return xyxy

def yolov8_post_process(input_data):
    """
    YOLOv8模型的后处理函数，包括边界框解析、过滤和NMS。

    参数:
        input_data (list): RKNN模型推理的原始输出数据。

    返回:
        tuple: 最终的边界框、类别和分数。如果没有检测到目标，则返回(None, None, None)。
    """
    boxes, scores, classes_conf = [], [], []
    defualt_branch=3
    pair_per_branch = len(input_data)//defualt_branch
    # Python 忽略 score_sum 输出
    # 遍历每个分支的输出，进行边界框处理和置信度提取
    for i in range(defualt_branch):
        boxes.append(box_process(input_data[pair_per_branch*i]))
        classes_conf.append(input_data[pair_per_branch*i+1])
        scores.append(np.ones_like(input_data[pair_per_branch*i+1][:,:1,:,:], dtype=np.float32))

    def sp_flatten(_in):
        """
        将输入数组展平。
        """
        ch = _in.shape[1]
        _in = _in.transpose(0,2,3,1)
        return _in.reshape(-1, ch)

    # 展平所有边界框、类别置信度和分数
    boxes = [sp_flatten(_v) for _v in boxes]
    classes_conf = [sp_flatten(_v) for _v in classes_conf]
    scores = [sp_flatten(_v) for _v in scores]

    boxes = np.concatenate(boxes)
    classes_conf = np.concatenate(classes_conf)
    scores = np.concatenate(scores)

    # 根据阈值过滤边界框
    boxes, classes, scores = filter_boxes(boxes, scores, classes_conf)

    # 执行NMS
    nboxes, nclasses, nscores = [], [], []
    for c in set(classes):
        inds = np.where(classes == c)
        b = boxes[inds]
        c = classes[inds]
        s = scores[inds]
        keep = nms_boxes(b, s)

        if len(keep) != 0:
            nboxes.append(b[keep])
            nclasses.append(c[keep])
            nscores.append(s[keep])

    # 如果没有检测到任何目标，返回None
    if not nclasses and not nscores:
        return None, None, None

    # 拼接所有保留的边界框、类别和分数
    boxes = np.concatenate(nboxes)
    classes = np.concatenate(nclasses)
    scores = np.concatenate(nscores)

    return boxes, classes, scores

def draw(image, boxes, scores, classes):
    """
    在图像上绘制检测到的边界框和类别标签。

    参数:
        image (np.ndarray): 原始图像。
        boxes (np.ndarray): 边界框坐标数组。
        scores (np.ndarray): 边界框的置信度分数数组。
        classes (np.ndarray): 边界框的类别数组。

    返回:
        list: 检测到的对象的中心点坐标列表。
    """
    center_list = []
    for box, score, cl in zip(boxes, scores, classes):
        # 提取边界框坐标
        top, left, right, bottom = box
        top = top
        left = left
        right = right
        bottom = bottom
        # 计算中心点坐标
        center_x = (top + right) / 2.0
        center_y = (left + bottom) / 2.0
        center_list.append([center_x, center_y])

        # 绘制矩形框
        cv2.rectangle(image, (int(top), int(left)), (int(right), int(bottom)), (255, 0, 0), 2)
        # 绘制类别和分数文本
        cv2.putText(image, '{0} {1:.2f}'.format(CLASSES[cl], score),
                    (int(top), int(left) - 6),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 0, 255), 2)
    return center_list

def letterbox(im, new_shape=(640, 640), color=(0, 0, 0)):
    """
    对图像进行等比例缩放和填充，使其适应新的形状，同时保持纵横比。

    参数:
        im (np.ndarray): 输入图像。
        new_shape (tuple): 目标形状 (height, width)。
        color (tuple): 填充颜色 (B, G, R)。

    返回:
        np.ndarray: 经过letterbox处理后的图像。
    """
    shape = im.shape[:2]  # current shape [height, width]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)

    # 计算缩放比例
    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])

    ratio = r, r  # width, height ratios
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - \
        new_unpad[1]  # wh padding

    dw /= 2  # divide padding into 2 sides
    dh /= 2

    if shape[::-1] != new_unpad:  # resize
        im = cv2.resize(im, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    im = cv2.copyMakeBorder(im, top, bottom, left, right,
                            cv2.BORDER_CONSTANT, value=color)  # add border
    return im
    # return im, ratio, (dw, dh)

def myFunc(rknn_lite, IMG):
    """
    对输入图像进行预处理、RKNN模型推理、后处理和结果可视化。

    参数:
        rknn_lite: RKNN推理引擎实例。
        IMG (np.ndarray): 输入的原始图像。

    返回:
        tuple: 绘制了检测结果的图像、类别数组和中心点坐标数组。
    """
    # 将BGR图像转换为RGB图像
    IMG = cv2.cvtColor(IMG, cv2.COLOR_BGR2RGB)
    # 等比例缩放图像
    IMG = letterbox(IMG)
    # 强制放缩 (如果需要，可以取消注释使用)
    # IMG = cv2.resize(IMG, (IMG_SIZE, IMG_SIZE))
    # 扩展维度以适应模型输入 (Batch, Height, Width, Channel)
    IMG2 = np.expand_dims(IMG, 0)
    
    # 执行RKNN模型推理
    outputs = rknn_lite.inference(inputs=[IMG2],data_format=['nhwc'])

    #print("oups1",len(outputs))
    #print("oups2",outputs[0].shape)

    # 对推理结果进行后处理，获取边界框、类别和分数
    boxes, classes, scores = yolov8_post_process(outputs)

    # 将RGB图像转换回BGR图像以便OpenCV显示
    IMG = cv2.cvtColor(IMG, cv2.COLOR_RGB2BGR)

    center_x_y = [] # 初始化为空列表，确保在没有检测到目标时也能正确处理
    if boxes is not None:
        # 在图像上绘制检测结果
        center_x_y = draw(IMG, boxes, scores, classes)

    # 初始化类别数组和中心坐标数组
    class_array = [0.0] * 7  # 初始化类别数组为默认值 0.0
    center_array = [[0.0, 0.0]] * 3  # 初始化中心坐标数组为默认值 [0.0, 0.0]

    # 填充类别数组
    for class_idx in range(7):
        if classes is not None and np.isin(class_idx, classes):
            class_array[class_idx] = float(class_idx) + 1.0
        
    # 提取特定类别的中心点坐标
    if classes is not None and np.isin(5, classes):
        teargas = np.where(classes == 5)[0]
        if len(teargas) == 1:
            teargas_center = center_x_y[teargas.item()]
            center_array[2] = teargas_center
    
    if classes is not None and np.isin(3, classes):
        spontoon = np.where(classes == 3)[0]
        if len(spontoon) == 1:
            spontoon_center = center_x_y[spontoon.item()]
            center_array[0] = spontoon_center
    
    if classes is not None and np.isin(4, classes): 
        bulltproo_vest = np.where(classes == 4)[0]
        if len(bulltproo_vest) == 1:
            bulltproo_vest_center = center_x_y[bulltproo_vest.item()]
            center_array[1] = bulltproo_vest_center

    return IMG, class_array, center_array
