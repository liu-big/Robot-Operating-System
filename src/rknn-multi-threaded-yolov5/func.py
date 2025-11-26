# -*- coding: utf-8 -*-
# 以下代码改自https://github.com/rockchip-linux/rknn-toolkit2/tree/master/examples/onnx/yolov5
# 该文件包含YOLOv5模型推理后的后处理函数，包括sigmoid激活函数、坐标转换、
# 边界框处理、非极大值抑制（NMS）以及在图像上绘制检测结果等。
# 主要用于对RKNN模型输出的原始数据进行解析和可视化。
import cv2
import numpy as np


# 定义目标检测的置信度阈值、非极大值抑制阈值和图像大小
# OBJ_THRESH: 目标置信度阈值，低于此值的目标将被过滤
# NMS_THRESH: 非极大值抑制阈值，用于过滤重叠的边界框
# IMG_SIZE: 模型输入图像的尺寸，这里是640x640
OBJ_THRESH, NMS_THRESH, IMG_SIZE = 0.25, 0.45, 640

# 定义YOLOv5模型识别的类别名称
# 这里的类别包括：恐怖分子1、恐怖分子2、恐怖分子3、警棍、防弹衣、催泪瓦斯、急救包
CLASSES = ("terrorist1","terrorist2","terrorist3","spontoon","bulltproo_vest","teargas","first_aid_kit")
#CLASSES = ("tank")
# 

def sigmoid(x):
    """
    Sigmoid激活函数。
    将输入值x映射到(0, 1)之间，常用于计算概率。

    参数:
        x (np.array): 输入的Numpy数组。

    返回值:
        np.array: 经过Sigmoid函数处理后的数组。
    """
    return 1 / (1 + np.exp(-x))


def xywh2xyxy(x):
    """
    将边界框的中心点坐标和宽高(xywh)格式转换为左上角和右下角坐标(xyxy)格式。

    参数:
        x (np.array): 边界框数组，形状为(N, 4)，每行包含[x_center, y_center, width, height]。

    返回值:
        np.array: 转换后的边界框数组，形状为(N, 4)，每行包含[x1, y1, x2, y2]。
    """
    # Convert [x, y, w, h] to [x1, y1, x2, y2]
    y = np.copy(x)
    y[:, 0] = x[:, 0] - x[:, 2] / 2  # 左上角x坐标
    y[:, 1] = x[:, 1] - x[:, 3] / 2  # 左上角y坐标
    y[:, 2] = x[:, 0] + x[:, 2] / 2  # 右下角x坐标
    y[:, 3] = x[:, 1] + x[:, 3] / 2  # 右下角y坐标
    return y


def process(input, mask, anchors):
    """
    处理YOLOv5模型的原始输出，将其转换为边界框、置信度和类别概率。
    此函数负责解析模型在特定特征图上的输出，并结合预设的锚点框计算出初步的检测结果。

    参数:
        input (np.array): 模型输出的原始数据，包含边界框偏移、置信度和类别分数。
        mask (list): 锚点框的索引掩码，用于选择当前特征图对应的锚点。
        anchors (list): 预设的锚点框尺寸列表。

    返回值:
        tuple: 包含以下元素的元组：
            - box (np.array): 转换后的边界框坐标。
            - box_confidence (np.array): 边界框的置信度。
            - box_class_probs (np.array): 边界框的类别概率。
    """
    anchors = [anchors[i] for i in mask]
    grid_h, grid_w = map(int, input.shape[0:2])

    # 计算边界框的置信度
    box_confidence = sigmoid(input[..., 4])
    box_confidence = np.expand_dims(box_confidence, axis=-1)

    # 计算边界框的类别概率
    box_class_probs = sigmoid(input[..., 5:])

    # 计算边界框的中心点坐标(xy)
    box_xy = sigmoid(input[..., :2]) * 2 - 0.5

    # 生成网格坐标
    col = np.tile(np.arange(0, grid_w), grid_w).reshape(-1, grid_w)
    row = np.tile(np.arange(0, grid_h).reshape(-1, 1), grid_h)
    col = col.reshape(grid_h, grid_w, 1, 1).repeat(3, axis=-2)
    row = row.reshape(grid_h, grid_w, 1, 1).repeat(3, axis=-2)
    grid = np.concatenate((col, row), axis=-1)
    box_xy += grid
    box_xy *= int(IMG_SIZE / grid_h)

    # 计算边界框的宽度和高度(wh)
    box_wh = pow(sigmoid(input[..., 2:4]) * 2, 2)
    box_wh = box_wh * anchors

    # 合并xy和wh得到完整的边界框信息
    box = np.concatenate((box_xy, box_wh), axis=-1)

    return box, box_confidence, box_class_probs



def filter_boxes(boxes, box_confidences, box_class_probs):
    """
    根据置信度阈值过滤边界框。
    此函数会筛选掉置信度低于预设阈值的边界框，并返回有效的边界框、对应的类别和分数。

    参数:
        boxes (np.array): 原始边界框数组。
        box_confidences (np.array): 边界框的置信度数组。
        box_class_probs (np.array): 边界框的类别概率数组。

    返回值:
        tuple: 包含以下元素的元组：
            - boxes (np.array): 过滤后的边界框。
            - classes (np.array): 过滤后的边界框对应的类别。
            - scores (np.array): 过滤后的边界框的分数（置信度*类别最大概率）。
    """
    boxes = boxes.reshape(-1, 4)
    box_confidences = box_confidences.reshape(-1)
    box_class_probs = box_class_probs.reshape(-1, box_class_probs.shape[-1])

    # 根据目标置信度阈值过滤边界框
    _box_pos = np.where(box_confidences >= OBJ_THRESH)
    boxes = boxes[_box_pos]
    box_confidences = box_confidences[_box_pos]
    box_class_probs = box_class_probs[_box_pos]

    # 获取每个边界框的最大类别分数及其对应的类别索引
    class_max_score = np.max(box_class_probs, axis=-1)
    classes = np.argmax(box_class_probs, axis=-1)
    # 再次根据类别最大分数过滤边界框
    _class_pos = np.where(class_max_score >= OBJ_THRESH)

    return boxes[_class_pos], classes[_class_pos], (class_max_score * box_confidences)[_class_pos]


def nms_boxes(boxes, scores):
    """
    执行非极大值抑制（NMS），用于消除重叠的边界框。
    根据边界框的得分，保留最佳的边界框并抑制与其高度重叠的其他边界框。

    参数:
        boxes (np.array): 边界框数组，格式为[x1, y1, x2, y2]。
        scores (np.array): 边界框的得分数组。

    返回值:
        np.array: 经过NMS处理后保留的边界框的索引。
    """
    x = boxes[:, 0]
    y = boxes[:, 1]
    w = boxes[:, 2] - boxes[:, 0]
    h = boxes[:, 3] - boxes[:, 1]

    areas = w * h
    order = scores.argsort()[::-1] # 按照得分降序排列

    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(i)

        # 计算当前边界框与其余边界框的交集坐标
        xx1 = np.maximum(x[i], x[order[1:]])
        yy1 = np.maximum(y[i], y[order[1:]])
        xx2 = np.minimum(x[i] + w[i], x[order[1:]] + w[order[1:]])
        yy2 = np.minimum(y[i] + h[i], y[order[1:]] + h[order[1:]])

        # 计算交集区域的宽度和高度
        w1 = np.maximum(0.0, xx2 - xx1 + 0.00001)
        h1 = np.maximum(0.0, yy2 - yy1 + 0.00001)
        inter = w1 * h1 # 交集面积

        # 计算交并比(IoU)
        ovr = inter / (areas[i] + areas[order[1:]] - inter)
        # 找出IoU低于NMS阈值的边界框索引
        inds = np.where(ovr <= NMS_THRESH)[0]
        order = order[inds + 1] # 更新order列表，移除被抑制的边界框
    return np.array(keep)


def yolov5_post_process(input_data):
    """
    YOLOv5模型的后处理函数。
    该函数整合了边界框处理、置信度过滤和非极大值抑制等步骤，
    将模型的原始输出转换为最终的检测结果（边界框、类别和分数）。

    参数:
        input_data (list): 包含模型三个输出层数据的列表。

    返回值:
        tuple: 包含以下元素的元组：
            - boxes (np.array): 最终检测到的边界框。
            - classes (np.array): 最终检测到的边界框对应的类别。
            - scores (np.array): 最终检测到的边界框的分数。
            如果未检测到任何目标，则返回(None, None, None)。
    """
    # YOLOv5的锚点掩码和锚点尺寸，用于不同尺度的特征图
    masks = [[0, 1, 2], [3, 4, 5], [6, 7, 8]]
    anchors = [[43, 43], [49, 84], [77, 64], [87, 103], [134, 119],
               [97, 178], [161, 194], [211, 240], [378, 311]]

    boxes, classes, scores = [], [], []
    # 遍历每个输出层进行处理
    for input, mask in zip(input_data, masks):
        b, c, s = process(input, mask, anchors)
        b, c, s = filter_boxes(b, c, s)
        boxes.append(b)
        classes.append(c)
        scores.append(s)

    # 合并所有层的检测结果
    boxes = np.concatenate(boxes)
    boxes = xywh2xyxy(boxes) # 转换为xyxy格式
    classes = np.concatenate(classes)
    scores = np.concatenate(scores)

    nboxes, nclasses, nscores = [], [], []
    # 对每个类别独立进行NMS
    for c in set(classes):
        inds = np.where(classes == c)
        b = boxes[inds]
        c = classes[inds]
        s = scores[inds]

        keep = nms_boxes(b, s)

        nboxes.append(b[keep])
        nclasses.append(c[keep])
        nscores.append(s[keep])

    # 如果没有检测到任何目标，返回None
    if not nclasses and not nscores:
        return None, None, None

    # 合并NMS后的结果并返回
    return np.concatenate(nboxes), np.concatenate(nclasses), np.concatenate(nscores)


def draw(image, boxes, scores, classes):
    """
    在图像上绘制检测到的边界框、类别和置信度，并返回每个检测框的中心坐标。

    参数:
        image (np.array): 原始图像，OpenCV格式。
        boxes (np.array): 边界框数组，格式为[x1, y1, x2, y2]。
        scores (np.array): 边界框的置信度得分。
        classes (np.array): 边界框的类别索引。

    返回值:
        list: 包含每个检测框中心坐标的列表，每个中心坐标为[center_x, center_y]。
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
        
        # 在图像上绘制矩形框
        cv2.rectangle(image, (int(top), int(left)), (int(right), int(bottom)), (255, 0, 0), 2)
        # 在图像上绘制类别和置信度文本
        cv2.putText(image, '{0} {1:.2f}'.format(CLASSES[cl], score),
                    (int(top), int(left) - 6),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 0, 255), 2)
    print(center_list)
    return center_list

def letterbox(im, new_shape=(640, 640), color=(0, 0, 0)):
    """
    将图像进行等比例缩放并填充，使其尺寸符合模型输入要求。
    保持图像的宽高比，不足部分用指定颜色填充。

    参数:
        im (np.array): 输入图像，OpenCV格式。
        new_shape (tuple): 目标图像尺寸，默认为(640, 640)。
        color (tuple): 填充颜色，默认为黑色(0, 0, 0)。

    返回值:
        np.array: 经过letterbox处理后的图像。
    """
    shape = im.shape[:2]  # 当前图像的形状 [height, width]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)

    # 计算缩放比例
    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])

    ratio = r, r  # 宽度和高度的缩放比例
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    # 计算需要填充的宽度和高度
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - \
        new_unpad[1]  # wh padding

    dw /= 2  # 将填充平均分配到两边
    dh /= 2

    if shape[::-1] != new_unpad:  # 如果尺寸不匹配，则进行resize
        im = cv2.resize(im, new_unpad, interpolation=cv2.INTER_LINEAR)
    # 计算上下左右的填充量
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    # 添加边界填充
    im = cv2.copyMakeBorder(im, top, bottom, left, right,
                            cv2.BORDER_CONSTANT, value=color)  # add border
    return im
    # return im, ratio, (dw, dh)

def myFunc(rknn_lite, IMG):
    """
    主处理函数，用于执行RKNN模型的推理，并对结果进行后处理和可视化。
    该函数将输入图像进行预处理，送入RKNN模型进行推理，然后对推理结果进行解析、
    过滤、NMS，并在图像上绘制检测结果，最后返回处理后的图像、类别数组和中心坐标数组。

    参数:
        rknn_lite: RKNN模型推理器实例。
        IMG (np.array): 输入图像，OpenCV格式。

    返回值:
        tuple: 包含以下元素的元组：
            - IMG (np.array): 绘制了检测结果的图像。
            - class_array (list): 包含检测到的类别索引的数组，未检测到的类别为8.0。
            - center_array (list): 包含特定类别目标中心坐标的数组，未检测到的为[8.0, 8.0]。
    """
    # 将图像从BGR格式转换为RGB格式
    IMG = cv2.cvtColor(IMG, cv2.COLOR_BGR2RGB)
    # 等比例缩放 (目前被注释掉，使用强制放缩)
    # IMG = letterbox(IMG)
    # 强制放缩图像到模型输入尺寸
    IMG = cv2.resize(IMG, (IMG_SIZE, IMG_SIZE))
    # 执行RKNN模型推理
    outputs = rknn_lite.inference(inputs=[IMG])

    # 对模型输出进行reshape和转置，以符合后处理函数的输入要求
    input0_data = outputs[0].reshape([3, -1]+list(outputs[0].shape[-2:]))
    input1_data = outputs[1].reshape([3, -1]+list(outputs[1].shape[-2:]))
    input2_data = outputs[2].reshape([3, -1]+list(outputs[2].shape[-2:]))

    input_data = list()
    input_data.append(np.transpose(input0_data, (2, 3, 0, 1)))
    input_data.append(np.transpose(input1_data, (2, 3, 0, 1)))
    input_data.append(np.transpose(input2_data, (2, 3, 0, 1)))

    # 执行YOLOv5后处理，获取边界框、类别和分数
    boxes, classes, scores = yolov5_post_process(input_data)

    # 将图像从RGB格式转换回BGR格式
    IMG = cv2.cvtColor(IMG, cv2.COLOR_RGB2BGR)
    # 再次resize图像到640x640，用于显示
    IMG = cv2.resize(IMG, (640, 640))
    
    # 初始化类别数组和中心坐标数组，默认值为8.0或[8.0, 8.0]表示未识别到
    class_array = [8.0] * 7  # 初始化类别数组为默认值 8.0
    center_array = [[8.0, 8.0]] * 3  # 初始化中心坐标数组为默认值 [8.0, 8.0]

    # 如果检测到目标且置信度高于0.75，则绘制边界框并获取中心坐标
    if boxes is not None:
        # 检查scores是否为空，或者是否所有分数都大于等于0.75
        # scores.all()在scores为空时会返回True，需要额外判断
        if scores.size > 0 and np.all(scores >= 0.75):
            # 绘制检测结果并获取中心坐标列表
            center_x_y = draw(IMG, boxes, scores, classes)
        else:
            # 如果没有满足条件的分数，则将center_x_y设置为空列表，避免后续索引错误
            center_x_y = []
    else:
        # 如果boxes为None，则将center_x_y设置为空列表
        center_x_y = []

    # 填充类别数组
    for class_idx in range(7):
        if np.isin(class_idx, classes):
            class_array[class_idx] = float(class_idx)
        
    # 根据特定类别填充中心坐标数组
    # 类别3：teargas (催泪瓦斯)
    if np.isin(3, classes):
        teargas_indices = np.where(classes == 3)[0]
        if len(teargas_indices) == 1:
            teargas_center = center_x_y[teargas_indices.item()]
            center_array[0] = teargas_center
    
    # 类别4：spontoon (警棍)
    if np.isin(4, classes):
        spontoon_indices = np.where(classes == 4)[0]
        if len(spontoon_indices) == 1:
            spontoon_center = center_x_y[spontoon_indices.item()]
            center_array[1] = spontoon_center
    
    # 类别6：bulltproo_vest (防弹衣)
    if np.isin(6, classes):
        bulltproo_vest_indices = np.where(classes == 6)[0]
        if len(bulltproo_vest_indices) == 1:
            bulltproo_vest_center = center_x_y[bulltproo_vest_indices.item()]
            center_array[2] = bulltproo_vest_center

    return IMG, class_array, center_array


