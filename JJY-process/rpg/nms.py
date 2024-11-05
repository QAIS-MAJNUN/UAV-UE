import numpy as np

# 定义车辆模型标签信息
class Car:
    def __init__(self, name, xmin, ymin, xmax, ymax):
        # 是否分割
        if 0 <= int(xmin) <= delta or 0 <= int(ymin) <= delta \
                or HEIGHT - delta <= int(xmax) <= HEIGHT or WIDTH - delta <= int(ymax) <= WIDTH:
            self.truncated = 1
        else:
            self.truncated = 0
        self.name = name
        # 检测框坐标
        self.xmin = str(xmin)
        self.ymin = str(ymin)
        self.xmax = str(xmax)
        self.ymax = str(ymax)

    def __repr__(self):
        return f"{self.name} {self.xmin} {self.ymin} {self.xmax} {self.ymax}"

# 定义宽度、高度和深度
WIDTH, HEIGHT, DEPTH = 1920, 1080, 3
# 偏移量
delta = 5

# IoU阈值
overlapThresh = 0.5

def bb_intersection_over_union(boxA, boxB):
    # 将字符串坐标转换为整数
    boxA = [int(coord) for coord in boxA]
    boxB = [int(coord) for coord in boxB]

    # 确定两个框的左上角和右下角坐标
    xA = max(boxA[0], boxB[0])
    yA = max(boxA[1], boxB[1])
    xB = min(boxA[2], boxB[2])
    yB = min(boxA[3], boxB[3])

    # 计算交集面积
    intersection_area = max(0, xB - xA + 1) * max(0, yB - yA + 1)

    # 计算两个框的面积
    boxA_area = (boxA[2] - boxA[0] + 1) * (boxA[3] - boxA[1] + 1)
    boxB_area = (boxB[2] - boxB[0] + 1) * (boxB[3] - boxB[1] + 1)

    # 计算并集面积
    iou = intersection_area / float(boxA_area + boxB_area - intersection_area)

    return iou

def non_max_suppression_helper(boxes, scores, overlapThresh):
    # 如果没有框，则返回空列表
    if len(boxes) == 0:
        return []

    # 初始化索引列表
    pick = []

    # 将字符串坐标转换为整数
    boxes = np.array([[int(coord) for coord in box] for box in boxes])

    # 如果边界框是整数，转换为浮点数
    if boxes.dtype.kind == "i":
        boxes = boxes.astype("float")

    # 根据得分排序
    idxs = scores.argsort()[::-1]

    while len(idxs) > 0:
        # 拾取当前得分最高的边界框的索引
        i = idxs[0]
        pick.append(i)

        # 找到剩余所有框的最大左上角和最小右下角
        xx1 = np.maximum(boxes[idxs, 0], boxes[i, 0])
        yy1 = np.maximum(boxes[idxs, 1], boxes[i, 1])
        xx2 = np.minimum(boxes[idxs, 2], boxes[i, 2])
        yy2 = np.minimum(boxes[idxs, 3], boxes[i, 3])

        # 计算宽度和高度
        w = np.maximum(0, xx2 - xx1 + 1)
        h = np.maximum(0, yy2 - yy1 + 1)

        # 计算IoU
        overlap = (w * h) / ((boxes[idxs, 2] - boxes[idxs, 0] + 1) *
                             (boxes[idxs, 3] - boxes[idxs, 1] + 1))

        # 删除重叠大于阈值的框
        idxs = idxs[np.where(overlap <= overlapThresh)]

    # 返回选定的边界框索引
    return boxes[pick].astype("int")

def non_max_suppression(cars, overlapThresh=0.5):
    # 分组
    grouped_cars = {}
    for car in cars:
        if car.name not in grouped_cars:
            grouped_cars[car.name] = []
        grouped_cars[car.name].append([car.xmin, car.ymin, car.xmax, car.ymax])

    # 处理每个类别的边界框
    processed_cars = []
    for label, boxes in grouped_cars.items():
        # 将字符串坐标转换为整数
        boxes = np.array([[int(coord) for coord in box] for box in boxes])
        scores = np.ones(len(boxes))  # 假设所有框的置信度分数相同
        selected_boxes = non_max_suppression_helper(boxes, scores, overlapThresh)
        for box in selected_boxes:
            processed_cars.append(Car(label, *box))

    return processed_cars
