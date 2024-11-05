import glob
import os
from PIL import Image
import cv2
import numpy as np
from tqdm import tqdm
import re
import time

# 实景图、分割图及标签的存储位置
scenepath = "C://Users//HP//Desktop//Airsim示例照片//SceneImage//"
segmentationpath = "C://Users//HP//Desktop//Airsim示例照片//SegmentationImage//"
annotationpath = ".//Annotation//"
# 图片的尺寸
WIDTH, HEIGHT, DEPTH = 1920, 1080, 3
# 偏移量
delta = 5
# 图片存储主路径
PREPATHNAME = r"C:\Users\HP\Desktop\Airsim示例照片"

# 车辆及对应的颜色
models = {
    #"Car": [178, 221, 213],
    #"Truck": [160, 113, 101],
    #"Pickup": [177, 106, 230],
    #"Motor": [130, 56, 55],
    #"Van": [202, 97, 155],
    "BoxTruck": [85, 152, 34],
    "Bulldozer": [51, 155, 241],
    "Excavator": [114, 161, 30],
    "ForkLift": [46, 104, 76],
    "Jeep": [3, 177, 32],
    "Motor": [138, 223, 226],
    "Pickup": [154, 159, 251],
    "RoadRoller": [235, 208, 124],
    "Sedan": [70, 209, 228],
    "SUV": [230, 136, 198],
    "Trailer": [129, 235, 107],
    "Truck": [10, 160, 82],
    "Van": [95, 224, 39]
}

# 车辆模型标签信息
class Car:
    def __init__(self, name, xmin, ymin, xmax, ymax):
        # 是否分割
        if 0 <= xmin <= delta or 0 <= ymin <= delta \
                or HEIGHT - delta <= xmax <= HEIGHT or WIDTH - delta <= ymax <= WIDTH:
            self.truncated = 1
        else:
            self.truncated = 0
        self.name = name
        # 检测框坐标
        self.xmin = str(xmin)
        self.ymin = str(ymin)
        self.xmax = str(xmax)
        self.ymax = str(ymax)


# 图片信息
class Picture:
    def __init__(self, path, cars=[], pictureinfo=[]):
        # 图片路径
        folder, filename = os.path.split(path)
        self.path = PREPATHNAME + path
        self.folder = folder
        self.filename = filename
        # 图片其他信息
        self.database = "Unknown"
        self.width = WIDTH
        self.height = HEIGHT
        self.depth = DEPTH
        # 图片中的车辆模型信息
        self.cars = cars
        self.cameraheight = pictureinfo[0]
        self.camerarotation = pictureinfo[1]
        self.scene = pictureinfo[2]


files = os.listdir(segmentationpath)
picture_nums = len(files)
print("当前数据集中共有" + str(picture_nums) + "张相片")

empty_data = []

# for pictureid in range(1, picture_nums + 1):
def imageprocess(pictureid):
    # 加载图片
    pattern = segmentationpath + "Scene" + str(pictureid) + f"_*_*_*" + ".png"  #1111111111 
    print(pattern)
    # image_path = segmentationpath + "Segmentation" + str(pictureid) + ".png"
    matching_files = glob.glob(pattern)
    #print(matching_files)
    image_path = matching_files[0]
    imgpth = "Scene" + image_path[30:-4]
    image = Image.open(image_path)
    image_np = np.array(image)

    # print(image_path)
    pattn = r"Scene\d+_(\d+)_(\d+)_(\w+)\.png"   #1111111111 Segmentation
    match = re.search(pattn, image_path)
    num1, num2, scene = match.groups()
    pictureinfo = [num1, num2, scene]
    # if match:
    #     num1, num2, scene = match.groups()
    #     pictureinfo = [num1, num2, scene]
    #     print(f"Extracted numbers and landscape: {num1}, {num2}, {scene}")
    # else:
    #     print("No match found")

    # 处理图片中的车辆模型信息
    car_list = []
    for key, value in models.items():
        target_color = np.array(value)
        matches = np.all(image_np[:, :, :3] == target_color, axis=-1)
        contours, _ = cv2.findContours(matches.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        bounding_boxes = [cv2.boundingRect(contour) for contour in contours]
        bounding_boxescopy = []
        for bounding_box in bounding_boxes:
            tuplecopy = list(bounding_box)
            # 过于边缘的模型删掉
            if tuplecopy[2] <= delta or tuplecopy[3] <= delta:
                continue
            tuplecopy[2] = tuplecopy[0] + tuplecopy[2]
            tuplecopy[3] = tuplecopy[1] + tuplecopy[3]
            car_list.append(Car(key, tuplecopy[0], tuplecopy[1], tuplecopy[2], tuplecopy[3]))
            bounding_boxescopy.append(tuplecopy)
        # print(key, bounding_boxescopy)

    if car_list==[]:
        empty_data.append(pictureid)

    for p in car_list:
        print(p.name, p.xmin, p.ymin, p.xmax, p.ymax)


    # 输出到标签中
    # print(pictureid)
    # print(car_list)
    pathname = scenepath + imgpth + ".png"
    picture = Picture(pathname, car_list, pictureinfo)
    parts = picture.filename.split(".")
    xmlname = parts[0]
    xmlname = xmlname + ".xml"
    xmlpath = annotationpath + xmlname
    # print(xmlname)
    with open(xmlpath, 'w') as f:
        f.write("<annotation>\n")
        f.write("\t<folder>" + picture.folder + "</folder>\n")
        f.write("\t<filename>" + picture.filename + "</filename>\n")
        f.write("\t<path>" + picture.path + "</path>\n")
        f.write("\t<source>\n")
        f.write("\t\t<database>" + picture.database + "</database>\n")
        f.write("\t</source>\n")
        f.write("\t<size>\n")
        f.write("\t\t<width>" + str(picture.width) + "</width>\n")
        f.write("\t\t<height>" + str(picture.height) + "</height>\n")
        f.write("\t\t<depth>" + str(picture.depth) + "</depth>\n")
        f.write("\t</size>\n")
        f.write("\t<segmented>0</segmented>\n")
        f.write("\t<scene>" + picture.scene + "</scene>\n")
        f.write("\t<cameraHeight>" + picture.cameraheight + "</cameraHeight>\n")
        f.write("\t<cameraRotation>" + picture.camerarotation + "</cameraRotation>\n")
        for obj in picture.cars:
            f.write("\t<object>\n")
            f.write("\t\t<name>" + obj.name + "</name>\n")
            f.write("\t\t<pose>Unspecified</pose>\n")
            f.write("\t\t<truncated>" + str(obj.truncated) + "</truncated>\n")
            f.write("\t\t<difficult>0</difficult>\n")
            f.write("\t\t<bndbox>\n")
            f.write("\t\t\t<xmin>" + obj.xmin + "</xmin>\n")
            f.write("\t\t\t<ymin>" + obj.ymin + "</ymin>\n")
            f.write("\t\t\t<xmax>" + obj.xmax + "</xmax>\n")
            f.write("\t\t\t<ymax>" + obj.ymax + "</ymax>\n")
            f.write("\t\t</bndbox>\n")
            f.write("\t</object>\n")
        f.write("</annotation>\n")

start_id = 101650 

num_list = list(range(picture_nums))
print(num_list)


for num in tqdm(num_list, desc='Processing data', unit='data'):
    imageprocess(num + start_id)

print("数据集处理完成!")

print("有效数据量为" + str(picture_nums - len(empty_data)) + "张")

print("如下数据为空")
print(empty_data)
with open("empty.txt", 'w', encoding="utf-8") as file:
    for i in empty_data:
        file.write(str(i) + '\n')