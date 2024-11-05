import xml.etree.ElementTree as ET
import numpy as np

def parse_xml(file_path):
    # 解析XML文件
    tree = ET.parse(file_path)
    root = tree.getroot()

    # 初始化数据结构
    annotation = {
        'folder': root.find('folder').text,
        'filename': root.find('filename').text,
        'path': root.find('path').text,
        'database': root.find('source/database').text,
        'size': {
            'width': int(root.find('size/width').text),
            'height': int(root.find('size/height').text),
            'depth': int(root.find('size/depth').text)
        },
        'segmented': int(root.find('segmented').text),
        'scene': root.find('scene').text,
        'camera_height': float(root.find('cameraHeight').text),
        'camera_rotation': float(root.find('cameraRotation').text),
        'objects': []
    }

    # 提取对象信息
    for obj in root.findall('object'):
        name = obj.find('name').text
        pose = obj.find('pose').text
        truncated = int(obj.find('truncated').text)
        difficult = int(obj.find('difficult').text)
        bndbox = {
            'xmin': int(obj.find('bndbox/xmin').text),
            'ymin': int(obj.find('bndbox/ymin').text),
            'xmax': int(obj.find('bndbox/xmax').text),
            'ymax': int(obj.find('bndbox/ymax').text)
        }
        annotation['objects'].append({
            'name': name,
            'pose': pose,
            'truncated': truncated,
            'difficult': difficult,
            'bndbox': bndbox
        })

    # 输出基本文件信息
    print("Basic Annotation Information:")
    print(annotation)

    # 提取边界框信息
    boxes = []
    scores = []

    for obj in annotation['objects']:
        boxes.append([obj['bndbox']['xmin'], obj['bndbox']['ymin'], obj['bndbox']['xmax'], obj['bndbox']['ymax']])
        scores.append(0.9)  # 假设所有框的置信度分数相同

    boxes = np.array(boxes)
    scores = np.array(scores)

    #print(boxes)
    #print(scores)

    return (boxes, scores, annotation)

