import xml.etree.ElementTree as ET
import xml.dom.minidom

def write_xml(annotation, output_file):
    # 创建XML树
    annotation_el = ET.Element('annotation')

    # 添加基本节点
    ET.SubElement(annotation_el, 'folder').text = annotation['folder']
    ET.SubElement(annotation_el, 'filename').text = annotation['filename']
    ET.SubElement(annotation_el, 'path').text = annotation['path']
    source_el = ET.SubElement(annotation_el, 'source')
    ET.SubElement(source_el, 'database').text = annotation['database']
    size_el = ET.SubElement(annotation_el, 'size')
    ET.SubElement(size_el, 'width').text = str(annotation['size']['width'])
    ET.SubElement(size_el, 'height').text = str(annotation['size']['height'])
    ET.SubElement(size_el, 'depth').text = str(annotation['size']['depth'])
    ET.SubElement(annotation_el, 'segmented').text = str(annotation['segmented'])
    ET.SubElement(annotation_el, 'scene').text = annotation['scene']
    ET.SubElement(annotation_el, 'cameraHeight').text = str(annotation['camera_height'])
    ET.SubElement(annotation_el, 'cameraRotation').text = str(annotation['camera_rotation'])

    # 添加对象节点
    for obj in annotation['objects']:
        object_el = ET.SubElement(annotation_el, 'object')
        ET.SubElement(object_el, 'name').text = obj['name']
        ET.SubElement(object_el, 'pose').text = obj['pose']
        ET.SubElement(object_el, 'truncated').text = str(obj['truncated'])
        ET.SubElement(object_el, 'difficult').text = str(obj['difficult'])
        bndbox_el = ET.SubElement(object_el, 'bndbox')
        ET.SubElement(bndbox_el, 'xmin').text = str(obj['bndbox']['xmin'])
        ET.SubElement(bndbox_el, 'ymin').text = str(obj['bndbox']['ymin'])
        ET.SubElement(bndbox_el, 'xmax').text = str(obj['bndbox']['xmax'])
        ET.SubElement(bndbox_el, 'ymax').text = str(obj['bndbox']['ymax'])

    # 将ElementTree转换为DOM对象
    rough_string = ET.tostring(annotation_el, 'utf-8')
    reparsed = xml.dom.minidom.parseString(rough_string)
    formatted_xml = reparsed.toprettyxml(indent="\t")

    # 写入文件
    with open(output_file, 'w') as f:
        f.write(formatted_xml)