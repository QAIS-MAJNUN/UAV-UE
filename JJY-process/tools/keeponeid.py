import os
import shutil

def extract_info_from_filename(filename):
    """
    从文件名中提取ID、高度和角度，假设文件名格式为Scene + id + 高度 + 角度 + 地图名 + .png
    例如: Scene2512_15_0_VRGame.png，提取ID=2512, 高度=15, 角度=0
    """
    if filename.startswith('Scene') and filename.endswith('.png'):
        # 例如: Scene2512_15_0_VRGame.png -> 提取 '2512', '15', '0'
        parts = filename[5:-4].split('_')
        if len(parts) >= 3:
            file_id = parts[0]  # ID 是第一个部分
            height = int(parts[1])  # 高度是第二个部分
            angle = int(parts[2])  # 角度是第三个部分
            return file_id, height, angle
    return None, None, None

def move_duplicate_files(source_folder, target_folder):
    """
    遍历文件夹中的文件，提取ID、高度和角度，并保留ID相同但高度和角度最大的文件，
    其他ID相同的文件将被移动到目标文件夹。
    
    :param source_folder: 源文件夹路径
    :param target_folder: 目标文件夹路径
    """
    if not os.path.exists(target_folder):
        os.makedirs(target_folder)
    
    id_dict = {}  # 用于存储每个ID对应的文件及其最大高度和角度信息
    
    # 遍历文件夹中的文件
    for filename in os.listdir(source_folder):
        file_path = os.path.join(source_folder, filename)
        
        if os.path.isfile(file_path):
            file_id, height, angle = extract_info_from_filename(filename)
            
            if file_id is not None:
                # 检查该ID是否已经出现过
                if file_id not in id_dict:
                    # 如果没有出现过，直接存储当前文件的信息
                    id_dict[file_id] = (file_path, height, angle)
                else:
                    # 如果已经存在同ID的文件，则比较高度和角度
                    existing_file_path, existing_height, existing_angle = id_dict[file_id]
                    # 比较高度和角度，保留更大的那个
                    if (height > existing_height) or (height == existing_height and angle > existing_angle):
                        # 移动旧的文件到目标文件夹
                        shutil.move(existing_file_path, os.path.join(target_folder, os.path.basename(existing_file_path)))
                        # 更新为当前更大的文件
                        id_dict[file_id] = (file_path, height, angle)
                    else:
                        # 移动当前较小的文件到目标文件夹
                        shutil.move(file_path, os.path.join(target_folder, filename))

    print("重复文件处理完成，保留高度和角度最大的文件。")

# 示例使用
source_folder = r"C:\Users\HP\Desktop\Airsim示例照片\SceneImage"  # 替换为你的源文件夹路径
target_folder = r"C:\Users\HP\Desktop\test"  # 替换为你的目标文件夹路径

move_duplicate_files(source_folder, target_folder)
