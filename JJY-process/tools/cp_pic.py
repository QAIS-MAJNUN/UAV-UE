import os  # 导入操作系统模块，用于文件和目录操作
import shutil  # 导入高级文件操作模块，用于移动文件

# 定义要搜索的文件夹列表
source_folders = [r"G:\airsim_pic\DepthPerspectiveImage", r"G:\airsim_pic\DepthPlanarImage", r"G:\airsim_pic\DepthVisImage",
                  r"G:\airsim_pic\InfraredImage",r"G:\airsim_pic\SceneImage",r"G:\airsim_pic\SegmentationImage",r"G:\airsim_pic\SurfaceNormalsImage"]

# 创建一个列表，包含要移动的文件的名称。
file_path = "cp_list.txt"   # 这里填入需要删除的txt文件的路径
# 保存要移动文件的列表
cp_list = []
try:
    with open(file_path, 'r') as file:
        for line in file:
            cp_list.append(line.strip())
except FileNotFoundError:
    print(f"文件 {file_path} 不存在。")

# 定义目标文件夹名称
target_folder_name = r"C:\Users\HP\Desktop\Airsim示例照片"
# 指定目标文件夹的名称，这些特定文件将被移动到这个文件夹中。

# 检查目标文件夹是否存在，如果不存在则创建
if not os.path.exists(target_folder_name):
    os.mkdir(target_folder_name)
    # 如果目标文件夹不存在，使用 os.mkdir 创建新的文件夹。

# 遍历每个源文件夹
for folder in source_folders:
    # 对于 source_folders 列表中的每个文件夹路径进行遍历。
    for filename in cp_list:
        # 对于 cp_list 列表中的每个文件名进行遍历。
        filenames = os.listdir(folder)
        for pic in filenames:
            if pic.find(filename) != -1:
                # 如果当前文件名在当前源文件夹中存在。
                source_path = os.path.join(folder, pic)
                # 构建源文件的完整路径。
                target_path = os.path.join(target_folder_name + "\\" + folder.split("\\")[-1], pic)
                if not os.path.exists(target_folder_name + "\\" + folder.split("\\")[-1]):
                    os.mkdir(target_folder_name + "\\" + folder.split("\\")[-1])
                # 构建目标文件的完整路径（在新文件夹中）。
                shutil.copy(source_path, target_path)  # 拷贝文件
