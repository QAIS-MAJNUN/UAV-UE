import os

def extract_filenames_from_folder(folder):
    """
    提取文件夹中的文件名（不含路径）
    """
    return set(os.listdir(folder))

def compare_folders(folder1, folder2, output_file):
    """
    比较两个文件夹中的文件名集合，并将不同的文件名写入到一个txt文件中
    :param folder1: 第一个文件夹的路径
    :param folder2: 第二个文件夹的路径
    :param output_file: 输出不同文件名的txt文件路径
    """
    # 提取两个文件夹的文件名集合
    filenames1 = extract_filenames_from_folder(folder1)
    filenames2 = extract_filenames_from_folder(folder2)
    
    # 比较文件名集合，找到不同的部分
    diff1 = filenames1 - filenames2  # 仅在 folder1 中存在的文件
    diff2 = filenames2 - filenames1  # 仅在 folder2 中存在的文件

    # 将不同的文件名写入到输出文件
    with open(output_file, 'w') as f:
        # f.write("Files only in folder1:\n")
        for filename in diff1:
            f.write(f"{filename}\n")
        
        # f.write("\nFiles only in folder2:\n")
        for filename in diff2:
            f.write(f"{filename}\n")

    print(f"比较结果已写入 {output_file}")

# 示例使用
folder1 = r"G:\airsim_pic\SurfaceNormalsImage"  # 替换为你的第一个文件夹路径
folder2 = r"G:\airsim_pic\SegmentationImage"  # 替换为你的第二个文件夹路径
folder3 = r"G:\airsim_pic\SceneImage"  # 替换为你的第二个文件夹路径
folder4 = r"G:\airsim_pic\InfraredImage"  # 替换为你的第二个文件夹路径
folder5 = r"G:\airsim_pic\DepthVisImage"  # 替换为你的第二个文件夹路径
folder6 = r"G:\airsim_pic\DepthPlanarImage"  # 替换为你的第二个文件夹路径
folder7 = r"G:\airsim_pic\DepthPerspectiveImage"  # 替换为你的第二个文件夹路径
output_file = r"H:\UAV-temp-staging\process\tools\mv_list.txt"  # 替换为你想输出结果的txt文件路径

compare_folders(folder1, folder7, output_file)
