import os



# 此函数用于将 名字为 Scene2_5_5_VRGame.png 中的 2 变为 addnum + 2
def reindex_all_files_in_folder(path,addnum):
    for root, dirs, files in os.walk(path):
        for file in files:
            old_file_path = os.path.join(root, file)
            id = int(file.split("_")[0].split("Scene")[1]) + addnum # 拆分出Scene中的2 + addnum
            real_filename = os.path.splitext(file)[0]
            file_extension = os.path.splitext(file)[1]
            new_file_name = "Scene" + str(id) + "_" + real_filename.split("_",maxsplit = 1)[1] + file_extension
            new_file_path = os.path.join(root, new_file_name)
            os.rename(old_file_path, new_file_path)


folderpath = r"I:\UAV-temp-staging\test1008\airsim_pic"
addnum = 101650 # 用于控制将Scene后的数字加上一个值

# 使用示例，将路径替换为你的实际文件夹路径
reindex_all_files_in_folder(folderpath,addnum)