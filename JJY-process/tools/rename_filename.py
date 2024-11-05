import os

# Scene7256_45_80_VRGame

def rename_all_files_in_folder(path,id_start=None,id_end=None,start_filename=None,end_filename=None):
    for root, dirs, files in os.walk(path):
        for file in files:
            if(file.endswith(add_end_filename + ".png")): continue
            old_file_path = os.path.join(root, file)
            id = int(file.split("_")[0].split("Scene")[1])
            if(id >= id_start and id <= id_end):
                real_filename = os.path.splitext(file)[0]
                file_extension = os.path.splitext(file)[1]
                new_file_name = start_filename + real_filename.rsplit("_",1)[0] + "_" + end_filename  + file_extension
                new_file_path = os.path.join(root, new_file_name)
                os.rename(old_file_path, new_file_path)


folderpath = r"G:\airsim_pic"
start_id = 1
end_id = 10000
add_start_filename = ""
add_end_filename = "Brushify"

# 使用示例，将路径替换为你的实际文件夹路径
rename_all_files_in_folder(folderpath,start_id,end_id,add_start_filename,add_end_filename)