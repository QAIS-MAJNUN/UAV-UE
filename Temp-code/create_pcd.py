import open3d as o3d
import numpy as np
import numpy as np
import os

target = 50
output = "E:/__WORKSPACE__/Python/UAV/demo/target.txt"
final = "E:/__WORKSPACE__/Python/UAV/demo/target.pcd"

def create_pcd(file_path1):
    pcd = []
    with open(file_path1, "r") as f:
        lines = f.readlines()
        for line in lines:
            data = line.strip().split(" ")
            x = float(data[0])
            y = float(data[1])
            z = float(data[2])
            seg = int(data[3])
            if seg == target:
                pcd.append(f"{x} {y} {z}\n")
    
    with open(output, "w") as f:
        for line in pcd:
            f.write(line)
    
    points = np.loadtxt(output)
    # 创建Open3D点云对象
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    os.remove(final)
    o3d.io.write_point_cloud(final, pcd, write_ascii=True)
    print("Done")


if __name__ == "__main__":
    #mayavi显示点云
    create_pcd("E:/__WORKSPACE__/Python/UAV/process/PointCloud/PointCloud18.txt")
