import open3d as o3d
import numpy as np
import numpy as np
import os

target = 50
final = "E:/__WORKSPACE__/Python/UAV/demo/visualize/output.pcd"


def create_pcd_from_npy(file_path1):
    points = np.load(file_path1)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    os.remove(final)
    o3d.io.write_point_cloud(final, pcd, write_ascii=True)
    print("Done")


if __name__ == "__main__":
    #mayavi显示点云
    create_pcd_from_npy("E:/__WORKSPACE__/Python/UAV/demo/visualize/fine.npy")
