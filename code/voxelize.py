import numpy as np
 
def point_cloud_to_voxel(point_cloud, voxel_resolution):
    voxel_grid = np.zeros(voxel_resolution, dtype=np.int32)
    
    # 计算点云中的坐标范围
    min_coords = np.min(point_cloud, axis=0)
    max_coords = np.max(point_cloud, axis=0)
    
    # 计算归一化的坐标范围
    range_coords = max_coords - min_coords
    
    # 归一化点云坐标
    normalized_point_cloud = (point_cloud - min_coords) / range_coords
    
    for point in normalized_point_cloud:
        x, y, z = point
        voxel_x = int(x * voxel_resolution[0])
        voxel_y = int(y * voxel_resolution[1])
        voxel_z = int(z * voxel_resolution[2])
        
        voxel_x = max(0, min(voxel_resolution[0] - 1, voxel_x))
        voxel_y = max(0, min(voxel_resolution[1] - 1, voxel_y))
        voxel_z = max(0, min(voxel_resolution[2] - 1, voxel_z))
        
        voxel_grid[voxel_x, voxel_y, voxel_z] = 1
    
    return voxel_grid
 
import numpy as np
 
def point_cloud_to_voxel(point_cloud, voxel_resolution):
    voxel_grid = np.zeros(voxel_resolution, dtype=np.int32)
    
    # 计算点云中的坐标范围
    min_coords = np.min(point_cloud, axis=0)
    max_coords = np.max(point_cloud, axis=0)
    
    # 计算归一化的坐标范围
    range_coords = max_coords - min_coords
    
    # 归一化点云坐标
    normalized_point_cloud = (point_cloud - min_coords) / range_coords
    
    for point in normalized_point_cloud:
        x, y, z = point
        voxel_x = int(x * voxel_resolution[0])
        voxel_y = int(y * voxel_resolution[1])
        voxel_z = int(z * voxel_resolution[2])
        
        voxel_x = max(0, min(voxel_resolution[0] - 1, voxel_x))
        voxel_y = max(0, min(voxel_resolution[1] - 1, voxel_y))
        voxel_z = max(0, min(voxel_resolution[2] - 1, voxel_z))
        
        voxel_grid[voxel_x, voxel_y, voxel_z] = 1
    
    return voxel_grid
 
# 示例点云数据，每个点表示为 (x, y, z) 坐标，可能包含负数坐标
point_cloud =np.loadtxt(r"/your/point/cloud/path")
 
# 定义体素分辨率，这里假设分辨率为 (10, 10, 10)
voxel_resolution = (10, 10, 10)
 
# 将点云转换为体素格式
voxel_data = point_cloud_to_voxel(point_cloud, voxel_resolution)
 
# 可视化体素数据
plot_voxel(voxel_data)