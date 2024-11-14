import numpy as np
import open3d as o3d


absolute_path            = "E:/__WORKSPACE__/Python/UAV/process/dataset/"
pcd_path                 = absolute_path + "PointCloud/"
voxel_path               = absolute_path + "VoxelGrid/"
semanticvox_path         = absolute_path + "SemanticVoxel/"
groundtruth_path         = absolute_path + "GroundTruth/"


def nn_correspondance(verts1, verts2):
    """ for each vertex in verts2 find the nearest vertex in verts1

        Args:
            nx3 np.array's
        Returns:
            ([indices], [distances])

    """
    indices = []
    distances = []

    if len(verts1) == 0 or len(verts2) == 0:
        return indices, distances

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(verts1)
    kdtree = o3d.geometry.KDTreeFlann(pcd)
    for vert in verts2:
        _, inds, dist = kdtree.search_knn_vector_3d(vert, 1)
        indices.append(inds[0])
        distances.append(np.sqrt(dist[0]))

    return indices, distances


def getSemantic(KEY):
    dense_voxels = []
    pcd = []

    with open(voxel_path + "Voxel" + str(KEY) + ".txt", 'r') as f:
        lines = f.readlines()
        for line in lines:
            data = line.split(" ")
            dense_voxels.append([float(data[0]), float(data[1]), float(data[2])])
    dense_voxels = np.array(dense_voxels)
    vox = dense_voxels.copy()

    with open(pcd_path + "PointCloud" + str(KEY) + ".txt", 'r') as f:
        lines = f.readlines()
        for line in lines:
            data = line.split(" ")
            pcd.append([float(data[0]), float(data[1]), float(data[2]), int(data[3])])
    pcd = np.array(pcd)

    indices, _ = nn_correspondance(pcd[:,:3], dense_voxels)


    semantic = pcd[:, 3][np.array(indices)]
    result = np.concatenate([vox, semantic[:, np.newaxis]], axis=1)

    # remove self vox
    with open(groundtruth_path + "GroundTruth" + str(KEY) + ".txt", 'r') as f:
        line = f.readline()
        data = line.split(" ")
        self_position = [float(data[0]), float(data[1]), float(data[2])]
    
    # radius to be tested
    radius = 1.5  

    self_mask = ((np.fabs(result[:, 0] - self_position[0]) > radius) 
                | (np.fabs(result[:, 1] - self_position[1]) > radius) 
                | (np.fabs(result[:, 2] - self_position[2]) > radius))
    
    result = result[self_mask]

    with open(semanticvox_path + "SemanticVoxel" + str(KEY) + ".txt", 'w') as f:
        for i in range(result.shape[0]):
            f.write(str(result[i][0]) + " " + str(result[i][1]) + " " + str(result[i][2]) + " " + str(int(result[i][3])) + "\n")



if __name__ == '__main__':
    
    KEY = 5

    getSemantic(KEY)

    print("Done")