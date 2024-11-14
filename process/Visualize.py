import numpy as np
from mayavi import mlab
import xml.etree.ElementTree as ET
import quaternion
import utils.binvox_rw as binvox


################################## 关键帧信息 ##################################

KEY = 6
# todo
INTER = []
OBJECT = {50:"SM_bus_articulated_14"}


################################### 存储位置 ###################################

absolute_path            = "E:/__WORKSPACE__/Python/UAV/process/dataset/"
scene_path               = absolute_path + "SceneImage/"
segmentation_path        = absolute_path + "SegmentationImage/"
segmentationnorm_path    = absolute_path + "SegmentationNormalImage/"
surface_path             = absolute_path + "SurfaceNormalsImage/"
depth_path               = absolute_path + "DepthVisImage/"
infred_path              = absolute_path + "InfraredImage/"
depthperspective_path    = absolute_path + "DepthPerspectiveImage/"
depthplanar_path         = absolute_path + "DepthPlanarImage/"
pcd_path                 = absolute_path + "PointCloud/"
detection_path           = absolute_path + "Detection/"
pose_path                = absolute_path + "Pose/"
voxel_path               = absolute_path + "VoxelGrid/"
semanticvox_path          = absolute_path + "SemanticVoxel/"
groundtruth_path         = absolute_path + "GroundTruth/"
temp_path                = absolute_path + "Temp/"


################################### 色彩LUT ###################################

palette = np.load("E:/__WORKSPACE__/Python/UAV/process/ColorLUT/palette.npy")
palette = np.hstack((palette, np.ones((palette.shape[0], 1), dtype=np.uint8) * 255))
palette[50] = [255, 0, 0, 255]



delta = None


def swap(a, b):
    return b, a


def get_boxes(xml_path):
    global delta
    tree = ET.parse(xml_path)
    root = tree.getroot()
    objects = root.findall('object')
    boxes = []
    for obj in objects:
        box = obj.find('box3D')
        xmin = float(box.find('xmin').text)
        ymin = -float(box.find('ymin').text)
        zmin = -float(box.find('zmin').text)
        xmax = float(box.find('xmax').text)
        ymax = -float(box.find('ymax').text)
        zmax = -float(box.find('zmax').text)
        delta = [xmax-xmin, ymax-ymin, zmax-zmin]
        boxes.append([[xmin, ymin, zmin], [xmax, ymin, zmin], [xmax, ymax, zmin], [xmin, ymax, zmin], [xmin, ymin, zmax], [xmax, ymin, zmax], [xmax, ymax, zmax], [xmin, ymax, zmax]])
    return boxes


def draw_boxes(fig, boxespath, color=(1,0,0), line_width=1):
    boxes = get_boxes(boxespath)
    boxes = np.array(boxes)
    for i in range(len(boxes)):
        box = boxes[i]
        for k in range(0,4):
            i,j=k,(k+1)%4
            mlab.plot3d([box[i,0], box[j,0]], [box[i,1], box[j,1]], [box[i,2], box[j,2]], color=color, tube_radius=None, line_width=line_width, figure=fig)
            i,j=k+4,(k+3)%4 + 4
            mlab.plot3d([box[i,0], box[j,0]], [box[i,1], box[j,1]], [box[i,2], box[j,2]], color=color, tube_radius=None, line_width=line_width, figure=fig)
            i,j=k,k+4
            mlab.plot3d([box[i,0], box[j,0]], [box[i,1], box[j,1]], [box[i,2], box[j,2]], color=color, tube_radius=None, line_width=line_width, figure=fig)
    

def draw_boxes_from_center(fig, center, color=(1,0,0), line_width=1):
    boxes = []
    xmin = center[0] - delta[0]/2
    ymin = center[1] - delta[1]/2
    zmin = center[2] - delta[2]/2
    xmax = center[0] + delta[0]/2
    ymax = center[1] + delta[1]/2
    zmax = center[2] + delta[2]/2
    boxes.append([[xmin, ymin, zmin], [xmax, ymin, zmin], [xmax, ymax, zmin], [xmin, ymax, zmin], [xmin, ymin, zmax], [xmax, ymin, zmax], [xmax, ymax, zmax], [xmin, ymax, zmax]])
    boxes = np.array(boxes)
    for i in range(len(boxes)):
        box = boxes[i]
        for k in range(0,4):
            i,j=k,(k+1)%4
            mlab.plot3d([box[i,0], box[j,0]], [box[i,1], box[j,1]], [box[i,2], box[j,2]], color=color, tube_radius=None, line_width=line_width, figure=fig)
            i,j=k+4,(k+3)%4 + 4
            mlab.plot3d([box[i,0], box[j,0]], [box[i,1], box[j,1]], [box[i,2], box[j,2]], color=color, tube_radius=None, line_width=line_width, figure=fig)
            i,j=k,k+4
            mlab.plot3d([box[i,0], box[j,0]], [box[i,1], box[j,1]], [box[i,2], box[j,2]], color=color, tube_radius=None, line_width=line_width, figure=fig)
    mlab.view(azimuth=180,elevation=None,distance=50,focalpoint=[12.0909996 , -1.04700089, -2.03249991])


def get_poses(xml_path):
    tree = ET.parse(xml_path)
    root = tree.getroot()
    objects = root.findall('object')
    poses = []
    for obj in objects:
        pose = []

        name = obj.find('name').text
        pose.append(name)

        orient = obj.find('orientation')
        w = float(orient.find('w').text)
        x = float(orient.find('x').text)
        y = float(orient.find('y').text)
        z = float(orient.find('z').text)
        pose.append(np.quaternion(w, x, y, z))

        pos = obj.find('position')
        x = float(pos.find('x').text)
        y = float(pos.find('y').text)
        z = float(pos.find('z').text)
        pose.append([x, y, z])

        poses.append(pose)
    return poses


def get_forward_vector(poses):
    forward = []
    v = np.array([0, 1, 0])
    for pose in poses:
        R = quaternion.as_rotation_matrix(pose[1])
        forward.append(v@R)
    return forward


def align(refering_pose, pose, points):

    xyz = np.hstack((points, np.ones((xyz.shape[0], 1))))
    q = pose[0]
    refering_q = refering_pose[0]

    translation = np.array([[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]])
    translation[0,4] = pose[1][0] - refering_pose[1][0]
    translation[1,4] = pose[1][1] - refering_pose[1][1]
    translation[2,4] = pose[1][2] - refering_pose[1][2]

    xyz = xyz @ translation

    rotation = refering_q*quaternion.inverse(q)
    R = quaternion.as_rotation_matrix(rotation)
    xyz = xyz @ R

    translation[0,4] = refering_pose[1][0]
    translation[1,4] = refering_pose[1][1]
    translation[2,4] = refering_pose[1][2]

    xyz = xyz @ translation

    return xyz[:,:3]


def draw_vector(fig, posespath, color=(0,0,1), line_width=6, z_translate=10, scale_factor=3):
    poses = get_poses(posespath)
    forward = get_forward_vector(poses)
    center = [pose[2] for pose in poses]
    for i,quiver in enumerate(forward):
        mlab.quiver3d(center[i][0], center[i][1], center[i][2]-z_translate, quiver[0], -quiver[1], quiver[2], line_width=line_width, scale_factor=scale_factor, figure=fig, color=color) 


def draw_pcd(fig, pcdpath):
    with open(pcdpath, 'r') as f:
        tmp = []
        lines = f.readlines()
        for line in lines:
            tmp.append(line.split(" "))
    pcd = np.array(tmp, dtype=np.dtype('f4')).reshape(-1, 4)
    pts = mlab.points3d(
        pcd[:, 0],
        pcd[:, 1],
        pcd[:, 2],
        pcd[:, 3],
        colormap="viridis",
        scale_factor = 0.75,
        # scale_factor = voxel_size - 0.05 * voxel_size,
        mode="sphere",
        opacity=1.0,
        vmin=0,
        vmax=19,
        figure=fig
    )

    pts.glyph.scale_mode = "scale_by_vector"
    pts.module_manager.scalar_lut_manager.lut._vtk_obj.SetTableRange(0, palette.shape[0])
    pts.module_manager.scalar_lut_manager.lut.number_of_colors = palette.shape[0]
    pts.module_manager.scalar_lut_manager.lut.table = palette

    
def draw_vox(fig, voxpath, color=(220/255, 220/255, 220/255), opacity=0.75, scale_factor=1):
    voxel = []
    with open(voxpath, 'r') as f:
        lines = f.readlines()
        for line in lines:
            data = line.split(" ")
            voxel.append([float(data[0]), float(data[1]), float(data[2])])
    voxel = np.array(voxel)
    mlab.points3d(voxel[:, 0], voxel[:, 1], voxel[:, 2], mode='cube', color=color, opacity=opacity, scale_factor=scale_factor, figure=fig)


def draw_semantic_vox(fig, semanticpath, scale_factor = 0.75, opacity=1.0):
    voxel = []
    with open(semanticpath) as f:
        lines = f.readlines()
        for line in lines:
            data = line.split(" ")
            voxel.append([float(data[0]), float(data[1]), float(data[2]), int(data[3])])

    voxel = np.array(voxel)
    voxel_size = 0.5
    
    vox = mlab.points3d(
        voxel[:, 0],
        voxel[:, 1],
        voxel[:, 2],
        voxel[:, 3],
        colormap="viridis",
        scale_factor = scale_factor,
        # scale_factor = voxel_size - 0.05 * voxel_size,
        mode="cube",
        opacity=opacity,
        vmin=0,
        vmax=19,
    )

    vox.glyph.scale_mode = "scale_by_vector"
    vox.module_manager.scalar_lut_manager.lut._vtk_obj.SetTableRange(0, palette.shape[0])
    vox.module_manager.scalar_lut_manager.lut.number_of_colors = palette.shape[0]
    vox.module_manager.scalar_lut_manager.lut.table = palette


def multi_concat():

    # TODO

    pcd = []
    poses = []

    # read KEY pcd & seg & pose
    tmp = []
    with open(pcd_path + str(KEY) + ".txt", 'r') as f:
        lines = f.readlines()
        for line in lines:
            tmp.append(line.split(" "))
    tmp = np.array(tmp, dtype=np.dtype('f4')).reshape(-1, 4).tolist()
    pcd.append(tmp)
    poses.append(get_poses(pose_path + str(KEY) + ".xml"))

    # read INTER pcd & seg & pose
    for i in INTER:
        tmp = []
        with open(pcd_path + str(i) + ".txt", 'r') as f:
            lines = f.readlines()
            for line in lines:
                tmp.append(line.split(" "))
        tmp = np.array(tmp, dtype=np.dtype('f4')).reshape(-1, 4).tolist()
        pcd.append(tmp)
        poses.append(get_poses(pose_path + str(i) + ".xml"))
    
    #  TODO concat

    # Only vis KEY
    pcd = np.array(pcd[0])
    draw_pcd(pcd[:,:3], pcd[:,3], get_boxes(detection_path + str(KEY) + ".xml"), poses[0])


# todo concat
def concat(pcd, poses):
    result = []
    static = np.ones()
    for id in OBJECT:
        mask_key = []
        mask_inter = []
        name = OBJECT[id]
        for i in range(len(pcd[0])):
            if pcd[0][i][3] == id:
                mask_key.append(i)


def sgl_concat():
    
    pcd = pcd_path + "PointCloud" + str(KEY) + ".txt"
    pose = pose_path + "Pose" + str(KEY) + ".xml"
    box = detection_path + "Detection" + str(KEY) + ".xml"
    vox = voxel_path + "Voxel" + str(KEY) + ".txt"
    semantic = semanticvox_path + "SemanticVoxel" + str(KEY) + ".txt"

    draw_pcd(fig, pcd)
    draw_boxes(fig, box, color=(1,0,0), line_width=1)
    draw_vector(fig, pose, color=(0,0,1), line_width=6, z_translate=10, scale_factor=3)
    # draw_vox(fig, vox, color=(220/255, 220/255, 220/255), opacity=0.75, scale_factor=1)
    draw_semantic_vox(fig, semantic, scale_factor = 0.75, opacity=1.0)





if __name__ == '__main__':

    # mlab.options.offscreen = True

    fig = mlab.figure(bgcolor=(1, 1, 1), size=(1920, 1080))

    # 绘制原点
    mlab.points3d(0, 0, 0, color=(1, 1, 1), mode="sphere",scale_factor=0.2)

    # 绘制坐标
    axes = np.array([[20.0, 0.0, 0.0, 0.0], [0.0, 20.0, 0.0, 0.0], [0.0, 0.0, 20.0, 0.0]],dtype=np.float64,)
    mlab.plot3d([0, axes[0, 0]],[0, axes[0, 1]],[0, axes[0, 2]],color=(1, 0, 0),tube_radius=None,figure=fig,)   #x轴
    mlab.plot3d([0, axes[1, 0]],[0, axes[1, 1]],[0, axes[1, 2]],color=(0, 1, 0),tube_radius=None,figure=fig,)   #y轴
    mlab.plot3d([0, axes[2, 0]],[0, axes[2, 1]],[0, axes[2, 2]],color=(0, 0, 1),tube_radius=None,figure=fig,)   #z轴

    sgl_concat()

    # mlab.view(azimuth=180,elevation=None,distance=50,focalpoint=[12.0909996 , -1.04700089, -2.03249991])

    # mlab.savefig('temp/mayavi.png')

    mlab.show()
    

    
