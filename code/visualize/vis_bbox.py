import open3d as o3d
import numpy as np
from mayavi import mlab
import xml.etree.ElementTree as ET
import quaternion
import binvox_rw as binvox


KEY = 2
INTER = []
OBJECT = {50:"SM_bus_articulated_14"}

pcdpath = "E:/__WORKSPACE__/Python/UAV/process/dataset/PointCloud/PointCloud"
bboxpath =  "E:/__WORKSPACE__/Python/UAV/process/dataset/Detection/Detection"
posepath = "E:/__WORKSPACE__/Python/UAV/process/dataset/Pose/Pose"
binvox_path = "E:/__WORKSPACE__/Python/UAV/process/dataset/VoxelGrid/Voxel"
groundtruthpath = "E:/__WORKSPACE__/Python/UAV/process/dataset/VoxelGrid/Groundtruth"

palette = np.load(r"E:\__WORKSPACE__\Python\UAV\code\visualize\palette.npy").tolist()
delta = None

def sawp(a, b):
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
        print(xmin-xmax, ymin-ymax, zmin-zmax)
    return boxes


def draw_boxes(boxes, fig, color=(1,0,0), line_width=1):
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


def draw_boxes_from_center(center, fig, color=(1,0,0), line_width=1):
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
    print(poses)
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


def draw_vector(poses, fig):
    forward = get_forward_vector(poses)
    center = [pose[2] for pose in poses]
    for i,quiver in enumerate(forward):
        mlab.quiver3d(center[i][0], center[i][1], center[i][2],
                    quiver[0], -quiver[1], quiver[2], line_width=6, scale_factor=3, figure=fig) 


def draw_pcd(pointcloud, seg, bbox, pose):
    # pointcloud = pointcloud.tolist()
    # seg = seg.tolist()
    color = []
    for i in seg:
        color += palette[int(i)]
        color.append(255)

    color = np.array(color).reshape(-1, 4)
    
    x = pointcloud[:, 0]  # x position of point
    xmin = np.amin(x, axis=0)
    xmax = np.amax(x, axis=0)
    y = pointcloud[:, 1]  # y position of point
    ymin = np.amin(y, axis=0)
    ymax = np.amax(y, axis=0)
    z = pointcloud[:, 2]  # z position of point
    zmin = np.amin(z, axis=0)
    zmax = np.amax(z, axis=0)
    d = np.sqrt(x ** 2 + y ** 2)  # Map Distance from sensor
    vals = 'height'
    if vals == "height":
        col = z
    else:
        col = d

    

    fig = mlab.figure(bgcolor=(0, 0, 0), size=(640, 500))
    pts = mlab.pipeline.scalar_scatter(x, y, z) # plot the points
    pts.add_attribute(color, 'colors') # assign the colors to each point
    pts.data.point_data.set_active_scalars('colors')
    g = mlab.pipeline.glyph(pts)
    g.glyph.glyph.scale_factor = 0.5 # set scaling for all the points
    g.glyph.scale_mode = 'data_scaling_off' # make all the points same size


    # 绘制原点
    mlab.points3d(0, 0, 0, color=(1, 1, 1), mode="sphere",scale_factor=0.2)
    # 绘制坐标
    axes = np.array(
        [[20.0, 0.0, 0.0, 0.0], [0.0, 20.0, 0.0, 0.0], [0.0, 0.0, 20.0, 0.0]],
        dtype=np.float64,
    )
    #x轴
    mlab.plot3d([0, axes[0, 0]],[0, axes[0, 1]],[0, axes[0, 2]],color=(1, 0, 0),tube_radius=None,figure=fig,)
    #y轴
    mlab.plot3d([0, axes[1, 0]],[0, axes[1, 1]],[0, axes[1, 2]],color=(0, 1, 0),tube_radius=None,figure=fig,)
    #z轴
    mlab.plot3d([0, axes[2, 0]],[0, axes[2, 1]],[0, axes[2, 2]],color=(0, 0, 1),tube_radius=None,figure=fig,)

    draw_boxes(bbox, fig)
    # draw_boxes_from_center([-36.981483459472656,-5.128476619720459,3.087404727935791], fig)
    draw_vector(pose, fig)
    visualize_voxels_mayavi(binvox_path + str(KEY) + ".binvox") 
    mlab.show()


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
        
        
def visualize_voxels_mayavi(binvox_path):
    with open(binvox_path, 'rb') as f:
        model = binvox.read_as_3d_array(f)
    voxels = model.data
    dims = model.dims
    translate = model.translate
    scale = model.scale
    
    # 创建体素网格
    x, y, z = np.indices(dims)
    filled = voxels.flatten()
    x = x.flatten()[filled]
    y = y.flatten()[filled]
    z = z.flatten()[filled]
    x, y = sawp(x, y)
    with open(groundtruthpath + str(KEY) + ".txt", 'r') as f:
        line = f.readline()
        tmp = line.split(" ")
        print(x)
        x = x + float(tmp[0]) + translate[0]
        print(x)
        y = y + float(tmp[1]) + translate[1]
        z = z - float(tmp[2]) + translate[2]
    
    # 绘制体素
    mlab.points3d(x, y, -z, mode='cube', color=(1, 0, 0), scale_factor=1)
    






if __name__ == '__main__':
    pcd = []
    poses = []

    # read KEY pcd & seg & pose
    tmp = []
    with open(pcdpath + str(KEY) + ".txt", 'r') as f:
        lines = f.readlines()
        for line in lines:
            tmp.append(line.split(" "))
    tmp = np.array(tmp, dtype=np.dtype('f4')).reshape(-1, 4).tolist()
    pcd.append(tmp)
    poses.append(get_poses(posepath + str(KEY) + ".xml"))

    # read INTER pcd & seg & pose
    for i in INTER:
        tmp = []
        with open(pcdpath + str(i) + ".txt", 'r') as f:
            lines = f.readlines()
            for line in lines:
                tmp.append(line.split(" "))
        tmp = np.array(tmp, dtype=np.dtype('f4')).reshape(-1, 4).tolist()
        pcd.append(tmp)
        poses.append(get_poses(posepath + str(i) + ".xml"))
    
    # todo concat

    # Only vis KEY
    pcd = np.array(pcd[0])
    draw_pcd(pcd[:,:3], pcd[:,3], get_boxes(bboxpath + str(KEY) + ".xml"), poses[0])
