import open3d as o3d
import numpy as np
import numpy as np
from mayavi import mlab


palette = np.load(r"E:\__WORKSPACE__\Python\UAV\code\visualize\palette.npy").tolist()


def read_pcd(file_path1):
    pointcloud = []
    seg = []
    color = []
    with open(file_path1, 'r') as f:
        lines = f.readlines()
        for line in lines:
            pointcloud.append(line.split(" ")[:3])
            seg.append(line.split(" ")[3])

    pointcloud = np.array(pointcloud, dtype=np.dtype('f4')).reshape(-1, 3)
    seg = np.array(seg, dtype=int).flatten()

    for i in seg:
        color += palette[i]
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
    mlab.plot3d(
        [0, axes[0, 0]],
        [0, axes[0, 1]],
        [0, axes[0, 2]],
        color=(1, 0, 0),
        tube_radius=None,
        figure=fig,
    )
    #y轴
    mlab.plot3d(
        [0, axes[1, 0]],
        [0, axes[1, 1]],
        [0, axes[1, 2]],
        color=(0, 1, 0),
        tube_radius=None,
        figure=fig,
    )
    #z轴
    mlab.plot3d(
        [0, axes[2, 0]],
        [0, axes[2, 1]],
        [0, axes[2, 2]],
        color=(0, 0, 1),
        tube_radius=None,
        figure=fig,
    )
    mlab.show()

#mayavi显示点云
read_pcd("E:/__WORKSPACE__/Python/UAV/process/PointCloud/PointCloud18.txt")
