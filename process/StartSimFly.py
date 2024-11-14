import sys
import time
import airsim
import pygame
import numpy as np
import os
import math
import utils.binvox_rw as binvox
from utils.GetSemantic import getSemantic


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


################################ 无人机基本参数 ################################

vehicle_name = "Drone"              # 无人机名称（需要与setting.json对应）
vehicle_velocity = 2.0              # 基础的控制速度(m/s)
speedup_ratio = 10.0                # 设置临时加速比例
speedup_flag = False                # 用来设置临时加速
vehicle_yaw_rate = 5.0              # 基础的偏航速率
camera_rotations = 0.               # 无人机摄像头角度
camera_rotation_rate = math.pi / 4  # 无人机摄像头最大旋转角度
# 建立连接
AirSim_client = airsim.MultirotorClient()
AirSim_client.confirmConnection()
AirSim_client.enableApiControl(True, vehicle_name)


################################# 拍摄基本信息 #################################

scene_name = "Real City SF"                         # 场景名称
tot_data_num = len(os.listdir(pcd_path))            # 已有数据集的数量
cur_data_num = 0                                    # 拍摄的数据集数量
key_counter = 0                                     # 用于控制拍摄频率，短时间内只会拍摄一次照片


################################### Mesh信息 ###################################

# 设置分割对象ID及对应颜色
AirSim_client.simSetSegmentationObjectID(".*", 0, True)
AirSim_client.simSetSegmentationObjectID("SM_bus_articulated[\w]*", 50, True)
AirSim_client.simSetSegmentationObjectID(".*tree.*", 245, True)
AirSim_client.simSetSegmentationObjectID(".*[Bb]ld.*", 149, True)
AirSim_client.simSetSegmentationObjectID(".*[Bb]uild.*", 149, True)
AirSim_client.simSetSegmentationObjectID(".*road.*", 109, True)
AirSim_client.simSetSegmentationObjectID(".*traffic.*light.*", 178, True)

# 设置物体检测参数（通配符）
AirSim_client.simSetDetectionFilterRadius("bottom_center", airsim.ImageType.Scene, 100 * 100, vehicle_name = 'Drone')
AirSim_client.simAddDetectionFilterMeshName("bottom_center", airsim.ImageType.Scene, "SM_bus_articulated*", vehicle_name = 'Drone')

# 获取移动物体列表（正则）
# movable_object = AirSim_client.simListSceneObjects(name_regex = "SM_bus_articulated[\w]*")
movable_object = AirSim_client.simListSceneObjects(name_regex = "SM_bus_articulated_0")


def swap(a, b):  # 交换两个变量的值
    return b, a


def parse_lidarData(data): # 解析点云数据
    
    points = np.array(data.point_cloud, dtype=np.dtype('f4')).reshape(-1, 3)
    seg = data.segmentation
    
    if (len(data.point_cloud) < 3):
        print("\tNo points received from Lidar data")
    elif (len(seg) != len(points)):
        print("\tSeg mismatch with points")
    else:
        with open(pcd_path + "PointCloud" + str(tot_data_num + cur_data_num) + ".txt", "w") as f:
            for i in range(points.shape[0]):
                f.write("%f %f %f %d\n" % (points[i, 0], points[i, 1], points[i, 2], seg[i]))


def save_detections(detections, groundtruth, SceneImage):
    
    # 转换到GlobalNED
    for obj in detections:
        
        obj.box3D.min.x_val, obj.box3D.min.z_val = swap(obj.box3D.min.x_val, obj.box3D.min.z_val)
        obj.box3D.max.x_val, obj.box3D.max.z_val = swap(obj.box3D.max.x_val, obj.box3D.max.z_val)

        obj.box2D.min.x_val += groundtruth.x_val
        obj.box2D.min.y_val += groundtruth.y_val
        obj.box2D.max.x_val += groundtruth.x_val
        obj.box2D.max.y_val += groundtruth.y_val

        obj.box3D.min.x_val += groundtruth.x_val
        obj.box3D.min.y_val += groundtruth.y_val
        obj.box3D.min.z_val += groundtruth.z_val
        obj.box3D.max.x_val += groundtruth.x_val
        obj.box3D.max.y_val += groundtruth.y_val
        obj.box3D.max.z_val += groundtruth.z_val

    # TODO: 添加 prev next 支持

    with open(detection_path + "Detection" + str(tot_data_num + cur_data_num) + ".xml", "w") as f:
        f.write("<annotation>\n")
        f.write("\t<file>" + SceneImage + "</file>\n")
        f.write("\t<scene>" + scene_name + "</scene>\n")
        f.write("\t<prev>" + "pass" + "</prev>\n")
        f.write("\t<next>" + "pass" + "</next>\n")
        f.write("\t<timestamp>" + "pass" + "</timestamp>\n")
        for obj in detections:
            f.write("\t<object>\n")
            f.write("\t\t<name>" + obj.name + "</name>\n")

            f.write("\t\t<box2D>\n")
            f.write("\t\t\t<xmin>" + str(obj.box2D.min.x_val) + "</xmin>\n")
            f.write("\t\t\t<ymin>" + str(obj.box2D.min.y_val) + "</ymin>\n")
            f.write("\t\t\t<xmax>" + str(obj.box2D.max.x_val) + "</xmax>\n")
            f.write("\t\t\t<ymax>" + str(obj.box2D.max.y_val) + "</ymax>\n")
            f.write("\t\t</box2D>\n")

            f.write("\t\t<box3D>\n")
            f.write("\t\t\t<xmin>" + str(obj.box3D.min.x_val) + "</xmin>\n")
            f.write("\t\t\t<ymin>" + str(obj.box3D.min.y_val) + "</ymin>\n")
            f.write("\t\t\t<zmin>" + str(obj.box3D.min.z_val) + "</zmin>\n")
            f.write("\t\t\t<xmax>" + str(obj.box3D.max.x_val) + "</xmax>\n")
            f.write("\t\t\t<ymax>" + str(obj.box3D.max.y_val) + "</ymax>\n")
            f.write("\t\t\t<zmax>" + str(obj.box3D.max.z_val) + "</zmax>\n")
            f.write("\t\t</box3D>\n")

            f.write("\t\t<geo_point>\n")
            f.write("\t\t\t<altitude>" + str(obj.geo_point.altitude) + "</altitude>\n")
            f.write("\t\t\t<latitude>" + str(obj.geo_point.latitude) + "</latitude>\n")
            f.write("\t\t\t<longitude>" + str(obj.geo_point.longitude) + "</longitude>\n")
            f.write("\t\t</geo_point>\n")

            f.write("\t\t<relative_pose>\n")
            f.write("\t\t\t<orientation>\n")
            f.write("\t\t\t\t<w>" + str(obj.relative_pose.orientation.w_val) + "</w>\n")
            f.write("\t\t\t\t<x>" + str(obj.relative_pose.orientation.x_val) + "</x>\n")
            f.write("\t\t\t\t<y>" + str(obj.relative_pose.orientation.y_val) + "</y>\n")
            f.write("\t\t\t\t<z>" + str(obj.relative_pose.orientation.z_val) + "</z>\n")
            f.write("\t\t\t</orientation>\n")
            f.write("\t\t\t<position>\n")
            f.write("\t\t\t\t<x>" + str(obj.relative_pose.position.x_val) + "</x>\n")
            f.write("\t\t\t\t<y>" + str(obj.relative_pose.position.y_val) + "</y>\n")
            f.write("\t\t\t\t<z>" + str(obj.relative_pose.position.z_val) + "</z>\n")
            f.write("\t\t\t</position>\n")
            f.write("\t\t</relative_pose>\n")

            f.write("\t</object>\n")

        f.write("</annotation>\n")


def save_poses(poses, SceneImage):

    # TODO: 添加 prev next 支持

    with open(pose_path + "Pose" + str(tot_data_num + cur_data_num) + ".xml", "w") as f:
        f.write("<annotation>\n")
        f.write("\t<file>" + SceneImage + "</file>\n")
        f.write("\t<scene>" + scene_name + "</scene>\n")
        f.write("\t<prev>" + "pass" + "</prev>\n")
        f.write("\t<next>" + "pass" + "</next>\n")
        f.write("\t<timestamp>" + "pass" + "</timestamp>\n")
        for i, obj in enumerate(poses):
            f.write("\t<object>\n")
            f.write("\t\t<name>" + movable_object[i] + "</name>\n")

            f.write("\t\t<orientation>\n")
            f.write("\t\t\t<w>" + str(obj.orientation.w_val) + "</w>\n")
            f.write("\t\t\t<x>" + str(obj.orientation.x_val) + "</x>\n")
            f.write("\t\t\t<y>" + str(obj.orientation.y_val) + "</y>\n")
            f.write("\t\t\t<z>" + str(obj.orientation.z_val) + "</z>\n")
            f.write("\t\t</orientation>\n")

            f.write("\t\t<position>\n")
            f.write("\t\t\t<x>" + str(obj.position.x_val) + "</x>\n")
            f.write("\t\t\t<y>" + str(obj.position.y_val) + "</y>\n")
            f.write("\t\t\t<z>" + str(obj.position.z_val) + "</z>\n")
            f.write("\t\t</position>\n")

            f.write("\t</object>\n")

        f.write("</annotation>\n")


def vox_process(tot_data_num):
    with open(temp_path + "Voxel" + str(tot_data_num) + ".binvox", 'rb') as f:
        model = binvox.read_as_3d_array(f)
    voxels = model.data
    dims = model.dims
    translate = model.translate
    scale = model.scale
    voxel_size = 1.0 / scale / dims[0]
    
    x, y, z = np.indices(dims)
    filled = voxels.flatten()
    x = x.flatten()[filled]
    y = y.flatten()[filled]
    z = z.flatten()[filled]

    x = (x + 0.5) * voxel_size
    y = (y + 0.5) * voxel_size
    z = (z + 0.5) * voxel_size

    x, y = swap(x, y)
    z = -z 

    with open(groundtruth_path + "GroundTruth" + str(tot_data_num) + ".txt", 'r') as f:
        line = f.readline()
        tmp = line.split(" ")
        x = x + float(tmp[0]) + translate[0]
        y = y + float(tmp[1]) + translate[1]
        z = z + float(tmp[2]) - translate[2]

    with open(voxel_path + "Voxel" + str(tot_data_num) + ".txt", 'w') as f:
        for j in range(len(x)):
            f.write(str(x[j]) + " " + str(y[j]) + " " + str(z[j]) + "\n")

    os.remove(temp_path + "Voxel" + str(tot_data_num) + ".binvox")
    print("\tVoxel Processed!")


def semantic_process(tot_data_num):
    getSemantic(tot_data_num)
    print("\tSemantic Processed!")


def image_capture(AirSim_client, cur_data_num):  # 拍摄数据集

    #* START
    print("Start capture!")

    # TODO: 添加 token 支持

    token = scene_path + "Scene" + str(tot_data_num + cur_data_num) + ".png"

    # Scene
    response = AirSim_client.simGetImage("bottom_center", airsim.ImageType.Segmentation) 
    with open(scene_path + "Scene" + str(tot_data_num + cur_data_num) + ".png", 'wb') as f:
        f.write(response) 

    # PointCloud
    pcd = AirSim_client.getLidarData()
    parse_lidarData(pcd)

    #? Pose and bbox may not be accurate when the drone is moving

    # Pose
    poses = []
    for obj in movable_object:
        pose = AirSim_client.simGetObjectPose(obj)
        poses.append(pose)
    save_poses(poses, token)

    # Bbox
    detections = AirSim_client.simGetDetections("bottom_center", airsim.ImageType.Scene, vehicle_name = 'Drone')
    groundtruth = AirSim_client.simGetGroundTruthEnvironment(vehicle_name='Drone').position
    save_detections(detections, groundtruth, token)

    # Voxel
    with open(groundtruth_path + "GroundTruth" + str(tot_data_num + cur_data_num) + ".txt", "w") as f:
        f.write(str(groundtruth.x_val) + " " + str(groundtruth.y_val) + " " + str(groundtruth.z_val))
    AirSim_client.simCreateVoxelGrid(groundtruth, 200, 200, 200, 1, temp_path + "Voxel" + str(tot_data_num + cur_data_num) + ".binvox")
    vox_process(tot_data_num + 1)

    # SemanticVoxel
    semantic_process(tot_data_num + 1)

    #* END
    print("Captured!")




if __name__ == "__main__":

    #! AirSim起飞
    AirSim_client.armDisarm(True, vehicle_name)
    AirSim_client.takeoffAsync(vehicle_name=vehicle_name).join()

    # AirSim_client.moveToZAsync(-60, 10, vehicle_name=vehicle_name).join()

    # 初始化pygame
    pygame.init()
    screen = pygame.display.set_mode((320, 240))
    pygame.display.set_caption('keyboard ctrl')
    screen.fill((0, 0, 0))

    # 提供了三种关闭方式：若用户点击窗口、ESC关闭按钮，程序会直接退出，无人机会悬停在空中；若用户通过.关闭，程序会等待无人机降落后再退出
    while True:
        yaw_rate = 0.0
        velocity_x = 0.0
        velocity_y = 0.0
        velocity_z = 0.0

        time.sleep(0.02)  # 检测时间间隔为0.02s

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                print("本次飞行共拍摄" + str(cur_data_num) + "张相片!")
                print("目前数据集中已有" + str(tot_data_num + cur_data_num) + "张相片!")
                pygame.quit()
                sys.exit()

        # 读取键盘指令
        scan_wrapper = pygame.key.get_pressed()

        # 按下空格键加速10倍
        if scan_wrapper[pygame.K_SPACE]:
            scale_ratio = speedup_ratio  # 加速倍率,若按空格则为10倍,否则是1倍
        else:
            scale_ratio = speedup_ratio / speedup_ratio

        # 根据 'A' 和 'D' 按键来设置偏航速率变量
        if scan_wrapper[pygame.K_a] or scan_wrapper[pygame.K_d]:
            yaw_rate = (scan_wrapper[pygame.K_d] - scan_wrapper[pygame.K_a]) * scale_ratio * vehicle_yaw_rate  # d-a为1顺时针偏航,否则逆时针

        # 根据 'UP' 和 'DOWN' 按键来设置pitch轴速度变量(NED坐标系，x为机头向前)	同时也是前进后退
        if scan_wrapper[pygame.K_UP] or scan_wrapper[pygame.K_DOWN]:
            velocity_x = (scan_wrapper[pygame.K_UP] - scan_wrapper[pygame.K_DOWN]) * scale_ratio

        # 根据 'LEFT' 和 'RIGHT' 按键来设置roll轴速度变量(NED坐标系，y为正右方)	 同时也是左右飞行
        if scan_wrapper[pygame.K_LEFT] or scan_wrapper[pygame.K_RIGHT]:
            velocity_y = -(scan_wrapper[pygame.K_LEFT] - scan_wrapper[pygame.K_RIGHT]) * scale_ratio

        # 根据 'W' 和 'S' 按键来设置z轴速度变量(NED坐标系，z轴向上为负)			同时也是上升下降
        if scan_wrapper[pygame.K_w] or scan_wrapper[pygame.K_s]:
            velocity_z = -(scan_wrapper[pygame.K_w] - scan_wrapper[pygame.K_s]) * scale_ratio

        # 根据 'C' 按键来拍摄数据集
        if scan_wrapper[pygame.K_c] and key_counter == 0:
            cur_data_num += 1
            image_capture(AirSim_client, cur_data_num)
            key_counter = 3

        # 控制0.06s才拍摄一次
        key_counter = (key_counter - 1) % 3

        if scan_wrapper[pygame.K_q] or scan_wrapper[pygame.K_e]:
            camera_rotations += (scan_wrapper[pygame.K_q] - scan_wrapper[pygame.K_e]) * math.pi / 6 * 0.02 * scale_ratio
            if camera_rotations > 0:
                camera_rotations = 0
            if camera_rotations < - math.pi / 2:
                camera_rotations = - math.pi / 2
            camera_pose = airsim.Pose(airsim.Vector3r(0, 0, 0), airsim.to_quaternion(camera_rotations, 0, 0))
            AirSim_client.simSetCameraPose(0, camera_pose)

        # 设置速度控制以及设置偏航控制
        AirSim_client.moveByVelocityBodyFrameAsync(vx=velocity_x, vy=velocity_y, vz=velocity_z, duration=0.02, yaw_mode=airsim.YawMode(True, yaw_or_rate=yaw_rate))

        if scan_wrapper[pygame.K_ESCAPE]:
            print("本次飞行共拍摄" + str(cur_data_num) + "张相片!")
            print("目前数据集中已有" + str(tot_data_num + cur_data_num) + "张相片!")
            pygame.quit()
            sys.exit()

        if scan_wrapper[pygame.K_PERIOD]:
            print("本次飞行共拍摄" + str(cur_data_num) + "张相片!")
            print("目前数据集中已有" + str(tot_data_num + cur_data_num) + "张相片!")
            pygame.quit()
            break

    #! AirSim降落
    AirSim_client.landAsync(vehicle_name=vehicle_name).join()
    AirSim_client.armDisarm(False, vehicle_name)
    AirSim_client.enableApiControl(False)

    

    sys.exit()
