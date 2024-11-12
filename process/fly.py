import sys
import time
import airsim
import pygame
import numpy as np
import os
import math


################################### 存储位置 ###################################

absolutepath            = "E:/__WORKSPACE__/Python/UAV/process/dataset/"
scenepath               = absolutepath + "SceneImage/"
segmentationpath        = absolutepath + "SegmentationImage/"
segmentationnormpath    = absolutepath + "SegmentationNormalImage/"
surfacepath             = absolutepath + "SurfaceNormalsImage/"
depthpath               = absolutepath + "DepthVisImage/"
infredpath              = absolutepath + "InfraredImage/"
depthperspectivepath    = absolutepath + "DepthPerspectiveImage/"
depthplanarpath         = absolutepath + "DepthPlanarImage/"
pointcloudpath          = absolutepath + "PointCloud/"
detectionpath           = absolutepath + "Detection/"


################################ 无人机基本参数 ################################

vehicle_velocity = 2.0              # 基础的控制速度(m/s)
speedup_ratio = 10.0                # 设置临时加速比例
speedup_flag = False                # 用来设置临时加速
vehicle_yaw_rate = 5.0              # 基础的偏航速率
camera_rotations = 0.               # 无人机摄像头角度
camera_rotation_rate = math.pi / 4  # 无人机摄像头最大旋转角度


################################### 基本信息 ###################################

scene_name = "Real City SF"                     # 场景名称
picture_nums = len(os.listdir(pointcloudpath))  # 已有数据集的数量
picture_num = 0                                 # 拍摄的数据集数量
key_counter = 0                                 # 用于控制拍摄频率，短时间内只会拍摄一次照片




# pygame初始化设置
def pygame_init():
    pygame.init()
    # 创建320*240像素的窗口
    screen = pygame.display.set_mode((320, 240))
    # 窗口标题
    pygame.display.set_caption('keyboard ctrl')
    # 用黑色填充窗口
    screen.fill((0, 0, 0))


# 解析点云数据
def parse_lidarData(data):
    # reshape array of floats to array of [X,Y,Z]
    points = np.array(data.point_cloud, dtype=np.dtype('f4')).reshape(-1, 3)
    seg = data.segmentation
    
    if (len(data.point_cloud) < 3):
        print("\tNo points received from Lidar data")
    elif (len(seg) != len(points)):
        print("\tSeg mismatch with points")
    else:
        with open(pointcloudpath + "PointCloud" + str(picture_nums + picture_num) + ".txt", "w") as f:
            for i in range(points.shape[0]):
                f.write("%f %f %f %d\n" % (points[i, 0], points[i, 1], points[i, 2], seg[i]))


def save_detections(detections, SceneImage):
    with open(detectionpath + "Detection" + str(picture_nums + picture_num) + ".xml", "w") as f:
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


# 拍摄数据集
def image_capture(AirSim_client, picture_num):

    # Scene
    response = AirSim_client.simGetImage("bottom_center", airsim.ImageType.Segmentation) 
    with open(scenepath + "Scene" + str(picture_nums + picture_num) + ".png", 'wb') as f:
        f.write(response) 

    # PointCloud
    parse_lidarData(AirSim_client.getLidarData())

    # Bbox
    detections = AirSim_client.simGetDetections("bottom_center", airsim.ImageType.Scene, vehicle_name = 'Drone')
    save_detections(detections, scenepath + "Scene" + str(picture_nums + picture_num) + ".png")

    print("Captured")
    


if __name__ == "__main__":

    #! AirSim起飞
    # vehicle_name修改为要控制的无人机名称(与settings.json对应)
    vehicle_name = "Drone"
    AirSim_client = airsim.MultirotorClient()
    AirSim_client.confirmConnection()
    AirSim_client.enableApiControl(True, vehicle_name)
    AirSim_client.armDisarm(True, vehicle_name)
    AirSim_client.takeoffAsync(vehicle_name=vehicle_name).join()

    #* 移动到60m高度
    # AirSim_client.moveToZAsync(-60, 10, vehicle_name=vehicle_name).join()

    # 设置分割对象ID及对应颜色
    AirSim_client.simSetSegmentationObjectID(".*", 0, True)
    AirSim_client.simSetSegmentationObjectID("SM_bus_articulated[\w]*", 50, True)
    AirSim_client.simSetSegmentationObjectID(".*tree.*", 245, True)
    AirSim_client.simSetSegmentationObjectID(".*[Bb]ld.*", 149, True)
    AirSim_client.simSetSegmentationObjectID(".*[Bb]uild.*", 149, True)
    AirSim_client.simSetSegmentationObjectID(".*road.*", 109, True)
    AirSim_client.simSetSegmentationObjectID(".*traffic.*light.*", 178, True)

    # 设置物体检测参数
    AirSim_client.simSetDetectionFilterRadius("bottom_center", airsim.ImageType.Scene, 100 * 100, vehicle_name = 'Drone')
    AirSim_client.simAddDetectionFilterMeshName("bottom_center", airsim.ImageType.Scene, "SM_bus_articulated*", vehicle_name = 'Drone')

    pygame_init()

    # 提供了三种关闭方式：若用户点击窗口、ESC关闭按钮，程序会直接退出，无人机会悬停在空中；若用户通过.关闭，程序会等待无人机降落后再退出
    while True:
        yaw_rate = 0.0
        velocity_x = 0.0
        velocity_y = 0.0
        velocity_z = 0.0

        time.sleep(0.02)  # 检测时间间隔为0.02s

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                print("本次飞行共拍摄" + str(picture_num) + "张相片!")
                print("目前数据集中已有" + str(picture_nums + picture_num) + "张相片!")
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
            yaw_rate = (scan_wrapper[pygame.K_d] - scan_wrapper[
                pygame.K_a]) * scale_ratio * vehicle_yaw_rate  # d-a为1顺时针偏航,否则逆时针

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
            picture_num += 1
            image_capture(AirSim_client, picture_num)
            key_counter = 3

        # 控制0.06s才拍摄一次
        key_counter = (key_counter - 1) % 3

        if scan_wrapper[pygame.K_q] or scan_wrapper[pygame.K_e]:
            camera_rotations += (scan_wrapper[pygame.K_q] - scan_wrapper[pygame.K_e]) * math.pi / 6 * 0.02 * scale_ratio
            if camera_rotations > 0:
                camera_rotations = 0
            if camera_rotations < - math.pi / 2:
                camera_rotations = - math.pi / 2
            camera_pose = airsim.Pose(airsim.Vector3r(0, 0, 0),
                                    airsim.to_quaternion(camera_rotations, 0, 0))
            AirSim_client.simSetCameraPose(0, camera_pose)

        # 设置速度控制以及设置偏航控制
        AirSim_client.moveByVelocityBodyFrameAsync(vx=velocity_x, vy=velocity_y, vz=velocity_z, duration=0.02,
                                                yaw_mode=airsim.YawMode(True, yaw_or_rate=yaw_rate))

        if scan_wrapper[pygame.K_ESCAPE]:
            print("本次飞行共拍摄" + str(picture_num) + "张相片!")
            print("目前数据集中已有" + str(picture_nums + picture_num) + "张相片!")
            pygame.quit()
            sys.exit()

        if scan_wrapper[pygame.K_PERIOD]:
            print("本次飞行共拍摄" + str(picture_num) + "张相片!")
            print("目前数据集中已有" + str(picture_nums + picture_num) + "张相片!")
            pygame.quit()
            break

    #! AirSim降落
    AirSim_client.landAsync(vehicle_name=vehicle_name).join()
    AirSim_client.armDisarm(False, vehicle_name)
    AirSim_client.enableApiControl(False)

    # 关闭pygame窗口并退出程序
    sys.exit()
