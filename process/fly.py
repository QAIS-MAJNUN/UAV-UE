import sys
import time
import airsim
import pygame
import numpy as np
import cv2
import os
import math

# 基本参数
# 基础的控制速度(m/s)
vehicle_velocity = 2.0
# 设置临时加速比例
speedup_ratio = 20.0
# 用来设置临时加速
speedup_flag = False
# 基础的偏航速率
vehicle_yaw_rate = 5.0
# 无人机摄像头角度
camera_rotations = 0.
# 无人机摄像头最大旋转角度
camera_rotation_rate = math.pi / 4
# 实景图和分割图的存储位置
scenepath = "SceneImage/"
segmentationpath = "SegmentationImage/"
segmentationnormpath = "SegmentationNormalImage/"
surfacepath = "SurfaceNormalsImage/"
depthpath = "DepthVisImage/"
infredpath = "InfraredImage/"
depthperspectivepath = "DepthPerspectiveImage/"
depthplanarpath = "DepthPlanarImage/"
# 已有数据集的数量
files = os.listdir(segmentationpath)
picture_nums = len(files)
# 拍摄的数据集数量
picture_num = 0
# 一般短时间内只会拍摄一次照片，按一次按键需要0.06s，会重复多拍
key_counter = 0


# pygame初始化设置
def pygame_init():
    pygame.init()
    # 创建320*240像素的窗口
    screen = pygame.display.set_mode((320, 240))
    # 窗口标题
    pygame.display.set_caption('keyboard ctrl')
    # 用黑色填充窗口
    screen.fill((0, 0, 0))


# 拍摄数据集
def image_capture(picture_num):
    # 拍摄未压缩的Scene图像
    responsed = AirSim_client.simGetImages([airsim.ImageRequest("front_center", airsim.ImageType.Scene, False, False)])

    if responsed:
        # 获取第一个图像响应
        image_response = responsed[0]

        # 检查图像数据是否有效
        if image_response is not None and len(image_response.image_data_uint8) > 0:
            # 从返回的图像数据创建一个numpy数组
            img1d = np.frombuffer(image_response.image_data_uint8, dtype=np.uint8)

            # 将一维数组重塑为H X W X 3数组
            img_rgb = img1d.reshape(image_response.height, image_response.width, 3)

            # 保存图像
            cv2.imwrite(scenepath + "Scene" + str(picture_nums + picture_num) + ".png", img_rgb)
        else:
            print("No Scene image data received.")
    else:
        print("Scene Image Failed to get.")

    # 拍摄未压缩的Segmentation图像
    responses = AirSim_client.simGetImages(
        [airsim.ImageRequest("front_center", airsim.ImageType.Segmentation, False, False)])

    if responses:
        # 因为返回是一个列表，我们只请求了一张图片，取第一个元素
        response = responses[0]

        # 检查是否有图像数据
        if response is not None and len(response.image_data_uint8) > 0:
            # 从返回的图像数据创建一个numpy数组
            img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)

            # 将一维数组重塑为H X W X 4数组
            img_rgb = img1d.reshape(response.height, response.width, 3)

            # 保存图像
            cv2.imwrite(segmentationnormpath + "SegmentationNormal" + str(picture_nums + picture_num) + ".png", img_rgb)

        else:
            print("No Segmentation image data received.")
    else:
        print("Segmentation Image Failed to get.")

    # model = {
    #     "SUV": [178, 221, 213],
    #     "Sedan": [160, 113, 101],
    #     "BoxTruck": [177, 106, 230],
    #     "Campervan": [130, 56, 55]
    # }
    #       SUV RGB设置为[178, 221, 213]
    #     Sedan RGB设置为[160, 113, 101]
    #  BoxTruck RGB设置为[177, 106, 230]
    # Campervan RGB设置为[130,  56,  55]
    # AirSim_client.simSetSegmentationObjectID(".*", -1, True)
    found1 = AirSim_client.simSetSegmentationObjectID("BoxTruck[\w]*", 48, True)
    found2 = AirSim_client.simSetSegmentationObjectID("Bulldozer[\w]*", 57, True)
    found3 = AirSim_client.simSetSegmentationObjectID("Excavator[\w]*", 90, True)
    found4 = AirSim_client.simSetSegmentationObjectID("ForkLift[\w]*", 93, True)
    found5 = AirSim_client.simSetSegmentationObjectID("Jeep[\w]*", 109, True)
    found6 = AirSim_client.simSetSegmentationObjectID("Motor[\w]*", 178, True)
    found7 = AirSim_client.simSetSegmentationObjectID("Pickup[\w]*", 192, True)
    found8 = AirSim_client.simSetSegmentationObjectID("RoadRoller[\w]*", 189, True)
    found9 = AirSim_client.simSetSegmentationObjectID("Sedan[\w]*", 245, True)
    found10 = AirSim_client.simSetSegmentationObjectID("SUV[\w]*", 172, True)
    found11 = AirSim_client.simSetSegmentationObjectID("Trailer[\w]*", 156, True)
    found12 = AirSim_client.simSetSegmentationObjectID("Truck[\w]*", 239, True)
    found13 = AirSim_client.simSetSegmentationObjectID("Van[\w]*", 149, True)

    # 拍摄未压缩的Segmentation图像
    responses = AirSim_client.simGetImages(
        [airsim.ImageRequest("front_center", airsim.ImageType.Segmentation, False, False)])

    if responses:
        # 因为返回是一个列表，我们只请求了一张图片，取第一个元素
        response = responses[0]

        # 检查是否有图像数据
        if response is not None and len(response.image_data_uint8) > 0:
            # 从返回的图像数据创建一个numpy数组
            img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)

            # 将一维数组重塑为H X W X 4数组
            img_rgb = img1d.reshape(response.height, response.width, 3)

            # 保存图像
            cv2.imwrite(segmentationpath + "Segmentation" + str(picture_nums + picture_num) + ".png", img_rgb)

        else:
            print("No Segmentation image data received.")
    else:
        print("Segmentation Image Failed to get.")


    # SurfaceNormalsImage
    responses = AirSim_client.simGetImages(
        [airsim.ImageRequest("front_center", airsim.ImageType.SurfaceNormals, False, False)])

    if responses:
        # 因为返回是一个列表，我们只请求了一张图片，取第一个元素
        response = responses[0]

        # 检查是否有图像数据
        if response is not None and len(response.image_data_uint8) > 0:
            # 从返回的图像数据创建一个numpy数组
            img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)

            # 将一维数组重塑为H X W X 4数组
            img_rgb = img1d.reshape(response.height, response.width, 3)

            # 保存图像
            cv2.imwrite(surfacepath + "SurfaceNormals" + str(picture_nums + picture_num) + ".png", img_rgb)

        else:
            print("No SurfaceNormals image data received.")
    else:
        print("SurfaceNormals Image Failed to get.")


    # DepthVisImage
    responses = AirSim_client.simGetImages(
        [airsim.ImageRequest("front_center", airsim.ImageType.DepthVis, False, False)])

    if responses:
        # 因为返回是一个列表，我们只请求了一张图片，取第一个元素
        response = responses[0]

        # 检查是否有图像数据
        if response is not None and len(response.image_data_uint8) > 0:
            # 从返回的图像数据创建一个numpy数组
            img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)

            # 将一维数组重塑为H X W X 4数组
            img_rgb = img1d.reshape(response.height, response.width, 3)

            # 保存图像
            cv2.imwrite(depthpath + "DepthVis" + str(picture_nums + picture_num) + ".png", img_rgb)

        else:
            print("No DepthVis image data received.")
    else:
        print("DepthVis Image Failed to get.")

    # DepthVisImage
    responses = AirSim_client.simGetImages(
        [airsim.ImageRequest("front_center", airsim.ImageType.DepthPlanar, False, False)])

    if responses:
        # 因为返回是一个列表，我们只请求了一张图片，取第一个元素
        response = responses[0]

        # 检查是否有图像数据
        if response is not None and len(response.image_data_uint8) > 0:
            # 从返回的图像数据创建一个numpy数组
            img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)

            # 将一维数组重塑为H X W X 4数组
            img_rgb = img1d.reshape(response.height, response.width, 3)

            # 保存图像
            cv2.imwrite(depthplanarpath + "DepthPlanar" + str(picture_nums + picture_num) + ".png", img_rgb)

        else:
            print("No DepthVis image data received.")
    else:
        print("DepthVis Image Failed to get.")

    # DepthVisImage
    responses = AirSim_client.simGetImages(
        [airsim.ImageRequest("front_center", airsim.ImageType.DepthPerspective, False, False)])

    if responses:
        # 因为返回是一个列表，我们只请求了一张图片，取第一个元素
        response = responses[0]

        # 检查是否有图像数据
        if response is not None and len(response.image_data_uint8) > 0:
            # 从返回的图像数据创建一个numpy数组
            img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)

            # 将一维数组重塑为H X W X 4数组
            img_rgb = img1d.reshape(response.height, response.width, 3)

            # 保存图像
            cv2.imwrite(depthperspectivepath + "DepthPerspective" + str(picture_nums + picture_num) + ".png", img_rgb)

        else:
            print("No DepthVis image data received.")
    else:
        print("DepthVis Image Failed to get.")

    # Infrared
    responses = AirSim_client.simGetImages(
        [airsim.ImageRequest("front_center", airsim.ImageType.Infrared, False, False)])

    if responses:
        # 因为返回是一个列表，我们只请求了一张图片，取第一个元素
        response = responses[0]

        # 检查是否有图像数据
        if response is not None and len(response.image_data_uint8) > 0:
            # 从返回的图像数据创建一个numpy数组
            img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)

            # 将一维数组重塑为H X W X 4数组
            img_rgb = img1d.reshape(response.height, response.width, 3)

            # 保存图像
            cv2.imwrite(infredpath + "Infrared" + str(picture_nums + picture_num) + ".png",
                            img_rgb)

        else:
            print("No Infrared image data received.")
    else:
        print("Infrared Image Failed to get.")

# AirSim起飞
# vehicle_name修改为要控制的无人机名称(与settings.json对应)
vehicle_name = ""
AirSim_client = airsim.MultirotorClient()
AirSim_client.confirmConnection()
AirSim_client.enableApiControl(True, vehicle_name)
AirSim_client.armDisarm(True, vehicle_name)
AirSim_client.takeoffAsync(vehicle_name=vehicle_name).join()

pygame_init()

# 提供了三种关闭方式
# 若用户点击窗口关闭按钮，程序会直接退出，无人机会悬停在空中
# 若用户通过Esc关闭，程序会直接退出，无人机会悬停在空中
# 若用户通过.关闭，程序会等待无人机降落后再退出
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
        image_capture(picture_num)  # 拍摄图像均为1080P
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

# AirSim降落
AirSim_client.landAsync(vehicle_name=vehicle_name).join()
AirSim_client.armDisarm(False, vehicle_name)
AirSim_client.enableApiControl(False)

# 关闭pygame窗口并退出程序
sys.exit()
