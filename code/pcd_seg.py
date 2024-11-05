import sys
import time
import airsim
import pygame

# pygame初始化设置
def pygame_init():
    pygame.init()
    # 创建320*240像素的窗口
    screen = pygame.display.set_mode((320, 240))
    # 窗口标题
    pygame.display.set_caption('keyboard ctrl')
    # 用黑色填充窗口
    screen.fill((0, 0, 0))


if __name__ == "__main__":

    # AirSim起飞
    # vehicle_name修改为要控制的无人机名称(与settings.json对应)
    vehicle_name = "Drone"
    AirSim_client = airsim.MultirotorClient()
    AirSim_client.confirmConnection()
    AirSim_client.enableApiControl(True, vehicle_name)
    AirSim_client.armDisarm(True, vehicle_name)
    AirSim_client.takeoffAsync(vehicle_name=vehicle_name).join()

    pygame_init()

    while True:

        time.sleep(0.02)  # 检测时间间隔为0.02s

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

        # 读取键盘指令
        scan_wrapper = pygame.key.get_pressed()

    
        # 根据 'C' 按键来拍摄数据集
        if scan_wrapper[pygame.K_c]:
            count = 0
            for i in range(1000):
                lidarData = AirSim_client.getLidarData()
                # lidarSegData = client.simGetLidarSegmentation()
                lidarSegData = lidarData.segmentation
                
                if not lidarData.point_cloud:
                    print("Null entry, skipping")
                    continue

                print(f"Lidar: {len(lidarData.point_cloud)}")
                print(f"LidarSeg: {len(lidarSegData)}")

                if(len(lidarSegData) != len(lidarData.point_cloud)//3):
                    print("Not equal")
                    count+=1
                time.sleep(0.1)

            print(count)
            break

      

    # AirSim降落
    AirSim_client.landAsync(vehicle_name=vehicle_name).join()
    AirSim_client.armDisarm(False, vehicle_name)
    AirSim_client.enableApiControl(False)

    # 关闭pygame窗口并退出程序
    sys.exit()
