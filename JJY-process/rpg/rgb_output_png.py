import cv2
import numpy as np

def png_output(array_3d):
    # 假设这是你的三维 RGB 数组
    #array_3d_rgb = np.random.randint(0, 255, size=(1920, 1080, 3), dtype=np.uint8)

    # 保存为 PNG 文件
    cv2.imwrite('output.png', array_3d)

    print("RGB图像已保存为PNG格式")