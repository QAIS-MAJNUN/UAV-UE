import numpy as np
import cv2

def three_to_two(image):
    weights = np.array([1000000, 1000, 1])
    weighted_array = image * weights
    weighted_image = np.sum(weighted_array, axis=2)
    #print(image)
    #print(weighted_image.shape)
    #print(weighted_image)
    return weighted_image

def two_to_three(array_2d):
    # 假设这是你的二维数组
    #array_2d = np.random.randint(0, 1000000000, size=(1920, 1080))

    # 计算每个维度上的值
    dim1 = array_2d // 1000000
    dim2 = (array_2d // 1000) % 1000
    dim3 = array_2d % 1000

    # 将这些值堆叠在一起形成三维数组
    array_3d = np.stack((dim1, dim2, dim3), axis=-1)

    # 打印结果
    print("还原后的三维数组的形状:", array_3d.shape)

    # 如果你想检查一个具体的元素
    #print("一个元素的三维表示:", array_3d[0, 0])
    return array_3d

# filename = 'output.txt'
# np.savetxt(filename, weighted_image, delimiter=' ', fmt='%d')