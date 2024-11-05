import cv2
import numpy as np


def cmp(a, b):
    for i in range(3):
        if a[i] != b[i]:
            return False
    return True


if __name__ == "__main__":
    # img = cv2.imread(r"E:\__WORKSPACE__\Python\UAV\demo\seg_color_palette.png")
    # img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    # color = np.array(img).flatten().reshape(-1, 3).tolist()
    # tmp = [-1,-1,-1]
    # res = []
    # for i in range(len(color)):
    #     if cmp(tmp, color[i]):
    #         continue
    #     else:
    #         res.append(color[i])
    #         tmp = color[i]
    # np.save(r"E:\__WORKSPACE__\Python\UAV\demo\color.npy", np.array(res))
    color = np.load(r"E:\__WORKSPACE__\Python\UAV\demo\palette.npy")
    print(color)