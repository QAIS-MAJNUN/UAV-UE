

import cv2
from collections import deque

from rpg.rgb_input_png import three_to_two, two_to_three
from rpg.rgb_output_png import png_output

background_rgb = 6108153
out_of_range_rgb = 256256256

def bfs(start_x, start_y, matrix, vis):
    n, m = matrix.shape
    q = deque()
    q.append((start_x, start_y))
    area = []
    color = set()
    vis[start_x][start_y] = True
    base_color = matrix[start_x][start_y]
    color.add(base_color)
    cnt = 0
    while q:
        x, y = q.popleft()
        cnt += 1
        area.append((x, y))
        for dx, dy in [[1, 0], [-1, 0], [0, 1], [0, -1]]:
            a, b = x + dx, y + dy
            if (0 <= a < n and 0 <= b < m):
                color.add(matrix[a][b])
                if (matrix[a][b] == base_color and not vis[a][b]):
                    vis[a][b] = True
                    area.append((a, b))
                    q.append((a, b))
            else:
                color.add(out_of_range_rgb)
    color.discard(base_color)
    # color.discard(background_rgb)
    # print(cnt, base_color, len(color))
    return area, color



def merge(matrix):
    n, m = matrix.shape
    vis = [[False for j in range(m)]for i in range(n)]
    for i in range(n):
        for j in range(m):
            if vis[i][j]:
                continue
            area, color = bfs(i, j, matrix, vis)
            if len(color) == 1:
                new_color = list(color)[0]
                if new_color == background_rgb or new_color == out_of_range_rgb:
                    continue
                print("Yes, change")
                cnt = 0
                #print(len(area))
                for x, y in area:
                    #print(x, y)
                    cnt += 1
                    matrix[x][y] = new_color
                print("将%d个点修改为 %d" %(cnt, new_color))
    return matrix

image_path = "test.png"
#读入需要处理的图片
image = cv2.imread(image_path)

print(image.shape)
# 3维rgb映射为2维
rgb_matrix = three_to_two(image)

background_rgb = rgb_matrix[0][0]
print(background_rgb)
# for i in range(100):
#     for j in range(100):
#         rgb_matrix[i][j] = 100100100

# 合并处理
new_rgb_matrix = merge(rgb_matrix)

# 2维转3维rgb
new_image = two_to_three(rgb_matrix)

# 输出图像
png_output(new_image)

#print(image == new_image)

#png_output(new_rgb_matrix)
