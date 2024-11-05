import cv2
import sys
import math
import numpy as np

outputFile = "cloud.asc" 
rawImage = "DepthPerspective1.png"
color = (0,255,0)
rgb = "%d %d %d" % color
projectionMatrix = np.array([[-0.501202762, 0.000000000, 0.000000000, 0.000000000],
                              [0.000000000, -0.501202762, 0.000000000, 0.000000000],
                              [0.000000000, 0.000000000, 10.00000000, 100.00000000],
                              [0.000000000, 0.000000000, -10.0000000, 0.000000000]])
   
def savePointCloud(image, fileName):
   with open(fileName, "w") as f:
    for x in range(image.shape[0]):
        for y in range(image.shape[1]):
            pt = image[x,y]
            if (math.isinf(pt[0]) or math.isnan(pt[0])):
            # skip it
                None
            else: 
                f.write("%f %f %f %s\n" % (pt[0], pt[1], pt[2]-1, rgb))

if __name__ == "__main__":
    with open(rawImage, "rb") as f:
        rawImage = f.read()
        png = cv2.imdecode(np.frombuffer(rawImage, np.uint8) , cv2.IMREAD_UNCHANGED)
        gray = cv2.cvtColor(png, cv2.COLOR_BGR2GRAY)
        Image3D = cv2.reprojectImageTo3D(gray, projectionMatrix)
        savePointCloud(Image3D, outputFile)
        print("saved " + outputFile)
