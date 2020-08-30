import numpy as np
import cv2

mtx = np.mat([[1.35007461e+03, 0.00000000e+00, 9.64381787e+02],[0.00000000e+00, 1.34859409e+03, 6.10803328e+02],[0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
dist = np.mat([[ 0.10485946, -0.24625137,  0.00903166, -0.00257263, -0.09894589]])

img = cv2.imread('resource/2arucos.jpg')

arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
markers, ids, rejected = cv2.aruco.detectMarkers(img,arucoDict)

rvecs,tvecs,objPoints = cv2.aruco.estimatePoseSingleMarkers(markers,0.0935,mtx,dist)

print(rvecs)
print('test')
print(tvecs)

cv2.aruco.drawDetectedMarkers(img, markers)

for i in range(len(rvecs)):
    cv2.aruco.drawAxis(img, mtx, dist, rvecs[i],tvecs[i],0.05)

cv2.imshow("output",img)
cv2.waitKey(0)