import numpy as np
import cv2
from collections import namedtuple

Point = namedtuple('Point', ['x', 'y'])

def draw(frame, points):
    point0 = Point(int(points[0][0]), int(points[0][1]))
    point1 = Point(int(points[1][0]), int(points[1][1]))
    point2 = Point(int(points[2][0]), int(points[2][1]))
    point3 = Point(int(points[3][0]), int(points[3][1]))

    cv2.line(frame, point0, point1, [0, 0, 255], 2)
    cv2.line(frame, point1, point2, [0, 0, 255], 2)
    cv2.line(frame, point2, point3, [0, 0, 255], 2)
    cv2.line(frame, point3, point0, [0, 0, 255], 2)

#CameraMtx = [[1.35007461e+03 0.00000000e+00 9.64381787e+02],[0.00000000e+00 1.34859409e+03 6.10803328e+02],[0.00000000e+00 0.00000000e+00 1.00000000e+00]]
#err = [[ 0.10485946 -0.24625137  0.00903166 -0.00257263 -0.09894589]]



img = cv2.imread('resource/2arucos.jpg')

arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
markers, ids, rejected = cv2.aruco.detectMarkers(img,arucoDict)

print(markers[1])
#print(ids)
#print(rejected)

for marks in markers:
    for corners in marks:
        print(corners)
        print('loop')
        draw(img, corners)

cv2.imshow("output",img)
cv2.waitKey(0)