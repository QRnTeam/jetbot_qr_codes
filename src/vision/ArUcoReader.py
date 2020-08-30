import numpy as np
import cv2

mtx = np.mat([[1.35007461e+03, 0.00000000e+00, 9.64381787e+02],[0.00000000e+00, 1.34859409e+03, 6.10803328e+02],[0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
dist = np.mat([ 0.10485946, -0.24625137, 0.00903166, -0.00257263, -0.09894589])
parameters = cv2.aruco.DetectorParameters_create()
arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

cam =cv2.VideoCapture(1)

while True:
    ret, frame = cam.read()
    markers, ids, rejected = cv2.aruco.detectMarkers(image=frame,dictionary=arucoDict,cameraMatrix=mtx,distCoeff=dist)

    print(ids)

    cv2.aruco.drawDetectedMarkers(frame,markers)

    cv2.imshow("Live", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

