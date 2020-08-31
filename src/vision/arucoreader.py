import numpy as np
import cv2

class ArucoReader(object):
    def __init__(self, mtx, dist, side_length):
        self._mtx = mtx
        self._dist = dist
        self._side_length = side_length
        self._dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

    def detect_markers(self, image):
        markers, ids, rejected = cv2.aruco.detectMarkers(image=image, dictionary=self._dict, cameraMatrix=self._mtx, distCoeff=self._dist)

        if not np.any(markers):
            return False, None, None, None

        rvecs, tvecs, objPoints = cv2.aruco.estimatePoseSingleMarkers(markers, self._side_length, self._mtx, self._dist)

        if not np.any(tvecs):
            return False, None, None, None

        assert (len(markers) == len(ids) == len(tvecs)), "Must have same number of markers, ids and tvecs"

        image_copy = np.copy(image)
        cv2.aruco.drawDetectedMarkers(image_copy, markers)
        for i in range(len(ids)):
            cv2.aruco.drawAxis(image_copy, self._mtx, self._dist, rvecs[i], tvecs[i], 0.05)

        return True, tvecs, ids, image_copy

if __name__ == '__main__':
    mtx = np.mat([[1.35007461e+03, 0.00000000e+00, 9.64381787e+02],[0.00000000e+00, 1.34859409e+03, 6.10803328e+02],[0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
    dist = np.mat([ 0.10485946, -0.24625137, 0.00903166, -0.00257263, -0.09894589])
    side_length = 0.0935

    reader = ArucoReader(mtx, dist, side_length)

    cam = cv2.VideoCapture(0)
    while True:
        ret, image = cam.read()
        if not ret:
            continue

        ret, tvecs, ids, image_with_markers = reader.detect_markers(image)
        if ret:
            image = image_with_markers

        cv2.imshow("Live", image)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
