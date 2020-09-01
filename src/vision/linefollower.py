#!/usr/bin/env python

import rospy
import cv2
import numpy as np

def follow(image):
    # based heavily on code from Robot Ignite Academy
    #
    # image should be BGR-encoded for opencv
    height, width, channels = image.shape

    mid = height / 2
    image = image[mid:,:]

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV).astype(np.float)

    lower_color = np.array([20, 100, 100])
    upper_color = np.array([50, 255, 255])
    
    mask = cv2.inRange(hsv, lower_color, upper_color)
    m = cv2.moments(mask, False)
    try:
        cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
    except ZeroDivisionError:
        cx, cy = width/2, height/2

    res = cv2.bitwise_and(image, image, mask=mask)
    cv2.circle(res, (int(cx), int(cy)), 10, (0, 0, 255), -1)

    return _heading(cx, width)

def _heading(cx, width):
    error_x = cx - width / 2;
    angular_z = -error_x / 100;
    return -error_x / 100;

if __name__ == '__main__':
    import sys

    image_path = sys.argv[1]
    print("processing image: {}".format(image_path))

    image = cv2.imread(image_path)
    image = cv2.flip(image, 1)
    heading = follow(image)

    print("proposed heading from image: {}".format(heading))
