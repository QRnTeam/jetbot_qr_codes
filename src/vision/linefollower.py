#!/usr/bin/env python

import rospy
import cv2
import numpy as np

class LineFollower(object):
    def __init__(self, lower, upper):
        # hsv filters should be provided as np.array([h, s, v])
        #
        # Args:
        #   lower: the lower range for hsv filtering
        #   upper: the upper range for hsv filtering
        self._lower = lower
        self._upper = upper

    def follow(self, image):
        # based heavily on code from Robot Ignite Academy
        #
        # image should be BGR-encoded for opencv
        height, width, channels = image.shape

        mid = height / 2
        image = image[mid:,:]

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV).astype(np.float)

        lower = self._lower
        upper = self._upper
        
        # TODO: Filter to make sure we have found a good portion of line, otherwise just go straight.
        mask = cv2.inRange(hsv, lower, upper)
        m = cv2.moments(mask, False)
        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
        except ZeroDivisionError:
            cx, cy = width/2, height/2

        res = cv2.bitwise_and(image, image, mask=mask)
        cv2.circle(res, (int(cx), int(cy)), 10, (0, 0, 255), -1)

        # calculate heading
        error_x = cx - width / 2;
        return -error_x / 100;

if __name__ == '__main__':
    import sys

    image_path = sys.argv[1]
    print("processing image: {}".format(image_path))

    image = cv2.imread(image_path)
    image = cv2.flip(image, 1)

    lower = np.array([20, 100, 100])
    upper = np.array([50, 255, 255])
        
    follower = LineFollower(lower, upper)
    heading = follower.follow(image)

    print("proposed heading from image: {}".format(heading))
