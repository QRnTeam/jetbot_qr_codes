#!/usr/bin/env python

import cv_bridge
import cv2
import numpy as np
import rospy

from sensor_msgs.msg import Image

from vision.arucoreader import ArucoReader

raw_camera_topic = rospy.get_param('raw_camera_topic')

class MarkersTopic(object):
    def __init__(self, mtx, dist, aruco_side_length):
        self._reader = ArucoReader(mtx, dist, aruco_side_length)
        self._image_sub = rospy.Subscriber(raw_camera_topic, Image, self.image_callback)

    def image_callback(self, image):
        image = cv_bridge.CvBridge().imgmsg_to_cv2(image)

        rospy.logerr("Shape: {}".format(image.shape))

        ret, markers, new_image = self._reader.detect_markers(image)

        if ret:
            image = new_image


if __name__ == '__main__':
    rospy.init_node("arucos")

    mtx = rospy.get_param('camera_matrix')
    dist = rospy.get_param('camera_dist')
    aruco_side_length = rospy.get_param('aruco_side_length')

    mtx = np.mat(mtx)
    dist = np.mat(dist)

    MarkersTopic(mtx, dist, aruco_side_length)
    rospy.spin()
