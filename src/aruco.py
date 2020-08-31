#!/usr/bin/env python

import rospy
import cv_bridge
from sensor_msgs.msg import Image

import numpy as np
from .vision.arucoreader import ArucoReader

mtx = rospy.get_param('camera_matrix')
dist = rospy.get_param('camera_dist')
aruco_side_length = rospy.get_param('aruco_side_length')

raw_camera_topic = rospy.get_param('raw_camera_topic')

camera_matrix = np.mat(camera_matrix)
camera_dist = np.mat(camera_dist)

class MarkersTopic(object):
    def __init__(self):
        self._reader = ArucoReader(mtx, dist, aruco_side_length)
        self._image_sub = rospy.Subscriber(raw_camera_topic, Image, self.image_callback)

    def image_callback(self, image):
        image = cv_bridge.CvBridge().imgmsg_to_cv2(image)
        cv2.imshow('live', image)
        cv2.waitKey(0)

if __name__ == '__main__':
    rospy.init_node("arucos")
    MarkersTopic()
    rospy.spin()
