#!/usr/bin/env python

import rospy
import numpy as np
from .vision.ArUcoReader import ArucoReader

from sensor_msgs.msg import Image

mtx = rospy.get_param('camera_matrix')
dist = rospy.get_param('camera_dist')
raw_camera_topic = rospy.get_param('raw_camera_topic')

camera_matrix = np.mat(camera_matrix)
camera_dist = np.mat(camera_dist)

class MarkersTopic(object):
    def __init__(self):
        self._reader = ArucoReader(mtx, dist)
        self._image_sub = rospy.Subscriber(raw_camera_topic, Image, self.image_callback)

    def image_callback(self, image):
        pass
