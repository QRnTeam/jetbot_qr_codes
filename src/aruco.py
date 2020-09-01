#!/usr/bin/env python

import cv_bridge
import cv2
import numpy as np
import rospy

from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from jetbot_qr_codes.msg import Marker, Markers

from vision.arucoreader import ArucoReader

raw_camera_topic = rospy.get_param('raw_camera_topic')
markers_image_topic = rospy.get_param('markers_image_topic')
markers_topic = rospy.get_param('markers_topic')

class MarkersTopic(object):
    def __init__(self, mtx, dist, aruco_side_length):
        self._reader = ArucoReader(mtx, dist, aruco_side_length)
        self._cv_bridge = cv_bridge.CvBridge()

        self._image_sub = rospy.Subscriber(raw_camera_topic, Image, self.image_callback)

        self._processed_image_pub = rospy.Publisher(
                markers_image_topic, Image, queue_size=10)
        self._markers_pub = rospy.Publisher(markers_topic, Markers, queue_size=10)

    def image_callback(self, image):
        image = self._cv_bridge.imgmsg_to_cv2(image, "bgr8")
        image = cv2.flip(image, -1)

        ret, tvecs, ids, new_image = self._reader.detect_markers(image)
        
        if not np.any(new_image):
            new_image = image

        markers = Markers()
        markers.markers = []

        if np.any(ids):
            for i in range(len(ids)):
                pos = Vector3(
                    x=tvecs[i][0][0],
                    y=tvecs[i][0][1],
                    z=tvecs[i][0][2],
                )
                markers.markers.append(Marker(position=pos, id=ids[i]))

        # publish image and markers
        new_image = self._cv_bridge.cv2_to_imgmsg(new_image, "bgr8")
        self._processed_image_pub.publish(new_image)
        self._markers_pub.publish(markers)

if __name__ == '__main__':
    rospy.init_node("arucos")

    mtx = rospy.get_param('camera_matrix')
    dist = rospy.get_param('camera_dist')
    aruco_side_length = rospy.get_param('aruco_side_length')

    mtx = np.mat(mtx)
    dist = np.mat(dist)

    MarkersTopic(mtx, dist, aruco_side_length)
    rospy.spin()
