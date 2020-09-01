#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

from vision.linefollower import LineFollower

raw_camera_topic = rospy.get_param('raw_camera_topic')
turning_topic = rospy.get_param('turning_topic')
turning_image_topic = rospy.get_param('turning_image_topic')

class TurningTopic(object):
    def __init__(self, lower_color, upper_color):
        self._cv_bridge = CvBridge()
        self._follower = LineFollower(lower_color, upper_color)
        self._raw_camera_sub = rospy.Subscriber(raw_camera_topic, Image, self.image_callback)
        self._turning_pub = rospy.Publisher(turning_topic, Twist, queue_size=10)
        self._turning_image_pub = rospy.Publisher(turning_image_topic, Image, queue_size=10)

    def image_callback(self, image):
        try:
            image = self._cv_bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
    	except CvBridgeError as e:
            rospy.logerr("could not decode imgmsg to cv2: {}".format(e))
            return

        # jetbot camera is upside down
        image = cv2.flip(image, -1)
        twist = Twist()
        twist.angular.z, new_image = self._follower.follow(image)
        new_image = self._cv_bridge.cv2_to_imgmsg(new_image, "bgr8")

        rospy.loginfo("Turning topic publishing: {}".format(twist))
        self._turning_pub.publish(twist)
        self._turning_image_pub.publish(new_image)

if __name__ == '__main__':
    rospy.init_node('turning')
    lower = np.array(rospy.get_param('lower_color'))
    upper = np.array(rospy.get_param('upper_color'))

    # convert from [359, 100, 100] to [255, 255, 255]
    conv  = np.array([255. / 360, 255. / 100, 255. / 100])
    lower = np.floor(lower * conv).astype('int')
    upper = np.ceil(upper * conv).astype('int')

    TurningTopic(lower, upper)
    rospy.spin()
