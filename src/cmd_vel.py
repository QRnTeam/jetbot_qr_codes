#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class CmdVel:
    def __init__(self):
        self._raw = rospy.Publisher('/jetbot_motors/cmd_raw', String, queue_size=10)
        self._vel = rospy.Subscriber('/cmd_vel', Twist, self.callback)

    def callback(self, msg):
        rospy.loginfo("Received /cmd_vel message: {}".format(msg))

        # TODO: Apply kinematics of two-wheeled differential drive robot.
        #
        #   The output will be published to cmd_raw, which accepts the left 
        #   and right motor power encoded as a comma-separated string.
        #
        #     Example: "-1.0,0.5"
        #       (full reverse for left and half forward for right)

if __name__ == '__main__':
    rospy.init_node('~')
    CmdVel()
    rospy.spin()
