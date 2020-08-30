#!/usr/bin/env python

import rospy
from std_msgs.msg import String

cmd_str_topic = rospy.get_param('cmd_str_topic')
cmd_raw_topic = rospy.get_param('cmd_raw_topic')

class CmdStr(object):
    def __init__(self):
        self._pub = rospy.Publisher(cmd_raw_topic, String, queue_size=10)
        self._sub = rospy.Subscriber(cmd_str_topic, String, self.callback)
        self._speed = 0.5

    def callback(self, msg):
        cmd = msg.data
	rospy.loginfo(rospy.get_caller_id() + ' cmd_str=%s', cmd)

        commands = {
            'forward': [self._speed, self._speed],
            'backward': [-self._speed, -self._speed],
            'left': [self._speed, -self._speed],
            'right': [-self._speed, self._speed],
            'stop': [0, 0],
            }

        speeds = commands.get(cmd, None)
        if speeds == None:
            rospy.logerr(rospy.get_caller_id() + ' cmd_str={}, not recognized, must be in commands: {}'.format(cmd, commands.keys()))
            return

        left, right = speeds
        self._pub.publish("{},{}".format(left, right))

if __name__ == '__main__':
    rospy.init_node("jetbot_cmd_str")
    CmdStr()
    rospy.spin()
