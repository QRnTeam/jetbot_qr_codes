#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

cmd_raw_topic = rospy.get_param('cmd_raw_topic')
cmd_vel_topic = rospy.get_param('cmd_vel_topic')

class CmdVel:
    def __init__(self):
        self._raw = rospy.Publisher(cmd_raw_topic, String, queue_size=10)
        self._vel = rospy.Subscriber(cmd_vel_topic, Twist, self.callback)
        self._wheelbase_meters = 0.10
        self._wheel_radius_meters = 0.03

    def callback(self, twist):
        # Apply twist to two-wheeled differential drive robot.
        #
        # The output will be published to cmd_raw, which accepts the left 
        # and right motor power encoded as a comma-separated string.
        #
        # Example: "-1.0,0.5"
        #   (full reverse for left and half forward for right)
        rospy.loginfo("Received cmd_vel message: {}".format(twist))

        v_l, v_r = self.unicycle(twist)
        speed_l, speed_r = abs(v_l), abs(v_r)

        if speed_l > 1.0 and speed_l > speed_r:
            v_l /= speed_l
            v_r /= speed_l
        elif speed_r > 1.0:
            v_l /= speed_r
            v_r /= speed_r

        cmd = "{},{}".format(v_l, v_r)
        self._raw.publish(cmd)

    def unicycle(self, twist):
        # Apply unicycle model to two-wheeled differential drive robot.
        #
        # From http://faculty.salina.k-state.edu/tim/robotics_sg/Control/kinematics/unicycle.html
        #
        # Given:
        #   v: twist.linear.x   (in meters per second)
        #   w: twist.angular.z  (in radians per second)
        #   L: wheelbase width  (in meters per radian)
        #   R: wheel radius     (in meters per radian)
        #
        # Calculate:
        #   v_r, v_l: velocity for left and right wheel
        # 
        # Equations:
        #   v_r = ((2 * v) + (w * L)) / (2 * R)
        #   v_l = ((2 * v) - (w * L)) / (2 * R)
        v = twist.linear.x
        w = twist.angular.z
        L = self._wheelbase_meters
        R = self._wheel_radius_meters

        v_l = ((2 * v) - (w * L)) / (2 * R)
        v_r = ((2 * v) + (w * L)) / (2 * R)
        return v_l, v_r


if __name__ == '__main__':
    rospy.init_node('jetbot_cmd_vel')
    CmdVel()
    rospy.spin()
