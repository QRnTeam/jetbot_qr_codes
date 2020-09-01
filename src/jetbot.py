#!/usr/bin/env python
import rospy
import numpy as np

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from jetbot_qr_codes.msg import Markers

LIMITER_SIGNS = {
    #id: [front_limit, back_limit]
    0: (0.50, 0.50),
    1: (0.75, 0.75),
    2: (1.00, 1.00),
    # 3, 4 are reserved for special signs
}

def abs_clamp(val, _max):
    if val >= 0:
        val = min(_max, val)
    else:
        val = max(-_max, val)

    return val

cmd_raw_topic = rospy.get_param('cmd_raw_topic')
cmd_vel_topic = rospy.get_param('cmd_vel_topic')
turning_topic = rospy.get_param('turning_topic')
markers_topic = rospy.get_param('markers_topic')

class Jetbot(object):
    def __init__(self, wheelbase, wheel_radius, max_speed, left_adj, right_adj):
        self._wheelbase_meters = wheelbase
        self._wheel_radius_meters = wheel_radius
        self._max_speed = max_speed
        self._left_adj = left_adj
        self._right_adj = right_adj

        self._rate = rospy.Rate(2)

        self._vel = 0
        self._vel_sub = rospy.Subscriber(cmd_vel_topic, Twist, self.vel_callback)

        self._turn = 0
        self._turn_sub = rospy.Subscriber(turning_topic, Twist, self.turning_callback)

        self._markers = []
        self._markers_sub = rospy.Subscriber(markers_topic, Markers, self.markers_callback)

        # TODO: Move cmd_raw into this class.
        self._raw_pub = rospy.Publisher(cmd_raw_topic, String, queue_size=10)

    def vel_callback(self, twist):
        # we only use the linear.x, turning is provided by the turning topic
        rospy.loginfo("Received cmd_vel: {}".format(twist))
        self._vel = twist.linear.x

    def turning_callback(self, twist):
        rospy.loginfo("Received turning: {}".format(twist))
        # we only use the angular.z, velocity is provided by cmd_vel topic
        self._turn = twist.angular.z

    def markers_callback(self, markers):
        rospy.loginfo("Received markers: {}".format(markers))
        markers = markers.markers
        markers.sort(key=lambda x: x.position.z)
        self._markers = markers

    def run(self):
        while not rospy.is_shutdown():
            vel = self._vel

            rospy.logdebug("Velocity requested: {}".format(vel))

            # clamp to max speed
            vel = abs_clamp(vel, self._max_speed)

            rospy.logdebug("Velocity clamped to max: {}".format(vel))

            # find speed limit from id of closest marker
            front_limit, back_limit = (0, 0)
            if np.any(self._markers):
                marker = self._markers[0]
                front_limit, back_limit = LIMITER_SIGNS.get(marker.id, (0, 0))

            rospy.logdebug("Limits: {}, {}".format(front_limit, back_limit))

            # find speed limit based on distance from closest marker
            if np.any(self._markers):
                marker = self._markers[0]
                dist = marker.position.z

                # TODO: Move into config/*.yaml.
                stop_close, stop_far = 0.7, 1.5
                if dist < stop_close:
                    front_limit = 0
                elif dist < stop_far:
                    stop_limit = (dist - stop_close) / (stop_far - stop_close)
                    front_limit = min(stop_limit, front_limit)
                else:
                    front_limit = front_limit

            # apply speed limit
            if vel >= 0:
                vel = abs_clamp(vel, front_limit*self._max_speed)
            else:
                vel = abs_clamp(vel, back_limit*self._max_speed)

            rospy.logdebug("Velocity final: {}".format(vel))

            twist = Twist()
            twist.linear.x = vel
            # TODO: Fix and re-enable line tracking.
            # twist.angular.z = self._turn
            self.move(twist)
            self._rate.sleep()

    def move(self, twist):
        # Apply twist to two-wheeled differential drive robot.
        #
        # The output will be published to cmd_raw, which accepts the left 
        # and right motor power encoded as a comma-separated string.
        #
        # Example: "-1.0,0.5"
        #   (full reverse for left and half forward for right)
        v_l, v_r = self.unicycle(twist)
        speed_l, speed_r = abs(v_l), abs(v_r)

        if speed_l > 1.0 and speed_l > speed_r:
            v_l /= speed_l
            v_r /= speed_l
        elif speed_r > 1.0:
            v_l /= speed_r
            v_r /= speed_r

        v_l *= self._left_adj
        v_r *= self._right_adj

        cmd = "{},{}".format(v_l, v_r)

        self._raw_pub.publish(cmd)

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
    wheelbase = rospy.get_param('wheelbase_meters')
    wheel_radius = rospy.get_param('wheel_radius_meters')
    max_speed = rospy.get_param('max_speed')
    left_adj = rospy.get_param('left_adj')
    right_adj = rospy.get_param('right_adj')

    rospy.init_node('jetbot')
    jetbot = Jetbot(wheelbase, wheel_radius, max_speed, left_adj, right_adj)
    jetbot.run()

    rospy.spin()
