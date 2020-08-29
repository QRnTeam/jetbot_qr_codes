#!/usr/bin/env python
import rospy
from Adafruit_MotorHAT import Adafruit_MotorHAT 
from std_msgs.msg import String

class CmdRaw(object):
    def __init__(self):
        self._driver = Adafruit_MotorHAT(i2c_bus=1)
        self._channels = [[1, 0], [2, 3]]
        self._motors = {
            "left": {
                "motor": self._driver.getMotor(1),
                "ina": 1,
                "inb": 0,
            },
            "right": {
                "motor": self._driver.getMotor(2),
                "ina": 2,
                "inb": 3,
            },
        }
        self._sub = rospy.Subscriber("/cmd_raw", String, self.callback)


    def set_speed(self, name, value):
        mapped_value = int(255.0 * value)
        speed = min(max(abs(mapped_value), 0), 255)
        motor = self._motors[name]
        motor["motor"].setSpeed(speed)

        if mapped_value < 0:
            motor["motor"].run(Adafruit_MotorHAT.FORWARD)
            self._driver._pwm.setPWM(motor["ina"],0,0)
            self._driver._pwm.setPWM(motor["inb"],0,speed*16)
        else:
            motor["motor"].run(Adafruit_MotorHAT.BACKWARD)
            self._driver._pwm.setPWM(motor["ina"],0,speed*16)
            self._driver._pwm.setPWM(motor["inb"],0,0)


    # raw L/R motor commands (speed, speed)
    def callback(self, msg):
        # msg.data = "-1.0,0.5"
	rospy.loginfo(rospy.get_caller_id() + ' cmd_raw=%s', msg.data)

        speeds = msg.data.split(',')
        if len(speeds) != 2:
            rospy.logerr(rospy.get_caller_id() + ' invalid cmd_raw="%s", should be in form "-1.0,0.5"', msg.data)
            return

        try:
            left, right = float(speeds[0]), float(speeds[1])
        except ValueError:
            rospy.logerr(rospy.get_caller_id() + ' invalid cmd_raw="%s", could not parse as float', msg.data)
            return

        if not (-1.0 <= left <= 1.0) or not (-1.0 <= right <= 1.0):
            rospy.logerr(rospy.get_caller_id() + ' invalid cmd_raw="%s", values must be between -1.0 and 1.0', msg.data)
            return

        rospy.loginfo(rospy.get_caller_id() + ' cmd_dir speeds left=%f, right=%f', left, right)
        self.set_speed("left", left)
        self.set_speed("right", right)

        
if __name__ == '__main__':
    rospy.init_node("~", log_level=rospy.INFO)
    CmdRaw()
    rospy.spin()
