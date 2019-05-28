#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist


class WireFollower(object):
    def __init__(self, on_threshold=0.2, pW=0.1, pV=0.1):
        self.on_threshold = on_threshold
        self.p_w = pW
        self.p_v = pV
        self._sub = rospy.Subscriber("perimeter_wire", Float32MultiArray, self._wireCb)
        self._pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

    def _wireCb(self, msg):
        cmd = Twist()
        zr = msg.data[2]
        zl = msg.data[1]
        y = msg.data[3]
        x = msg.data[0]

        zdiff = zl - zr
        power = math.sqrt(x ** 2 + y ** 2)
        rospy.loginfo("zr: %f, zl: %f, zdiff: %f, y: %f, x: %f, power: %f", zr, zl, zdiff, y, x, power)
        if power >= self.on_threshold:
            cmd.angular.z = self.p_w * zdiff
            cmd.linear.x = self.p_v * power
            self._pub.publish(cmd)
        else:
            rospy.logwarn_throttle(2.0, "Not on track")

def initNode():
    rospy.init_node('wire_follower')

    pW = rospy.get_param("~p_w", 0.2)
    pV = rospy.get_param("~p_v", 0.2)
    follower = WireFollower(pW=pW, pV=pV)
    rospy.spin()

if __name__ == '__main__':
    initNode()
