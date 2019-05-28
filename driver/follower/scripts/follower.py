#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist


class WireHeadingController(object):
    '''
    This controller uses heading information from x coil to control the angular rate.
    y coil is used to detect the wire.
    w = pd(x / |(x,y)|)
    '''
    def __init__(self, onThreshold=0.2, pW=0.1, dW=0.0, pV=0.1):
        self.on_threshold = onThreshold
        self.p_w = pW
        self.d_w = dW
        self.p_v = pV
        self._last_heading = 0
        self._sub = rospy.Subscriber("perimeter_wire", Float32MultiArray, self._wireCb)
        self._pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

    def _wireCb(self, msg):
        cmd = Twist()

        power = abs(msg.data[3])
        if power >= self.on_threshold:
            heading = msg.data[1] / msg.data[8]
            diff = heading - self._last_heading
            self._last_heading = heading
            cmd.angular.z = self.p_w * heading + self.d_w * diff
            cmd.linear.x = self.p_v * 1.0
        else:
            rospy.logwarn_throttle(2.0, "Not on track")

        self._pub.publish(cmd)


class WireDistanceController(object):
    '''
    This controller estimates the ditance to the wire using y coil and sign of x coil.
    y coil is used  wire
    w = pd(d)
    '''
    def __init__(self, onThreshold=0.2, pW=0.1, dW=0.0, pV=0.1, maxY=0.1):
        self.on_threshold = onThreshold
        self.p_w = pW
        self.d_w = dW
        self.p_v = pV
        self._max_y = maxY
        self._last_dist = 0
        self._sub = rospy.Subscriber("perimeter_wire", Float32MultiArray, self._wireCb)
        self._pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

    def _wireCb(self, msg):
        cmd = Twist()

        y = abs(msg.data[3])
        if y > self._max_y:
            self._max_y = y

        if y >= self.on_threshold:
            x = msg.data[1]
            d = math.copysign(self._max_y - y, x)

            diff = d - self._last_dist
            self._last_dist = d
            cmd.angular.z = self.p_w * d + self.d_w * diff
            cmd.linear.x = self.p_v * 1.0
        else:
            rospy.logwarn_throttle(2.0, "Not on track")

        self._pub.publish(cmd)


def initNode():
    rospy.init_node('wire_follower')

    pW = rospy.get_param("~p_w", 0.2)
    dW = rospy.get_param("~d_w", 0.0)
    pV = rospy.get_param("~p_v", 0.1)
    onThreshold = rospy.get_param("~on_threshold", 0.02)
    follower = WireHeadingController(onThreshold=onThreshold, pW=pW, dW=dW, pV=pV)
    rospy.spin()

if __name__ == '__main__':
    initNode()
