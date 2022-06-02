#! /usr/bin/env python

"""Republishes navigation commands at user-controllable speeds.
"""

import geometry_msgs.msg
import numpy as np
import rospy
import std_msgs.msg
import math

class NavigationController(object):
    def __init__(self, cmd_vel_pub, max_vel_pub):
        self._cmd_vel_pub = cmd_vel_pub
        self._max_vel_pub = max_vel_pub
        self._max_speed = 1.0 # In meters/second
        self._amplitude = 0.4
        self._frequency = 1/61.125
        self._vertical_shift = 0.1
        self._counter = 0

    def nav_cmd_vel_callback(self, twist):
        vec = np.array([twist.linear.x, twist.linear.y])
        commanded_speed = np.linalg.norm(vec)
        self._max_speed = self._amplitude*math.sin(self._frequency*self._counter)+self._vertical_shift + self._amplitude
        if commanded_speed > self._max_speed:
            squared_speed = commanded_speed * commanded_speed
            max_squared = self._max_speed * self._max_speed
            slow_factor = squared_speed / max_squared
            twist.linear.x /= slow_factor
            twist.linear.y /= slow_factor

        self._cmd_vel_pub.publish(twist)
        self._max_vel_pub.publish(np.linalg.norm(np.array([twist.linear.x, twist.linear.y])))
        self._counter+=1

    def max_speed_callback(self, msg):
        rospy.loginfo('Setting max speed to {}'.format(msg.data))
        self._max_speed = msg.data

def main():
    rospy.init_node('navigation_speed_controller')
    cmd_vel_pub = rospy.Publisher('cmd_vel', geometry_msgs.msg.Twist, queue_size=10)
    max_vel_pub = rospy.Publisher('max_speed', std_msgs.msg.Float32, queue_size=10)
    controller = NavigationController(cmd_vel_pub, max_vel_pub)
    sub = rospy.Subscriber('move_base_cmd_vel', geometry_msgs.msg.Twist, controller.nav_cmd_vel_callback)
    sub = rospy.Subscriber('navigation_controller/max_speed', std_msgs.msg.Float32, controller.max_speed_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
