#!/usr/bin/env python
import rospy
import sys
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import atan2

def angle_between(p1, p2):
    xDiff = p2[0] - p1[0]
    yDiff = p2[1] - p1[1]
    return atan2(yDiff, xDiff)

def get_position(msg):
    global x, y, z, roll, pitch, yaw
    orientation_q = msg.pose.pose.orientation
    position = msg.pose.pose.position
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    x, y, z = position.x, position.y, position.z

def go_forward():
   global x, y, z, roll, pitch, yaw
   print("tmp")

if __name__ == '__main__':
  if len(sys.argv) == 3:
    des_x = float(sys.argv[1])
    des_y = float(sys.argv[2])
    e = 0.1
    x, y, z = 0, 0, 0
    roll, pitch, yaw = 0, 0, 0
    rospy.init_node('mybot_to_point_control')
    sub = rospy.Subscriber ('/odom', Odometry, get_position)
    vel_pub = rospy.Publisher('/mybot/cmd_vel', Twist, queue_size=10)
    r = rospy.Rate(10)
    msg = Twist()
    msg.linear.x = 0
    msg.linear.y = 0
    msg.linear.z = 0
    msg.angular.x = 0
    msg.angular.y = 0
    msg.angular.z = 0.1
    while abs(yaw - angle_between((x,y), (des_x, des_y))) > 0.01:
      print(abs(yaw - angle_between((x,y),(des_x, des_y))))
      vel_pub.publish(msg)
    msg.angular.z = 0
    vel_pub.publish(msg)
    msg.linear.x = 0.1
    while (x - float(sys.argv[1]))*(x - float(sys.argv[1])) + (y - float(sys.argv[2]))*(y - float(sys.argv[2])) > e*e:
      print(x)
      vel_pub.publish(msg)
      r.sleep()
    msg.linear.x = 0
    vel_pub.publish(msg)
  else:
    print("Need two arguments: x,y")
