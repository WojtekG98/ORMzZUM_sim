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
    points = []
    for i in range(int((len(sys.argv)-1)/2)):
      points.append((float(sys.argv[2*(i+1)-1]), float(sys.argv[2*(i+1)])))
    print("points:", points)
    e = 0.1
    vel_fast = 0.1
    vel_slow = 0.1#0.05
    vel_fast_angular = 0.1
    vel_slow_angular = 0.1#0.05
    x, y, z = 0, 0, 0
    roll, pitch, yaw = 0, 0, 0
    rospy.init_node('mybot_to_point_control')
    sub = rospy.Subscriber ('/odom', Odometry, get_position)
    vel_pub = rospy.Publisher('/mybot/cmd_vel', Twist, queue_size=2)
    r = rospy.Rate(10)
    msg = Twist()
    msg.linear.x = 0
    msg.linear.y = 0
    msg.linear.z = 0
    msg.angular.x = 0
    msg.angular.y = 0
    for point in points:
      des_x = float(point[0])
      des_y = float(point[1])
      print("Now going to: ", des_x, des_y)
      msg.angular.z = vel_fast_angular
      vel_pub.publish(msg)
      while abs(yaw - angle_between((x,y), (des_x, des_y))) > e*e:
        if abs(yaw - angle_between((x,y), (des_x, des_y))) < e:
          msg.angular.z = vel_slow_angular
          vel_pub.publish(msg)
        else:
          msg.angular.z = vel_fast_angular
          vel_pub.publish(msg)
        r.sleep()
      #  vel_pub.publish(msg)
      msg.angular.z = 0
      vel_pub.publish(msg)
      msg.linear.x = vel_fast
      vel_pub.publish(msg)
      while (x - float(des_x ))*(x - float(des_x)) + (y - float(des_y))*(y - float(des_y)) > e*e:
        if (x - float(des_x))*(x - float(des_x)) + (y - float(des_y))*(y - float(des_y)) < 4*e*e:
          msg.linear.x = vel_slow
          vel_pub.publish(msg)
        else:
          msg.linear.x = vel_fast
          vel_pub.publish(msg)
        r.sleep()
      msg.linear.x = 0
      vel_pub.publish(msg)
