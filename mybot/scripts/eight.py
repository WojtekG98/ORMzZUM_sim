#!/usr/bin/env python
import rospy
import sys
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import time
import math
vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
x = 0
y = 0
z = 0
roll = 0
pitch = 0
yaw = 0

def get_position(msg):
    global x, y, z, roll, pitch, yaw
    orientation_q = msg.pose.pose.orientation
    position = msg.pose.pose.position
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    x, y, z = position.x, position.y, position.z
    print("yaw:", abs(yaw))

def mybot_velocity_commands():
  # Velocity publisher
  global vel_pub, x, y, z, roll, pitch, yaw


  msg = Twist()
  msg.linear.x = 0.5
  msg.linear.y = 0
  msg.linear.z = 0
  msg.angular.x = 0
  msg.angular.y = 0
  msg.angular.z = 0
  time_to_move_forward = 3
  for i in range(1, 5):
    t_end = time.time() + time_to_move_forward 
    while time.time() < t_end:
      vel_pub.publish(msg)
    
    msg.angular.z = -1.4
    msg.linear.x = 0.3
    vel_pub.publish(msg)
    while abs(yaw) < math.pi - 0.1:
      vel_pub.publish(msg)
    while abs(yaw) > 2.356:
      vel_pub.publish(msg)
    
    msg.angular.z = 0
    msg.linear.x = 0.5
    t_end = time.time() + 2*time_to_move_forward 
    while time.time() < t_end:
      vel_pub.publish(msg)
    
    msg.angular.z = 1.4
    msg.linear.x = 0.3
    vel_pub.publish(msg)
    while abs(yaw) > 0.1:
      vel_pub.publish(msg)
    #while abs(yaw) < 0.785:
    #  vel_pub.publish(msg)
    
    msg.angular.z = 0
    msg.linear.x = 0.5
    t_end = time.time() + time_to_move_forward 
    while time.time() < t_end:
      vel_pub.publish(msg)
  
  msg.linear.x = 0
  vel_pub.publish(msg)

if __name__ == '__main__':
  rospy.init_node('mybot_cmd_vel', anonymous=True)
  sub = rospy.Subscriber ('/mybot/state', Odometry, get_position)
  mybot_velocity_commands()

