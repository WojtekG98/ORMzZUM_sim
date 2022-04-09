#!/usr/bin/env python
import rospy
import sys
from geometry_msgs.msg import Twist

def mybot_velocity_commands(velocity, time):
  # Velocity publisher
  vel_pub = rospy.Publisher('/mybot/cmd_vel', Twist, queue_size=10)
  rospy.init_node('mybot_cmd_vel', anonymous=True)

  msg = Twist()
  msg.linear.x = float(velocity)
  msg.linear.y = 0
  msg.linear.z = 0
  msg.angular.x = 0
  msg.angular.y = 0
  msg.angular.z = 0
  while vel_pub.get_num_connections() < 1:
     pass
     # wait for a connection to publisher
     # you can do whatever you like here or simply do nothing

  vel_pub.publish(msg)
  rospy.sleep(int(round(float(time))))
  msg.linear.x = 0
  vel_pub.publish(msg)

if __name__ == '__main__':
  if len(sys.argv) == 3:
    try:
      mybot_velocity_commands(sys.argv[1], sys.argv[2])      
    except rospy.ROSInterruptException:
      pass
