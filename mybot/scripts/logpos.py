#!/usr/bin/env python
import rospy
import sys
from math import sqrt, atan2, asin
from nav_msgs.msg import Odometry
from datetime import datetime
new_odom = False
odom = None
original_stdout = sys.stdout
path = "/home/wojtek/robot_laser/src/mybot/rosbag_recordings/"
time_stamp = datetime.now().strftime("%d_%m_%Y_%H_%M_%S")
filename = path + "pos_" + time_stamp + ".csv"
file_opened = open(filename, 'w')


def euler_from_quaternion(orientation):
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = atan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

def listener():
    rospy.init_node('logpos')  #node
    rospy.Subscriber("/mybot/state", Odometry, log_odometry)  #subcribin to topic

def log_odometry(msg):
    #print("odom")
    global odom, new_odom
    odom = msg
    new_odom = True

def logtofile():
    global odom, new_odom, original_stdout
    if new_odom:
        # print(len(scan.ranges))
        euler_angles = euler_from_quaternion(odom.pose.pose.orientation)
        sys.stdout = file_opened
        print(odom.pose.pose.position.x, ",", odom.pose.pose.position.y, ",", euler_angles[2])
        sys.stdout = original_stdout
        #print("scan: ", scan.ranges)
        #print("\nodom: ", odom.pose.pose)
        new_odom = False

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass

    while True:
        logtofile()
        #rospy.sleep(0.1)
