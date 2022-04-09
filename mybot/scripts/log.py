#!/usr/bin/env python
import rospy
import sys
from math import sqrt
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from datetime import datetime
new_odom = False
new_scan = False
odom = None
scan = None
original_stdout = sys.stdout
path = "/home/wojtek/robot_laser/src/mybot/rosbag_recordings/"
time_stamp = datetime.now().strftime("%d_%m_%Y_%H_%M_%S")
linear_log_file = path + "lin_" + time_stamp + ".csv"
angular_log_file = path + "ang_" + time_stamp + ".csv"
linear_file = open(linear_log_file, 'w')
angular_file = open(angular_log_file, 'w')


def listener():
    rospy.init_node('log')  #node
    rospy.Subscriber("/mybot/state", Odometry, log_odometry)  #subcribin to topic1
    rospy.Subscriber("/mybot/laser_scan", LaserScan, log_scan)  #subscribing to topic2

def log_odometry(msg):
    #print("odom")
    global odom, new_odom
    odom = msg
    new_odom = True

def log_scan(msg):
    #print("scan")
    global scan,new_scan
    scan = msg
    new_scan = True

def logtofile():
    global scan, odom, new_scan, new_odom, original_stdout
    if new_scan and new_odom:
        # print(len(scan.ranges))
        sys.stdout = linear_file
        print(sqrt(odom.twist.twist.linear.x**2+odom.twist.twist.linear.y**2), ",", scan.ranges)
        sys.stdout = angular_file
        print(odom.twist.twist.angular.z,",", scan.ranges)
        sys.stdout = original_stdout
        #print("scan: ", scan.ranges)
        #print("\nodom: ", odom.pose.pose)
        new_scan = False
        new_odom = False

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass

    while True:
        logtofile()
        #rospy.sleep(0.1)
