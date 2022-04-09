#!/usr/bin/env python
import rospy
import sys
import tensorflow as tf
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
new_scan = False
scan = None
last_scan = None
model = None
ang_vel_pub = rospy.Publisher('/mybot/ang_vel_ml', Float64, queue_size=10)

def listener():
    global model
    rospy.init_node('ang_vel_ml')  #node
    rospy.Subscriber("/mybot/laser_scan", LaserScan, log_scan)  #subscribing to topic
    model = tf.keras.models.load_model("/home/wojtek/robot_laser/src/mybot/scripts/ang_vel_regress.model")
    # print(model.summary())

def log_scan(msg):
    global scan,new_scan, last_scan
    last_scan = scan
    scan = msg
    new_scan = True

def printall():
    global scan, new_scan, last_scan
    if new_scan and last_scan is not None:
        all_diffs = []
        for i in range(0, len(scan.ranges)):
            all_diffs.append(scan.ranges[i] - last_scan.ranges[i])
        ind = [x for x in range(0, 720, int(720/8))]
        eight_diffs = [all_diffs[i] for i in ind]
        #txt = [str(x) for x in eight_diffs]
        #print("\n-------\n", txt, "\n-------\n")
        predict_vel(eight_diffs)
        new_scan = False

def predict_vel(lengths):
    global model, ang_vel_pub
    lengths = np.array([lengths])
    predict = model.predict(lengths)
    ang_vel_pub.publish(predict[0][0])

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass

    while True:
        printall()
        rospy.sleep(0.1)

