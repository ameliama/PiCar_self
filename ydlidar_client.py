#!/usr/bin/env python

import rospy
import serial
import time
from sensor_msgs.msg import LaserScan

arduino = serial.Serial('/dev/ttyACM0', 9600)

def rad2deg(x):
    return (x*180/3.14159)

def callback(scan):
    #print len(scan.ranges)

    count = int(scan.scan_time / scan.time_increment)
    #create and initialize array for mean filter
    #store the mean of current value + 9 previous values into the current index
    filter = [0]*count
    #printf("[YDLIDAR INFO]: I heard a laser scan %s[%d]:\n", scan->header.frame_id.c_str(), count);
    #printf("[YDLIDAR INFO]: angle_range : [%f, %f]\n", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));


    for i in range (0, count):
        sum = scan.ranges[i];
        i_copy = i;
        for j in range (0, 9):
            i_copy = i_copy - 1
            if i_copy < 0:
                i_copy = count - 1
            sum = sum + filter[i_copy]
        sum = sum / 10
        filter[i] = sum
        degree = rad2deg(scan.angle_min + scan.angle_increment*i)
        if (degree >= 1 and degree <= 1.5):
            if (filter[i] < 0.1 and filter [i-1] < 0.1 and filter [i-2] < 0.1):
                print ("stuff")
                arduino.write(b'2')
                time.sleep(1)
            else: 
                print ("nothing")



def ydlidar_client():
    rospy.init_node('ydlidar_client')
    rospy.Subscriber("scan", LaserScan, callback)
    rospy.spin()

if __name__ == '__main__':
    ydlidar_client()
