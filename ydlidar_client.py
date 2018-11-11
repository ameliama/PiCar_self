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

    output = "nothing"
    for i in range (0, count):
#        time.sleep(1)
        filter[i] = scan.ranges[i]
        degree = rad2deg(scan.angle_min + scan.angle_increment*i)
        if (degree >=1 and degree < 1.5):
            if (filter[i] < 0.4 and filter [i-1] < 0.4 and filter [i-2] < 0.4):
                if (output == "nothing"):
                    output = "stuff"
                    arduino.write(b'1')
#                time.sleep(1)
            else: 
                if (output == "stuff"):
                    output = "nothing"
		    arduino.write(b'2')
#                time.sleep(1)
            print(output)


def ydlidar_client():
    rospy.init_node('ydlidar_client')
    rospy.Subscriber("scan_filtered", LaserScan, callback)
    rospy.spin()

if __name__ == '__main__':
    ydlidar_client()
