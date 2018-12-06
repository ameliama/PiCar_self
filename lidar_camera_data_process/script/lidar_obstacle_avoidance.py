#!/usr/bin/env python

import rospy
import serial
import time
from sensor_msgs.msg import LaserScan

#Setup arduino port for the communication
arduino = serial.Serial('/dev/ttyACM0', 9600)

#Define some constants for later use:

#OUTPUT: is there anything in front of the lidar? 
NOTHING = 0
SOMETHING = 1
#initialize OUTPUT to be NOTHING 
OUTPUT = NOTHING

#LAST_TURN: which direction did it turn last time? 
#This is a really simple algorithm that only turns the car to the direction opposite to the last turning direction when some obstacle is detected in front of the car.
RIGHT = 0
LEFT = 1
#Initialize LAST_TURN to be RIGHT so it always turns left first
LAST_TURN = RIGHT

#The threshold distance for "seeing" things (how close the obstacle needs to be for Lidar to detect it)
DISTANCE = 1.3

#Function to convert radius to degeree
def rad2deg(x):
    return (x*180/3.14159)

#The scan function 
def callback(scan):
    #Need to define all the global variables
    global NOTHING
    global SOMETHING
    global OUTPUT
    global RIGHT
    global LEFT
    global LAST_TURN
    global DISTANCE

    #Number of sample points taken in from the Lidar. The values are already filtered by the filter node.
    count = int(scan.scan_time / scan.time_increment)
    #Create an array to store filtered values, initialized to all 0s 
    filter = [0]*count
    
    #For each sample point:
    for i in range (0, count):
        #Take the current value from the LaserScan Median filter
        filter[i] = scan.ranges[i]
        #Calculate degree from radius
        degree = rad2deg(scan.angle_min + scan.angle_increment*i)
        #Again, this is a really simple algorithm and it only examines the distance right in front of the Lidar
        if (degree >= 1 and degree < 1.5):
            #If three consecutive distances are smaller than a certain amount 
            #This is used to make sure that something is detected, safer than only examining one value
            if (filter[i] < DISTANCE and filter [i-1] < DISTANCE and filter [i-2] < DISTANCE):
                #Write to Arduino and change OUTPUT, only if there is a change in the state 
                #Because Arduino will not work correctly if data is sent from the Pi too frequently 
                #Changing from nothing detected to something detected 
                if (OUTPUT == NOTHING):
                    OUTPUT = SOMETHING
                    #Determine which way should it turn
                    if (LAST_TURN == LEFT):
                        LAST_TURN = RIGHT
                        #2 stands for "turn right". Serial communication only takes in bytes, and we haven't found a clean way to define the int as a global variable and send it as a byte to the Arduino. 
                        arduino.write(b'2')
                    else:
                        LAST_TURN = LEFT
                        #1 stands for "turn left"
                        arduino.write(b'1')
                    print(OUTPUT)
            else: # Changing from something detected to nothing detected
                if (OUTPUT == SOMETHING):
                    OUTPUT = NOTHING
                    #3 stands for going striaght
		    arduino.write(b'3')
                    print(OUTPUT)

#ROS routine for defining a Subscriber Node 
def ydlidar_client():
    #Node name
    rospy.init_node('lidar_obstacle_avoidance')
    #Topic that the node subsribes to 
    rospy.Subscriber("scan_filtered", LaserScan, callback)
    #Loop 
    rospy.spin()

if __name__ == '__main__':
    ydlidar_client()
