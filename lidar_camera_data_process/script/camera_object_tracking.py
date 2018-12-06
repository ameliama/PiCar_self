#!/usr/bin/env python

import rospy
import serial
import time
from std_msgs.msg import Float32MultiArray

arduino = serial.Serial('/dev/ttyACM0', 9600)

#Define some constants to indicate the location of the object being tracked (very rough)
LEFT = 1
RIGHT = 2
STRAIGHT = 3

#Initialize OUTPUT, the variable holding the state of the turning direction, to be straight
OUTPUT = STRAIGHT

def callback(array_msg):
    global LEFT
    global RIGHT 
    global STRAIGHT
    global OUTPUT

    #Getting published data about object coordinates
    data = array_msg.data
    #If the object is detected
    if (data):
        #Use the 7th coordinate, stored in array index 9, which gives information on the object's location on the x axis
        current_x = data[9]
        #Determine if the object is on the left, right, or right in front of the camera
        #And update the state of the wheel turning direction to make the wheels turn accordingly
        #OUTPUT will only change when the state is changed. This is to prevent sending too many data to the Arduino
        if (current_x < 300): #Indicating that the object is on the left side in the camera image frame
            if (OUTPUT != LEFT):
                OUTPUT = LEFT
                arduino.write(b'1')
                print "LEFT"
        elif (300 < current_x and current_x < 600): #If the object is in the middle
            if (OUTPUT != STRAIGHT):
                OUTPUT = STRAIGHT
                arduino.write(b'3')
                print "STRAIGHT"
        elif (sum > 600):
            if (OUTPUT != RIGHT): #If the object is on the right side
                OUTPUT = RIGHT
                arduino.write(b'2')
                print "RIGHT"
def object_track():
    rospy.init_node('camera_object_tracking')
    rospy.Subscriber("objects", Float32MultiArray, callback)
    rospy.spin()

if __name__ == '__main__':
    object_track()
