/*
 *  YDLIDAR SYSTEM
 *  YDLIDAR ROS Node Client 
 *
 *  Copyright 2015 - 2018 EAI TEAM
 *  http://www.ydlidar.com
 *
 *  Modified by Amelia Ma 10/21/2018 
 */

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#define RAD2DEG(x) ((x)*180./M_PI)

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int count = scan->scan_time / scan->time_increment;
    //create and initialize array for mean filter
    //store the mean of current value + 9 previous values into the current index
    float filter[count];
    for (int i = 0; i < count; i++){
	filter[i] = 0;
    }
    //printf("[YDLIDAR INFO]: I heard a laser scan %s[%d]:\n", scan->header.frame_id.c_str(), count);
    //printf("[YDLIDAR INFO]: angle_range : [%f, %f]\n", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));
  
    for(int i = 0; i < count; i++) {
	//calculate average value for current index
        float sum = scan->ranges[i]; //current value from sensor 
	int i_copy = i; //copy i
	for (int j = 0; j < 9; j++){ //Add 9 more previous values
	    i_copy--;
	    if (i_copy < 0) i_copy = count - 1; //if past the start of the array, reset to end
	    sum += filter[i_copy];
	}
	sum /= 10; 
	filter[i] = sum; //put average value into the array
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
	if(degree >=1 && degree <= 1.5){
	    //printf("average: %f\n", sum);
            //printf("[YDLIDAR INFO]: angle-distance : [%f, %f, %i]\n", degree, scan->ranges[i], i);
    	    //printf("[YDLIDAR INFO]: angle-distance filtered: [%f, %f, %i]\n", degree, filter[i], i);
	    if ((filter[i] <0.1) && (filter[i-1]<0.1)&&(filter[i-2]<0.1)) printf("stuff\n");
	    else printf("nothing\n");
	}
    }
}

//void mean_filter (vector<double> & filter, )

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ydlidar_client_modified");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);
    ros::spin();

    return 0;
}
