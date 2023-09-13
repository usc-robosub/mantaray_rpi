#include "ros/ros.h"
#include <ros/console.h>
#include <iostream>
#include <string>

int main(int argc, char **argv) {
    ros::init(argc, argv, "ekf_data_prep");
    ros::NodeHandle n;

    ros::Rate loop_rate(100);

    // Get Relevant ROS Params

    bool simulation = false;

    ROS_WARN_COND(!n.getParam("/simulation", simulation), "\'simulation\' wasn't defined as a param!");

    // Create publishers

    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("mantaray/imu", 1);
    

}