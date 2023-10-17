#include "include.h"
#include "fsm.h"

FSM_Base::FSM_Base(){
    ros::Subscriber robotStateFilteredListener;
    std::string topicName = SUB_NAME + "/odometry/filtered"
    robotStateFilteredListener = nh.subscribe<nav_msgs::Odometry>(topicName, 1, this->robotStateFilteredListenerCallback);
    
}

std::string FSM_Base::getName(){
    return this->name;
}

void FSM_Base::robotStateFilteredListenerCallback(nav_msgs::Odometry msg) {
    this->robotState[0] = msg.pose.pose.position.x
    this->robotState[1] = msg.pose.pose.position.y
    this->robotState[2] = msg.pose.pose.position.z

    tfScalar yaw, pitch, roll;
    tf::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
    tf::Matrix3x3 mat(q);
    mat.getRPY(this->robotState[3], this->robotState[4], this->robotState[5]);

    this->robotState[6] = msg.twist.twist.linear.x;
    this->robotState[7] = msg.twist.twist.linear.y;
    this->robotState[8] = msg.twist.twist.linear.z;
    this->robotState[9] = msg.twist.twist.angular.x;
    this->robotState[10] = msg.twist.twist.angular.y;
    this->robotState[11] = msg.twist.twist.angular.z;
    
    std::stringstream ss();
    for (int i = 0; i < 12; i++) {
        ss << std::toString(robotState[i]);
        ss << " "
    }

    ROS_INFO(ss.str());
}