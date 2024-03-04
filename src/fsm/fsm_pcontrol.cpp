#include "include.h"
#include "fsm.h"


extern double robotState[15];
extern tf::Quaternion robotQuat;

FSM_PControl::FSM_PControl(){
    
}

FSM_PControl::~FSM_PControl(){
    
}

void FSM_PControl::enter(){
    ROS_INFO("Entering PControl State");
}

void FSM_PControl::exit(){
    ROS_INFO("Exiting PControl State");
}

void FSM_PControl::run(int dt){
    Eigen::Quaternion<double> meas(robotQuat.getW(), robotQuat.getX(), robotQuat.getY(), robotQuat.getZ());
    Eigen::Quaternion<double> qsp(0,0,0,0);

    Eigen::Matrix<double, 3, 1> sp = this->ac.getAngularSetpoint(qsp, meas);
    
}




