#include "include.h"
#include "fsm.h"

FSM_Passive::FSM_Passive(){
    
}

FSM_Passive::~FSM_Passive(){
    
}

void FSM_Passive::enter(){
    ROS_INFO("Entering Passive State");
}

void FSM_Passive::exit(){
    ROS_INFO("Exiting Passive State");
}

void FSM_Passive::setSetpoint(const Eigen::Matrix<double, 15, 1> setpoint) {
    this->setpoint = setpoint;
}


void FSM_Passive::run(int dt){
    
}


