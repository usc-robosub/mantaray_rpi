#include "include.h"
#include "fsm.h"

extern double robotState[15];

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
    std::stringstream ss;
    for(int i = 0; i < 15; i++){
        ss << robotState[i] << " ";
    }

    ROS_INFO("Robot State: %s", ss.str().c_str());
}

std::string FSM_PControl::get_name(){
    return this->name;
}



