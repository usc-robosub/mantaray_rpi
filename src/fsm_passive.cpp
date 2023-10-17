#include "fsm_passive.h"

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

void FSM_Passive::run(int dt){
    
}

std::string FSM_Passive::get_name(){
    return this->name;
}

