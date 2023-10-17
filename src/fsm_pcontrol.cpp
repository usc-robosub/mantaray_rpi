#include "include.h"
#include "fsm.h"

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
    
}

std::string FSM_PControl::get_name(){
    return this->name;
}

