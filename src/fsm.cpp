#include "include.h"
#include "fsm.h"

FSM::FSM(){
    this->state_list.push_back(std::unique_ptr<FSM_Base>(new FSM_Passive()));
    this->state_list.push_back(std::unique_ptr<FSM_Base>(new FSM_PControl()));
    this->setState(0);
}

FSM::~FSM(){

}

void FSM::run(int dt){
    this->state_list[this->stateIndex]->run(dt);
}

void FSM::setState(int stateIndex){
    this->state_list[this->stateIndex]->exit();
    this->stateIndex = stateIndex;
    this->state_list[this->stateIndex]->enter();
}

FSM_Base* FSM::getState(){
    return this->state_list.at(this->stateIndex).get();
}

int FSM::getStateIndex(){
    return this->stateIndex;
}






