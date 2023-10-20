#pragma once
#include "include.h"


class FSM_Base {

    public:
    std::string getName();
    virtual void enter() = 0;
    virtual void exit() = 0;
    virtual void run(int dt) = 0;
    double* getThrusterValues();

    protected:
    double robot_state[15];
    double* thruster_values;
    private:
    std::string name_ = "Base";
    
    

};