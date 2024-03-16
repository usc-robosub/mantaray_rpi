#pragma once
#include "include.h"

class FSM_Base {

    public:
    // FSM_Base();
    // ~FSM_Base();
    std::string getName();
    virtual void enter() = 0;
    virtual void exit() = 0;
    virtual void run(int dt) = 0;
    double* getThrusterValues();
    virtual void setSetpoint(const Eigen::Matrix<double, 15, 1>& setpoint);

    protected:
    Eigen::Matrix <double, 15, 1> setpoint;
    double robot_state[15];
    double* thruster_values;
    private:
    std::string name_ = "Base";
    
    

};