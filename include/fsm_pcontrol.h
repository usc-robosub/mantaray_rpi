#pragma once
#include "include.h"
#include "attitude_controller.h"

class FSM_PControl : public FSM_Base
{
    public:
    FSM_PControl();
    ~FSM_PControl();
    void enter();
    void exit();
    void run(int dt);
    std::string get_name();
    void setSetpoint(const Eigen::Matrix<double, 15, 1> setpoint);

    private:
    std::string name_ = "PControl";
    AttitudeController ac;
    Eigen::Matrix<double, 15, 1> setpoint;
    
};