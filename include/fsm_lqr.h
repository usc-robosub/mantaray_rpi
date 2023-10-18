#pragma once

#include "include.h"

#define STATE_DIM 15
#define CONTROL_DIM 8

class FSM_LQR : public FSM_Base {
    public:
        FSM_LQR();
        ~FSM_LQR();
        void enter();
        void exit();
        void run(int dt);
        std::string get_name();
        
        Eigen::Matrix <double, CONTROL_DIM, 1> getControlOutput(Eigen::Matrix <double, STATE_DIM, 1> state, Eigen::Matrix <double, STATE_DIM, 1> setpoint, double dt);
        void computeLinearizedInputMatrix();
        void computeThrustCoefficients();



    private:
        static constexpr double dt = 0.01;
        std::string name = "LQR";
        AUV_STRUCTS::AUVParameters auvParams_;
        Eigen::Matrix <double, STATE_DIM, STATE_DIM> Q;
        Eigen::Matrix <double, CONTROL_DIM, CONTROL_DIM> R;
        Eigen::Matrix <double, STATE_DIM, STATE_DIM> A;
        Eigen::Matrix <double, STATE_DIM, CONTROL_DIM> B;
        Eigen::Matrix <double, 6, CONTROL_DIM> thrustCoeffs_;

};