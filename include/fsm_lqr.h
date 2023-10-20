#pragma once

#include "include.h"

#include <ct/optcon/optcon.h>

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
        
        Eigen::Matrix <double, CONTROL_DIM, 1> getControlOutput(Eigen::Matrix <double, STATE_DIM, 1> state, Eigen::Matrix <double, STATE_DIM, 1> setpoint);
        void computeLinearizedInputMatrix();
        void computeThrustCoefficients();

    private:
        static constexpr double dt_ = 0.01;
        std::string name_ = "LQR";
        AUV_STRUCTS::AUVParameters auvParams_;
        Eigen::Matrix <double, STATE_DIM, STATE_DIM> Q_;        // State Cost Matrix
        Eigen::Matrix <double, CONTROL_DIM, CONTROL_DIM> R_;    // Input Cost Matrix
        Eigen::Matrix <double, STATE_DIM, STATE_DIM> A_;        // System Matrix
        Eigen::Matrix <double, STATE_DIM, CONTROL_DIM> B_;      // Control Input Matrix
        Eigen::Matrix <double, CONTROL_DIM, STATE_DIM> K_;      // LQR Gain Matrix

        Eigen::Matrix <double, 6, CONTROL_DIM> thrustCoeffs_; 
        Eigen::Matrix <double, STATE_DIM, 1> error_;
        Eigen::Matrix <double, CONTROL_DIM, 1> U_;// Thrust Output Matrix
        Eigen::Matrix <double, STATE_DIM, 1> setpoint;
        ct::optcon::LQR<STATE_DIM, CONTROL_DIM> lqrSolver_;

};