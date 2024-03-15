#include "include.h"
#include "fsm.h"

double toRad(double deg){
    return deg * M_PI / 180;
}

extern double robotState[15];

FSM_LQR::FSM_LQR(){
    this->dt_ = 0.01;

    AUV_STRUCTS::AUVParameters params;
    params.thruster_pose << -0.25, -0.333, 0.145, toRad(300), 0,
                            0.25, -0.333, 0.145, toRad(240), 0,
                            0.25, 0.333, 0.145, toRad(120), 0,
                            -0.25, 0.333, 0.145, toRad(60), 0,
                            -0.25, 0.19, 0.19, toRad(90), 0, 
                            0.25, 0.19, 0.19, toRad(90), 0,
                            -0.25, -0.19, 0.19, toRad(270), 0,
                            0.25, -0.19, 0.19, toRad(270), 0;
    params.mass = 22.102392;
    params.CoB = Eigen::Vector3d(0, 0, 0);
    params.inertia << 1.343163, 0.000003, 0.000132,
                      0, 1.793012, 0.000084,
                      0, 0, 0.649198;
    params.Fg = params.mass * 9.81;
    params.Fb = params.Fg; // measure buoyant force

    this->auvParams_ = params;

    this->A_ <<  1, 0, 0, 0, 0, 0, this->dt_, 0, 0, 0, 0, 0, 0.5*this->dt_*this->dt_, 0, 0, 
                0, 1, 0, 0, 0, 0, 0, this->dt_, 0, 0, 0, 0, 0, 0.5*this->dt_*this->dt_, 0, 
                0, 0, 1, 0, 0, 0, 0, 0, this->dt_, 0, 0, 0, 0, 0, 0.5*this->dt_*this->dt_, 
                0, 0, 0, 1, 0, 0, 0, 0, 0, this->dt_, 0, 0, 0, 0, 0, 
                0, 0, 0, 0, 1, 0, 0, 0, 0, 0, this->dt_, 0, 0, 0, 0, 
                0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, this->dt_, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, this->dt_, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 0, this->dt_, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 1, 0 ,0, 0, 0, 0, this->dt_,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 1 ,0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,1, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;

    this->Q_.diagonal() << 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1;
    this->R_.diagonal() << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01;

    this->computeThrustCoefficients();
    this->computeLinearizedInputMatrix();

}

void FSM_LQR::setSetpoint(Eigen::Matrix <double, STATE_DIM, 1> setpoint) {
    this->setpoint = setpoint;
}

FSM_LQR::~FSM_LQR(){
    
}

void FSM_LQR::enter(){
    ROS_INFO("Entering LQR State");
}

void FSM_LQR::exit(){
    ROS_INFO("Exiting LQR State");
}   

void FSM_LQR::run(int dt){
    Eigen::Matrix <double, STATE_DIM, 1> robot_state;
    for (int i = 0; i < STATE_DIM; i++) {
        robot_state << robotState[i];
    }
    this->getControlOutput(robot_state, this->setpoint);
    this->thruster_values = this->U_.data();
}

void FSM_LQR::computeThrustCoefficients() {
    for (int i = 0; i < CONTROL_DIM; i++)
    {
        float psi = auvParams_.thruster_pose(3, i) * M_PI / 180;
        float theta = auvParams_.thruster_pose(4, i) * M_PI / 180;
        thrustCoeffs_(0, i) = cos(psi) * cos(theta); // cos(psi)cos(theta)
        thrustCoeffs_(1, i) = sin(psi) * cos(theta); // sin(psi)cos(theta)
        thrustCoeffs_(2, i) = -sin(theta);           // -sin(theta)

        // Cross-product
        thrustCoeffs_.block<3, 1>(3, i) = auvParams_.thruster_pose.block<3, 1>(0, i).cross(thrustCoeffs_.block<3, 1>(0, i));
    }
}

void FSM_LQR::computeLinearizedInputMatrix() {
    B_.block<3, 8>(3, 0) = thrustCoeffs_.block<3, 8>(0, 0);                                     // Force contributions
    B_.block<3, 8>(9, 0) = auvParams_.inertia.inverse() * thrustCoeffs_.block<3, 8>(3, 0);     // Moment contributions
}

Eigen::Matrix <double, CONTROL_DIM, 1> FSM_LQR::getControlOutput(Eigen::Matrix <double, STATE_DIM, 1> state, Eigen::Matrix <double, STATE_DIM, 1> setpoint){
    this->error_ = state - setpoint;

    this->lqrSolver_.compute(this->Q_, this->R_, this->A_, this->B_, this->K_, true); // R is diagonal so set flag to true

    this->U_ = -K_*this->error_;
    return this->U_;
}