#include "include.h"
#include "fsm.h"

double toRad(double deg){
    return deg * M_PI / 180;
}

FSM_LQR::FSM_LQR(){

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

    this->A <<  1, 0, 0, 0, 0, 0, FSM_LQR::dt, 0, 0, 0, 0, 0, 0.5*FSM_LQR::dt*FSM_LQR::dt, 0, 0, 
                0, 1, 0, 0, 0, 0, 0, FSM_LQR::dt, 0, 0, 0, 0, 0, 0.5*FSM_LQR::dt*FSM_LQR::dt, 0, 
                0, 0, 1, 0, 0, 0, 0, 0, FSM_LQR::dt, 0, 0, 0, 0, 0, 0.5*FSM_LQR::dt*FSM_LQR::dt, 
                0, 0, 0, 1, 0, 0, 0, 0, 0, FSM_LQR::dt, 0, 0, 0, 0, 0, 
                0, 0, 0, 0, 1, 0, 0, 0, 0, 0, FSM_LQR::dt, 0, 0, 0, 0, 
                0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, FSM_LQR::dt, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, FSM_LQR::dt, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 0, FSM_LQR::dt, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 1, 0 ,0, 0, 0, 0, FSM_LQR::dt,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 1 ,0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,1, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;

    this->computeThrustCoefficients();
    this->computeLinearizedInputMatrix();

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
    
}

std::string FSM_LQR::get_name(){
    return this->name;
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
    B.block<3, 8>(3, 0) = thrustCoeffs_.block<3, 8>(0, 0);                                     // Force contributions
    B.block<3, 8>(9, 0) = auvParams_.inertia.inverse() * thrustCoeffs_.block<3, 8>(3, 0);     // Moment contributions
}

Eigen::Matrix <double, CONTROL_DIM, 1> FSM_LQR::getControlOutput(Eigen::Matrix <double, STATE_DIM, 1> state, Eigen::Matrix <double, STATE_DIM, 1> setpoint, double dt){
    Eigen::Matrix <double, STATE_DIM, 1> error = setpoint - state;
    int N = 100;

    Eigen::Matrix <double, STATE_DIM, STATE_DIM> * P = new Eigen::Matrix <double, STATE_DIM, STATE_DIM>[N];
    Eigen::Matrix <double, STATE_DIM, STATE_DIM> * K = new Eigen::Matrix <double, 1, STATE_DIM>[N];
    Eigen::Matrix <double, CONTROL_DIM, 1> * u = new Eigen::Matrix <double, CONTROL_DIM, 1>[N];

    P[N-1] = Q;

    for(int i = N-1; i > 0; i--){
        P[i-1] = Q + A.transpose() * P[i] * A - A.transpose() * P[i] * B * (R + B.transpose() * P[i] * B).inverse() * B.transpose() * P[i] * A;
    }

    for(int i = 0; i < N; i++){
        K[i] = (R + B.transpose() * P[i+1] * B).inverse() * B.transpose() * P[i+1] * A;
        u[i] = -K[i] * error;
    }

    Eigen::Matrix <double, CONTROL_DIM, 1> control_output = u[N-1];

    delete[] P;
    delete[] K;
    delete[] u;

    return control_output;
}