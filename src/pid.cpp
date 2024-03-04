#include "pid.h"

PIDController::PIDController(int dim, double dt, double integral_bound){
    kp.resize(dim, dim);
    ki.resize(dim, dim);
    kd.resize(dim, dim);

    error.resize(dim, 1);
    integral.resize(dim, 1);
    deriv.resize(dim, 1);
    last_error.resize(dim, 1);
    last_error.fill(0);

    this->dt = dt;
    this->integral_bound = integral_bound;
}

void PIDController::setKp(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> kp){
    this->kp = kp;
}

void PIDController::setKi(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> ki){
    this->ki = ki;
}

void PIDController::setKd(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> kd){
    this->kd = kd;
}

Eigen::Matrix<double, Eigen::Dynamic, 1> PIDController::computeOutput(Eigen::Matrix<double, Eigen::Dynamic, 1> sp, Eigen::Matrix<double, Eigen::Dynamic, 1> meas){
    this->error = sp-meas;
    this->integral += error*dt;
    this->integral.cwiseMin(integral_bound)
         .cwiseMax(-integral_bound);
    this->deriv = (error - last_error)/dt;

    this->last_error = error;
    
    return kp*error + ki*integral + kd*deriv;
}
