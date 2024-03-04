#pragma once
#include "include.h"

class PIDController {
public:
    PIDController(int dim, double dt, double integral_bound);
    void setKp(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>);
    void setKi(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>);
    void setKd(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>);
    Eigen::Matrix<double, Eigen::Dynamic, 1> computeOutput(Eigen::Matrix<double, Eigen::Dynamic, 1>, Eigen::Matrix<double, Eigen::Dynamic, 1>);
private:
    double dt;
    double integral_bound;
    Eigen::Matrix<double, Eigen::Dynamic, 1> error, last_error, integral, deriv;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> kp, ki, kd;
};
