#pragma once
#include "include.h"

class AttitudeController {

    public:
    Eigen::Matrix <double, 3, 1> getAngularSetpoint(Eigen::Quaternion<double> sp, Eigen::Quaternion<double> meas);

    private:
    double kP = 1;
    

};

