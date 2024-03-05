#include "attitude_controller.h"

Eigen::Matrix <double, 3, 1> AttitudeController::getAngularSetpoint(Eigen::Quaternion<double> sp, Eigen::Quaternion<double> meas) {
    Eigen::Quaternion<double> err = meas.conjugate() * sp;
    Eigen::Quaternion<double> errShort;
    if (err.w() < 0) {
        Eigen::Quaternion<double> flipped(-err.w(), -err.x(), -err.y(), -err.z());
        errShort = flipped;
    } else {
        errShort = err;
    }
    Eigen::Matrix <double, 3, 1> angularSetpoint;
    angularSetpoint << errShort.x(), errShort.y(), errShort.z();
    ROS_INFO("here32");
    ROS_INFO("%f", (errShort.x()));
    angularSetpoint *= (2*this->kP);
    return angularSetpoint;
} 