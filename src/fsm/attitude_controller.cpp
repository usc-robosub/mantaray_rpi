#include "attitude_controller.h"

Eigen::Matrix <double, 3, 1> AttitudeController::getAngularSetpoint(Eigen::Quaternion<double> sp, Eigen::Quaternion<double> meas) {
  
    Eigen::Quaternion<double> err = sp * meas.conjugate(); // error around each axis of rotation
    Eigen::Quaternion<double> errShort;

    if (err.w() < 0) {
        Eigen::Quaternion<double> flipped(-err.w(), -err.x(), -err.y(), -err.z());
        errShort = flipped;
    } else {
        errShort = err;
    }
    ROS_INFO("Error: (%f, %f, %f)", errShort.x(), errShort.y(), errShort.z());
    Eigen::Matrix <double, 3, 1> angularSetpoint;
    angularSetpoint << errShort.x(), errShort.y(), errShort.z();
    angularSetpoint *= 1;
    return angularSetpoint;
} 