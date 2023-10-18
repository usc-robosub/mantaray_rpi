#pragma once

#include "eigen3/Eigen/Dense"

namespace AUV_STRUCTS{

typedef struct {
    double mass;                //kg
    double Fg;                  //N 
    double Fb;                  //N
    Eigen::Vector3d CoB;        // Center of buoyancy relative to center of mass (X [m], Y [m], Z [m])
    Eigen::Matrix3d inertia;    // 3x3 inertia matrix kg-m^2
    Eigen::Matrix <double, 6, 2> drag_coefficients; // 6x2 matrix of drag coefficients
    Eigen::Matrix <double, 5, 8> thruster_pose; // 5x8 matrix of thruster pose

}  AUVParameters;

typedef struct
{
   double maxXYDistance;       // [m]
   double maxZDistance;        // [m]
   double maxAlignInclination; // [rad]
   // double closingTolXYZ;      // [m]
   // double closingTolRot;      // [rad]
   Eigen::Vector3d maxTransVel;   // [m/s]
   double maxRotVel;              // [rad/s]
   Eigen::Vector3d maxTransAccel; // [m/s^2]
   double maxRotAccel;            // [rad/s^2]
   double transJerk;              // [m/s^3]
   // double xyzClosingJerk;     // [m/s^3]
   double rotJerk; // [rad/s^3]
   // double rotClosingJerk;     // [rad/^3]
} AUVConstraints;

}