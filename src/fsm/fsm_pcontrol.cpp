#include "include.h"
#include "fsm.h"
#include "std_msgs/Float64.h"


extern double robotState[15];
extern tf::Quaternion robotQuat;

FSM_PControl::FSM_PControl(){
    this->thruster_values = new double[8];
}

FSM_PControl::~FSM_PControl(){
    delete [] thruster_values;
}

void FSM_PControl::enter(){
    ROS_INFO("Entering PControl State");
}

void FSM_PControl::exit(){
    ROS_INFO("Exiting PControl State");
}

void FSM_PControl::run(int dt){
    Eigen::Quaternion<double> meas(robotQuat.getW(), robotQuat.getX(), robotQuat.getY(), robotQuat.getZ());
    Eigen::Quaternion<double> qsp;
    double roll = 0, pitch = 0, yaw = 1.57;    
    qsp = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
    * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
    * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

    Eigen::Matrix<double, 3, 1> sp = this->ac.getAngularSetpoint(qsp, meas);
    
    
    // this->thruster_values[0] = sp(2,0);
    // this->thruster_values[1] = sp(2,0);
    // this->thruster_values[2] = sp(2,0);
    // this->thruster_values[3] = sp(2,0);
    // this->thruster_values[4] = sp(0,0) - sp(1,0);
    // this->thruster_values[5] = sp(0,0) - sp(1,0);
    // this->thruster_values[6] = sp(0,0) + sp(1,0);
    // this->thruster_values[7] = sp(0,0) + sp(1,0);


    
    ROS_INFO("Robot State: %f %f %f", robotState[3], robotState[4], robotState[5]);
    
 }