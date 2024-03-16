#include "include.h"
#include "fsm.h"


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
    ROS_INFO("Measured Quaternion: (W=%f, X=%f, Y=%f, Z=%f)", meas.w(), meas.x(), meas.y(), meas.z());

    Eigen::Quaternion<double> qsp;
    
    qsp = Eigen::AngleAxisd(this->setpoint(3,0), Eigen::Vector3d::UnitX())
    * Eigen::AngleAxisd(this->setpoint(4,0), Eigen::Vector3d::UnitY())
    * Eigen::AngleAxisd(this->setpoint(5,0), Eigen::Vector3d::UnitZ());
    
    ROS_INFO("Desired Setpoint Quaternion: (W=%f, X=%f, Y=%f, Z=%f)", qsp.w(), qsp.x(), qsp.y(), qsp.z());


    Eigen::Matrix<double, 3, 1> sp = this->ac.getAngularSetpoint(qsp, meas);
    
    this->thruster_values[0] = sp(2,0) * 100;
    this->thruster_values[1] = -sp(2,0) * 100;
    this->thruster_values[2] = sp(2,0) * 100;
    this->thruster_values[3] = -sp(2,0) * 100;
    this->thruster_values[4] = sp(0,0) * 100 - sp(1,0) * 100;
    this->thruster_values[5] = sp(0,0) * 100 - sp(1,0) * 100;
    this->thruster_values[6] = sp(0,0) * 100 + sp(1,0) * 100;
    this->thruster_values[7] = sp(0,0) * 100 + sp(1,0) * 100;
    ROS_INFO("Robot State: %f %f %f", sp(0,0), sp(1,0), sp(2,0));  
    ROS_INFO("_____________");
 }