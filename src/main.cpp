#include "include.h"
#include "std_msgs/Float64.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/AccelWithCovarianceStamped.h"
#include "fsm.h"

ros::Publisher* thruster_pubs;

double robotState[15] = {0.0};

void odometryFilteredListenerCallback(nav_msgs::Odometry msg);
void accelFilteredListenerCallback(geometry_msgs::AccelWithCovarianceStamped msg);

void initializeThrusters() {
    ROS_INFO("Initializing thrusters");
    std::this_thread::sleep_for(std::chrono::milliseconds(THRUSTER_INIT_DELAY));
    ROS_INFO("Thrusters initialized");
}

int main(int argc, char **argv){
    ROS_INFO("Starting mantaray_control");
    initializeThrusters();

    ros::init(argc, argv, "control");
    ros::NodeHandle nh;


    ros::Subscriber odometryFilteredListener;
    
    odometryFilteredListener = nh.subscribe<nav_msgs::Odometry>("odometry/filtered", 1, odometryFilteredListenerCallback);
    
    ros::Subscriber accelFilteredListener;
    accelFilteredListener = nh.subscribe<geometry_msgs::AccelWithCovarianceStamped>("accel/filtered", 1, accelFilteredListenerCallback);

    thruster_pubs = new ros::Publisher[THRUSTER_COUNT];
    for(int i = 0; i < THRUSTER_COUNT; i++){
        std::string topic_name = "thruster/" + std::to_string(i) + "/input";
        thruster_pubs[i] = nh.advertise<std_msgs::Float64>(topic_name, 1);
    }

    ros::Rate loop_rate(10);

    FSM fsm;
    fsm.setState(2);

    Eigen::Matrix<double, 15, 1> point;
    
    

    point << 0,0,30,0,0,0,0,0,0,0,0,0,0,0,0;
    // FSM_Base* thang = fsm.getState();
    // FSM_LQR* thing = static_cast<FSM_LQR*> (thang);
    // thing->setSetpoint(point);
    while(ros::ok()){
        
        fsm.run(100);
        
        for(int i = 0; i < THRUSTER_COUNT; i++){
            std_msgs::Float64 msg;
            double* thrustVals = fsm.getState()->getThrusterValues();
            if(thrustVals != nullptr) {
                msg.data = thrustVals[i];
            }
            thruster_pubs[i].publish(msg);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

}

void odometryFilteredListenerCallback(nav_msgs::Odometry msg) {
    robotState[0] = msg.pose.pose.position.x;
    robotState[1] = msg.pose.pose.position.y;
    robotState[2] = msg.pose.pose.position.z;

    tfScalar yaw, pitch, roll;
    tf::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
    tf::Matrix3x3 mat(q);
    mat.getRPY(robotState[3], robotState[4], robotState[5]);

    robotState[6] = msg.twist.twist.linear.x;
    robotState[7] = msg.twist.twist.linear.y;
    robotState[8] = msg.twist.twist.linear.z;
    robotState[9] = msg.twist.twist.angular.x;
    robotState[10] = msg.twist.twist.angular.y;
    robotState[11] = msg.twist.twist.angular.z;
    
}

void accelFilteredListenerCallback(geometry_msgs::AccelWithCovarianceStamped msg) {
    robotState[12] = msg.accel.accel.linear.x;
    robotState[13] = msg.accel.accel.linear.y;
    robotState[14] = msg.accel.accel.linear.z;
}